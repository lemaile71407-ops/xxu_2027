#include "xxu_bringup/sensor_math.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace xxu_bringup {
class GimbalImuCompensator final : public rclcpp::Node
{
public:
  explicit GimbalImuCompensator(const rclcpp::NodeOptions & options)
  : Node("gimbal_imu_compensator", options)
  {
    history_.name = declare_parameter("joint_name", "gimbal_joint");
    fixed_ = declare_parameter("fixed_sensor", false);
    interval_samples_ = declare_parameter("imu_interval_samples", 1);
    if ((interval_samples_ != 1 && interval_samples_ != 10) || (interval_samples_ > 1 && !fixed_)) {
      throw std::invalid_argument("interval mean requires fixed_sensor and 1 or 10 samples");
    }
    expected_ = declare_parameter("expected_source_frame", "");
    target_ = declare_parameter("target_frame", "lio_base_sensor");
    axis_ = vector_parameter(*this, "joint_axis", {0., 0., 1.});
    if (axis_.norm() < 1.e-12) {throw std::invalid_argument("zero joint axis");}
    axis_.normalize();
    offset_ = vector_parameter(*this, "sensor_offset", {0., .08637, 0.});
    sensor_rotation_ = rpy_rotation(vector_parameter(*this, "sensor_rpy", {-.2967059728, 0., 0.}));
    position_offset_ = declare_parameter("joint_position_offset", 0.);
    if (!std::isfinite(position_offset_)) {throw std::invalid_argument("nonfinite joint position offset");}
    scale_ = checked_seconds(*this, "input_acceleration_scale", 1.);
    alpha_tau_ = checked_seconds(*this, "angular_acceleration_time_constant", .05, true);
    history_.acceleration_tau = checked_seconds(*this, "joint_acceleration_time_constant", .05, true);
    history_.maximum_acceleration = checked_seconds(*this, "max_joint_acceleration", 100.);
    history_.gap = checked_seconds(*this, "max_joint_sample_gap", .05);
    history_.reset_threshold = checked_seconds(*this, "time_reset_threshold", .5);
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", rclcpp::SensorDataQoS());
    if (!fixed_) {
      joints_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          if (history_.add(*m, true)) {reset_derivatives(); pending_.clear();}
          process_pending();
        });
    }
    input_ = create_subscription<sensor_msgs::msg::Imu>("imu_in",
      interval_samples_ > 1 ? rclcpp::SensorDataQoS().keep_last(4000) : rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {on_input(*m);});
  }
private:
  static Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & v) {return {v.x, v.y, v.z};}
  static void assign(geometry_msgs::msg::Vector3 & v, const Eigen::Vector3d & x)
  {v.x = x.x(); v.y = x.y(); v.z = x.z();}
  static void rotate_cov(std::array<double, 9> & covariance, const Eigen::Matrix3d & r, double scale)
  {
    if (covariance[0] < 0) {return;}
    using RowMatrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    const RowMatrix input = Eigen::Map<const RowMatrix>(covariance.data());
    Eigen::Map<RowMatrix>(covariance.data()) = scale * scale * r * input * r.transpose();
  }
  void on_input(const sensor_msgs::msg::Imu & message)
  {
    if ((!expected_.empty() && message.header.frame_id != expected_) ||
      stamp_ns(message.header.stamp) < 0 || !vector(message.angular_velocity).allFinite() ||
      !vector(message.linear_acceleration).allFinite())
    {
      interval_.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting invalid IMU frame/time/values");
      return;
    }
    if (interval_samples_ == 1) {on_imu(message); return;}
    if (!interval_.empty() && (stamp_ns(message.header.stamp) - stamp_ns(interval_.back().header.stamp) != 1000000 ||
      message.header.frame_id != interval_.back().header.frame_id)) {interval_.clear();}
    interval_.push_back(message);
    if (interval_.size() < static_cast<std::size_t>(interval_samples_)) {return;}
    auto output = interval_.back();
    const auto midpoint = (stamp_ns(interval_.front().header.stamp) + stamp_ns(message.header.stamp)) / 2;
    output.header.stamp = rclcpp::Time(midpoint);
    Eigen::Vector3d omega = Eigen::Vector3d::Zero(), accel = Eigen::Vector3d::Zero();
    for (const auto & row : interval_) {omega += vector(row.angular_velocity); accel += vector(row.linear_acceleration);}
    assign(output.angular_velocity, omega / interval_samples_);
    assign(output.linear_acceleration, accel / interval_samples_);
    for (auto * covariance : {&output.angular_velocity_covariance, &output.linear_acceleration_covariance}) {
      if ((*covariance)[0] >= 0) {for (auto & value : *covariance) {value /= interval_samples_;}}
    }
    interval_.clear();
    on_imu(output);
  }
  void reset_derivatives()
  {
    previous_time_.reset(); previous_omega_.reset(); filtered_alpha_.setZero();
  }
  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const double time = seconds(message.header.stamp);
    std::optional<double> latest = previous_time_;
    if (!pending_.empty()) {latest = seconds(pending_.back().header.stamp);}
    if (latest && time <= *latest) {
      if (*latest - time < history_.reset_threshold) {return;}
      pending_.clear(); reset_derivatives(); history_.reset_derivative();
    }
    if (fixed_) {publish(message, {time, 0., 0., 0.}); return;}
    if (pending_.size() >= 256) {pending_.pop_front();}
    pending_.push_back(message);
    process_pending();
  }
  void process_pending()
  {
    while (!pending_.empty()) {
      const double time = seconds(pending_.front().header.stamp);
      const auto joint = history_.interpolate(time);
      if (!joint) {
        if (!history_.samples.empty() && time <= history_.samples.back().time) {pending_.pop_front(); continue;}
        break;
      }
      publish(pending_.front(), *joint);
      pending_.pop_front();
    }
  }
  void publish(const sensor_msgs::msg::Imu & message, const JointHistory::Sample & joint)
  {
    const Eigen::Matrix3d joint_rotation = Eigen::AngleAxisd(joint.position + position_offset_, axis_).toRotationMatrix();
    const Eigen::Matrix3d r = joint_rotation * sensor_rotation_;
    const Eigen::Vector3d lever = joint_rotation * offset_;
    const Eigen::Vector3d relative_omega = axis_ * joint.velocity;
    const Eigen::Vector3d omega = r * vector(message.angular_velocity) - relative_omega;
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
    if (previous_omega_ && previous_time_ && joint.time > *previous_time_) {
      const double dt = joint.time - *previous_time_;
      filtered_alpha_ += (alpha_tau_ == 0 ? 1. : dt / (alpha_tau_ + dt)) *
        ((omega - *previous_omega_) / dt - filtered_alpha_);
      alpha = filtered_alpha_;
    }
    previous_time_ = joint.time; previous_omega_ = omega;
    const Eigen::Vector3d origin_accel = alpha.cross(lever) + omega.cross(omega.cross(lever)) +
      2. * omega.cross(relative_omega.cross(lever)) + (axis_ * joint.acceleration).cross(lever) +
      relative_omega.cross(relative_omega.cross(lever));
    const Eigen::Vector3d force = r * (scale_ * vector(message.linear_acceleration)) - origin_accel;
    auto output = message;
    output.header.frame_id = target_;
    assign(output.angular_velocity, omega); assign(output.linear_acceleration, force);
    output.orientation = geometry_msgs::msg::Quaternion(); output.orientation.w = 1.;
    output.orientation_covariance.fill(0.); output.orientation_covariance[0] = -1.;
    rotate_cov(output.angular_velocity_covariance, r, 1.);
    rotate_cov(output.linear_acceleration_covariance, r, scale_);
    publisher_->publish(output);
  }
  JointHistory history_;
  bool fixed_;
  int interval_samples_;
  double position_offset_, scale_, alpha_tau_;
  std::string expected_, target_;
  Eigen::Vector3d axis_, offset_, filtered_alpha_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sensor_rotation_;
  std::optional<double> previous_time_;
  std::optional<Eigen::Vector3d> previous_omega_;
  std::vector<sensor_msgs::msg::Imu> interval_;
  std::deque<sensor_msgs::msg::Imu> pending_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr input_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
};
}  // namespace xxu_bringup
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::GimbalImuCompensator)
