#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace xxu_sim_bringup {
double wrap(double x) {return std::atan2(std::sin(x), std::cos(x));}
double yaw(const geometry_msgs::msg::Quaternion & q)
{return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));}
double steady()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

class RmSimAdapter final : public rclcpp::Node
{
public:
  explicit RmSimAdapter(const rclcpp::NodeOptions & options) : Node("rm_sim_adapter", options)
  {
    gimbal_ = create_publisher<std_msgs::msg::Float64>(
      declare_parameter("gimbal_topic", "/gimbal/big_yaw/cmd_pos"), 10);
    velocity_ = create_publisher<geometry_msgs::msg::Twist>(declare_parameter("output_topic", "/cmd_vel"), 10);
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      declare_parameter("input_topic", "/rm_sim/cmd_vel_safe"), rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {command_ = *m; received_ = steady();});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter("joint_states_topic", "/joint_states"), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        const auto it = std::find(m->name.begin(), m->name.end(), "big_yaw_joint");
        const auto i = static_cast<std::size_t>(it - m->name.begin());
        if (it != m->name.end() && i < m->position.size() && std::isfinite(m->position[i])) {position_ = m->position[i];}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/odom"),
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        if (!position_) {return;}
        const double y = yaw(m->pose.pose.orientation);
        if (!std::isfinite(y)) {return;}
        target_ = last_yaw_ ? target_ - wrap(y - *last_yaw_) : *position_;
        last_yaw_ = y;
        std_msgs::msg::Float64 out; out.data = target_; gimbal_->publish(out);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {
      geometry_msgs::msg::Twist output;
      const auto & t = command_.twist;
      const double age = (now() - rclcpp::Time(command_.header.stamp)).seconds();
      if (received_ && steady() - *received_ <= .3 && age >= 0 && age <= .3 &&
        std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
        std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z)) {output = t;}
      velocity_->publish(output);
    });
  }
private:
  std::optional<double> position_, last_yaw_, received_;
  double target_ = 0.;
  geometry_msgs::msg::TwistStamped command_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class GimbalStabilizer final : public rclcpp::Node
{
public:
  explicit GimbalStabilizer(const rclcpp::NodeOptions & options) : Node("gimbal_stabilizer", options)
  {
    joint_ = declare_parameter("joint_name", "gimbal_joint");
    mode_ = declare_parameter("mode", "spin");
    rate_ = declare_parameter("spin_rate", 6.283185307179586);
    delay_ = declare_parameter("spin_start_delay", 1.);
    wait_ = declare_parameter("wait_for_odom", true);
    gain_ = declare_parameter("hold_gain", 4.);
    limit_ = std::abs(declare_parameter("max_hold_velocity", 2.));
    const double command_rate = declare_parameter("command_rate", 50.);
    if ((mode_ != "spin" && mode_ != "hold") || !std::isfinite(command_rate) || command_rate <= 0) {
      throw std::invalid_argument("invalid gimbal mode/rate");
    }
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      declare_parameter("command_topic", "/gimbal_velocity_controller/commands"), 10);
    joints_ = create_subscription<sensor_msgs::msg::JointState>(declare_parameter("joint_states_topic", "/joint_states"),
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        auto it = std::find(m->name.begin(), m->name.end(), joint_);
        auto i = static_cast<std::size_t>(it - m->name.begin());
        if (it != m->name.end() && i < m->position.size() && std::isfinite(m->position[i])) {position_ = m->position[i];}
      });
    imu_ = create_subscription<sensor_msgs::msg::Imu>(declare_parameter("imu_topic", "/imu_raw"),
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        if (mode_ != "hold" || !position_) {return;}
        auto q = m->orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(norm) || norm < 1.e-6) {return;}
        q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
        const double y = yaw(q);
        if (!hold_yaw_) {hold_yaw_ = y;}
        publish(std::clamp(gain_ * wrap(*hold_yaw_ - y), -limit_, limit_));
      });
    odom_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter("odom_topic", "/odom"),
      rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
        if (!odom_ready_) {odom_ready_ = now().seconds();}
      });
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1. / command_rate), [this] {
      if (mode_ != "spin" || !position_) {return;}
      const double time = now().seconds();
      if (!start_) {start_ = time;}
      if (wait_ && !odom_ready_) {publish(0.); return;}
      publish(time - (wait_ ? *odom_ready_ : *start_) < delay_ ? 0. : rate_);
    });
  }
private:
  void publish(double velocity)
  {std_msgs::msg::Float64MultiArray m; m.data = {velocity}; publisher_->publish(m);}
  std::string joint_, mode_;
  double rate_, delay_, gain_, limit_;
  bool wait_;
  std::optional<double> position_, hold_yaw_, odom_ready_, start_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};
class OmniDrive final : public rclcpp::Node
{
public:
  explicit OmniDrive(const rclcpp::NodeOptions & options) : Node("omni_drive", options)
  {
    const std::array<std::string, 4> wheels{"wheel_fl", "wheel_rl", "wheel_rr", "wheel_fr"};
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      publishers_[i] = create_publisher<std_msgs::msg::Float64>("/" + wheels[i] + "/cmd_vel", 10);
    }
    input_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        const bool valid = std::isfinite(m->linear.x) && std::isfinite(m->linear.y) && std::isfinite(m->angular.z);
        const std::array<double, 4> degrees{45., 135., -135., -45.};
        for (std::size_t i = 0; i < degrees.size(); ++i) {
          const double angle = degrees[i] * std::acos(-1.) / 180.;
          std_msgs::msg::Float64 out;
          out.data = valid ? -(-m->linear.x * std::sin(angle) + m->linear.y * std::cos(angle) + m->angular.z * .245) / .076 : 0.;
          publishers_[i]->publish(out);
        }
      });
  }
private:
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> publishers_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
};
}  // namespace xxu_sim_bringup
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::RmSimAdapter)
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::GimbalStabilizer)
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::OmniDrive)
