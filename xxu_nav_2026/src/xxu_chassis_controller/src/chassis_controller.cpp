#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

namespace xxu_chassis_controller
{

class ChassisController : public rclcpp::Node
{
public:
  ChassisController()
  : Node("chassis_controller")
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.0762);
    wheel_command_sign_ = this->declare_parameter<double>("wheel_command_sign", 1.0);
    max_wheel_speed_ = this->declare_parameter<double>("max_wheel_speed", 100.0);
    timeout_ = this->declare_parameter<double>("timeout", 0.3);
    max_wheel_acceleration_ = this->declare_parameter<double>("max_wheel_acceleration", 0.0);
    require_fresh_stamp_ = this->declare_parameter<bool>("require_fresh_stamp", false);
    const auto wheel_topics = this->declare_parameter<std::vector<std::string>>(
      "wheel_command_topics", std::vector<std::string>{});
    const auto wheel_radii = this->declare_parameter<std::vector<double>>(
      "wheel_radii", std::vector<double>{});
    if (!wheel_topics.empty() && wheel_topics.size() != 4) {
      throw std::invalid_argument("wheel_command_topics must be empty or contain 4 topics");
    }
    if (!wheel_radii.empty() && wheel_radii.size() != 4) {
      throw std::invalid_argument("wheel_radii must be empty or contain 4 positive radii");
    }
    if (!std::isfinite(max_wheel_acceleration_) || max_wheel_acceleration_ < 0.0) {
      throw std::invalid_argument("max_wheel_acceleration must be finite and nonnegative");
    }
    const double publish_rate = this->declare_parameter<double>("publish_rate", 50.0);
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    wheel_command_topic_ = this->declare_parameter<std::string>(
      "wheel_command_topic", "/wheel_velocity_controller/commands");
    expected_frame_id_ = this->declare_parameter<std::string>("expected_frame_id", "base_link");
    accept_empty_frame_id_ = this->declare_parameter<bool>("accept_empty_frame_id", true);

    wheel_x_ = readWheelArray(
      "wheel_x",
      {0.223275, 0.223275, -0.223275, -0.223275});
    wheel_y_ = readWheelArray(
      "wheel_y",
      {0.223275, -0.223275, 0.223275, -0.223275});
    drive_direction_angle_ = readWheelArray(
      "drive_direction_angle",
      {-0.785398, -2.356194, 0.785398, 2.356194});

    if (wheel_radius_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "wheel_radius must be positive; using fallback 0.0762");
      wheel_radius_ = 0.0762;
    }
    if (max_wheel_speed_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "max_wheel_speed must be positive; using fallback 100.0");
      max_wheel_speed_ = 100.0;
    }

    wheel_radii_.fill(wheel_radius_);
    if (!wheel_radii.empty()) {
      for (size_t i = 0; i < wheel_radii_.size(); ++i) {
        if (!std::isfinite(wheel_radii[i]) || wheel_radii[i] <= 0.0) {
          throw std::invalid_argument("wheel_radii must contain positive finite values");
        }
        wheel_radii_[i] = wheel_radii[i];
      }
    }
    previous_tick_ = this->now();
    const double safe_publish_rate = publish_rate > 0.0 ? publish_rate : 50.0;

    if (wheel_topics.empty()) {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        wheel_command_topic_, rclcpp::QoS(rclcpp::KeepLast(10)));
    } else {
      for (const auto & topic : wheel_topics) {
        wheel_publishers_.push_back(this->create_publisher<std_msgs::msg::Float64>(topic, 10));
      }
    }

    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&ChassisController::cmdVelCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / safe_publish_rate),
      std::bind(&ChassisController::publishWheelCommands, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Omni chassis controller: wheel_radius=%.4f, max_wheel_speed=%.2f, "
      "timeout=%.2f, publish_rate=%.1f, input=%s, output=%s",
      wheel_radius_, max_wheel_speed_, timeout_, safe_publish_rate,
      cmd_vel_topic_.c_str(), wheel_command_topic_.c_str());
  }

private:
  using WheelArray = std::array<double, 4>;

  WheelArray readWheelArray(
    const std::string & parameter_name,
    const WheelArray & defaults)
  {
    const std::vector<double> default_vector(defaults.begin(), defaults.end());
    const auto values = this->declare_parameter<std::vector<double>>(
      parameter_name,
      default_vector);

    if (values.size() != defaults.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s must contain 4 values; using defaults",
        parameter_name.c_str());
      return defaults;
    }

    WheelArray result{};
    std::copy(values.begin(), values.end(), result.begin());
    return result;
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!accept_empty_frame_id_ && msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting TwistStamped with an empty frame_id");
      return;
    }
    if (!msg->header.frame_id.empty() &&
      !expected_frame_id_.empty() && msg->header.frame_id != expected_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting TwistStamped in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), expected_frame_id_.c_str());
      return;
    }
    const auto & twist = msg->twist;
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
      !std::isfinite(twist.linear.z) || !std::isfinite(twist.angular.x) ||
      !std::isfinite(twist.angular.y) || !std::isfinite(twist.angular.z))
    {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting non-finite TwistStamped command");
      return;
    }
    if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting TwistStamped with an invalid timestamp");
      has_cmd_ = false;
      return;
    }
    last_cmd_ = msg->twist;
    last_cmd_time_ = this->now();
    last_cmd_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    last_received_steady_ = std::chrono::steady_clock::now();
    has_cmd_ = true;
  }

  void publishWheelCommands()
  {
    geometry_msgs::msg::Twist cmd;
    if (has_cmd_) {
      const double age = (this->now() - last_cmd_time_).seconds();
      bool fresh = age >= 0.0 && age <= timeout_;
      if (require_fresh_stamp_) {
        const double stamp_age = (this->now() - last_cmd_stamp_).seconds();
        const double receive_age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_received_steady_).count();
        fresh = fresh && stamp_age >= 0.0 && stamp_age <= timeout_ && receive_age <= timeout_;
      }
      if (fresh) {
        cmd = last_cmd_;
      }
    }

    auto speeds = computeWheelSpeeds(cmd);
    limitWheelSpeeds(speeds);

    limitWheelAcceleration(speeds, cmd);
    if (publisher_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data.assign(speeds.begin(), speeds.end());
      publisher_->publish(msg);
    } else {
      for (size_t i = 0; i < speeds.size(); ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = speeds[i];
        wheel_publishers_[i]->publish(msg);
      }
    }
  }

  WheelArray computeWheelSpeeds(const geometry_msgs::msg::Twist & cmd) const
  {
    WheelArray speeds{};
    const double vx = cmd.linear.x;
    const double vy = cmd.linear.y;
    const double wz = cmd.angular.z;

    for (size_t i = 0; i < speeds.size(); ++i) {
      const double contact_vx = vx - wz * wheel_y_[i];
      const double contact_vy = vy + wz * wheel_x_[i];
      const double tx = std::cos(drive_direction_angle_[i]);
      const double ty = std::sin(drive_direction_angle_[i]);
      speeds[i] = wheel_command_sign_ * (tx * contact_vx + ty * contact_vy) / wheel_radii_[i];
    }

    return speeds;
  }

  void limitWheelAcceleration(WheelArray & speeds, const geometry_msgs::msg::Twist & cmd)
  {
    const auto now = this->now();
    const double dt = std::clamp((now - previous_tick_).seconds(), 0.0, 0.05);
    previous_tick_ = now;
    const bool stopping = cmd.linear.x == 0.0 && cmd.linear.y == 0.0 && cmd.angular.z == 0.0;
    if (max_wheel_acceleration_ > 0.0 && !stopping) {
      double max_change = 1e-12;
      for (size_t i = 0; i < speeds.size(); ++i) {
        max_change = std::max(max_change, std::abs(speeds[i] - previous_speeds_[i]));
      }
      const double factor = std::min(1.0, max_wheel_acceleration_ * dt / max_change);
      for (size_t i = 0; i < speeds.size(); ++i) {
        speeds[i] = previous_speeds_[i] + factor * (speeds[i] - previous_speeds_[i]);
      }
    }
    // A safety stop bypasses the ramp, including command/clock timeout stops.
    previous_speeds_ = speeds;
  }

  void limitWheelSpeeds(WheelArray & speeds) const
  {
    double max_abs_speed = 0.0;
    for (const double speed : speeds) {
      max_abs_speed = std::max(max_abs_speed, std::abs(speed));
    }

    if (max_abs_speed <= max_wheel_speed_) {
      return;
    }

    const double scale = max_wheel_speed_ / max_abs_speed;
    for (double & speed : speeds) {
      speed *= scale;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};
  bool require_fresh_stamp_{false};
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_tick_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_received_steady_;
  WheelArray wheel_radii_{};
  WheelArray previous_speeds_{};
  double max_wheel_acceleration_{0.0};

  WheelArray wheel_x_{};
  WheelArray wheel_y_{};
  WheelArray drive_direction_angle_{};
  double wheel_radius_{0.0762};
  double wheel_command_sign_{1.0};
  double max_wheel_speed_{100.0};
  double timeout_{0.3};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string wheel_command_topic_{"/wheel_velocity_controller/commands"};
  std::string expected_frame_id_{"base_link"};
  bool accept_empty_frame_id_{true};
};

}  // namespace xxu_chassis_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xxu_chassis_controller::ChassisController>());
  rclcpp::shutdown();
  return 0;
}
