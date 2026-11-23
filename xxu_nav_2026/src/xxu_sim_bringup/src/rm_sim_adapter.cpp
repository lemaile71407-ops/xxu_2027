#include "sim_math.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace xxu_sim_bringup
{
namespace
{

struct RmSimAdapterConfig {
  std::string gimbal_topic;
  std::string output_topic;
  std::string input_topic;
  std::string joint_states_topic;
  std::string odom_topic;
};

bool is_finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

}  // namespace

class RmSimAdapter final : public rclcpp::Node
{
public:
  explicit RmSimAdapter(const rclcpp::NodeOptions & options) : Node("rm_sim_adapter", options)
  {
    load_config();
    init_interfaces();
  }

private:
  void load_config()
  {
    config_.gimbal_topic = declare_parameter("gimbal_topic", "/gimbal/big_yaw/cmd_pos");
    config_.output_topic = declare_parameter("output_topic", "/cmd_vel");
    config_.input_topic = declare_parameter("input_topic", "/rm_sim/cmd_vel_safe");
    config_.joint_states_topic = declare_parameter("joint_states_topic", "/joint_states");
    config_.odom_topic = declare_parameter("odom_topic", "/odom");
  }

  void init_interfaces()
  {
    gimbal_pub_ = create_publisher<std_msgs::msg::Float64>(config_.gimbal_topic, 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(config_.output_topic, 10);

    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      config_.input_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
        latest_command_ = *message;
        command_received_sec_ = steady_seconds();
      });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) { on_joint_state(*message); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odometry(*message); });

    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { publish_velocity(); });
  }

  void on_joint_state(const sensor_msgs::msg::JointState & message)
  {
    const auto it = std::find(message.name.begin(), message.name.end(), "big_yaw_joint");
    const auto index = static_cast<std::size_t>(it - message.name.begin());
    if (it != message.name.end() && index < message.position.size() &&
        std::isfinite(message.position[index])) {
      big_yaw_position_rad_ = message.position[index];
    }
  }

  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    if (!big_yaw_position_rad_) {
      return;
    }
    const double current_yaw_rad = quaternion_yaw(message.pose.pose.orientation);
    if (!std::isfinite(current_yaw_rad)) {
      return;
    }

    gimbal_target_rad_ = last_odom_yaw_rad_
                           ? gimbal_target_rad_ - wrap_angle(current_yaw_rad - *last_odom_yaw_rad_)
                           : *big_yaw_position_rad_;
    last_odom_yaw_rad_ = current_yaw_rad;

    std_msgs::msg::Float64 command;
    command.data = gimbal_target_rad_;
    gimbal_pub_->publish(command);
  }

  void publish_velocity()
  {
    geometry_msgs::msg::Twist output;
    const auto & stamp = latest_command_.header.stamp;
    const bool valid_stamp = stamp.sec >= 0 && stamp.nanosec < 1000000000u;
    const bool wall_fresh =
      command_received_sec_ && (steady_seconds() - *command_received_sec_ <= 0.3);

    if (wall_fresh && valid_stamp && is_finite_twist(latest_command_.twist)) {
      const double source_age_sec = (now() - rclcpp::Time(stamp)).seconds();
      if (source_age_sec >= 0.0 && source_age_sec <= 0.3) {
        output = latest_command_.twist;
      }
    }
    velocity_pub_->publish(output);
  }

  RmSimAdapterConfig config_;
  std::optional<double> big_yaw_position_rad_;
  std::optional<double> last_odom_yaw_rad_;
  std::optional<double> command_received_sec_;
  double gimbal_target_rad_ = 0.0;
  geometry_msgs::msg::TwistStamped latest_command_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace xxu_sim_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::RmSimAdapter)
