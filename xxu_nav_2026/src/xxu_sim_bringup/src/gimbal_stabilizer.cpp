#include "sim_math.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace xxu_sim_bringup
{
namespace
{

struct StabilizerConfig {
  std::string joint_name;
  std::string mode;
  std::string command_topic;
  std::string joint_states_topic;
  std::string imu_topic;
  std::string odom_topic;
  double spin_rate_rad_s = 6.283185307179586;
  double spin_start_delay_sec = 1.0;
  bool wait_for_odom = true;
  double hold_gain = 4.0;
  double max_hold_velocity_rad_s = 2.0;
  double command_rate_hz = 50.0;
};

}  // namespace

class GimbalStabilizer final : public rclcpp::Node
{
public:
  explicit GimbalStabilizer(const rclcpp::NodeOptions & options)
      : Node("gimbal_stabilizer", options)
  {
    load_config();
    init_interfaces();
  }

private:
  void load_config()
  {
    config_.joint_name = declare_parameter("joint_name", "gimbal_joint");
    config_.mode = declare_parameter("mode", "spin");
    config_.spin_rate_rad_s = declare_parameter("spin_rate", 6.283185307179586);
    config_.spin_start_delay_sec = declare_parameter("spin_start_delay", 1.0);
    config_.wait_for_odom = declare_parameter("wait_for_odom", true);
    config_.hold_gain = declare_parameter("hold_gain", 4.0);
    config_.max_hold_velocity_rad_s = std::abs(declare_parameter("max_hold_velocity", 2.0));
    config_.command_rate_hz = declare_parameter("command_rate", 50.0);

    if ((config_.mode != "spin" && config_.mode != "hold") ||
        !std::isfinite(config_.command_rate_hz) || config_.command_rate_hz <= 0.0) {
      throw std::invalid_argument("invalid gimbal mode/rate");
    }

    config_.command_topic =
      declare_parameter("command_topic", "/gimbal_velocity_controller/commands");
    config_.joint_states_topic = declare_parameter("joint_states_topic", "/joint_states");
    config_.imu_topic = declare_parameter("imu_topic", "/imu_raw");
    config_.odom_topic = declare_parameter("odom_topic", "/odom");
  }

  void init_interfaces()
  {
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(config_.command_topic, 10);

    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      config_.joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) { on_joint_state(*message); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      config_.imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { on_imu(*message); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
        if (!odom_ready_sec_) {
          odom_ready_sec_ = now().seconds();
        }
      });

    spin_timer_ = rclcpp::create_timer(this, get_clock(),
                                       std::chrono::duration<double>(1.0 / config_.command_rate_hz),
                                       [this] { on_spin_tick(); });
  }

  void on_joint_state(const sensor_msgs::msg::JointState & message)
  {
    const auto it = std::find(message.name.begin(), message.name.end(), config_.joint_name);
    const auto index = static_cast<std::size_t>(it - message.name.begin());
    if (it != message.name.end() && index < message.position.size() &&
        std::isfinite(message.position[index])) {
      joint_position_rad_ = message.position[index];
    }
  }

  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    if (config_.mode != "hold" || !joint_position_rad_) {
      return;
    }

    auto q = message.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-6) {
      return;
    }
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;

    const double current_yaw_rad = quaternion_yaw(q);
    if (!hold_yaw_rad_) {
      hold_yaw_rad_ = current_yaw_rad;
    }

    const double velocity =
      std::clamp(config_.hold_gain * wrap_angle(*hold_yaw_rad_ - current_yaw_rad),
                 -config_.max_hold_velocity_rad_s, config_.max_hold_velocity_rad_s);
    publish_velocity(velocity);
  }

  void on_spin_tick()
  {
    if (config_.mode != "spin" || !joint_position_rad_) {
      return;
    }

    const double current_time_sec = now().seconds();
    if (!spin_start_sec_) {
      spin_start_sec_ = current_time_sec;
    }
    if (config_.wait_for_odom && !odom_ready_sec_) {
      publish_velocity(0.0);
      return;
    }

    const double reference_sec = config_.wait_for_odom ? *odom_ready_sec_ : *spin_start_sec_;
    const double command_velocity =
      (current_time_sec - reference_sec < config_.spin_start_delay_sec) ? 0.0
                                                                        : config_.spin_rate_rad_s;
    publish_velocity(command_velocity);
  }

  void publish_velocity(double velocity_rad_s)
  {
    std_msgs::msg::Float64MultiArray message;
    message.data = {velocity_rad_s};
    command_pub_->publish(message);
  }

  StabilizerConfig config_;
  std::optional<double> joint_position_rad_;
  std::optional<double> hold_yaw_rad_;
  std::optional<double> odom_ready_sec_;
  std::optional<double> spin_start_sec_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
};

}  // namespace xxu_sim_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::GimbalStabilizer)
