#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace xxu_sim_bringup
{

class OmniDrive final : public rclcpp::Node
{
public:
  explicit OmniDrive(const rclcpp::NodeOptions & options) : Node("omni_drive", options)
  {
    const std::array<std::string, 4> wheel_names{"wheel_fl", "wheel_rl", "wheel_rr", "wheel_fr"};
    for (std::size_t i = 0; i < wheel_names.size(); ++i) {
      wheel_pubs_[i] =
        create_publisher<std_msgs::msg::Float64>("/" + wheel_names[i] + "/cmd_vel", 10);
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) { on_cmd_vel(*message); });
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & message)
  {
    constexpr double kWheelBaseRadiusMeters = 0.245;
    constexpr double kWheelRadiusMeters = 0.076;
    const double deg_to_rad = std::acos(-1.0) / 180.0;
    const bool valid = std::isfinite(message.linear.x) && std::isfinite(message.linear.y) &&
                       std::isfinite(message.angular.z);
    const std::array<double, 4> wheel_angles_deg{45.0, 135.0, -135.0, -45.0};

    for (std::size_t i = 0; i < wheel_angles_deg.size(); ++i) {
      const double angle_rad = wheel_angles_deg[i] * deg_to_rad;
      std_msgs::msg::Float64 wheel_cmd;
      if (valid) {
        const double tangential_velocity = -message.linear.x * std::sin(angle_rad) +
                                           message.linear.y * std::cos(angle_rad) +
                                           message.angular.z * kWheelBaseRadiusMeters;
        wheel_cmd.data = -tangential_velocity / kWheelRadiusMeters;
      } else {
        wheel_cmd.data = 0.0;
      }
      wheel_pubs_[i]->publish(wheel_cmd);
    }
  }

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> wheel_pubs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace xxu_sim_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::OmniDrive)
