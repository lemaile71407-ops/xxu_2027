#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wait_for_odom");
  bool received = false;
  const auto topic = node->declare_parameter("topic", "/odom");
  auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
    topic, rclcpp::SensorDataQoS(), [&](nav_msgs::msg::Odometry::ConstSharedPtr) {received = true;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !received) {executor.spin_once(std::chrono::milliseconds(100));}
  const bool success = received;
  rclcpp::shutdown();
  return success ? 0 : 1;
}
