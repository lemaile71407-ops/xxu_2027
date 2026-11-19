#include <memory>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager_isolated.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  auto node = std::make_shared<
    rclcpp_components::ComponentManagerIsolated<
      rclcpp::experimental::executors::EventsExecutor>>(exec);
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  rclcpp::shutdown();
  return 0;
}
