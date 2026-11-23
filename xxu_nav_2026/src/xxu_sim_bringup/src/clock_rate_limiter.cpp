#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace xxu_sim_bringup
{
namespace
{

struct ClockLimiterConfig {
  std::string input_topic;
  std::string output_topic;
  std::int64_t period_ns = 0;
};

}  // namespace

// Decimate source simulation time, independently of ROS time and physics stepping.
class ClockRateLimiter final : public rclcpp::Node
{
public:
  explicit ClockRateLimiter(const rclcpp::NodeOptions & options) : Node("rm_clock_limiter", options)
  {
    load_config();
    init_interfaces();
  }

private:
  void load_config()
  {
    const double publish_rate_hz = declare_parameter("publish_rate", 200.0);
    if (!std::isfinite(publish_rate_hz) || publish_rate_hz < 1.0 || publish_rate_hz > 1.0e9) {
      throw std::invalid_argument("clock publish_rate must be finite and within [1, 1e9] Hz");
    }
    config_.period_ns = static_cast<std::int64_t>(std::llround(1.0e9 / publish_rate_hz));

    config_.input_topic = declare_parameter("input_topic", "/sentry/simulation/clock_raw");
    config_.output_topic = declare_parameter("output_topic", "/clock");
    if (config_.input_topic == config_.output_topic) {
      throw std::invalid_argument("clock input and output must differ");
    }
  }

  void init_interfaces()
  {
    publisher_ =
      create_publisher<rosgraph_msgs::msg::Clock>(config_.output_topic, rclcpp::ClockQoS());
    subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      config_.input_topic, rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) { on_clock(*message); });
  }

  void on_clock(const rosgraph_msgs::msg::Clock & message)
  {
    if (message.clock.sec < 0 || message.clock.nanosec >= 1000000000u) {
      return;
    }

    const std::int64_t stamp_ns =
      static_cast<std::int64_t>(message.clock.sec) * 1000000000LL + message.clock.nanosec;
    if (previous_stamp_ns_ && stamp_ns <= *previous_stamp_ns_) {
      return;
    }
    previous_stamp_ns_ = stamp_ns;

    if (!next_due_ns_) {
      next_due_ns_ = stamp_ns + config_.period_ns;
    } else if (stamp_ns >= *next_due_ns_) {
      // Skip missed slots rather than replaying ticks or accumulating drift.
      *next_due_ns_ += ((stamp_ns - *next_due_ns_) / config_.period_ns + 1) * config_.period_ns;
    } else {
      return;
    }

    publisher_->publish(message);
  }

  ClockLimiterConfig config_;
  std::optional<std::int64_t> previous_stamp_ns_;
  std::optional<std::int64_t> next_due_ns_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

}  // namespace xxu_sim_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_sim_bringup::ClockRateLimiter)
