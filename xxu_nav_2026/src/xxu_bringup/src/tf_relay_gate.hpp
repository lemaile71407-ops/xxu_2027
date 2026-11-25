#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace xxu_bringup
{

struct TfRelayConfig {
  std::string global_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string target_frame = "odom";
  std::string source_frame = "base_link";
  bool require_tf = true;
  double tf_timeout_sec = 0.5;
  double time_reset_threshold_sec = 0.5;
  double tf_publish_rate_hz = 50.0;
  std::vector<std::string> input_topics;
  std::vector<std::pair<std::string, std::string>> required_pairs;
};

struct TfGateStatus {
  bool map_to_odom_ready = false;
  bool target_to_source_ready = false;
  bool streams_fresh = true;
};

/// Relays private dynamic TF streams to /tf at a bounded rate while enforcing
/// single-parent consistency and tracking required TF path readiness.
class TfRelayGate
{
public:
  TfRelayGate(rclcpp::Node & node, const TfRelayConfig & config);

  /// Normalize frame names by stripping leading slashes and surrounding whitespace.
  static std::string normalize_frame_name(const std::string & text);

  /// Parse "<parent>-><child>" strings into validated frame pairs.
  static std::vector<std::pair<std::string, std::string>> parse_required_pairs(
    const std::vector<std::string> & pair_specs);

  /// Evaluate TF path availability and stream freshness at steady_now (seconds).
  TfGateStatus evaluate(double steady_now);

  /// Collect required TF pairs whose latest dynamic update is stale at steady_now.
  std::vector<std::pair<std::string, std::string>> timed_out_pairs(double steady_now) const;

private:
  struct RelayEdge {
    double next_due_sec = 0.0;
    std::int64_t last_stamp_ns = -1;
    std::optional<geometry_msgs::msg::TransformStamped> pending_transform;
  };

  void check_clock_reset();
  void reset_dynamic_state();
  void advance_deadline(RelayEdge & edge, double steady_now) const;
  void schedule_timer(double due_sec, double steady_now);
  void relay_pending();
  void on_tf(const tf2_msgs::msg::TFMessage & message, const std::string & authority,
             bool is_static);
  bool is_navigation_dynamic_edge(const std::string & parent, const std::string & child) const;
  bool creates_cycle(const std::string & parent, const std::string & child) const;
  bool has_parent(const std::string & child, const std::string & parent) const;
  void refresh_availability();

  rclcpp::Node & node_;
  TfRelayConfig config_;
  double relay_interval_sec_ = 0.02;

  bool has_parent_conflict_ = false;
  bool canonical_frames_ = true;
  bool availability_dirty_ = true;
  bool paths_stable_ = false;
  bool map_to_odom_available_ = false;
  bool target_to_source_available_ = false;

  std::map<std::pair<std::string, std::string>, std::optional<double>> pair_receipt_times_;
  std::map<std::string, std::string> known_parents_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_transforms_;
  std::map<std::string, RelayEdge> relayed_edges_;
  std::optional<double> next_timer_due_sec_;
  std::optional<std::int64_t> last_ros_time_ns_;

  tf2::BufferCore buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr relay_timer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> dynamic_subs_;
};

}  // namespace xxu_bringup
