#include "tf_relay_gate.hpp"

#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <rcl/timer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace xxu_bringup
{

std::string TfRelayGate::normalize_frame_name(const std::string & text)
{
  const auto begin = text.find_first_not_of(" /\t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::vector<std::pair<std::string, std::string>> TfRelayGate::parse_required_pairs(
  const std::vector<std::string> & pair_specs)
{
  std::vector<std::pair<std::string, std::string>> pairs;
  pairs.reserve(pair_specs.size());
  for (const auto & spec : pair_specs) {
    const auto arrow = spec.find("->");
    if (arrow == std::string::npos) {
      throw std::invalid_argument("malformed required TF pair");
    }
    const std::string parent = normalize_frame_name(spec.substr(0, arrow));
    const std::string child = normalize_frame_name(spec.substr(arrow + 2));
    if (parent.empty() || child.empty()) {
      throw std::invalid_argument("empty required TF pair");
    }
    pairs.emplace_back(parent, child);
  }
  return pairs;
}

TfRelayGate::TfRelayGate(rclcpp::Node & node, const TfRelayConfig & config)
    : node_(node), config_(config)
{
  if (!std::isfinite(config_.tf_publish_rate_hz) || config_.tf_publish_rate_hz < 20.0 ||
      config_.tf_publish_rate_hz > 50.0) {
    throw std::invalid_argument("tf_publish_rate must be within [20, 50] Hz");
  }
  relay_interval_sec_ = 1.0 / config_.tf_publish_rate_hz;

  for (const auto & pair : config_.required_pairs) {
    pair_receipt_times_.emplace(pair, std::nullopt);
  }

  // Each TF publisher is remapped to exactly one private input. This relay
  // keeps standard /tf intact for Nav2, without subscribing to global /tf or
  // creating any TransformListener. Receipt times are recorded at this gate.
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  relay_timer_ = node_.create_wall_timer(
    std::chrono::duration<double>(relay_interval_sec_), [this] { relay_pending(); }, nullptr,
    false);

  std::set<std::string> unique_topics;
  for (const auto & topic : config_.input_topics) {
    if (topic.empty() || topic == "/tf" || topic == "tf" || !unique_topics.insert(topic).second) {
      throw std::invalid_argument(
        "TF inputs must be distinct private publisher streams, never /tf");
    }
    dynamic_subs_.push_back(node_.create_subscription<tf2_msgs::msg::TFMessage>(
      topic, rclcpp::SensorDataQoS().keep_last(100),
      [this, topic](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        on_tf(*message, topic, false);
      }));
  }

  static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
    [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { on_tf(*message, "static", true); });
}

void TfRelayGate::reset_dynamic_state()
{
  buffer_.clear();
  for (const auto & [child, transform] : static_transforms_) {
    (void)child;
    buffer_.setTransform(transform, "static", true);
  }
  relayed_edges_.clear();
  for (auto & [frames, received_time] : pair_receipt_times_) {
    (void)frames;
    received_time.reset();
  }
  if (relay_timer_ && !relay_timer_->is_canceled()) {
    relay_timer_->cancel();
  }
  next_timer_due_sec_.reset();
  availability_dirty_ = true;
  paths_stable_ = false;
  map_to_odom_available_ = false;
  target_to_source_available_ = false;
}

void TfRelayGate::check_clock_reset()
{
  const std::int64_t ros_now_ns = node_.get_clock()->now().nanoseconds();
  if (ros_now_ns <= 0) {
    return;
  }
  const auto threshold_ns = static_cast<std::int64_t>(config_.time_reset_threshold_sec * 1.0e9);
  if (last_ros_time_ns_ && *last_ros_time_ns_ - ros_now_ns >= threshold_ns) {
    RCLCPP_WARN(node_.get_logger(), "Detected ROS clock rewind; resetting dynamic TF relay state");
    reset_dynamic_state();
  }
  last_ros_time_ns_ = ros_now_ns;
}

TfGateStatus TfRelayGate::evaluate(double steady_now)
{
  check_clock_reset();
  if (!has_parent_conflict_ && availability_dirty_) {
    refresh_availability();
  }

  TfGateStatus status;
  status.map_to_odom_ready = !has_parent_conflict_ && map_to_odom_available_;
  status.target_to_source_ready = !has_parent_conflict_ && target_to_source_available_;
  if (config_.require_tf) {
    status.streams_fresh =
      std::all_of(pair_receipt_times_.begin(), pair_receipt_times_.end(),
                  [this, steady_now](const auto & item) {
                    return fresh(item.second, config_.tf_timeout_sec, steady_now);
                  });
  }
  return status;
}

std::vector<std::pair<std::string, std::string>> TfRelayGate::timed_out_pairs(
  double steady_now) const
{
  std::vector<std::pair<std::string, std::string>> stale_pairs;
  if (!config_.require_tf) {
    return stale_pairs;
  }
  for (const auto & [frames, received_time] : pair_receipt_times_) {
    if (!fresh(received_time, config_.tf_timeout_sec, steady_now)) {
      stale_pairs.push_back(frames);
    }
  }
  return stale_pairs;
}

void TfRelayGate::advance_deadline(RelayEdge & edge, double steady_now) const
{
  // Preserve the phase through small delivery delays and restart it after gaps.
  if (edge.next_due_sec > 0.0 && steady_now - edge.next_due_sec < relay_interval_sec_) {
    edge.next_due_sec += relay_interval_sec_;
  } else {
    edge.next_due_sec = steady_now + relay_interval_sec_;
  }
}

void TfRelayGate::schedule_timer(double due_sec, double steady_now)
{
  // Arm a single-shot wakeup for the earliest pending edge deadline instead of 1 kHz polling.
  const double remaining_sec = std::max(0.0, due_sec - steady_now);
  const auto period_ns =
    std::max<std::int64_t>(1, static_cast<std::int64_t>(std::ceil(remaining_sec * 1.0e9)));
  std::int64_t previous_ns = 0;
  if (rcl_timer_exchange_period(relay_timer_->get_timer_handle().get(), period_ns, &previous_ns) !=
      RCL_RET_OK) {
    throw std::runtime_error("Failed to update TF relay timer period");
  }
  relay_timer_->reset();
  next_timer_due_sec_ = due_sec;
}

void TfRelayGate::relay_pending()
{
  const double steady_now = steady_seconds();
  std::optional<double> earliest_due_sec;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_to_send;

  for (auto & [child, edge] : relayed_edges_) {
    (void)child;
    if (!edge.pending_transform) {
      continue;
    }
    if (steady_now >= edge.next_due_sec) {
      transforms_to_send.push_back(std::move(*edge.pending_transform));
      edge.pending_transform.reset();
      advance_deadline(edge, steady_now);
    } else if (!earliest_due_sec || edge.next_due_sec < *earliest_due_sec) {
      earliest_due_sec = edge.next_due_sec;
    }
  }

  if (!transforms_to_send.empty()) {
    broadcaster_->sendTransform(transforms_to_send);
  }
  if (earliest_due_sec) {
    schedule_timer(*earliest_due_sec, steady_now);
  } else {
    relay_timer_->cancel();
    next_timer_due_sec_.reset();
  }
}

void TfRelayGate::on_tf(const tf2_msgs::msg::TFMessage & message, const std::string & authority,
                        bool is_static)
{
  check_clock_reset();
  for (const auto & transform : message.transforms) {
    const std::string parent = normalize_frame_name(transform.header.frame_id);
    const std::string child = normalize_frame_name(transform.child_frame_id);
    if (!is_static && !is_navigation_dynamic_edge(parent, child)) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                           "Ignoring non-navigation dynamic TF %s -> %s", parent.c_str(),
                           child.c_str());
      continue;
    }

    const std::int64_t stamp = is_static ? 0 : stamp_ns(transform.header.stamp);
    bool stamp_advanced = true;
    if (!is_static) {
      const auto edge_it = relayed_edges_.find(child);
      if (stamp < 0 || (edge_it != relayed_edges_.end() && stamp < edge_it->second.last_stamp_ns)) {
        continue;
      }
      stamp_advanced = (edge_it == relayed_edges_.end()) || (stamp > edge_it->second.last_stamp_ns);
    }

    const auto known_parent = known_parents_.find(child);
    if (known_parent != known_parents_.end() && known_parent->second != parent) {
      has_parent_conflict_ = true;
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                            "Rejecting multiple TF parents for %s", child.c_str());
      continue;
    }
    if (known_parent == known_parents_.end() && creates_cycle(parent, child)) {
      has_parent_conflict_ = true;
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                            "Rejecting cyclic TF edge %s -> %s", parent.c_str(), child.c_str());
      continue;
    }

    try {
      // Even a rejected old sample can prune an overlapping history in BufferCore.
      availability_dirty_ |= is_static || !paths_stable_;
      if (!buffer_.setTransform(transform, authority, is_static)) {
        continue;
      }
      if (parent != transform.header.frame_id || child != transform.child_frame_id) {
        canonical_frames_ = false;
        availability_dirty_ = true;
      }
    } catch (const tf2::TransformException &) {
      continue;
    }

    if (known_parent == known_parents_.end()) {
      known_parents_.emplace(child, parent);
    }
    if (is_static) {
      static_transforms_[child] = transform;
      continue;
    }

    const double steady_now = steady_seconds();
    if (stamp_advanced) {
      const auto pair_it = pair_receipt_times_.find({parent, child});
      if (pair_it != pair_receipt_times_.end()) {
        pair_it->second = steady_now;
      }
    }

    RelayEdge & edge = relayed_edges_[child];
    edge.last_stamp_ns = stamp;
    if (!edge.pending_transform && steady_now >= edge.next_due_sec) {
      broadcaster_->sendTransform(transform);
      advance_deadline(edge, steady_now);
    } else {
      edge.pending_transform = transform;
      if (!next_timer_due_sec_ || edge.next_due_sec < *next_timer_due_sec_) {
        schedule_timer(edge.next_due_sec, steady_now);
      }
    }
  }
}

bool TfRelayGate::is_navigation_dynamic_edge(const std::string & parent,
                                             const std::string & child) const
{
  const bool is_global_to_odom = (parent == config_.global_frame && child == config_.odom_frame);
  const bool is_odom_to_base = (parent == config_.odom_frame && child == config_.base_frame);
  return is_global_to_odom || is_odom_to_base;
}

bool TfRelayGate::creates_cycle(const std::string & parent, const std::string & child) const
{
  std::string cursor = parent;
  for (std::size_t hops = 0; hops <= known_parents_.size(); ++hops) {
    if (cursor == child) {
      return true;
    }
    const auto it = known_parents_.find(cursor);
    if (it == known_parents_.end()) {
      return false;
    }
    cursor = it->second;
  }
  return true;
}

bool TfRelayGate::has_parent(const std::string & child, const std::string & parent) const
{
  const auto known_parent = known_parents_.find(child);
  return known_parent != known_parents_.end() && known_parent->second == parent;
}

void TfRelayGate::refresh_availability()
{
  map_to_odom_available_ =
    buffer_.canTransform(config_.global_frame, config_.odom_frame, tf2::TimePointZero);
  target_to_source_available_ =
    config_.require_tf &&
    buffer_.canTransform(config_.target_frame, config_.source_frame, tf2::TimePointZero);

  // In these paths each query uses at most one dynamic edge. New samples for
  // that same parent cannot exhaust the latest common time. For custom paths
  // with two dynamic edges, changes to either history can remove overlap.
  const bool target_is_base_or_child = config_.source_frame == config_.base_frame ||
                                       (config_.source_frame != config_.odom_frame &&
                                        has_parent(config_.source_frame, config_.base_frame));
  const bool custom_path_stable =
    !config_.require_tf ||
    (target_to_source_available_ && config_.target_frame == config_.odom_frame &&
     has_parent(config_.base_frame, config_.odom_frame) && target_is_base_or_child);

  paths_stable_ = canonical_frames_ && map_to_odom_available_ &&
                  has_parent(config_.odom_frame, config_.global_frame) && custom_path_stable;
  availability_dirty_ = false;
}

}  // namespace xxu_bringup
