#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/sensor_math.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl/timer.h>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <sstream>

#ifdef XXU_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) (void)0
#define TracySetProgramName(name) (void)0
#endif
#include <stdexcept>

namespace xxu_bringup {
namespace {
std::string frame_name(std::string text)
{
  const auto begin = text.find_first_not_of(" /\t\r\n");
  if (begin == std::string::npos) {return "";}
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}
bool finite_twist(const geometry_msgs::msg::Twist & m)
{
  return std::isfinite(m.linear.x) && std::isfinite(m.linear.y) && std::isfinite(m.linear.z) &&
    std::isfinite(m.angular.x) && std::isfinite(m.angular.y) && std::isfinite(m.angular.z);
}
}  // namespace

class CmdVelWatchdog final : public rclcpp::Node
{
public:
  explicit CmdVelWatchdog(const rclcpp::NodeOptions & options)
  : Node("cmd_vel_watchdog", options)
  {
    TracySetProgramName("cmd_vel_watchdog");
    const auto input = declare_parameter("input_topic", "/cmd_vel_collision");
    const auto output = declare_parameter("output_topic", "/cmd_vel");
    if (input == output) {throw std::invalid_argument("watchdog input/output loop");}
    frame_ = declare_parameter("output_frame_id", "base_link");
    timeout_ = checked_seconds(*this, "timeout", .3, true);
    const double rate = checked_seconds(*this, "publish_rate", 20.);
    require_odom_ = declare_parameter("require_odom", true);
    require_scan_ = declare_parameter("require_scan", true);
    require_joints_ = declare_parameter("require_joint_states", true);
    require_tf_ = declare_parameter("require_tf", true);
    require_localization_ = declare_parameter("require_localization", false);
    odom_timeout_ = checked_seconds(*this, "odom_timeout", .5, true);
    scan_timeout_ = checked_seconds(*this, "scan_timeout", .5, true);
    joints_timeout_ = checked_seconds(*this, "joint_states_timeout", .5, true);
    tf_timeout_ = checked_seconds(*this, "tf_timeout", .5, true);
    tf_target_ = frame_name(declare_parameter("tf_target_frame", "odom"));
    tf_source_ = frame_name(declare_parameter("tf_source_frame", "base_link"));
    global_ = declare_parameter("global_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    for (const auto & text : declare_parameter<std::vector<std::string>>("required_tf_pairs",
      std::vector<std::string>{"odom->base_link"}))
    {
      const auto arrow = text.find("->");
      if (arrow == std::string::npos) {throw std::invalid_argument("malformed required TF pair");}
      const auto pair = std::make_pair(frame_name(text.substr(0, arrow)), frame_name(text.substr(arrow + 2)));
      if (pair.first.empty() || pair.second.empty()) {throw std::invalid_argument("empty required TF pair");}
      tf_received_.emplace(pair, std::nullopt);
    }
    const double localization_timeout = checked_seconds(*this, "localization_timeout", .5);
    localization_ = std::make_unique<LocalizationHealth>(localization_timeout,
      checked_seconds(*this, "localization_recovery_hold", 1., true));
    readiness_interval_ = std::min(.15, localization_timeout / 3.);
    const auto state_topic = declare_parameter("localization_state_topic", "/backend_node/localization_state");
    const auto pose_topic = declare_parameter("localization_pose_topic", "/localization/odometry");
    if (require_localization_) {
      state_sub_ = create_subscription<std_msgs::msg::String>(state_topic,
        rclcpp::QoS(1).transient_local().reliable(), [this](std_msgs::msg::String::ConstSharedPtr m) {
          localization_->observe_state(m->data, steady_seconds());
        });
      pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(pose_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {localization_->observe_pose(*m, steady_seconds());});
    }
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(output, rclcpp::QoS(10));
    status_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter("status_topic", "/cmd_vel_watchdog/healthy"), rclcpp::QoS(10));
    readiness_ = create_publisher<std_msgs::msg::Bool>("~/localization_ready", rclcpp::QoS(1).transient_local());
    tf_readiness_ = create_publisher<std_msgs::msg::Bool>("~/tf_ready", rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(input, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
        if (!finite_twist(m->twist)) {command_received_.reset(); return;}
        command_ = *m; command_received_ = steady_seconds();
      });
    const auto odom_topic = declare_parameter("odom_topic", "/odom");
    const auto scan_topic = declare_parameter("scan_topic", "/scan");
    const auto joint_topic = declare_parameter("joint_states_topic", "/joint_states");
    // Poll keepalive streams in tick() via GenericSubscription in an unattached
    // callback group so high-rate messages neither deserialize nor wake the executor.
    keepalive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions keepalive_options;
    keepalive_options.callback_group = keepalive_group_;
    const auto keepalive_qos = rclcpp::SensorDataQoS().keep_last(1);
    auto make_keepalive = [this, &keepalive_qos, &keepalive_options](
      const std::string & topic, const std::string & type)
      {
        return create_generic_subscription(topic, type, keepalive_qos,
          [](const rclcpp::SerializedMessage &) {}, keepalive_options);
      };
    if (require_odom_) {odom_sub_ = make_keepalive(odom_topic, "nav_msgs/msg/Odometry");}
    if (require_scan_) {scan_sub_ = make_keepalive(scan_topic, "sensor_msgs/msg/LaserScan");}
    if (require_joints_) {joint_sub_ = make_keepalive(joint_topic, "sensor_msgs/msg/JointState");}
    // Each TF publisher is remapped to exactly one private input. This relay
    // keeps standard /tf intact for Nav2, without subscribing to global /tf or
    // creating any TransformListener. Receipt times are recorded at this gate.
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const double tf_rate = checked_seconds(*this, "tf_publish_rate", 50.);
    if (tf_rate < 20. || tf_rate > 50.) {
      throw std::invalid_argument("tf_publish_rate must be within [20, 50] Hz");
    }
    tf_interval_ = 1. / tf_rate;
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(tf_interval_), [this] {relay_pending_tf();}, nullptr, false);
    const auto topics = declare_parameter<std::vector<std::string>>("tf_input_topics",
      std::vector<std::string>{"/sentry/tf/odometry", "/sentry/tf/localization"});
    std::set<std::string> unique;
    for (const auto & topic : topics) {
      if (topic.empty() || topic == "/tf" || topic == "tf" || !unique.insert(topic).second) {
        throw std::invalid_argument("TF inputs must be distinct private publisher streams, never /tf");
      }
      tf_subs_.push_back(create_subscription<tf2_msgs::msg::TFMessage>(topic,
        rclcpp::SensorDataQoS().keep_last(100), [this, topic](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          on_tf(*m, topic, false);
        }));
    }
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(100).transient_local().reliable(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        on_tf(*m, "static", true);
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] {tick();});
  }

private:
  struct RelayEdge
  {
    double next_due = 0.;
    std::int64_t last_stamp_ns = -1;
    std::optional<geometry_msgs::msg::TransformStamped> pending;
  };

  struct ReadinessState
  {
    std::optional<bool> value;
    double last_sent = 0.;
  };

  void advance_tf_deadline(RelayEdge & edge, double steady)
  {
    // Preserve the phase through small delivery delays and restart it after gaps.
    if (edge.next_due > 0. && steady - edge.next_due < tf_interval_) {
      edge.next_due += tf_interval_;
    } else {
      edge.next_due = steady + tf_interval_;
    }
  }

  void schedule_tf_timer(double due, double steady)
  {
    // Arm a single-shot wakeup for the earliest pending edge deadline instead of 1 kHz polling.
    const double remaining = std::max(0., due - steady);
    const auto period_ns = std::max<std::int64_t>(
      1, static_cast<std::int64_t>(std::ceil(remaining * 1.e9)));
    std::int64_t previous_ns = 0;
    if (rcl_timer_exchange_period(tf_timer_->get_timer_handle().get(), period_ns, &previous_ns) !=
      RCL_RET_OK)
    {
      throw std::runtime_error("Failed to update TF relay timer period");
    }
    tf_timer_->reset();
    tf_timer_due_ = due;
  }

  void relay_pending_tf()
  {
    const double steady = steady_seconds();
    std::optional<double> earliest_due;
    std::vector<geometry_msgs::msg::TransformStamped> latest;
    for (auto & item : relayed_tf_) {
      auto & edge = item.second;
      if (!edge.pending) {continue;}
      if (steady >= edge.next_due) {
        latest.push_back(std::move(*edge.pending));
        edge.pending.reset();
        advance_tf_deadline(edge, steady);
      } else if (!earliest_due || edge.next_due < *earliest_due) {
        earliest_due = edge.next_due;
      }
    }
    if (!latest.empty()) {broadcaster_->sendTransform(latest);}
    if (earliest_due) {
      schedule_tf_timer(*earliest_due, steady);
    } else {
      tf_timer_->cancel();
      tf_timer_due_.reset();
    }
  }

  void on_tf(const tf2_msgs::msg::TFMessage & message, const std::string & authority, bool is_static)
  {
    for (const auto & t : message.transforms) {
      const auto parent = frame_name(t.header.frame_id), child = frame_name(t.child_frame_id);
      if (!is_static && !((parent == global_ && child == odom_frame_) ||
        (parent == odom_frame_ && child == frame_)))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Ignoring non-navigation dynamic TF %s -> %s", parent.c_str(), child.c_str());
        continue;
      }
      const auto stamp = is_static ? 0 : stamp_ns(t.header.stamp);
      if (!is_static) {
        const auto edge = relayed_tf_.find(child);
        if (stamp < 0 || (edge != relayed_tf_.end() && stamp < edge->second.last_stamp_ns)) {
          continue;
        }
      }
      const auto known = parents_.find(child);
      if (known != parents_.end() && known->second != parent) {
        tf_conflict_ = true;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting multiple TF parents for %s", child.c_str());
        continue;
      }
      try {
        // Even a rejected old sample can prune an overlapping history in BufferCore.
        tf_availability_dirty_ |= is_static || !tf_paths_stable_;
        if (!buffer_.setTransform(t, authority, is_static)) {continue;}
        if (parent != t.header.frame_id || child != t.child_frame_id) {
          canonical_tf_frames_ = false;
          tf_availability_dirty_ = true;
        }
      } catch (const tf2::TransformException &) {continue;}
      if (known == parents_.end()) {parents_.emplace(child, parent);}
      if (!is_static) {
        const double steady = steady_seconds();
        const auto pair = tf_received_.find({parent, child});
        if (pair != tf_received_.end()) {pair->second = steady;}
        auto & edge = relayed_tf_[child];
        edge.last_stamp_ns = stamp;
        if (!edge.pending && steady >= edge.next_due) {
          broadcaster_->sendTransform(t);
          advance_tf_deadline(edge, steady);
        } else {
          edge.pending = t;
          if (!tf_timer_due_ || edge.next_due < *tf_timer_due_) {
            schedule_tf_timer(edge.next_due, steady);
          }
        }
      }
    }
  }

  bool child_of(const std::string & child, const std::string & parent) const
  {
    const auto known = parents_.find(child);
    return known != parents_.end() && known->second == parent;
  }

  void refresh_tf_availability()
  {
    map_to_odom_available_ = buffer_.canTransform(global_, odom_frame_, tf2::TimePointZero);
    target_to_source_available_ = require_tf_ &&
      buffer_.canTransform(tf_target_, tf_source_, tf2::TimePointZero);
    // In these paths each query uses at most one dynamic edge. New samples for
    // that same parent cannot exhaust the latest common time. For custom paths
    // with two dynamic edges, changes to either history can remove overlap.
    tf_paths_stable_ = canonical_tf_frames_ && map_to_odom_available_ && child_of(odom_frame_, global_) &&
      (!require_tf_ || (target_to_source_available_ && tf_target_ == odom_frame_ &&
      child_of(frame_, odom_frame_) && (tf_source_ == frame_ ||
      (tf_source_ != odom_frame_ && child_of(tf_source_, frame_)))));
    tf_availability_dirty_ = false;
  }

  void publish_readiness(
    rclcpp::Publisher<std_msgs::msg::Bool> & publisher, ReadinessState & state,
    bool ready, double steady)
  {
    if (state.value && *state.value == ready && steady - state.last_sent < readiness_interval_) {return;}
    std_msgs::msg::Bool message;
    message.data = ready;
    publisher.publish(message);
    state.value = ready;
    state.last_sent = steady;
  }

  void tick()
  {
    ZoneScopedN("CmdVelWatchdog::tick");
    const double steady = steady_seconds();
    const auto ros_time = now();
    if (!tf_conflict_ && tf_availability_dirty_) {refresh_tf_availability();}
    const bool tf_ready = !tf_conflict_ && map_to_odom_available_;
    const bool target_ready = !tf_conflict_ && target_to_source_available_;
    LocalizationHealth::Evaluation localization;
    if (require_localization_) {
      localization = localization_->evaluate(steady, ros_time.nanoseconds());
      if (!localization.ready() || !localization_ready_) {command_received_.reset();}
      localization_ready_ = localization.ready();
    }
    publish_readiness(*readiness_, localization_readiness_state_,
      require_localization_ && localization.ready() && tf_ready, steady);
    publish_readiness(*tf_readiness_, tf_readiness_state_, tf_ready, steady);

    const bool command_fresh = fresh(command_received_, timeout_, steady);
    const auto stamp = stamp_ns(command_.header.stamp);
    const auto age = ros_time.nanoseconds() - stamp;
    const bool command_stamp_valid = stamp >= 0 && age >= 0 && static_cast<double>(age) <= timeout_ * 1.e9;
    auto poll_keepalive = [this, steady](
      const rclcpp::GenericSubscription::SharedPtr & subscription,
      std::optional<double> & received)
      {
        if (subscription && subscription->take_serialized(keepalive_buffer_, keepalive_info_)) {
          received = steady;
        }
      };
    poll_keepalive(odom_sub_, odom_received_);
    poll_keepalive(scan_sub_, scan_received_);
    poll_keepalive(joint_sub_, joints_received_);
    const bool odom_fresh = !require_odom_ || fresh(odom_received_, odom_timeout_, steady);
    const bool scan_fresh = !require_scan_ || fresh(scan_received_, scan_timeout_, steady);
    const bool joints_fresh = !require_joints_ || fresh(joints_received_, joints_timeout_, steady);
    const bool tf_path_ready = !require_tf_ || target_ready;
    const bool tf_streams_fresh = !require_tf_ || std::all_of(tf_received_.begin(), tf_received_.end(),
      [this, steady](const auto & pair) {return fresh(pair.second, tf_timeout_, steady);});
    const bool healthy = localization.ready() && command_fresh && command_stamp_valid &&
      odom_fresh && scan_fresh && joints_fresh && tf_path_ready && tf_streams_fresh;
    auto output = healthy ? command_ : geometry_msgs::msg::TwistStamped();
    if (output.header.frame_id.empty()) {output.header.frame_id = frame_;}
    output.header.stamp = ros_time;
    publisher_->publish(output);
    std_msgs::msg::Bool state; state.data = healthy; status_->publish(state);
    if (!previous_health_ || *previous_health_ != healthy) {
      if (healthy) {RCLCPP_INFO(get_logger(), "Velocity safety gate opened");}
      else {
        std::ostringstream text;
        for (const auto & reason : localization.reasons()) {text << reason << "; ";}
        if (!command_fresh) {text << "command timeout; ";}
        if (!command_stamp_valid) {text << "command source timestamp expired or invalid; ";}
        if (!odom_fresh) {text << "odometry timeout; ";}
        if (!scan_fresh) {text << "scan timeout; ";}
        if (!joints_fresh) {text << "joint_states timeout; ";}
        if (!tf_path_ready) {text << "missing/conflicting TF " << tf_target_ << "->" << tf_source_ << "; ";}
        if (!tf_streams_fresh) {
          for (const auto & [frames, received] : tf_received_) {
            if (!fresh(received, tf_timeout_, steady)) {
              text << "TF stream timeout " << frames.first << "->" << frames.second << "; ";
            }
          }
        }
        RCLCPP_WARN(get_logger(), "Velocity safety gate closed; publishing zero: %s", text.str().c_str());
      }
      previous_health_ = healthy;
    }
  }
  std::string frame_, tf_target_, tf_source_, global_, odom_frame_;
  double timeout_, odom_timeout_, scan_timeout_, joints_timeout_, tf_timeout_;
  double readiness_interval_, tf_interval_;
  bool require_odom_, require_scan_, require_joints_, require_tf_, require_localization_;
  bool localization_ready_ = false, tf_conflict_ = false;
  bool canonical_tf_frames_ = true, tf_availability_dirty_ = true, tf_paths_stable_ = false;
  bool map_to_odom_available_ = false, target_to_source_available_ = false;
  ReadinessState localization_readiness_state_, tf_readiness_state_;
  std::optional<bool> previous_health_;
  std::optional<double> command_received_, odom_received_, scan_received_, joints_received_;
  std::map<std::pair<std::string, std::string>, std::optional<double>> tf_received_;
  std::map<std::string, std::string> parents_;
  geometry_msgs::msg::TwistStamped command_;
  std::unique_ptr<LocalizationHealth> localization_;
  tf2::BufferCore buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::map<std::string, RelayEdge> relayed_tf_;
  std::optional<double> tf_timer_due_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_, readiness_, tf_readiness_;
  rclcpp::CallbackGroup::SharedPtr keepalive_group_;
  rclcpp::SerializedMessage keepalive_buffer_;
  rclcpp::MessageInfo keepalive_info_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::GenericSubscription::SharedPtr odom_sub_, scan_sub_, joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace xxu_bringup
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::CmdVelWatchdog)
