#include "tf_relay_gate.hpp"
#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/node_parameters.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#ifdef XXU_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) (void)0
#define TracySetProgramName(name) (void)0
#endif

namespace xxu_bringup
{
namespace
{

bool is_finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

struct WatchdogConfig {
  std::string input_topic;
  std::string output_topic;
  std::string output_frame_id;
  std::string status_topic;
  double command_timeout_sec = 0.3;
  double publish_rate_hz = 20.0;

  bool require_odom = true;
  bool require_scan = true;
  bool require_joint_states = true;
  std::string odom_topic;
  std::string scan_topic;
  std::string joint_states_topic;
  double odom_timeout_sec = 0.5;
  double scan_timeout_sec = 0.5;
  double joint_states_timeout_sec = 0.5;

  bool require_localization = false;
  double localization_timeout_sec = 0.5;
  double localization_recovery_hold_sec = 1.0;
  std::string localization_state_topic;
  std::string localization_pose_topic;

  TfRelayConfig tf_relay;
};

}  // namespace

class CmdVelWatchdog final : public rclcpp::Node
{
public:
  explicit CmdVelWatchdog(const rclcpp::NodeOptions & options) : Node("cmd_vel_watchdog", options)
  {
    TracySetProgramName("cmd_vel_watchdog");
    load_config();
    init_interfaces();
  }

private:
  struct ReadinessState {
    std::optional<bool> value;
    double last_sent_sec = 0.0;
  };

  void load_config()
  {
    config_.input_topic = declare_parameter("input_topic", "/cmd_vel_collision");
    config_.output_topic = declare_parameter("output_topic", "/cmd_vel");
    if (config_.input_topic == config_.output_topic) {
      throw std::invalid_argument("watchdog input/output loop");
    }

    config_.output_frame_id = declare_parameter("output_frame_id", "base_link");
    config_.status_topic = declare_parameter("status_topic", "/cmd_vel_watchdog/healthy");
    config_.command_timeout_sec = checked_seconds(*this, "timeout", 0.3, true);
    config_.publish_rate_hz = checked_rate_hz(*this, "publish_rate", 20.0);

    config_.require_odom = declare_parameter("require_odom", true);
    config_.require_scan = declare_parameter("require_scan", true);
    config_.require_joint_states = declare_parameter("require_joint_states", true);
    config_.odom_topic = declare_parameter("odom_topic", "/odom");
    config_.scan_topic = declare_parameter("scan_topic", "/scan");
    config_.joint_states_topic = declare_parameter("joint_states_topic", "/joint_states");
    config_.odom_timeout_sec = checked_seconds(*this, "odom_timeout", 0.5, true);
    config_.scan_timeout_sec = checked_seconds(*this, "scan_timeout", 0.5, true);
    config_.joint_states_timeout_sec = checked_seconds(*this, "joint_states_timeout", 0.5, true);

    config_.require_localization = declare_parameter("require_localization", false);
    config_.localization_timeout_sec = checked_seconds(*this, "localization_timeout", 0.5);
    config_.localization_recovery_hold_sec =
      checked_seconds(*this, "localization_recovery_hold", 1.0, true);
    config_.localization_state_topic =
      declare_parameter("localization_state_topic", "/backend_node/localization_state");
    config_.localization_pose_topic =
      declare_parameter("localization_pose_topic", "/localization/odometry");
    readiness_interval_sec_ = std::min(0.15, config_.localization_timeout_sec / 3.0);

    config_.tf_relay.require_tf = declare_parameter("require_tf", true);
    config_.tf_relay.tf_timeout_sec = checked_seconds(*this, "tf_timeout", 0.5, true);
    config_.tf_relay.target_frame =
      TfRelayGate::normalize_frame_name(declare_parameter("tf_target_frame", "odom"));
    config_.tf_relay.source_frame =
      TfRelayGate::normalize_frame_name(declare_parameter("tf_source_frame", "base_link"));
    config_.tf_relay.global_frame = declare_parameter("global_frame", "map");
    config_.tf_relay.odom_frame = declare_parameter("odom_frame", "odom");
    config_.tf_relay.base_frame = config_.output_frame_id;
    config_.tf_relay.required_pairs =
      TfRelayGate::parse_required_pairs(declare_parameter<std::vector<std::string>>(
        "required_tf_pairs", std::vector<std::string>{"odom->base_link"}));
    config_.tf_relay.tf_publish_rate_hz = checked_rate_hz(*this, "tf_publish_rate", 50.0);
    config_.tf_relay.input_topics = declare_parameter<std::vector<std::string>>(
      "tf_input_topics",
      std::vector<std::string>{"/sentry/tf/odometry", "/sentry/tf/localization"});
  }

  void init_interfaces()
  {
    localization_ = std::make_unique<LocalizationHealth>(config_.localization_timeout_sec,
                                                         config_.localization_recovery_hold_sec);

    if (config_.require_localization) {
      state_sub_ = create_subscription<std_msgs::msg::String>(
        config_.localization_state_topic, rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          localization_->observe_state(message->data, steady_seconds());
        });
      pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.localization_pose_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          localization_->observe_pose(*message, steady_seconds());
        });
    }

    command_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(config_.output_topic, rclcpp::QoS(10));
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>(config_.status_topic, rclcpp::QoS(10));
    localization_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/localization_ready", rclcpp::QoS(1).transient_local());
    tf_ready_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/tf_ready", rclcpp::QoS(1).transient_local());

    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      config_.input_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
        const std::string command_frame =
          TfRelayGate::normalize_frame_name(message->header.frame_id);
        const std::string expected_frame =
          TfRelayGate::normalize_frame_name(config_.output_frame_id);
        const bool valid_frame = command_frame.empty() || command_frame == expected_frame;
        if (!is_finite_twist(message->twist) || !valid_frame) {
          if (!valid_frame) {
            RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000,
                                 "Rejecting velocity command in unexpected frame '%s' (expected '%s')",
                                 command_frame.c_str(), expected_frame.c_str());
          }
          command_received_sec_.reset();
          return;
        }
        latest_command_ = *message;
        command_received_sec_ = steady_seconds();
      });

    init_keepalive_subscriptions();
    tf_relay_ = std::make_unique<TfRelayGate>(*this, config_.tf_relay);
    tick_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.publish_rate_hz),
                                    [this] { tick(); });
  }

  void init_keepalive_subscriptions()
  {
    // Poll keepalive streams in tick() via GenericSubscription in an unattached
    // callback group so high-rate messages neither deserialize nor wake the executor.
    keepalive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions keepalive_options;
    keepalive_options.callback_group = keepalive_group_;
    const auto keepalive_qos = rclcpp::SensorDataQoS().keep_last(1);

    auto make_keepalive = [this, &keepalive_qos, &keepalive_options](const std::string & topic,
                                                                     const std::string & type) {
      return create_generic_subscription(
        topic, type, keepalive_qos, [](const rclcpp::SerializedMessage &) {}, keepalive_options);
    };

    if (config_.require_odom) {
      odom_sub_ = make_keepalive(config_.odom_topic, "nav_msgs/msg/Odometry");
    }
    if (config_.require_scan) {
      scan_sub_ = make_keepalive(config_.scan_topic, "sensor_msgs/msg/LaserScan");
    }
    if (config_.require_joint_states) {
      joint_sub_ = make_keepalive(config_.joint_states_topic, "sensor_msgs/msg/JointState");
    }
  }

  void publish_readiness(rclcpp::Publisher<std_msgs::msg::Bool> & publisher, ReadinessState & state,
                         bool ready, double steady_now)
  {
    if (state.value && *state.value == ready &&
        steady_now - state.last_sent_sec < readiness_interval_sec_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = ready;
    publisher.publish(message);
    state.value = ready;
    state.last_sent_sec = steady_now;
  }

  void poll_keepalive(const rclcpp::GenericSubscription::SharedPtr & subscription,
                      std::optional<double> & received_sec, double steady_now)
  {
    if (subscription && subscription->take_serialized(keepalive_buffer_, keepalive_info_)) {
      received_sec = steady_now;
    }
  }

  bool is_command_stamp_valid(const rclcpp::Time & ros_now) const
  {
    const auto stamp = stamp_ns(latest_command_.header.stamp);
    const auto age_ns = ros_now.nanoseconds() - stamp;
    return stamp > 0 && age_ns >= -50000000LL &&
           static_cast<double>(age_ns) <= config_.command_timeout_sec * 1.0e9;
  }

  void log_health_transition(bool healthy, double steady_now,
                             const LocalizationHealth::Evaluation & localization,
                             bool command_fresh, bool command_stamp_valid, bool odom_fresh,
                             bool scan_fresh, bool joints_fresh, bool tf_path_ready,
                             const TfGateStatus & tf_status)
  {
    if (previous_health_ && *previous_health_ == healthy) {
      return;
    }
    if (healthy) {
      RCLCPP_INFO(get_logger(), "Velocity safety gate opened");
    } else {
      std::ostringstream reason_text;
      for (const auto & reason : localization.reasons()) {
        reason_text << reason << "; ";
      }
      if (!command_fresh) {
        reason_text << "command timeout; ";
      }
      if (!command_stamp_valid) {
        reason_text << "command source timestamp expired or invalid; ";
      }
      if (!odom_fresh) {
        reason_text << "odometry timeout; ";
      }
      if (!scan_fresh) {
        reason_text << "scan timeout; ";
      }
      if (!joints_fresh) {
        reason_text << "joint_states timeout; ";
      }
      if (!tf_path_ready) {
        reason_text << "missing/conflicting TF " << config_.tf_relay.target_frame << "->"
                    << config_.tf_relay.source_frame << "; ";
      }
      if (!tf_status.streams_fresh) {
        for (const auto & [parent, child] : tf_relay_->timed_out_pairs(steady_now)) {
          reason_text << "TF stream timeout " << parent << "->" << child << "; ";
        }
      }
      RCLCPP_WARN(get_logger(), "Velocity safety gate closed; publishing zero: %s",
                  reason_text.str().c_str());
    }
    previous_health_ = healthy;
  }

  void tick()
  {
    ZoneScopedN("CmdVelWatchdog::tick");
    const double steady_now = steady_seconds();
    const auto ros_now = now();

    const TfGateStatus tf_status = tf_relay_->evaluate(steady_now);

    LocalizationHealth::Evaluation localization;
    if (config_.require_localization) {
      localization = localization_->evaluate(steady_now, ros_now.nanoseconds());
      if (!localization.ready() || !localization_ready_) {
        command_received_sec_.reset();
      }
      localization_ready_ = localization.ready();
    }

    publish_readiness(
      *localization_ready_pub_, localization_readiness_state_,
      config_.require_localization && localization.ready() && tf_status.map_to_odom_ready,
      steady_now);
    publish_readiness(*tf_ready_pub_, tf_readiness_state_, tf_status.map_to_odom_ready, steady_now);

    poll_keepalive(odom_sub_, odom_received_sec_, steady_now);
    poll_keepalive(scan_sub_, scan_received_sec_, steady_now);
    poll_keepalive(joint_sub_, joints_received_sec_, steady_now);

    const bool command_fresh =
      fresh(command_received_sec_, config_.command_timeout_sec, steady_now);
    const bool command_stamp_valid = is_command_stamp_valid(ros_now);
    const bool odom_fresh =
      !config_.require_odom || fresh(odom_received_sec_, config_.odom_timeout_sec, steady_now);
    const bool scan_fresh =
      !config_.require_scan || fresh(scan_received_sec_, config_.scan_timeout_sec, steady_now);
    const bool joints_fresh =
      !config_.require_joint_states ||
      fresh(joints_received_sec_, config_.joint_states_timeout_sec, steady_now);
    const bool tf_path_ready = !config_.tf_relay.require_tf || tf_status.target_to_source_ready;

    const bool healthy = localization.ready() && command_fresh && command_stamp_valid &&
                         odom_fresh && scan_fresh && joints_fresh && tf_path_ready &&
                         tf_status.streams_fresh;

    auto output = healthy ? latest_command_ : geometry_msgs::msg::TwistStamped();
    if (output.header.frame_id.empty()) {
      output.header.frame_id = config_.output_frame_id;
    }
    output.header.stamp = ros_now;
    command_pub_->publish(output);

    std_msgs::msg::Bool state_message;
    state_message.data = healthy;
    healthy_pub_->publish(state_message);

    log_health_transition(healthy, steady_now, localization, command_fresh, command_stamp_valid,
                          odom_fresh, scan_fresh, joints_fresh, tf_path_ready, tf_status);
  }

  // Configuration
  WatchdogConfig config_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  double readiness_interval_sec_ = 0.15;

  // Health and command state
  bool localization_ready_ = false;
  ReadinessState localization_readiness_state_;
  ReadinessState tf_readiness_state_;
  std::optional<bool> previous_health_;
  std::optional<double> command_received_sec_;
  std::optional<double> odom_received_sec_;
  std::optional<double> scan_received_sec_;
  std::optional<double> joints_received_sec_;
  geometry_msgs::msg::TwistStamped latest_command_;
  std::unique_ptr<LocalizationHealth> localization_;
  std::unique_ptr<TfRelayGate> tf_relay_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tf_ready_pub_;

  rclcpp::CallbackGroup::SharedPtr keepalive_group_;
  rclcpp::SerializedMessage keepalive_buffer_;
  rclcpp::MessageInfo keepalive_info_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::GenericSubscription::SharedPtr odom_sub_;
  rclcpp::GenericSubscription::SharedPtr scan_sub_;
  rclcpp::GenericSubscription::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace xxu_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::CmdVelWatchdog)
