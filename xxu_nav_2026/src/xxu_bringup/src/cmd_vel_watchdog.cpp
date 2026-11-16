#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/sensor_math.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <set>
#include <sstream>

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
    localization_ = std::make_unique<LocalizationHealth>(
      checked_seconds(*this, "localization_timeout", .5),
      checked_seconds(*this, "localization_recovery_hold", 1., true));
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
    if (require_odom_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr) {odom_received_ = steady_seconds();});
    }
    if (require_scan_) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {scan_received_ = steady_seconds();});
    }
    if (require_joints_) {
      joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(joint_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr) {joints_received_ = steady_seconds();});
    }
    // Each TF publisher is remapped to exactly one private input. This relay
    // keeps standard /tf intact for Nav2, without subscribing to global /tf or
    // creating any TransformListener. Receipt times are recorded at this gate.
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const double tf_rate = checked_seconds(*this, "tf_publish_rate", 50.);
    if (tf_rate < 20. || tf_rate > 50.) {
      throw std::invalid_argument("tf_publish_rate must be within [20, 50] Hz");
    }
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1. / tf_rate), [this] {
      std::vector<geometry_msgs::msg::TransformStamped> latest;
      for (const auto & item : pending_tf_) {latest.push_back(item.second);}
      pending_tf_.clear();
      if (!latest.empty()) {broadcaster_->sendTransform(latest);}
    });
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
      const auto known = parents_.find(child);
      if (known != parents_.end() && known->second != parent) {
        tf_conflict_ = true;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting multiple TF parents for %s", child.c_str());
        continue;
      }
      try {
        if (!buffer_->setTransform(t, authority, is_static)) {continue;}
      } catch (const tf2::TransformException &) {continue;}
      parents_[child] = parent;
      if (!is_static) {
        const auto pair = tf_received_.find({parent, child});
        if (pair != tf_received_.end()) {pair->second = steady_seconds();}
        pending_tf_[child] = t;
      }
    }
  }

  void tick()
  {
    const double steady = steady_seconds();
    const auto ros_time = now();
    std::vector<std::string> reasons;
    bool localization_ready = true;
    if (require_localization_) {
      auto state_reasons = localization_->check(steady, ros_time.nanoseconds());
      localization_ready = state_reasons.empty();
      if (!localization_ready || !localization_ready_) {command_received_.reset();}
      localization_ready_ = localization_ready;
      reasons.insert(reasons.end(), state_reasons.begin(), state_reasons.end());
    }
    std_msgs::msg::Bool tf_ready;
    tf_ready.data = !tf_conflict_ &&
      buffer_->canTransform(global_, odom_frame_, tf2::TimePointZero);
    std_msgs::msg::Bool ready;
    ready.data = require_localization_ && localization_ready && tf_ready.data;
    if (steady - last_readiness_ >= .1) {
      readiness_->publish(ready);
      tf_readiness_->publish(tf_ready);
      last_readiness_ = steady;
    }
    if (!fresh(command_received_, timeout_, steady)) {reasons.emplace_back("command timeout");}
    const auto stamp = stamp_ns(command_.header.stamp);
    const auto age = ros_time.nanoseconds() - stamp;
    if (stamp < 0 || age < 0 || static_cast<double>(age) > timeout_ * 1.e9) {
      reasons.emplace_back("command source timestamp expired or invalid");
    }
    if (require_odom_ && !fresh(odom_received_, odom_timeout_, steady)) {reasons.emplace_back("odometry timeout");}
    if (require_scan_ && !fresh(scan_received_, scan_timeout_, steady)) {reasons.emplace_back("scan timeout");}
    if (require_joints_ && !fresh(joints_received_, joints_timeout_, steady)) {reasons.emplace_back("joint_states timeout");}
    if (require_tf_) {
      if (tf_conflict_ || !buffer_->canTransform(tf_target_, tf_source_, tf2::TimePointZero)) {
        reasons.emplace_back("missing/conflicting TF " + tf_target_ + "->" + tf_source_);
      }
      for (const auto & pair : tf_received_) {
        if (!fresh(pair.second, tf_timeout_, steady)) {
          reasons.emplace_back("TF stream timeout " + pair.first.first + "->" + pair.first.second);
        }
      }
    }
    const bool healthy = reasons.empty();
    auto output = healthy ? command_ : geometry_msgs::msg::TwistStamped();
    if (output.header.frame_id.empty()) {output.header.frame_id = frame_;}
    output.header.stamp = ros_time;
    publisher_->publish(output);
    std_msgs::msg::Bool state; state.data = healthy; status_->publish(state);
    if (!previous_health_ || *previous_health_ != healthy) {
      if (healthy) {RCLCPP_INFO(get_logger(), "Velocity safety gate opened");}
      else {
        std::ostringstream text;
        for (const auto & reason : reasons) {text << reason << "; ";}
        RCLCPP_WARN(get_logger(), "Velocity safety gate closed; publishing zero: %s", text.str().c_str());
      }
      previous_health_ = healthy;
    }
  }
  std::string frame_, tf_target_, tf_source_, global_, odom_frame_;
  double timeout_, odom_timeout_, scan_timeout_, joints_timeout_, tf_timeout_;
  double last_readiness_ = 0.;
  bool require_odom_, require_scan_, require_joints_, require_tf_, require_localization_;
  bool localization_ready_ = false, tf_conflict_ = false;
  std::optional<bool> previous_health_;
  std::optional<double> command_received_, odom_received_, scan_received_, joints_received_;
  std::map<std::pair<std::string, std::string>, std::optional<double>> tf_received_;
  std::map<std::string, std::string> parents_;
  geometry_msgs::msg::TwistStamped command_;
  std::unique_ptr<LocalizationHealth> localization_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> pending_tf_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_, readiness_, tf_readiness_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  std::vector<rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr> tf_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace xxu_bringup
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::CmdVelWatchdog)
