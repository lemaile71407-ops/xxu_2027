#include "xxu_bringup/overhead_clearance.hpp"

#include "xxu_bringup/node_parameters.hpp"
#include "xxu_bringup/odometry_conversion.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.hpp>

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace xxu_bringup
{
namespace
{

struct NodeConfig {
  std::string cloud_topic = "/cloud_deskewed";
  std::string odom_topic = "/odom";
  std::string stow_topic = "/overhead_clearance/stow_gimbal";
  std::string status_topic = "/overhead_clearance/status";
  std::string base_frame = "base_link";
  std::string sensor_frame = "lio_base_sensor";
  std::string extrinsics_source = "tf_static";
  double status_check_rate_hz = 5.0;
  Eigen::Isometry3d base_from_sensor = Eigen::Isometry3d::Identity();
  bool extrinsics_ready = false;
  OverheadClearanceConfig detector;
};

}  // namespace

class OverheadClearanceNode final : public rclcpp::Node
{
public:
  explicit OverheadClearanceNode(const rclcpp::NodeOptions & options)
      : Node("overhead_clearance", options)
  {
    load_config();
    detector_ = std::make_unique<OverheadClearanceDetector>(config_.detector);
    init_interfaces();
    publish_status(detector_->status());
  }

private:
  void load_config()
  {
    config_.cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_deskewed");
    config_.odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    config_.stow_topic =
      declare_parameter<std::string>("stow_topic", "/overhead_clearance/stow_gimbal");
    config_.status_topic =
      declare_parameter<std::string>("status_topic", "/overhead_clearance/status");
    config_.base_frame = declare_parameter<std::string>("base_frame", "base_link");
    config_.sensor_frame = declare_parameter<std::string>("sensor_frame", "lio_base_sensor");
    if (config_.cloud_topic.empty() || config_.odom_topic.empty() || config_.stow_topic.empty() ||
        config_.status_topic.empty() || config_.base_frame.empty() ||
        config_.sensor_frame.empty()) {
      throw std::invalid_argument(
        "overhead_clearance topic and frame parameters must be non-empty");
    }

    config_.detector.base_height_m = checked_positive_number(*this, "base_height_m", 0.0, true);
    config_.detector.min_overhead_height_m =
      checked_positive_number(*this, "min_overhead_height_m", 0.22, true);
    config_.detector.max_overhead_height_m =
      checked_positive_number(*this, "max_overhead_height_m", 0.55, false);
    config_.detector.min_xy_radius_m =
      checked_positive_number(*this, "min_xy_radius_m", 0.15, true);
    config_.detector.detection_radius_m =
      checked_positive_number(*this, "detection_radius_m", 1.50, false);
    config_.detector.min_points = declare_parameter<int>("min_points", 8);
    config_.detector.trigger_frames = declare_parameter<int>("trigger_frames", 2);
    config_.detector.release_hold_sec = checked_seconds(*this, "release_hold_sec", 1.5, true);
    config_.detector.release_travel_distance_m =
      checked_positive_number(*this, "release_travel_distance_m", 0.80, true);
    config_.detector.max_odom_step_m =
      checked_positive_number(*this, "max_odom_step_m", 1.0, false);
    config_.detector.cloud_timeout_sec = checked_seconds(*this, "cloud_timeout_sec", 0.5, false);
    config_.status_check_rate_hz = checked_rate_hz(*this, "status_check_rate_hz", 5.0);

    validate_overhead_clearance_config(config_.detector);

    config_.extrinsics_source = declare_parameter<std::string>("extrinsics_source", "tf_static");
    const auto xyz =
      declare_parameter<std::vector<double>>("base_from_sensor_xyz", std::vector<double>{});
    const auto rpy =
      declare_parameter<std::vector<double>>("base_from_sensor_rpy", std::vector<double>{});

    if (config_.extrinsics_source == "identity") {
      config_.base_from_sensor = Eigen::Isometry3d::Identity();
      config_.extrinsics_ready = true;
    } else if (config_.extrinsics_source == "parameters") {
      if (xyz.size() != 3 || rpy.size() != 3) {
        throw std::invalid_argument(
          "base_from_sensor_xyz and base_from_sensor_rpy must each contain 3 values");
      }
      config_.base_from_sensor.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
      config_.base_from_sensor.linear() = rpy_rotation(Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
      if (!config_.base_from_sensor.matrix().allFinite()) {
        throw std::invalid_argument("nonfinite base_from_sensor extrinsics");
      }
      config_.extrinsics_ready = true;
    } else if (config_.extrinsics_source == "tf_static") {
      if (config_.base_frame == config_.sensor_frame) {
        config_.base_from_sensor = Eigen::Isometry3d::Identity();
        config_.extrinsics_ready = true;
      }
    } else {
      throw std::invalid_argument("extrinsics_source must be tf_static, parameters, or identity");
    }
  }

  void init_interfaces()
  {
    const auto status_qos = rclcpp::QoS(1).transient_local().reliable();
    stow_pub_ = create_publisher<std_msgs::msg::Bool>(config_.stow_topic, status_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic, status_qos);

    if (config_.extrinsics_source == "tf_static" && !config_.extrinsics_ready) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) { on_static_tf(*message); });
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        detector_->observe_odometry(*message);
      });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { on_cloud(*message); });

    const auto period = std::chrono::duration<double>(1.0 / config_.status_check_rate_hz);
    staleness_timer_ = create_wall_timer(period, [this] {
      const auto previous = detector_->status();
      const auto updated = detector_->check_staleness(steady_seconds());
      if (updated.cloud_stale != previous.cloud_stale || updated.reason != previous.reason) {
        publish_status(updated);
      }
    });
  }

  void on_static_tf(const tf2_msgs::msg::TFMessage & message)
  {
    if (config_.extrinsics_ready || !tf_buffer_) {
      return;
    }
    for (const auto & transform : message.transforms) {
      tf_buffer_->setTransform(transform, "static_mount", true);
    }
    try {
      const auto t =
        tf_buffer_->lookupTransform(config_.base_frame, config_.sensor_frame, tf2::TimePointZero)
          .transform;
      Eigen::Isometry3d cached = Eigen::Isometry3d::Identity();
      cached.linear() = rotation(t.rotation).toRotationMatrix();
      cached.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
      if (!cached.matrix().allFinite()) {
        throw std::invalid_argument("nonfinite static mount");
      }
      config_.base_from_sensor = cached;
      config_.extrinsics_ready = true;
      static_sub_.reset();
      tf_buffer_.reset();
      RCLCPP_INFO(get_logger(), "Cached static extrinsics %s <- %s; /tf_static listener released",
                  config_.base_frame.c_str(), config_.sensor_frame.c_str());
    } catch (const tf2::TransformException &) {
      // Wait for the remaining static transform publisher without assuming identity.
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (cloud.header.frame_id == config_.base_frame) {
      transform = Eigen::Isometry3d::Identity();
    } else if (cloud.header.frame_id == config_.sensor_frame) {
      if (!config_.extrinsics_ready) {
        RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000,
                             "Waiting for static extrinsics %s <- %s before evaluating %s",
                             config_.base_frame.c_str(), config_.sensor_frame.c_str(),
                             config_.cloud_topic.c_str());
        return;
      }
      transform = config_.base_from_sensor;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000,
                           "Ignoring cloud in unexpected frame '%s' (expected '%s' or '%s')",
                           cloud.header.frame_id.c_str(), config_.base_frame.c_str(),
                           config_.sensor_frame.c_str());
      return;
    }

    try {
      const auto status = detector_->evaluate_cloud(cloud, transform, steady_seconds());
      publish_status(status);
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "%s", error.what());
    }
  }

  void publish_status(const OverheadClearanceStatus & status)
  {
    std_msgs::msg::Bool stow_msg;
    stow_msg.data = status.stow_gimbal;
    stow_pub_->publish(stow_msg);

    std_msgs::msg::String status_msg;
    status_msg.data = format_overhead_clearance_status(status);
    status_pub_->publish(status_msg);
  }

  NodeConfig config_;
  std::unique_ptr<OverheadClearanceDetector> detector_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stow_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr staleness_timer_;
};

}  // namespace xxu_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::OverheadClearanceNode)
