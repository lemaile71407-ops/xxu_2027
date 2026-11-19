#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

#include "xxu_bringup/odometry_conversion.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace xxu_bringup {

class ZlioBaseOdometry final : public rclcpp::Node
{
public:
  explicit ZlioBaseOdometry(const rclcpp::NodeOptions & options)
  : Node("zlio_base_odometry", options)
  {
    const double tf_rate = declare_parameter<double>("tf_publish_rate", 30.0);
    if (!std::isfinite(tf_rate) || tf_rate < 20.0 || tf_rate > 50.0) {
      throw std::invalid_argument("tf_publish_rate must be within [20, 50] Hz");
    }
    sensor_ = declare_parameter("sensor_frame", "lio_base_sensor");
    source_ = declare_parameter("source_child_frame", "imu");
    odom_ = declare_parameter("odom_frame", "odom");
    base_ = declare_parameter("base_frame", "base_link");
    const auto input = declare_parameter("input_topic", "/odometry");
    const auto output = declare_parameter("output_topic", "/odom");
    if (sensor_.empty() || source_.empty() || odom_.empty() || base_.empty() ||
      odom_ == base_ || base_ == sensor_ || source_ == base_ || source_ == odom_ ||
      sensor_ == odom_ || input == output)
    {
      throw std::invalid_argument("invalid bridge frames or input/output loop");
    }

    const auto mode = declare_parameter("extrinsics_source", "tf_static");
    const auto xyz = declare_parameter<std::vector<double>>("base_from_sensor_xyz", std::vector<double>{});
    const auto rpy = declare_parameter<std::vector<double>>("base_from_sensor_rpy", std::vector<double>{});
    if (mode == "parameters") {
      if (xyz.size() != 3 || rpy.size() != 3) {
        throw std::invalid_argument("measured base_from_sensor_xyz/rpy must each contain 3 values");
      }
      base_from_sensor_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
      base_from_sensor_.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
      if (!base_from_sensor_.matrix().allFinite()) {
        throw std::invalid_argument("nonfinite measured extrinsics");
      }
      ready_ = true;
    } else if (mode == "tf_static") {
      // Subscribe ONLY to latched static transforms at startup. A dynamic
      // gimbal edge cannot accidentally become a cached rigid calibration.
      buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          if (ready_) {return;}
          for (const auto & transform : message->transforms) {
            buffer_->setTransform(transform, "static_mount", true);
          }
          try {
            const auto t = buffer_->lookupTransform(base_, sensor_, tf2::TimePointZero).transform;
            base_from_sensor_.linear() = rotation(t.rotation).toRotationMatrix();
            base_from_sensor_.translation() = Eigen::Vector3d(
              t.translation.x, t.translation.y, t.translation.z);
            if (!base_from_sensor_.matrix().allFinite()) {
              throw std::invalid_argument("nonfinite static mount");
            }
            ready_ = true;
            static_sub_.reset();
            buffer_.reset();
            RCLCPP_INFO(get_logger(), "Cached rigid %s <- %s; all TF subscriptions released",
              base_.c_str(), sensor_.c_str());
          } catch (const tf2::TransformException &) {
            // Other static publishers may arrive later. Never fall back to identity.
          }
        });
    } else {
      throw std::invalid_argument("extrinsics_source must be parameters or tf_static");
    }
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, rclcpp::SensorDataQoS());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(input, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odometry(*message);});
    // Sample the latest new pose at a fixed rate. A callback-time throttle
    // aliases 30 Hz input to 15 Hz under small arrival jitter. Never refresh
    // a stale pose merely because this wall timer is still running.
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_rate), [this] {
      if (pending_tf_) {
        broadcaster_->sendTransform(*pending_tf_);
        pending_tf_.reset();
      }
    });
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    if (!ready_ || message.header.frame_id != odom_ || message.child_frame_id != source_ ||
      message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u ||
      (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000,
        "Waiting for rigid mount or rejecting unexpected odometry frame/time");
      return;
    }
    try {
      auto output = convert_odometry(message, base_from_sensor_, base_);
      if (last_input_stamp_ && *last_input_stamp_ == message.header.stamp) {return;}
      last_input_stamp_ = message.header.stamp;

      geometry_msgs::msg::TransformStamped tf;
      tf.header = output.header;
      tf.child_frame_id = output.child_frame_id;
      tf.transform.translation.x = output.pose.pose.position.x;
      tf.transform.translation.y = output.pose.pose.position.y;
      tf.transform.translation.z = output.pose.pose.position.z;
      tf.transform.rotation = output.pose.pose.orientation;
      auto output_message = std::make_unique<nav_msgs::msg::Odometry>(std::move(output));
      publisher_->publish(std::move(output_message));
      pending_tf_ = tf;
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "%s", error.what());
    }
  }

  std::string sensor_, source_, odom_, base_;
  Eigen::Isometry3d base_from_sensor_ = Eigen::Isometry3d::Identity();
  bool ready_ = false;
  std::optional<geometry_msgs::msg::TransformStamped> pending_tf_;
  std::optional<builtin_interfaces::msg::Time> last_input_stamp_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};
}  // namespace xxu_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::ZlioBaseOdometry)
