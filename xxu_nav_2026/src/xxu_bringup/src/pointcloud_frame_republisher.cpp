#include "robot_self_filter.hpp"
#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/joint_history.hpp"
#include "xxu_bringup/node_parameters.hpp"
#include "xxu_bringup/odometry_conversion.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace xxu_bringup
{
namespace
{

struct RepublisherConfig {
  std::string target_frame;
  std::string expected_source_frame;
  bool use_point_timestamps = false;
  bool use_joint_interpolation = false;
  double joint_position_offset = 0.0;
  Eigen::Vector3d joint_axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d sensor_offset = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sensor_rotation = Eigen::Matrix3d::Identity();
  double pending_timeout_sec = 0.25;
  int pending_queue_size = 3;
};

}  // namespace

class PointCloudFrameRepublisher final : public rclcpp::Node
{
public:
  explicit PointCloudFrameRepublisher(const rclcpp::NodeOptions & options)
      : Node("pointcloud_frame_republisher", options)
  {
    load_config();
    init_interfaces();
  }

private:
  struct PendingCloud {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    CloudLayout layout;
    double received_sec = 0.0;
    std::optional<std::pair<double, double>> time_range_sec;
  };

  void load_config()
  {
    config_.target_frame = declare_parameter("target_frame", "mid360_link");
    config_.expected_source_frame = declare_parameter("expected_source_frame", "");
    config_.use_point_timestamps = declare_parameter("use_point_timestamps", false);
    config_.use_joint_interpolation = declare_parameter("use_joint_interpolation", false);
    if (config_.use_joint_interpolation && !config_.use_point_timestamps) {
      throw std::invalid_argument("joint interpolation requires point timestamps");
    }

    declare_parameter("tf_lookup_timeout", 0.0);
    declare_parameter("max_tf_timestamp_groups", 256);

    joint_history_.name = declare_parameter("joint_name", "gimbal_joint");
    joint_history_.max_sample_gap_sec = checked_seconds(*this, "max_joint_sample_gap", 0.05);
    joint_history_.time_reset_threshold_sec = checked_seconds(*this, "time_reset_threshold", 0.5);

    config_.joint_position_offset = declare_parameter("joint_position_offset", 0.0);
    if (!std::isfinite(config_.joint_position_offset)) {
      throw std::invalid_argument("invalid joint position offset");
    }

    config_.joint_axis = vector_parameter(*this, "joint_axis", {0.0, 0.0, 1.0});
    if (config_.joint_axis.norm() < 1.0e-12) {
      throw std::invalid_argument("zero joint axis");
    }
    config_.joint_axis.normalize();

    config_.sensor_offset = vector_parameter(*this, "sensor_offset", {0.0, 0.08637, 0.0});
    config_.sensor_rotation =
      rpy_rotation(vector_parameter(*this, "sensor_rpy", {-0.2967059728, 0.0, 0.0}));

    config_.pending_timeout_sec = checked_seconds(*this, "pending_timeout", 0.25);
    config_.pending_queue_size = declare_parameter("pending_queue_size", 3);
    if (config_.pending_queue_size <= 0) {
      throw std::invalid_argument("pending_queue_size must be positive");
    }

    const std::string geometry_path = declare_parameter("self_filter_geometry_file", "");
    if (!geometry_path.empty()) {
      self_filter_.load(geometry_path, config_.target_frame);
    }
  }

  void init_interfaces()
  {
    if (!config_.use_joint_interpolation) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
          if (!tf_buffer_) {
            return;
          }
          for (const auto & transform : message->transforms) {
            tf_buffer_->setTransform(transform, "static_mount", true);
          }
          process_pending();
        });
    } else {
      joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          if (joint_history_.add(*message, false)) {
            pending_.clear();
          }
          process_pending();
        });
    }

    output_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("points_out", rclcpp::SensorDataQoS());
    input_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_in", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        on_cloud(std::move(message));
      });
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this] { process_pending(); }, nullptr, false);
  }

  std::optional<std::pair<double, double>> compute_time_range(
    const sensor_msgs::msg::PointCloud2 & input, const CloudLayout & layout) const
  {
    if (!config_.use_point_timestamps || input.width == 0 || input.height == 0) {
      return std::nullopt;
    }

    double min_ns = 9.22e18;
    double max_ns = 0.0;
    for (std::uint32_t row = 0; row < input.height; ++row) {
      const auto * record = input.data.data() + static_cast<std::size_t>(row) * input.row_step;
      for (std::uint32_t col = 0; col < input.width; ++col, record += input.point_step) {
        const double point_ns = read_scalar<double>(record + layout.timestamp, input.is_bigendian);
        if (!std::isfinite(point_ns) || point_ns < 0.0 || point_ns >= 9.22e18) {
          throw std::invalid_argument("invalid point timestamp");
        }
        min_ns = std::min(min_ns, point_ns);
        max_ns = std::max(max_ns, point_ns);
      }
    }
    return std::make_pair(std::round(min_ns) * 1.0e-9, std::round(max_ns) * 1.0e-9);
  }

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const bool empty_frame = message->header.frame_id.empty();
    const bool mismatched_frame = !config_.expected_source_frame.empty() &&
                                  message->header.frame_id != config_.expected_source_frame;
    if (empty_frame || mismatched_frame) {
      return;
    }

    const double received_sec = steady_seconds();
    try {
      const auto & input = *message;
      const CloudLayout layout(input, config_.use_point_timestamps);
      const auto time_range_sec = compute_time_range(input, layout);
      pending_.push_back({std::move(message), layout, received_sec, time_range_sec});
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Dropping cloud: %s", error.what());
      return;
    }

    if (pending_.size() > static_cast<std::size_t>(config_.pending_queue_size)) {
      pending_.pop_front();
    }
    process_pending();
  }

  bool cache_static_transform(const std::string & source_frame)
  {
    if (cached_source_frame_) {
      return *cached_source_frame_ == source_frame;
    }
    if (source_frame == config_.target_frame) {
      static_transform_.setIdentity();
    } else {
      if (!tf_buffer_) {
        return false;
      }
      try {
        const auto t =
          tf_buffer_->lookupTransform(config_.target_frame, source_frame, tf2::TimePointZero)
            .transform;
        static_transform_.linear() = rotation(t.rotation).toRotationMatrix();
        static_transform_.translation() =
          Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
        if (!static_transform_.matrix().allFinite()) {
          throw std::invalid_argument("nonfinite static transform");
        }
      } catch (const tf2::TransformException &) {
        return false;
      }
    }
    cached_source_frame_ = source_frame;
    static_sub_.reset();
    tf_buffer_.reset();
    return true;
  }

  std::unique_ptr<sensor_msgs::msg::PointCloud2> allocate_output_cloud(
    const sensor_msgs::msg::PointCloud2 & input, bool filter_enabled) const
  {
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (filter_enabled) {
      const std::size_t point_count = static_cast<std::size_t>(input.width) * input.height;
      output->header = input.header;
      output->height = 1;
      output->fields = input.fields;
      output->is_bigendian = input.is_bigendian;
      output->point_step = input.point_step;
      output->is_dense = input.is_dense;
      output->data.reserve(point_count * input.point_step);
    } else {
      *output = input;
    }
    output->header.frame_id = config_.target_frame;
    return output;
  }

  bool process_cloud(const PendingCloud & entry)
  {
    const auto & input = *entry.message;
    const auto & layout = entry.layout;
    if (!config_.use_joint_interpolation && !cache_static_transform(input.header.frame_id)) {
      return false;
    }

    // JointHistory starts a new contiguous history whenever the sample gap is too large.
    if (config_.use_joint_interpolation && entry.time_range_sec) {
      const bool range_covered = joint_history_.interpolate(entry.time_range_sec->first, false) &&
                                 joint_history_.interpolate(entry.time_range_sec->second, false);
      if (!range_covered) {
        return false;
      }
    }

    const bool filter_enabled = self_filter_.enabled();
    auto output = allocate_output_cloud(input, filter_enabled);

    std::optional<double> previous_point_time_sec;
    Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();

    for (std::uint32_t row = 0; row < input.height; ++row) {
      for (std::uint32_t col = 0; col < input.width; ++col) {
        const std::size_t input_offset =
          static_cast<std::size_t>(row) * input.row_step + col * input.point_step;
        const auto * source_point = input.data.data() + input_offset;

        if (config_.use_joint_interpolation) {
          const double point_ns =
            read_scalar<double>(source_point + layout.timestamp, input.is_bigendian);
          const double point_time_sec = std::round(point_ns) * 1.0e-9;
          if (previous_point_time_sec != point_time_sec) {
            const auto joint = joint_history_.interpolate(point_time_sec, false);
            if (!joint) {
              return false;
            }
            const Eigen::Matrix3d joint_rotation =
              Eigen::AngleAxisd(joint->position + config_.joint_position_offset, config_.joint_axis)
                .toRotationMatrix();
            joint_transform.linear() = joint_rotation * config_.sensor_rotation;
            joint_transform.translation() = joint_rotation * config_.sensor_offset;
            previous_point_time_sec = point_time_sec;
          }
        }

        Eigen::Vector3d point;
        for (int axis = 0; axis < 3; ++axis) {
          point[axis] = read_scalar<float>(source_point + layout.xyz[axis], input.is_bigendian);
        }
        point = (config_.use_joint_interpolation ? joint_transform : static_transform_) * point;

        std::size_t output_offset = input_offset;
        if (filter_enabled) {
          if (self_filter_.contains(point)) {
            continue;
          }
          output_offset = output->data.size();
          output->data.insert(output->data.end(), source_point, source_point + input.point_step);
        }

        auto * destination_point = output->data.data() + output_offset;
        for (int axis = 0; axis < 3; ++axis) {
          write_scalar<float>(destination_point + layout.xyz[axis], static_cast<float>(point[axis]),
                              input.is_bigendian);
        }
      }
    }

    if (filter_enabled) {
      output->width = output->data.size() / output->point_step;
      output->row_step = output->width * output->point_step;
    }
    output_pub_->publish(std::move(output));
    return true;
  }

  void process_pending()
  {
    while (!pending_.empty()) {
      try {
        const bool processed = process_cloud(pending_.front());
        const bool within_timeout =
          steady_seconds() - pending_.front().received_sec <= config_.pending_timeout_sec;
        if (!processed && within_timeout) {
          if (retry_timer_->is_canceled()) {
            retry_timer_->reset();
          }
          return;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Dropping cloud: %s", error.what());
      }
      pending_.pop_front();
    }
    if (!retry_timer_->is_canceled()) {
      retry_timer_->cancel();
    }
  }

  // Configuration and helpers
  RepublisherConfig config_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  JointHistory joint_history_;
  RobotSelfFilter self_filter_;

  // Runtime transform and pending queue state
  Eigen::Isometry3d static_transform_ = Eigen::Isometry3d::Identity();
  std::optional<std::string> cached_source_frame_;
  std::deque<PendingCloud> pending_;

  // ROS interfaces
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace xxu_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::PointCloudFrameRepublisher)
