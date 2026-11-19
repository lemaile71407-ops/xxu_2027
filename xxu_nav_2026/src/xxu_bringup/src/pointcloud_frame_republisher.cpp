#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/odometry_conversion.hpp"
#include "xxu_bringup/sensor_math.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

namespace xxu_bringup {
class PointCloudFrameRepublisher final : public rclcpp::Node
{
public:
  explicit PointCloudFrameRepublisher(const rclcpp::NodeOptions & options)
  : Node("pointcloud_frame_republisher", options)
  {
    target_ = declare_parameter("target_frame", "mid360_link");
    expected_ = declare_parameter("expected_source_frame", "");
    timed_ = declare_parameter("use_point_timestamps", false);
    joint_mode_ = declare_parameter("use_joint_interpolation", false);
    if (joint_mode_ && !timed_) {throw std::invalid_argument("joint interpolation requires point timestamps");}
    declare_parameter("tf_lookup_timeout", 0.);
    declare_parameter("max_tf_timestamp_groups", 256);
    history_.name = declare_parameter("joint_name", "gimbal_joint");
    history_.gap = checked_seconds(*this, "max_joint_sample_gap", .05);
    history_.reset_threshold = checked_seconds(*this, "time_reset_threshold", .5);
    position_offset_ = declare_parameter("joint_position_offset", 0.);
    if (!std::isfinite(position_offset_)) {throw std::invalid_argument("invalid joint position offset");}
    axis_ = vector_parameter(*this, "joint_axis", {0., 0., 1.});
    if (axis_.norm() < 1.e-12) {throw std::invalid_argument("zero joint axis");}
    axis_.normalize();
    offset_ = vector_parameter(*this, "sensor_offset", {0., .08637, 0.});
    sensor_rotation_ = rpy_rotation(vector_parameter(*this, "sensor_rpy", {-.2967059728, 0., 0.}));
    pending_timeout_ = checked_seconds(*this, "pending_timeout", .25);
    queue_limit_ = declare_parameter("pending_queue_size", 3);
    if (queue_limit_ <= 0) {throw std::invalid_argument("pending_queue_size must be positive");}
    const auto geometry = declare_parameter("self_filter_geometry_file", "");
    if (!geometry.empty()) {load_geometry(geometry);}
    if (!joint_mode_) {
      buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
        rclcpp::QoS(100).transient_local().reliable(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
          if (!buffer_) {return;}
          for (const auto & t : m->transforms) {buffer_->setTransform(t, "static_mount", true);}
          process_pending();
        });
    } else {
      joints_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
          if (history_.add(*m, false)) {pending_.clear();}
          process_pending();
        });
    }
    output_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_out", rclcpp::SensorDataQoS());
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>("points_in", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {on_cloud(std::move(m));});
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {process_pending();},
      nullptr, false);
  }
private:
  struct Wheel {Eigen::Vector3d center, axis; double half_width, radius2;};
  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    CloudLayout layout;
    double received;
    std::optional<std::pair<double, double>> time_range;
  };

  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    if (message->header.frame_id.empty() ||
      (!expected_.empty() && message->header.frame_id != expected_)) {return;}
    const double received = steady_seconds();
    try {
      const auto & input = *message;
      const CloudLayout layout(input, timed_);
      std::optional<std::pair<double, double>> time_range;
      if (timed_ && input.width != 0 && input.height != 0) {
        double min_ns = 9.22e18, max_ns = 0.;
        for (std::uint32_t row = 0; row < input.height; ++row) {
          const auto * record = input.data.data() + static_cast<std::size_t>(row) * input.row_step;
          for (std::uint32_t col = 0; col < input.width; ++col, record += input.point_step) {
            const double ns = read_scalar<double>(record + layout.timestamp, input.is_bigendian);
            if (!std::isfinite(ns) || ns < 0 || ns >= 9.22e18) {
              throw std::invalid_argument("invalid point timestamp");
            }
            min_ns = std::min(min_ns, ns);
            max_ns = std::max(max_ns, ns);
          }
        }
        time_range.emplace(std::round(min_ns) * 1.e-9, std::round(max_ns) * 1.e-9);
      }
      pending_.push_back({std::move(message), layout, received, time_range});
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Dropping cloud: %s", error.what());
      return;
    }
    if (pending_.size() > static_cast<std::size_t>(queue_limit_)) {pending_.pop_front();}
    process_pending();
  }

  void load_geometry(const std::string & path)
  {
    const auto data = YAML::LoadFile(path);
    const auto spec = data["self_filter"];
    if (spec["frame"].as<std::string>() != target_) {throw std::invalid_argument("self-filter frame mismatch");}
    tolerance_ = spec["tolerance"].as<double>();
    for (const auto & planes : spec["hull_planes"]) {
      std::vector<Eigen::Vector4d> hull;
      for (const auto & p : planes) {hull.emplace_back(p[0].as<double>(), p[1].as<double>(), p[2].as<double>(), p[3].as<double>());}
      hulls_.push_back(std::move(hull));
    }
    auto v = [](const YAML::Node & n) {return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());};
    for (const auto & w : data["wheels"]) {
      const double radius = w["radius"].as<double>() + tolerance_;
      wheels_.push_back({v(w["center"]), v(w["axis"]), w["width"].as<double>() / 2 + tolerance_, radius * radius});
    }
  }
  bool inside_robot(const Eigen::Vector3d & point) const
  {
    for (const auto & hull : hulls_) {
      bool inside = true;
      for (const auto & p : hull) {if (p.head<3>().dot(point) + p.w() > tolerance_) {inside = false; break;}}
      if (inside && point.allFinite()) {return true;}
    }
    for (const auto & wheel : wheels_) {
      const Eigen::Vector3d delta = point - wheel.center;
      const double axial = delta.dot(wheel.axis);
      if (std::abs(axial) <= wheel.half_width && delta.squaredNorm() - axial * axial <= wheel.radius2) {return true;}
    }
    return false;
  }
  bool cache_transform(const std::string & source)
  {
    if (cached_source_) {return *cached_source_ == source;}
    if (source == target_) {transform_.setIdentity();}
    else {
      if (!buffer_) {return false;}
      try {
        const auto t = buffer_->lookupTransform(target_, source, tf2::TimePointZero).transform;
        transform_.linear() = rotation(t.rotation).toRotationMatrix();
        transform_.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
        if (!transform_.matrix().allFinite()) {throw std::invalid_argument("nonfinite static transform");}
      } catch (const tf2::TransformException &) {return false;}
    }
    cached_source_ = source;
    static_sub_.reset(); buffer_.reset();
    return true;
  }
  bool process(const PendingCloud & entry)
  {
    const auto & input = *entry.message;
    const auto & layout = entry.layout;
    if (!joint_mode_ && !cache_transform(input.header.frame_id)) {return false;}
    // JointHistory starts a new contiguous history whenever the sample gap is too large.
    if (joint_mode_ && entry.time_range &&
      (!history_.interpolate(entry.time_range->first, false) ||
      !history_.interpolate(entry.time_range->second, false)))
    {
      return false;
    }

    const std::size_t point_count = static_cast<std::size_t>(input.width) * input.height;
    const bool filter = !hulls_.empty() || !wheels_.empty();
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (filter) {
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
    output->header.frame_id = target_;

    std::optional<double> previous_point_time;
    Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
    for (std::uint32_t row = 0; row < input.height; ++row) {
      for (std::uint32_t col = 0; col < input.width; ++col) {
        const auto offset = static_cast<std::size_t>(row) * input.row_step + col * input.point_step;
        const auto * data = input.data.data() + offset;
        if (joint_mode_) {
          const double ns = read_scalar<double>(data + layout.timestamp, input.is_bigendian);
          const double time = std::round(ns) * 1.e-9;
          if (previous_point_time != time) {
            const auto joint = history_.interpolate(time, false);
            if (!joint) {return false;}
            const Eigen::Matrix3d rotation =
              Eigen::AngleAxisd(joint->position + position_offset_, axis_).toRotationMatrix();
            joint_transform.linear() = rotation * sensor_rotation_;
            joint_transform.translation() = rotation * offset_;
            previous_point_time = time;
          }
        }

        Eigen::Vector3d point;
        for (int axis = 0; axis < 3; ++axis) {point[axis] = read_scalar<float>(data + layout.xyz[axis], input.is_bigendian);}
        point = (joint_mode_ ? joint_transform : transform_) * point;

        std::size_t output_offset = offset;
        if (filter) {
          if (inside_robot(point)) {continue;}
          output_offset = output->data.size();
          output->data.insert(output->data.end(), data, data + input.point_step);
        }
        auto * record = output->data.data() + output_offset;
        for (int axis = 0; axis < 3; ++axis) {
          write_scalar<float>(record + layout.xyz[axis], static_cast<float>(point[axis]), input.is_bigendian);
        }
      }
    }
    if (filter) {
      output->width = output->data.size() / output->point_step;
      output->row_step = output->width * output->point_step;
    }
    output_->publish(std::move(output));
    return true;
  }
  void process_pending()
  {
    while (!pending_.empty()) {
      try {
        if (!process(pending_.front()) &&
          steady_seconds() - pending_.front().received <= pending_timeout_)
        {
          if (timer_->is_canceled()) {timer_->reset();}
          return;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), log_clock_, 2000, "Dropping cloud: %s", error.what());
      }
      pending_.pop_front();
    }
    if (!timer_->is_canceled()) {timer_->cancel();}
  }
  std::string target_, expected_;
  rclcpp::Clock log_clock_{RCL_STEADY_TIME};
  bool timed_, joint_mode_;
  int queue_limit_;
  double position_offset_, pending_timeout_, tolerance_ = 0.;
  JointHistory history_;
  Eigen::Vector3d axis_, offset_;
  Eigen::Matrix3d sensor_rotation_;
  Eigen::Isometry3d transform_ = Eigen::Isometry3d::Identity();
  std::optional<std::string> cached_source_;
  std::vector<std::vector<Eigen::Vector4d>> hulls_;
  std::vector<Wheel> wheels_;
  std::deque<PendingCloud> pending_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace xxu_bringup
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::PointCloudFrameRepublisher)
