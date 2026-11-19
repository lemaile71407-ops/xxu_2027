/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace pointcloud_to_laserscan
{
namespace
{
struct PointFields
{
  uint32_t x_offset{0};
  uint32_t y_offset{0};
  uint32_t z_offset{0};
};

bool findXYZFields(const sensor_msgs::msg::PointCloud2 & msg, PointFields & fields)
{
  if (msg.point_step < 4 || static_cast<std::uint64_t>(msg.row_step) <
    static_cast<std::uint64_t>(msg.width) * msg.point_step ||
    static_cast<std::uint64_t>(msg.row_step) * msg.height != msg.data.size()) {return false;}
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  for (const auto & field : msg.fields) {
    const bool valid_float =
      field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1 &&
      field.offset <= msg.point_step - sizeof(float);

    if (field.name == "x" && valid_float) {
      if (has_x) {return false;}
      fields.x_offset = field.offset;
      has_x = true;
    } else if (field.name == "y" && valid_float) {
      if (has_y) {return false;}
      fields.y_offset = field.offset;
      has_y = true;
    } else if (field.name == "z" && valid_float) {
      if (has_z) {return false;}
      fields.z_offset = field.offset;
      has_z = true;
    }
  }

  return has_x && has_y && has_z;
}

bool isHostBigendian()
{
  const uint16_t value = 0x0102;
  return *reinterpret_cast<const uint8_t *>(&value) == 0x01;
}

float readFloat32(const uint8_t * data, bool swap_bytes)
{
  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, data, sizeof(float));
  if (swap_bytes) {
    std::reverse(bytes, bytes + sizeof(float));
  }
  float value;
  std::memcpy(&value, bytes, sizeof(float));
  return value;
}

void writeFloat32(uint8_t * data, bool swap_bytes, float value)
{
  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, &value, sizeof(float));
  if (swap_bytes) {
    std::reverse(bytes, bytes + sizeof(float));
  }
  std::memcpy(data, bytes, sizeof(float));
}
}  // namespace

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  robot_self_filter_radius_ = this->declare_parameter("robot_self_filter_radius", 0.0);
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  publish_processed_cloud_ = this->declare_parameter("publish_processed_cloud", false);
  processed_cloud_frame_ = this->declare_parameter("processed_cloud_frame", target_frame_);
  if (publish_processed_cloud_ && !processed_cloud_frame_.empty() && processed_cloud_frame_ != target_frame_) {
    throw std::invalid_argument("processed cloud must use the cached static target_frame");
  }
  processed_cloud_project_to_2d_ =
    this->declare_parameter("processed_cloud_project_to_2d", true);
  processed_cloud_z_value_ = this->declare_parameter("processed_cloud_z_value", 0.0);
  crop_min_x_ = this->declare_parameter("crop_min_x", -std::numeric_limits<double>::max());
  crop_max_x_ = this->declare_parameter("crop_max_x", std::numeric_limits<double>::max());
  crop_min_y_ = this->declare_parameter("crop_min_y", -std::numeric_limits<double>::max());
  crop_max_y_ = this->declare_parameter("crop_max_y", std::numeric_limits<double>::max());
  const auto scan_qos_reliability =
    this->declare_parameter<std::string>("scan_qos_reliability", "best_effort");

  if (scan_qos_reliability != "best_effort") {
    throw std::invalid_argument("navigation scan requires best_effort SensorDataQoS");
  }
  auto scan_qos = rclcpp::SensorDataQoS();

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", scan_qos);
  RCLCPP_INFO(
    this->get_logger(), "Publishing LaserScan with %s QoS",
    scan_qos_reliability == "best_effort" ? "best_effort" : "reliable");

  if (publish_processed_cloud_) {
    processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "processed_cloud", rclcpp::SensorDataQoS());
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing processed cloud frame=%s, crop_x=[%.3f, %.3f], crop_y=[%.3f, %.3f], "
      "height=[%.3f, %.3f], range=[%.3f, %.3f], self_filter_radius=%.3f, project_to_2d=%s",
      processed_cloud_frame_.c_str(), crop_min_x_, crop_max_x_, crop_min_y_, crop_max_y_,
      min_height_, max_height_, range_min_, range_max_, robot_self_filter_radius_,
      processed_cloud_project_to_2d_ ? "true" : "false");
  }

  using std::placeholders::_1;
  // Navigation receives a deskewed cloud in one rigid sensor frame. Collect
  // static edges only at startup; never listen to the global dynamic TF stream.
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    static_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        if (!tf2_) {return;}
        for (const auto & transform : message->transforms) {
          tf2_->setTransform(transform, "static_mount", true);
        }
      });
  }
  sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  subscription_listener_thread_.join();
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count();
    if (processed_cloud_pub_) {
      subscription_count += processed_cloud_pub_->get_subscription_count() +
        processed_cloud_pub_->get_intra_process_subscription_count();
    }
    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else if (sub_.getSubscriber()) {
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe();
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  PointFields fields;
  if (!findXYZFields(*cloud_msg, fields)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), steady_clock_, 2000, "Invalid PointCloud2 layout or FLOAT32 x/y/z fields");
    return;
  }
  if (cached_source_frame_.empty() && cloud_msg->header.frame_id == target_frame_) {
    cached_source_frame_ = target_frame_; static_subscription_.reset(); tf2_.reset();
  }
  if (!cached_source_frame_.empty() && cached_source_frame_ != cloud_msg->header.frame_id) {return;}
  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  // The input cloud is already a single-time-slice cloud. Every point has
  // been deskewed to the cloud header stamp upstream, so LaserScan rays do
  // not represent a rolling acquisition.
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  const bool transform_cloud = scan_msg->header.frame_id != cloud_msg->header.frame_id;
  if (transform_cloud) {
    try {
      if (cached_source_frame_.empty()) {
        const auto transform = tf2_->lookupTransform(
          target_frame_, cloud_msg->header.frame_id, tf2::TimePointZero).transform;
        Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x,
          transform.rotation.y, transform.rotation.z);
        if (!q.coeffs().allFinite() || q.norm() < 1.e-12) {return;}
        target_from_source_.linear() = q.normalized().toRotationMatrix();
        target_from_source_.translation() = Eigen::Vector3d(transform.translation.x,
          transform.translation.y, transform.translation.z);
        if (!target_from_source_.matrix().allFinite()) {return;}
        cached_source_frame_ = cloud_msg->header.frame_id;
        static_subscription_.reset();
        tf2_.reset();
      }
      if (cached_source_frame_ != cloud_msg->header.frame_id) {return;}
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  sensor_msgs::msg::PointCloud2::UniquePtr processed_cloud;
  if (processed_cloud_pub_ && (processed_cloud_pub_->get_subscription_count() > 0 ||
    processed_cloud_pub_->get_intra_process_subscription_count() > 0))
  {
    processed_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    processed_cloud->header = scan_msg->header;
    if (!processed_cloud_frame_.empty()) {
      processed_cloud->header.frame_id = processed_cloud_frame_;
    }
    processed_cloud->height = 1;
    processed_cloud->fields = cloud_msg->fields;
    processed_cloud->is_bigendian = cloud_msg->is_bigendian;
    processed_cloud->point_step = cloud_msg->point_step;
    processed_cloud->is_dense = false;
    processed_cloud->data.reserve(
      static_cast<std::size_t>(cloud_msg->height) * cloud_msg->width * cloud_msg->point_step);
  }

  // Respect row padding and byte order; never interpret padding as points.
  const bool swap_bytes = isHostBigendian() != cloud_msg->is_bigendian;
  for (std::uint32_t row = 0; row < cloud_msg->height; ++row) {
    for (std::uint32_t col = 0; col < cloud_msg->width; ++col) {
      const auto * point = cloud_msg->data.data() +
        static_cast<std::size_t>(row) * cloud_msg->row_step + col * cloud_msg->point_step;
      float x = readFloat32(point + fields.x_offset, swap_bytes);
      float y = readFloat32(point + fields.y_offset, swap_bytes);
      float z = readFloat32(point + fields.z_offset, swap_bytes);
      if (transform_cloud) {
        const Eigen::Vector3d transformed = target_from_source_ * Eigen::Vector3d(x, y, z);
        // Retain the FLOAT32 rounding of a transformed PointCloud2 record.
        x = static_cast<float>(transformed.x());
        y = static_cast<float>(transformed.y());
        z = static_cast<float>(transformed.z());
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z > max_height_ || z < min_height_) {continue;}
      const double range = std::hypot(x, y);
      if (range < robot_self_filter_radius_ || range < range_min_ || range > range_max_) {continue;}
      if (processed_cloud && !(x < crop_min_x_ || x > crop_max_x_ ||
        y < crop_min_y_ || y > crop_max_y_))
      {
        auto & data = processed_cloud->data;
        const std::size_t start = data.size();
        data.insert(data.end(), point, point + cloud_msg->point_step);
        auto * record = data.data() + start;
        writeFloat32(record + fields.x_offset, swap_bytes, x);
        writeFloat32(record + fields.y_offset, swap_bytes, y);
        writeFloat32(
          record + fields.z_offset, swap_bytes,
          processed_cloud_project_to_2d_ ? static_cast<float>(processed_cloud_z_value_) : z);
      }
      const double angle = std::atan2(y, x);
      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {continue;}
      const int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
      if (index < 0 || static_cast<std::size_t>(index) >= scan_msg->ranges.size()) {continue;}
      if (range < scan_msg->ranges[index]) {scan_msg->ranges[index] = range;}
    }
  }

  if (processed_cloud) {
    processed_cloud->width = static_cast<uint32_t>(
      processed_cloud->data.size() / processed_cloud->point_step);
    processed_cloud->row_step = static_cast<uint32_t>(processed_cloud->data.size());
    processed_cloud_pub_->publish(std::move(processed_cloud));
  }
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
