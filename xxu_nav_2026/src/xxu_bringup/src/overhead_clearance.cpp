#include "xxu_bringup/overhead_clearance.hpp"

#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/localization_health.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace xxu_bringup
{
namespace
{

void check_finite_non_negative(double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

void check_finite_positive(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
}

std::string format_optional_metric(double value)
{
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

}  // namespace

void validate_overhead_clearance_config(const OverheadClearanceConfig & config)
{
  check_finite_non_negative(config.base_height_m, "base_height_m");
  check_finite_non_negative(config.min_overhead_height_m, "min_overhead_height_m");
  if (!std::isfinite(config.max_overhead_height_m) ||
      config.max_overhead_height_m <= config.min_overhead_height_m) {
    throw std::invalid_argument(
      "max_overhead_height_m must be finite and greater than min_overhead_height_m");
  }
  check_finite_non_negative(config.min_xy_radius_m, "min_xy_radius_m");
  if (!std::isfinite(config.detection_radius_m) ||
      config.detection_radius_m <= config.min_xy_radius_m) {
    throw std::invalid_argument(
      "detection_radius_m must be finite and greater than min_xy_radius_m");
  }
  if (config.min_points < 1) {
    throw std::invalid_argument("min_points must be at least 1");
  }
  if (config.trigger_frames < 1) {
    throw std::invalid_argument("trigger_frames must be at least 1");
  }
  check_finite_non_negative(config.release_hold_sec, "release_hold_sec");
  check_finite_non_negative(config.release_travel_distance_m, "release_travel_distance_m");
  check_finite_positive(config.max_odom_step_m, "max_odom_step_m");
  check_finite_positive(config.cloud_timeout_sec, "cloud_timeout_sec");
}

std::string format_overhead_clearance_status(const OverheadClearanceStatus & status)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(3);
  out << "stow_gimbal=" << (status.stow_gimbal ? "true" : "false")
      << ";overhead_detected_now=" << (status.overhead_detected_now ? "true" : "false")
      << ";cloud_stale=" << (status.cloud_stale ? "true" : "false")
      << ";hit_point_count=" << status.hit_point_count
      << ";consecutive_hit_frames=" << status.consecutive_hit_frames
      << ";min_hit_xy_distance_m=" << format_optional_metric(status.min_hit_xy_distance_m)
      << ";min_hit_height_m=" << format_optional_metric(status.min_hit_height_m)
      << ";clear_elapsed_sec=" << status.clear_elapsed_sec
      << ";clear_travel_m=" << status.clear_travel_m << ";reason=" << status.reason;
  return out.str();
}

OverheadClearanceDetector::OverheadClearanceDetector(const OverheadClearanceConfig & config)
    : config_(config)
{
  validate_overhead_clearance_config(config_);
}

void OverheadClearanceDetector::observe_odometry(const nav_msgs::msg::Odometry & odom)
{
  const auto & position = odom.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
    return;
  }

  const std::int64_t current_stamp_ns = stamp_ns(odom.header.stamp);
  if (current_stamp_ns < 0) {
    return;
  }

  const Eigen::Vector2d current_xy(position.x, position.y);
  if (last_odom_stamp_ns_.has_value() && current_stamp_ns > 0 && *last_odom_stamp_ns_ > 0 &&
      current_stamp_ns < *last_odom_stamp_ns_) {
    last_odom_xy_ = current_xy;
    last_odom_stamp_ns_ = current_stamp_ns;
    return;
  }
  if (current_stamp_ns > 0) {
    last_odom_stamp_ns_ = current_stamp_ns;
  }

  if (!last_odom_xy_.has_value()) {
    last_odom_xy_ = current_xy;
    return;
  }

  const double step_m = (current_xy - *last_odom_xy_).norm();
  last_odom_xy_ = current_xy;
  if (step_m > config_.max_odom_step_m) {
    return;
  }
  if (status_.stow_gimbal) {
    status_.clear_travel_m += step_m;
  }
}

OverheadClearanceStatus OverheadClearanceDetector::evaluate_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const Eigen::Isometry3d & base_from_sensor,
  double steady_now_sec)
{
  if (!std::isfinite(steady_now_sec)) {
    throw std::invalid_argument("steady_now_sec must be finite");
  }
  if (!base_from_sensor.matrix().allFinite()) {
    throw std::invalid_argument("base_from_sensor must be finite");
  }

  const CloudLayout layout(cloud, false);
  std::size_t hit_count = 0;
  double min_hit_xy = std::numeric_limits<double>::infinity();
  double min_hit_height = std::numeric_limits<double>::infinity();

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t point_offset =
        row_offset + static_cast<std::size_t>(col) * cloud.point_step;
      const std::uint8_t * ptr = cloud.data.data() + point_offset;
      const float x = read_scalar<float>(ptr + layout.xyz[0], cloud.is_bigendian);
      const float y = read_scalar<float>(ptr + layout.xyz[1], cloud.is_bigendian);
      const float z = read_scalar<float>(ptr + layout.xyz[2], cloud.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const Eigen::Vector3d point_base =
        base_from_sensor *
        Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
      const double z_ground = point_base.z() + config_.base_height_m;
      const double xy_distance = std::hypot(point_base.x(), point_base.y());
      if (z_ground < config_.min_overhead_height_m || z_ground > config_.max_overhead_height_m) {
        continue;
      }
      if (xy_distance < config_.min_xy_radius_m || xy_distance > config_.detection_radius_m) {
        continue;
      }

      ++hit_count;
      min_hit_xy = std::min(min_hit_xy, xy_distance);
      min_hit_height = std::min(min_hit_height, z_ground);
    }
  }

  last_cloud_steady_sec_ = steady_now_sec;
  status_.cloud_stale = false;
  status_.hit_point_count = hit_count;
  status_.min_hit_xy_distance_m =
    hit_count > 0 ? min_hit_xy : std::numeric_limits<double>::quiet_NaN();
  status_.min_hit_height_m =
    hit_count > 0 ? min_hit_height : std::numeric_limits<double>::quiet_NaN();

  const bool overhead_hit_now = hit_count >= static_cast<std::size_t>(config_.min_points);
  status_.overhead_detected_now = overhead_hit_now;

  if (overhead_hit_now) {
    status_.consecutive_hit_frames += 1;
    clear_start_sec_.reset();
    status_.clear_elapsed_sec = 0.0;
    status_.clear_travel_m = 0.0;
    if (!status_.stow_gimbal && status_.consecutive_hit_frames >= config_.trigger_frames) {
      status_.stow_gimbal = true;
    }
    status_.reason = status_.stow_gimbal ? "overhead_detected_stowed" : "overhead_debouncing";
    return status_;
  }

  status_.consecutive_hit_frames = 0;
  if (!status_.stow_gimbal) {
    clear_start_sec_.reset();
    status_.clear_elapsed_sec = 0.0;
    status_.clear_travel_m = 0.0;
    status_.reason = "clear";
    return status_;
  }

  if (!clear_start_sec_.has_value() || steady_now_sec < *clear_start_sec_) {
    clear_start_sec_ = steady_now_sec;
  }
  status_.clear_elapsed_sec = std::max(0.0, steady_now_sec - *clear_start_sec_);

  const bool hold_satisfied = status_.clear_elapsed_sec >= config_.release_hold_sec;
  const bool travel_satisfied = config_.release_travel_distance_m == 0.0 ||
                                status_.clear_travel_m >= config_.release_travel_distance_m;

  if (hold_satisfied && travel_satisfied) {
    status_.stow_gimbal = false;
    clear_start_sec_.reset();
    status_.clear_elapsed_sec = 0.0;
    status_.clear_travel_m = 0.0;
    status_.reason = "clear_released";
  } else if (!hold_satisfied && !travel_satisfied) {
    status_.reason = "latched_waiting_hold_and_travel";
  } else if (!hold_satisfied) {
    status_.reason = "latched_waiting_hold";
  } else {
    status_.reason = "latched_waiting_travel";
  }

  return status_;
}

OverheadClearanceStatus OverheadClearanceDetector::check_staleness(double steady_now_sec)
{
  if (!std::isfinite(steady_now_sec)) {
    throw std::invalid_argument("steady_now_sec must be finite");
  }
  if (!last_cloud_steady_sec_.has_value()) {
    status_.cloud_stale = true;
    status_.reason = "waiting_for_cloud";
    return status_;
  }

  const double elapsed_sec = steady_now_sec - *last_cloud_steady_sec_;
  if (elapsed_sec > config_.cloud_timeout_sec) {
    status_.cloud_stale = true;
    status_.overhead_detected_now = false;
    status_.consecutive_hit_frames = 0;
    // Do not count sensor blackout silence toward release hold, and never release while stale.
    clear_start_sec_.reset();
    status_.clear_elapsed_sec = 0.0;
    status_.reason = status_.stow_gimbal ? "cloud_stale_latched" : "cloud_stale";
  }
  return status_;
}

const OverheadClearanceStatus & OverheadClearanceDetector::status() const
{
  return status_;
}

const OverheadClearanceConfig & OverheadClearanceDetector::config() const
{
  return config_;
}

}  // namespace xxu_bringup
