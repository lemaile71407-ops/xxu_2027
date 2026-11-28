#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace xxu_bringup
{

struct OverheadClearanceConfig {
  double base_height_m = 0.0;
  double min_overhead_height_m = 0.22;
  double max_overhead_height_m = 0.55;
  double min_xy_radius_m = 0.15;
  double detection_radius_m = 1.50;
  int min_points = 8;
  int trigger_frames = 2;
  double release_hold_sec = 1.5;
  double release_travel_distance_m = 0.80;
  double max_odom_step_m = 1.0;
  double cloud_timeout_sec = 0.5;
};

struct OverheadClearanceStatus {
  bool stow_gimbal = false;
  bool overhead_detected_now = false;
  bool cloud_stale = true;
  std::size_t hit_point_count = 0;
  int consecutive_hit_frames = 0;
  double min_hit_xy_distance_m = std::numeric_limits<double>::quiet_NaN();
  double min_hit_height_m = std::numeric_limits<double>::quiet_NaN();
  double clear_elapsed_sec = 0.0;
  double clear_travel_m = 0.0;
  std::string reason = "waiting_for_cloud";
};

void validate_overhead_clearance_config(const OverheadClearanceConfig & config);

std::string format_overhead_clearance_status(const OverheadClearanceStatus & status);

// Pure C++ overhead obstacle detector with time + travel-distance hysteresis
// latching so a tunnel roof entering the near-range lidar blind zone does not
// immediately release gimbal stow.
class OverheadClearanceDetector
{
public:
  explicit OverheadClearanceDetector(const OverheadClearanceConfig & config);

  void observe_odometry(const nav_msgs::msg::Odometry & odom);

  OverheadClearanceStatus evaluate_cloud(const sensor_msgs::msg::PointCloud2 & cloud,
                                         const Eigen::Isometry3d & base_from_sensor,
                                         double steady_now_sec);

  OverheadClearanceStatus check_staleness(double steady_now_sec);

  const OverheadClearanceStatus & status() const;

  const OverheadClearanceConfig & config() const;

private:
  OverheadClearanceConfig config_;
  OverheadClearanceStatus status_;
  std::optional<double> last_cloud_steady_sec_;
  std::optional<double> clear_start_sec_;
  std::optional<Eigen::Vector2d> last_odom_xy_;
  std::optional<std::int64_t> last_odom_stamp_ns_;
};

}  // namespace xxu_bringup
