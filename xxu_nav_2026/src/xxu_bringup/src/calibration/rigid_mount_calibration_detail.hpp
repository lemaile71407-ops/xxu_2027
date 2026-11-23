#pragma once

#include "xxu_bringup/rigid_mount_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xxu_bringup::calibration_detail {

inline constexpr auto odom_tilt_reason =
  "odometry orientation disagrees with stationary gravity leveling (check raw vs compensated odometry)";

inline double degrees(double radians) {return radians * 180.0 / calibration_pi;}
inline double radians(double degrees) {return degrees * (calibration_pi / 180.0);}
inline double unit_angle(double cosine) {return std::acos(std::clamp(cosine, -1.0, 1.0));}
inline double wrap(double angle)
{
  const double value = std::remainder(angle, 2.0 * calibration_pi);
  return value <= -calibration_pi ? calibration_pi : value;
}
inline Eigen::Matrix3d yaw_rotation(double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}
inline bool valid_time(double time) {return std::isfinite(time) && time >= 0.0;}
inline bool valid_imu(const ImuSample & imu)
{
  return valid_time(imu.time) && imu.acceleration.allFinite() && imu.gyro.allFinite() &&
         std::isfinite(imu.acceleration.norm()) && std::isfinite(imu.gyro.norm());
}
inline bool valid_odom(const OdomSample & odom)
{
  const double norm = odom.orientation.norm();
  return valid_time(odom.time) && odom.position.allFinite() &&
         std::isfinite(norm) && norm > 1.e-12;
}
inline Eigen::Matrix3d odom_rotation(const OdomSample & odom)
{
  return odom.orientation.normalized().toRotationMatrix();
}
template<typename Sample, typename Predicate>
std::vector<Sample> usable(const std::vector<Sample> & input, Predicate valid)
{
  std::vector<Sample> result;
  result.reserve(input.size());
  for (const auto & sample : input) {
    if (valid(sample) && (result.empty() || sample.time > result.back().time)) {
      result.push_back(sample);
    }
  }
  return result;
}
template<typename Sample>
double max_sample_gap(const std::vector<Sample> & samples)
{
  double max_gap = 0.0;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    max_gap = std::max(max_gap, samples[i].time - samples[i - 1].time);
  }
  return max_gap;
}
inline bool valid_frame(const std::string & name)
{
  return !name.empty() && name.front() != '/' &&
         name.find_first_of(" \t\n\r\v\f") == std::string::npos;
}
inline void validate_mount_frame(const std::string & name)
{
  if (!valid_frame(name)) {throw std::invalid_argument("invalid TF frame: " + name);}
  if (name == "map" || name == "odom" || name == "base_footprint") {
    throw std::invalid_argument("reserved navigation TF frame: " + name);
  }
}
inline bool valid_topic(const std::string & name)
{
  return name.size() > 1 && name.front() == '/' &&
         name.find_first_of(" \t\n\r\v\f") == std::string::npos;
}

StationaryResult stationary_imu(
  const std::vector<ImuSample> & imu, const CalibrationConfig & config,
  std::optional<TimeWindow> * imu_interval = nullptr);
void fit_floor(
  StationaryResult & result, const std::vector<CloudSample> & clouds,
  const Eigen::Isometry3d & frame_from_lidar, const Eigen::Vector3d & imu_origin,
  const CalibrationConfig & config, const std::optional<TimeWindow> & imu_interval);
SpinResult evaluate_spin(
  const std::vector<OdomSample> & samples, const Eigen::Matrix3d & base_from_imu,
  double tz, const CalibrationConfig & config, const std::optional<Eigen::Vector2d> & fixed_xy);

inline constexpr std::pair<const char *, double CalibrationConfig::*> real_thresholds[] = {
  {"min_stationary_duration", &CalibrationConfig::min_stationary_duration},
  {"max_accel_norm_std", &CalibrationConfig::max_accel_norm_std},
  {"max_accel_deviation", &CalibrationConfig::max_accel_deviation},
  {"max_gyro_norm_std", &CalibrationConfig::max_gyro_norm_std},
  {"max_gyro_deviation", &CalibrationConfig::max_gyro_deviation},
  {"max_gyro_mean", &CalibrationConfig::max_gyro_mean},
  {"min_ground_radius", &CalibrationConfig::min_ground_radius},
  {"max_ground_radius", &CalibrationConfig::max_ground_radius},
  {"min_ground_z", &CalibrationConfig::min_ground_z},
  {"max_ground_z", &CalibrationConfig::max_ground_z},
  {"ground_inlier_distance", &CalibrationConfig::ground_inlier_distance},
  {"max_ground_rms", &CalibrationConfig::max_ground_rms},
  {"max_gravity_ground_angle_deg", &CalibrationConfig::max_gravity_ground_angle_deg},
  {"min_straight_distance", &CalibrationConfig::min_straight_distance},
  {"max_straight_yaw_change_deg", &CalibrationConfig::max_straight_yaw_change_deg},
  {"max_straight_lateral_rms", &CalibrationConfig::max_straight_lateral_rms},
  {"max_straight_vertical_ratio", &CalibrationConfig::max_straight_vertical_ratio},
  {"max_straight_pca_angle_deg", &CalibrationConfig::max_straight_pca_angle_deg},
  {"max_odom_tilt_deg", &CalibrationConfig::max_odom_tilt_deg},
  {"min_spin_yaw_span_deg", &CalibrationConfig::min_spin_yaw_span_deg},
  {"max_spin_residual_rms", &CalibrationConfig::max_spin_residual_rms},
  {"max_spin_tilt_span_deg", &CalibrationConfig::max_spin_tilt_span_deg},
  {"max_spin_condition", &CalibrationConfig::max_spin_condition},
  {"max_verify_gravity_error", &CalibrationConfig::max_verify_gravity_error},
  {"max_verify_floor_height_error", &CalibrationConfig::max_verify_floor_height_error},
  {"max_verify_straight_yaw_deg", &CalibrationConfig::max_verify_straight_yaw_deg},
  {"auto_stationary_speed", &CalibrationConfig::auto_stationary_speed},
  {"auto_straight_min_speed", &CalibrationConfig::auto_straight_min_speed},
  {"auto_straight_max_yaw_rate", &CalibrationConfig::auto_straight_max_yaw_rate},
  {"auto_spin_min_yaw_rate", &CalibrationConfig::auto_spin_min_yaw_rate},
  {"auto_spin_max_translation_per_rad", &CalibrationConfig::auto_spin_max_translation_per_rad},
  {"auto_max_sample_gap", &CalibrationConfig::auto_max_sample_gap},
};
inline constexpr std::pair<const char *, int CalibrationConfig::*> count_thresholds[] = {
  {"min_imu_samples", &CalibrationConfig::min_imu_samples},
  {"min_ground_clouds", &CalibrationConfig::min_ground_clouds},
  {"min_ground_inliers", &CalibrationConfig::min_ground_inliers},
  {"min_straight_samples", &CalibrationConfig::min_straight_samples},
  {"min_spin_samples", &CalibrationConfig::min_spin_samples},
};

}  // namespace xxu_bringup::calibration_detail
