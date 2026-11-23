#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace xxu_bringup {

inline constexpr double calibration_pi = 3.14159265358979323846;
inline constexpr double unmeasured = std::numeric_limits<double>::quiet_NaN();

struct ImuSample
{
  double time;
  Eigen::Vector3d acceleration, gyro;
};

struct OdomSample
{
  double time;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

using CalibrationPoints = std::vector<Eigen::Vector3d>;
struct CloudSample
{
  double time;
  CalibrationPoints points;
};

struct CalibrationConfig
{
  double imu_acceleration_scale = 0.0;
  double base_height_above_ground = 0.0;
  Eigen::Isometry3d imu_from_lidar = [] {
      auto pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
      return pose;
    }();
  int straight_direction = 1;
  std::string sensor_frame_mode = "base_aligned";
  std::string base_frame = "base_link", sensor_frame = "lio_base_sensor";
  std::string lidar_frame = "livox_frame", imu_source_frame = "livox_frame";
  std::string odom_frame = "odom";
  std::string odom_child_frame = "imu";
  std::string imu_topic = "/livox/imu", lidar_topic = "/livox/lidar";
  std::string odom_topic = "/calibration/lidar_odometry";
  bool allow_unisolated_odom_topic = false;
  // Offset about the leveled IMU's +Z, before applying the leveled rotation.
  std::optional<double> manual_psi_deg, manual_tx, manual_ty, manual_tz;
  bool confirm = false;

  int min_imu_samples = 100, min_ground_clouds = 3, min_ground_inliers = 200;
  double min_stationary_duration = 1.5;
  double max_accel_norm_std = 0.35, max_accel_deviation = 0.9;
  double max_gyro_norm_std = 0.025, max_gyro_deviation = 0.08, max_gyro_mean = 0.10;
  double min_ground_radius = 0.4, max_ground_radius = 5.0;
  double min_ground_z = -2.5, max_ground_z = -0.05;
  double ground_inlier_distance = 0.035, max_ground_rms = 0.03;
  double max_gravity_ground_angle_deg = 2.0;
  int min_straight_samples = 20;
  double min_straight_distance = 1.5, max_straight_yaw_change_deg = 10.0;
  double max_straight_lateral_rms = 0.06, max_straight_vertical_ratio = 0.10;
  double max_straight_pca_angle_deg = 8.0;
  double max_odom_tilt_deg = 5.0;
  int min_spin_samples = 60;
  double min_spin_yaw_span_deg = 180.0, max_spin_residual_rms = 0.025;
  double max_spin_tilt_span_deg = 5.0, max_spin_condition = 1.e4;
  double max_verify_gravity_error = 0.35;
  double max_verify_floor_height_error = 0.05, max_verify_straight_yaw_deg = 2.0;
  double auto_stationary_speed = 0.06, auto_straight_min_speed = 0.12;
  double auto_straight_max_yaw_rate = 0.12, auto_spin_min_yaw_rate = 0.25;
  double auto_spin_max_translation_per_rad = 0.60;
  double auto_max_sample_gap = 0.5;
};

void validate_calibration_config(const CalibrationConfig & config);

struct StageQuality
{
  bool attempted = false;
  std::vector<std::string> reasons;
  bool passed() const {return attempted && reasons.empty();}
};

struct StationaryResult : StageQuality
{
  bool floor_attempted = false;
  std::size_t imu_count = 0, cloud_count = 0, roi_count = 0, inlier_count = 0;
  double duration = unmeasured, accel_norm = unmeasured;
  double max_imu_gap = unmeasured;
  double accel_norm_std = unmeasured, max_accel_deviation = unmeasured;
  double gyro_norm_std = unmeasured, max_gyro_deviation = unmeasured;
  Eigen::Vector3d accel_mean = Eigen::Vector3d::Constant(unmeasured);
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Constant(unmeasured);
  Eigen::Vector3d accel_variance = Eigen::Vector3d::Constant(unmeasured);
  Eigen::Vector3d gyro_variance = Eigen::Vector3d::Constant(unmeasured);
  std::optional<double> scale, tz;
  std::optional<Eigen::Matrix3d> leveling;
  Eigen::Vector3d floor_normal = Eigen::Vector3d::Constant(unmeasured);
  double floor_offset = unmeasured, floor_rms = unmeasured;
  double gravity_ground_angle_deg = unmeasured;
};

struct StraightResult : StageQuality
{
  std::size_t sample_count = 0;
  double max_sample_gap = unmeasured;
  std::optional<double> psi;
  double distance = unmeasured, yaw_span_deg = unmeasured, lateral_rms = unmeasured;
  double vertical_ratio = unmeasured, pca_angle_deg = unmeasured;
  double yaw_error_deg = unmeasured;
};

struct SpinResult : StageQuality
{
  std::size_t sample_count = 0;
  double max_sample_gap = unmeasured;
  std::optional<double> tx, ty;
  Eigen::Vector2d center = Eigen::Vector2d::Constant(unmeasured);
  double yaw_span_deg = unmeasured, roll_span_deg = unmeasured, pitch_span_deg = unmeasured;
  double condition = unmeasured, std_tx = unmeasured, std_ty = unmeasured;
  int rank = 0;
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Constant(unmeasured);
  double raw_radius_rms = unmeasured, residual_rms = unmeasured;
};

struct TimeWindow
{
  double start, end;  // Seconds in the recording's shared timeline, inclusive.
  bool contains(double time) const {return time >= start && time <= end;}
};

struct Segmentation
{
  std::optional<TimeWindow> stationary, straight, spin;
  std::vector<std::string> reasons;
};

struct CalibrationSamples
{
  bool stationary_requested = true, straight_requested = false, spin_requested = false;
  std::vector<ImuSample> imu;
  std::vector<CloudSample> clouds;
  std::vector<OdomSample> straight, spin;
  std::set<std::string> imu_frames, lidar_frames, odom_frames, odom_child_frames;
  std::size_t rejected_imu = 0, rejected_clouds = 0, rejected_odom = 0;
  std::vector<std::string> input_errors;
};

struct CalibrationResult
{
  StationaryResult stationary;
  StraightResult straight;
  SpinResult spin;
  std::optional<double> scale, psi, tx, ty, tz;
  std::optional<Eigen::Matrix3d> base_from_imu;
  bool confirmed = false;
  // Each row is [psi change (rad), lever change (m), straight RMS (m), spin RMS (m)].
  std::vector<std::array<double, 4>> refinement;
  StageQuality refinement_quality;
  std::vector<std::string> reasons;
};

struct RigidMount
{
  Eigen::Isometry3d base_from_sensor, sensor_from_lidar;
  Eigen::Matrix3d sensor_from_imu;
  double imu_acceleration_scale;
  std::string base_frame, sensor_frame, lidar_frame, imu_source_frame;
  std::array<std::string, 4> topics;  // points in/out, IMU in/out
};

struct VerificationResult
{
  StationaryResult stationary;
  StraightResult straight;
  SpinResult spin;
  double gravity_angle_deg = unmeasured, gravity_error = unmeasured;
  double floor_height_error = unmeasured;
  std::vector<std::string> reasons;
  bool passed() const {return reasons.empty();}
};

Eigen::Vector3d rotation_to_rpy(const Eigen::Matrix3d & rotation);
Eigen::Matrix3d leveling_rotation_from_up(const Eigen::Vector3d & up);
std::optional<double> detect_imu_acceleration_scale(double accel_norm);

StationaryResult solve_stationary(
  const std::vector<ImuSample> & imu, const std::vector<CloudSample> & clouds,
  const CalibrationConfig & config);
StraightResult solve_straight(
  const std::vector<OdomSample> & samples, const Eigen::Matrix3d & level_from_imu,
  const CalibrationConfig & config, const std::optional<Eigen::Isometry3d> & base_from_imu = {});
SpinResult solve_spin(
  const std::vector<OdomSample> & samples, const Eigen::Matrix3d & base_from_imu,
  double tz, const CalibrationConfig & config);
Segmentation auto_segment_recording(
  const std::vector<ImuSample> & imu, const std::vector<OdomSample> & odom,
  const std::vector<double> & cloud_times, const CalibrationConfig & config);
CalibrationResult solve_all(const CalibrationSamples & samples, const CalibrationConfig & config);
VerificationResult verify_calibration(
  const RigidMount & mount, const CalibrationSamples & samples, const CalibrationConfig & config);

RigidMount load_rigid_mount_cpp(const std::string & path);
std::string render_rigid_mount_yaml(const CalibrationResult & result, const CalibrationConfig & config);
std::string render_backend_body_yaml(const CalibrationResult & result, const CalibrationConfig & config);
std::string render_report_yaml(
  const CalibrationSamples & samples, const CalibrationConfig & config,
  const CalibrationResult & result = {}, const std::optional<Segmentation> & segments = {},
  const std::optional<VerificationResult> & verification = {});

}  // namespace xxu_bringup
