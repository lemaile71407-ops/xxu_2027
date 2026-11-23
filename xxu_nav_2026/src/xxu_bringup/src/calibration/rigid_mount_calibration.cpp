#include "rigid_mount_calibration_detail.hpp"

#include <stdexcept>

namespace xxu_bringup {
using namespace calibration_detail;

Eigen::Vector3d rotation_to_rpy(const Eigen::Matrix3d & r)
{
  const double horizontal = std::hypot(r(0, 0), r(1, 0));
  const double pitch = std::atan2(-r(2, 0), horizontal);
  if (std::abs(r(2, 0)) >= 1.0 - 1.e-10) {
    const double yaw = horizontal <= 1.e-10 ? 0.0 : std::atan2(r(1, 0), r(0, 0));
    // Recover roll from well-scaled entries in Ry * Rx. The small bottom-row
    // components would make roll and yaw independently sensitive to roundoff.
    const Eigen::RowVector3d unyawed_y = std::cos(yaw) * r.row(1) - std::sin(yaw) * r.row(0);
    return {std::atan2(-unyawed_y.z(), unyawed_y.y()), pitch, yaw};
  }
  return {std::atan2(r(2, 1), r(2, 2)), pitch, std::atan2(r(1, 0), r(0, 0))};
}

Eigen::Matrix3d leveling_rotation_from_up(const Eigen::Vector3d & up)
{
  const double norm = up.stableNorm();
  if (!std::isfinite(norm) || norm <= 0.0) {
    throw std::invalid_argument("gravity direction must be finite and nonzero");
  }
  const Eigen::Vector3d direction = up / norm;
  const Eigen::Vector3d axis = direction.cross(Eigen::Vector3d::UnitZ());
  const double sine = axis.stableNorm();
  if (sine > 0.0) {
    return Eigen::AngleAxisd(std::atan2(sine, direction.z()), axis / sine).toRotationMatrix();
  }
  if (direction.z() < 0.0) {
    return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  }
  return Eigen::Matrix3d::Identity();
}

std::optional<double> detect_imu_acceleration_scale(double norm)
{
  if (norm >= 0.7 && norm <= 1.4) {return 9.81;}
  if (norm >= 7.0 && norm <= 13.5) {return 1.0;}
  return {};
}

void validate_calibration_config(const CalibrationConfig & c)
{
  for (const auto & [name, field] : real_thresholds) {
    const double value = c.*field;
    const bool height = field == &CalibrationConfig::min_ground_z ||
      field == &CalibrationConfig::max_ground_z;
    if (!std::isfinite(value) || (!height && value < 0.0)) {
      throw std::invalid_argument(std::string("invalid threshold: ") + name);
    }
  }
  for (const auto & [name, field] : count_thresholds) {
    if (c.*field < 1) {throw std::invalid_argument(std::string("invalid minimum: ") + name);}
  }
  if (c.min_imu_samples < 2 || c.min_ground_inliers < 3 || c.min_straight_samples < 2 ||
    c.min_spin_samples < 3 || c.max_ground_radius <= c.min_ground_radius ||
    c.max_ground_z <= c.min_ground_z || c.max_ground_z > 0.0 ||
    c.min_stationary_duration <= 0 || c.ground_inlier_distance <= 0 ||
    c.min_straight_distance <= 0 || c.min_spin_yaw_span_deg <= 0 ||
    c.max_spin_condition <= 1.0 || c.auto_max_sample_gap <= 0 ||
    c.auto_straight_min_speed <= c.auto_stationary_speed ||
    c.auto_spin_min_yaw_rate <= c.auto_straight_max_yaw_rate)
  {
    throw std::invalid_argument("inconsistent calibration thresholds");
  }
  if (!std::isfinite(c.imu_acceleration_scale) || c.imu_acceleration_scale < 0.0 ||
    !std::isfinite(c.base_height_above_ground) || c.base_height_above_ground < 0.0 ||
    (c.straight_direction != 1 && c.straight_direction != -1) ||
    (c.sensor_frame_mode != "base_aligned" && c.sensor_frame_mode != "imu_aligned"))
  {
    throw std::invalid_argument("invalid IMU scale, base height, straight direction or sensor mode");
  }
  for (const auto & value : {c.manual_psi_deg, c.manual_tx, c.manual_ty, c.manual_tz}) {
    if (value && !std::isfinite(*value)) {throw std::invalid_argument("nonfinite manual override");}
  }
  const Eigen::Matrix3d r = c.imu_from_lidar.linear();
  if (!c.imu_from_lidar.matrix().allFinite() ||
    !(r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1.e-9) ||
    std::abs(r.determinant() - 1.0) > 1.e-9)
  {
    throw std::invalid_argument("IMU-from-LiDAR extrinsic must be a finite rigid transform");
  }
  for (const auto & name : {c.base_frame, c.sensor_frame, c.lidar_frame, c.imu_source_frame}) {
    validate_mount_frame(name);
  }
  if (std::set<std::string>{c.base_frame, c.sensor_frame, c.lidar_frame}.size() != 3) {
    throw std::invalid_argument("base, sensor and lidar frames must be distinct");
  }
  if (!valid_frame(c.odom_frame) || c.odom_frame == "map" ||
    c.odom_frame == "base_footprint" || c.odom_frame == c.base_frame ||
    c.odom_frame == c.sensor_frame || c.odom_frame == c.lidar_frame)
  {
    throw std::invalid_argument("invalid odom_frame: " + c.odom_frame);
  }
  if (!valid_frame(c.odom_child_frame) || c.odom_child_frame == "map" ||
    c.odom_child_frame == "odom" || c.odom_child_frame == "base_footprint" ||
    c.odom_child_frame == c.base_frame || c.odom_child_frame == c.odom_frame)
  {
    throw std::invalid_argument("invalid odom_child_frame: " + c.odom_child_frame);
  }
  const std::set<std::string> topics{
    c.lidar_topic, c.imu_topic, c.odom_topic, c.lidar_topic + "_compensated",
    c.imu_topic + "_lio_compensated"};
  if (topics.size() != 5 || !std::all_of(topics.begin(), topics.end(), valid_topic)) {
    throw std::invalid_argument("topic names must be absolute and distinct");
  }
  if (!c.allow_unisolated_odom_topic &&
    (c.odom_topic == "/lidar_odometry" || c.odom_topic == "/odometry" ||
    c.odom_topic == "/frontend/odometry" || c.odom_topic == "/odom"))
  {
    throw std::invalid_argument(
      "production localization topics can publish compensated virtual-sensor poses under "
      "odom/imu frame IDs; use /calibration/lidar_odometry or explicitly opt in with "
      "allow_unisolated_odom_topic only for raw-input ZLIO");
  }
}

namespace {

bool has_straight(const CalibrationSamples & samples)
{
  return samples.straight_requested || !samples.straight.empty();
}

bool has_spin(const CalibrationSamples & samples)
{
  return samples.spin_requested || !samples.spin.empty();
}

std::vector<std::string> input_reasons(
  const CalibrationSamples & samples, const CalibrationConfig & c, const std::string & imu_frame,
  const std::string & lidar_frame, const std::string & base_frame)
{
  auto reasons = samples.input_errors;
  if (samples.imu_frames.size() > 1 || samples.lidar_frames.size() > 1 ||
    samples.odom_frames.size() > 1 || samples.odom_child_frames.size() > 1)
  {
    reasons.emplace_back("input frame IDs changed within the recording or captures");
  }
  if (!samples.imu_frames.empty() && *samples.imu_frames.begin() != imu_frame) {
    reasons.emplace_back("observed IMU frame does not match configured imu_source_frame");
  }
  if (!samples.lidar_frames.empty() && *samples.lidar_frames.begin() != lidar_frame) {
    reasons.emplace_back("observed LiDAR frame does not match configured lidar_frame");
  }
  if (samples.odom_child_frames.count(base_frame)) {
    reasons.emplace_back("base-frame odometry is not physical IMU odometry");
  }
  if (!samples.odom_frames.empty() && *samples.odom_frames.begin() != c.odom_frame) {
    reasons.emplace_back("observed odometry frame does not match configured odom_frame");
  }
  if (!samples.odom_child_frames.empty() && *samples.odom_child_frames.begin() != c.odom_child_frame) {
    reasons.emplace_back("observed odometry child_frame_id does not match configured odom_child_frame");
  }
  return reasons;
}

void append_failures(
  std::vector<std::string> & reasons, const std::string & stage, const StageQuality & quality)
{
  for (const auto & reason : quality.reasons) {reasons.push_back(stage + ": " + reason);}
}

Eigen::Isometry3d make_base_from_imu(
  const Eigen::Matrix3d & rotation, double tx, double ty, double tz)
{
  auto pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(tx, ty, tz);
  return pose;
}

double refinement_cost(const StraightResult & straight, const SpinResult & spin)
{
  return std::hypot(straight.lateral_rms, spin.residual_rms);
}

}  // namespace

CalibrationResult solve_all(const CalibrationSamples & samples, const CalibrationConfig & c)
{
  validate_calibration_config(c);
  CalibrationResult result;
  result.reasons = input_reasons(samples, c, c.imu_source_frame, c.lidar_frame, c.base_frame);
  if (samples.stationary_requested || !samples.imu.empty() || !samples.clouds.empty()) {
    result.stationary = solve_stationary(samples.imu, samples.clouds, c);
  }

  // Diagnostic scale/leveling can survive a floor failure. The stationary
  // quality gate still prevents confirmation.
  result.scale = result.stationary.scale;
  result.tz = c.manual_tz ? c.manual_tz : result.stationary.tz;
  if (c.manual_tz && result.stationary.tz &&
    std::abs(*c.manual_tz - *result.stationary.tz) > c.max_verify_floor_height_error)
  {
    result.stationary.reasons.emplace_back("manual tz disagrees with measured floor height");
  }
  result.tx = c.manual_tx;
  result.ty = c.manual_ty;
  const bool manual_yaw = c.manual_psi_deg.has_value();
  if (manual_yaw) {result.psi = wrap(radians(*c.manual_psi_deg));}
  const auto & level = result.stationary.leveling;
  if (has_straight(samples)) {
    if (level) {
      result.straight = solve_straight(samples.straight, *level, c);
    } else {
      result.straight.attempted = true;
      result.straight.sample_count = samples.straight.size();
      result.straight.reasons.emplace_back("requires stationary gravity leveling");
    }
  }
  std::optional<double> candidate_yaw = manual_yaw ? result.psi : result.straight.psi;
  if (has_spin(samples)) {
    if (level && candidate_yaw && result.tz) {
      result.spin = solve_spin(samples.spin, yaw_rotation(*candidate_yaw) * *level, *result.tz, c);
    } else {
      result.spin.attempted = true;
      result.spin.sample_count = samples.spin.size();
      result.spin.reasons.emplace_back("requires gravity leveling, straight/manual psi and floor/manual tz");
    }
  }

  if (result.straight.psi && result.spin.tx && result.spin.ty) {
    auto & quality = result.refinement_quality;
    quality.attempted = true;
    const double initial_cost = refinement_cost(result.straight, result.spin);
    for (int iteration = 0; iteration < 3; ++iteration) {
      const auto pose = make_base_from_imu(
        yaw_rotation(*candidate_yaw) * *level, *result.spin.tx, *result.spin.ty, *result.tz);
      auto straight = solve_straight(samples.straight, *level, c, pose);
      if (!straight.psi) {
        quality.reasons.emplace_back("corrected straight trajectory has no usable direction");
        break;
      }
      const double yaw = manual_yaw ? *candidate_yaw : *straight.psi;
      auto spin = solve_spin(samples.spin, yaw_rotation(yaw) * *level, *result.tz, c);
      if (!spin.tx || !spin.ty) {
        quality.reasons.emplace_back("updated spin system could not be solved");
        break;
      }
      const double lever_change = std::hypot(*spin.tx - *result.spin.tx, *spin.ty - *result.spin.ty);
      result.refinement.push_back({std::abs(wrap(yaw - *candidate_yaw)), lever_change,
        straight.lateral_rms, spin.residual_rms});
      candidate_yaw = yaw;
      result.straight = std::move(straight);
      result.spin = std::move(spin);
    }
    if (result.refinement.size() == 3) {
      const auto & last = result.refinement.back();
      if ((last[0] > radians(0.05) || last[1] > 0.001) &&
        refinement_cost(result.straight, result.spin) > initial_cost + 1.e-6)
      {
        quality.reasons.emplace_back("refinement neither converged nor reduced combined residual");
      }
    }
  } else if (level && result.straight.psi && result.tx && result.ty && result.tz) {
    const auto pose = make_base_from_imu(
      yaw_rotation(*candidate_yaw) * *level, *result.tx, *result.ty, *result.tz);
    result.straight = solve_straight(samples.straight, *level, c, pose);
    if (!manual_yaw) {candidate_yaw = result.straight.psi;}
  }

  if (manual_yaw && result.straight.psi) {
    result.straight.yaw_error_deg = std::abs(degrees(wrap(*result.straight.psi - *result.psi)));
    if (result.straight.yaw_error_deg > c.max_verify_straight_yaw_deg) {
      result.straight.reasons.emplace_back("manual psi disagrees with measured straight heading");
    }
  }
  if (!manual_yaw && result.straight.passed()) {result.psi = candidate_yaw;}
  if (result.spin.passed() && (manual_yaw || result.straight.passed())) {
    if (!c.manual_tx) {result.tx = result.spin.tx;}
    if (!c.manual_ty) {result.ty = result.spin.ty;}
  }
  if (level && result.psi) {result.base_from_imu = yaw_rotation(*result.psi) * *level;}

  append_failures(result.reasons, "stationary", result.stationary);
  append_failures(result.reasons, "straight", result.straight);
  append_failures(result.reasons, "spin", result.spin);
  append_failures(result.reasons, "refinement", result.refinement_quality);
  for (const auto & [label, present] : std::initializer_list<std::pair<const char *, bool>>{
    {"IMU scale", result.scale.has_value()}, {"gravity leveling", level.has_value()},
    {"psi", result.psi.has_value()}, {"tx", result.tx.has_value()}, {"ty", result.ty.has_value()},
    {"tz", result.tz.has_value()}})
  {
    if (!present) {result.reasons.emplace_back(std::string(label) + " is unmeasured");}
  }
  result.confirmed = c.confirm && result.stationary.passed() && result.reasons.empty();
  return result;
}

VerificationResult verify_calibration(
  const RigidMount & mount, const CalibrationSamples & samples, const CalibrationConfig & c)
{
  validate_calibration_config(c);
  VerificationResult result;
  result.reasons =
    input_reasons(samples, c, mount.imu_source_frame, mount.lidar_frame, mount.base_frame);
  Eigen::Isometry3d base_from_imu = mount.base_from_sensor;
  base_from_imu.linear() *= mount.sensor_from_imu;

  auto settings = c;
  settings.imu_acceleration_scale = mount.imu_acceleration_scale;
  std::optional<TimeWindow> imu_interval;
  result.stationary = stationary_imu(samples.imu, settings, &imu_interval);
  if (result.stationary.scale) {
    const Eigen::Vector3d gravity =
      base_from_imu.linear() * result.stationary.accel_mean * mount.imu_acceleration_scale;
    result.gravity_angle_deg = degrees(unit_angle(gravity.normalized().z()));
    result.gravity_error = (gravity - Eigen::Vector3d(0.0, 0.0, 9.81)).norm();
    if (result.gravity_angle_deg > c.max_gravity_ground_angle_deg) {
      result.stationary.reasons.emplace_back("mounted IMU gravity does not align with base +Z");
    }
    if (result.gravity_error > c.max_verify_gravity_error) {
      result.stationary.reasons.emplace_back("mounted IMU gravity vector differs from [0, 0, 9.81]");
    }
  }
  fit_floor(result.stationary, samples.clouds, mount.base_from_sensor * mount.sensor_from_lidar,
    base_from_imu.translation(), settings, imu_interval);
  if (std::isfinite(result.stationary.floor_offset)) {
    result.floor_height_error =
      std::abs(result.stationary.floor_offset - c.base_height_above_ground);
    if (result.floor_height_error > c.max_verify_floor_height_error) {
      result.stationary.reasons.emplace_back("mounted floor does not match base height above ground");
    }
  }
  if (samples.straight.empty()) {
    result.reasons.emplace_back("straight motion verification was not executed");
  } else {
    result.straight = solve_straight(samples.straight, base_from_imu.linear(), settings, base_from_imu);
    if (result.straight.psi) {
      result.straight.yaw_error_deg = std::abs(degrees(*result.straight.psi));
      if (result.straight.yaw_error_deg > c.max_verify_straight_yaw_deg) {
        result.straight.reasons.emplace_back("mounted straight trajectory does not align with base X");
      }
    }
  }
  if (samples.spin.empty()) {
    result.reasons.emplace_back("spin motion verification was not executed");
  } else {
    result.spin = evaluate_spin(samples.spin, base_from_imu.linear(), base_from_imu.translation().z(),
      settings, Eigen::Vector2d(base_from_imu.translation().head<2>()));
  }
  append_failures(result.reasons, "stationary", result.stationary);
  append_failures(result.reasons, "straight", result.straight);
  append_failures(result.reasons, "spin", result.spin);
  return result;
}

}  // namespace xxu_bringup
