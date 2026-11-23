#include "rigid_mount_calibration_detail.hpp"

#include <Eigen/Eigenvalues>
#include <random>

namespace xxu_bringup::calibration_detail {
namespace {

struct Plane
{
  Eigen::Vector3d normal;
  double offset, rms;
  std::size_t count;
};

struct ImuStatistics
{
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d variance = Eigen::Vector3d::Zero();
  double norm_std = 0.0, max_deviation = 0.0;
};

ImuStatistics statistics(
  const std::vector<ImuSample> & imu, Eigen::Vector3d ImuSample::* field)
{
  ImuStatistics result;
  double norm_mean = 0;
  for (const auto & sample : imu) {
    result.mean += sample.*field;
    norm_mean += (sample.*field).norm();
  }
  result.mean /= imu.size();
  norm_mean /= imu.size();
  double norm_sum = 0;
  for (const auto & sample : imu) {
    const Eigen::Vector3d difference = (sample.*field) - result.mean;
    result.variance += difference.cwiseAbs2();
    result.max_deviation = std::max(result.max_deviation, difference.norm());
    norm_sum += std::pow((sample.*field).norm() - norm_mean, 2);
  }
  if (imu.size() > 1) {
    result.variance /= imu.size() - 1;
    result.norm_std = std::sqrt(norm_sum / (imu.size() - 1));
  }
  return result;
}

std::optional<Plane> fit_plane(
  const CalibrationPoints & points, const Eigen::Vector3d & normal, double offset,
  double distance, std::size_t minimum)
{
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d second_moment = Eigen::Matrix3d::Zero();
  std::size_t count = 0;
  for (const auto & point : points) {
    if (std::abs(normal.dot(point) + offset) > distance) {continue;}
    sum += point;
    second_moment.noalias() += point * point.transpose();
    ++count;
  }
  if (count < minimum) {return {};}
  const Eigen::Vector3d centroid = sum / count;
  const Eigen::Matrix3d covariance = second_moment / count - centroid * centroid.transpose();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
  // A narrow line of returns cannot constrain a plane normal.
  if (eigen.info() != Eigen::Success || eigen.eigenvalues()[1] < 0.0025) {return {};}
  Eigen::Vector3d refined = eigen.eigenvectors().col(0);
  if (refined.z() < 0) {refined *= -1;}
  return Plane{refined, -refined.dot(centroid),
    std::sqrt(std::max(0.0, eigen.eigenvalues()[0])), count};
}

std::optional<Plane> dominant_plane(
  const CalibrationPoints & points, const CalibrationConfig & c, std::mt19937_64 & generator)
{
  // Search a bounded random subset, then refine inliers using the full ROI.
  const double distance = c.ground_inlier_distance;
  const double cosine = std::cos(radians(std::clamp(c.max_gravity_ground_angle_deg + 5.0, 12.0, 35.0)));
  std::uniform_int_distribution<std::size_t> index(0, points.size() - 1);
  CalibrationPoints probe;
  const std::size_t probe_size = std::min<std::size_t>(points.size(), 6000);
  probe.reserve(probe_size);
  for (std::size_t i = 0; i < probe_size; ++i) {
    probe.push_back(points[probe_size == points.size() ? i : index(generator)]);
  }
  std::size_t best_count = 0;
  std::optional<Plane> best;
  for (int iteration = 0; iteration < 480; ++iteration) {
    const auto & a = points[index(generator)];
    const auto & b = points[index(generator)];
    const auto & d = points[index(generator)];
    Eigen::Vector3d normal = (b - a).cross(d - a);
    const double length = normal.norm();
    if (length < 1.e-6 || std::abs(normal.z()) < cosine * length) {continue;}
    normal /= std::copysign(length, normal.z());
    const double offset = -normal.dot(a);
    const auto support = static_cast<std::size_t>(std::count_if(probe.begin(), probe.end(),
      [&](const auto & p) {return std::abs(normal.dot(p) + offset) <= distance;}));
    if (support <= best_count) {continue;}
    const auto refined = fit_plane(points, normal, offset, distance, c.min_ground_inliers);
    if (refined && refined->normal.z() >= cosine) {
      best_count = support;
      best = refined;
    }
  }
  if (best) {
    best = fit_plane(points, best->normal, best->offset, distance, c.min_ground_inliers);
  }
  return best;
}

}  // namespace

StationaryResult stationary_imu(
  const std::vector<ImuSample> & input, const CalibrationConfig & c,
  std::optional<TimeWindow> * imu_interval)
{
  StationaryResult result;
  result.attempted = true;
  const auto imu = usable(input, valid_imu);
  if (imu_interval != nullptr && !imu.empty()) {
    *imu_interval = TimeWindow{imu.front().time, imu.back().time};
  }
  result.imu_count = imu.size();
  if (imu.size() < static_cast<std::size_t>(c.min_imu_samples)) {
    result.reasons.emplace_back("too few valid IMU samples");
  }
  if (imu.empty()) {return result;}
  result.max_imu_gap = max_sample_gap(imu);
  if (result.max_imu_gap > c.auto_max_sample_gap) {
    result.reasons.emplace_back("stationary IMU stream contains a sample gap");
  }
  result.duration = imu.back().time - imu.front().time;
  if (result.duration < c.min_stationary_duration) {
    result.reasons.emplace_back("stationary IMU duration is too short");
  }

  const auto accel = statistics(imu, &ImuSample::acceleration);
  const auto gyro = statistics(imu, &ImuSample::gyro);
  result.accel_mean = accel.mean;
  result.accel_variance = accel.variance;
  result.gyro_bias = gyro.mean;
  result.gyro_variance = gyro.variance;
  result.accel_norm = accel.mean.norm();
  result.gyro_norm_std = gyro.norm_std;
  result.max_gyro_deviation = gyro.max_deviation;
  if (!gyro.variance.allFinite() || gyro.mean.norm() > c.max_gyro_mean ||
    gyro.norm_std > c.max_gyro_norm_std || gyro.max_deviation > c.max_gyro_deviation)
  {
    result.reasons.emplace_back("gyroscope indicates motion or unstable samples");
  }

  const auto scale = c.imu_acceleration_scale == 0.0 ?
    detect_imu_acceleration_scale(result.accel_norm) : std::optional<double>(c.imu_acceleration_scale);
  if (!scale) {
    result.reasons.emplace_back("cannot detect acceleration units: require mean norm near 1 g or 9.81 m/s^2");
    return result;
  }
  const double scaled_norm = result.accel_norm * *scale;
  if (!std::isfinite(scaled_norm) || std::abs(scaled_norm - 9.81) > 1.5) {
    result.reasons.emplace_back("scaled mean acceleration differs from 9.81 m/s^2 by more than 1.5");
    return result;
  }
  result.scale = scale;
  result.accel_norm_std = accel.norm_std * *scale;
  result.max_accel_deviation = accel.max_deviation * *scale;
  if (!accel.variance.allFinite() || result.accel_norm_std > c.max_accel_norm_std ||
    result.max_accel_deviation > c.max_accel_deviation)
  {
    result.reasons.emplace_back("accelerometer indicates motion or unstable samples");
  }
  if (result.passed()) {result.leveling = leveling_rotation_from_up(accel.mean);}
  return result;
}

void fit_floor(
  StationaryResult & result, const std::vector<CloudSample> & clouds,
  const Eigen::Isometry3d & frame_from_lidar, const Eigen::Vector3d & imu_origin,
  const CalibrationConfig & c, const std::optional<TimeWindow> & imu_interval)
{
  result.floor_attempted = true;
  CalibrationPoints roi;
  double last_time = -1.0;
  std::size_t outside_imu_interval = 0;
  for (const auto & cloud : clouds) {
    if (!valid_time(cloud.time) || cloud.time <= last_time || cloud.points.empty()) {continue;}
    if (!imu_interval || !imu_interval->contains(cloud.time)) {
      ++outside_imu_interval;
      continue;
    }
    bool usable_cloud = false;
    for (const auto & point : cloud.points) {
      if (!point.allFinite()) {continue;}
      usable_cloud = true;
      const Eigen::Vector3d transformed = frame_from_lidar * point;
      const Eigen::Vector3d relative = transformed - imu_origin;
      if (relative.z() < c.min_ground_z || relative.z() > c.max_ground_z) {continue;}
      const double radius_sq = relative.head<2>().squaredNorm();
      if (radius_sq >= c.min_ground_radius * c.min_ground_radius &&
        radius_sq <= c.max_ground_radius * c.max_ground_radius)
      {
        roi.push_back(transformed);
      }
    }
    if (usable_cloud) {
      ++result.cloud_count;
      last_time = cloud.time;
    }
  }
  result.roi_count = roi.size();
  if (result.cloud_count < static_cast<std::size_t>(c.min_ground_clouds)) {
    result.reasons.emplace_back(outside_imu_interval > 0 ?
      "too few valid stationary LiDAR clouds within the stationary IMU interval" :
      "too few valid stationary LiDAR clouds");
  }
  const auto minimum = static_cast<std::size_t>(c.min_ground_inliers);
  std::optional<Plane> floor;
  std::size_t substantial_support = minimum;
  std::mt19937_64 generator(0x585855ULL);
  // Peel off strong layers before choosing the lowest one with enough support.
  // This keeps a nearby dense roof from masking the floor.
  for (int layer = 0; layer < 6 && roi.size() >= substantial_support; ++layer) {
    const auto plane = dominant_plane(roi, c, generator);
    if (!plane || plane->count < substantial_support) {break;}
    if (layer == 0) {substantial_support = std::max(minimum, (plane->count + 6) / 7);}
    const auto depth = [&](const Plane & p) {return (p.offset + p.normal.dot(imu_origin)) / p.normal.z();};
    if (!floor || depth(*plane) > depth(*floor) + c.ground_inlier_distance) {floor = plane;}
    roi.erase(std::remove_if(roi.begin(), roi.end(), [&](const auto & point) {
        return std::abs(plane->normal.dot(point) + plane->offset) <= c.ground_inlier_distance;
      }), roi.end());
  }
  if (!floor) {
    result.reasons.emplace_back("no well-supported near-horizontal floor in the ground ROI");
    return;
  }
  result.floor_normal = floor->normal;
  result.floor_offset = floor->offset;
  result.inlier_count = floor->count;
  result.floor_rms = floor->rms;
  result.gravity_ground_angle_deg = degrees(unit_angle(floor->normal.z()));
  if (floor->rms > c.max_ground_rms) {result.reasons.emplace_back("floor plane RMS is too high");}
  if (result.gravity_ground_angle_deg > c.max_gravity_ground_angle_deg) {
    result.reasons.emplace_back("floor normal disagrees with gravity/base +Z");
  }
}

}  // namespace xxu_bringup::calibration_detail

namespace xxu_bringup {

StationaryResult solve_stationary(
  const std::vector<ImuSample> & imu, const std::vector<CloudSample> & clouds,
  const CalibrationConfig & c)
{
  std::optional<TimeWindow> imu_interval;
  auto result = calibration_detail::stationary_imu(imu, c, &imu_interval);
  if (!result.leveling) {return result;}
  Eigen::Isometry3d level_from_imu = Eigen::Isometry3d::Identity();
  level_from_imu.linear() = *result.leveling;
  calibration_detail::fit_floor(result, clouds, level_from_imu * c.imu_from_lidar,
    Eigen::Vector3d::Zero(), c, imu_interval);
  if (result.passed() && result.floor_offset > 0) {
    result.tz = result.floor_offset - c.base_height_above_ground;
  } else if (result.passed()) {
    result.reasons.emplace_back("floor plane is not below IMU origin");
  }
  return result;
}

}  // namespace xxu_bringup
