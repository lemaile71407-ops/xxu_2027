#include "rigid_mount_calibration_detail.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace xxu_bringup {
using namespace calibration_detail;
namespace {

struct AngleSpan
{
  bool initialized = false;
  double last = 0, unwrapped = 0, low = 0, high = 0;
  void add(double angle)
  {
    if (initialized) {
      unwrapped += wrap(angle - last);
      low = std::min(low, unwrapped);
      high = std::max(high, unwrapped);
    }
    initialized = true;
    last = angle;
  }
  double span_deg() const {return degrees(high - low);}
};

}  // namespace

StraightResult solve_straight(
  const std::vector<OdomSample> & input, const Eigen::Matrix3d & level_from_imu,
  const CalibrationConfig & c, const std::optional<Eigen::Isometry3d> & base_from_imu)
{
  StraightResult result;
  result.attempted = true;
  const auto samples = usable(input, valid_odom);
  result.sample_count = samples.size();
  if (samples.size() < static_cast<std::size_t>(c.min_straight_samples)) {
    result.reasons.emplace_back("too few valid straight odometry samples");
  }
  if (samples.size() < 2) {return result;}
  result.max_sample_gap = max_sample_gap(samples);
  if (result.max_sample_gap > c.auto_max_sample_gap) {
    result.reasons.emplace_back("straight odometry stream contains a sample gap");
  }

  CalibrationPoints positions;
  positions.reserve(samples.size());
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond first(odom_rotation(samples.front()) * level_from_imu.transpose());
  AngleSpan yaw;
  double max_level_tilt_deg = 0.0;
  for (const auto & sample : samples) {
    const Eigen::Matrix3d odom_from_imu = odom_rotation(sample);
    const Eigen::Matrix3d odom_from_level = odom_from_imu * level_from_imu.transpose();
    max_level_tilt_deg = std::max(max_level_tilt_deg, degrees(unit_angle(odom_from_level(2, 2))));
    const Eigen::Quaterniond orientation(odom_from_level);
    quaternion_sum += std::copysign(1.0, orientation.dot(first)) * orientation.coeffs();
    yaw.add(std::atan2(odom_from_level(1, 0), odom_from_level(0, 0)));
    Eigen::Vector3d position = sample.position;
    if (base_from_imu) {
      position -= odom_from_imu * base_from_imu->linear().transpose() * base_from_imu->translation();
    }
    positions.push_back(position);
    mean += position.head<2>();
  }
  if (max_level_tilt_deg > c.max_odom_tilt_deg) {
    result.reasons.emplace_back(odom_tilt_reason);
  }
  mean /= positions.size();
  result.yaw_span_deg = yaw.span_deg();
  if (result.yaw_span_deg > c.max_straight_yaw_change_deg) {
    result.reasons.emplace_back("straight run yaw span is too large");
  }
  const Eigen::Vector3d net = positions.back() - positions.front();
  result.distance = net.head<2>().norm();
  if (!std::isfinite(result.distance) || result.distance < c.min_straight_distance) {
    result.reasons.emplace_back("straight horizontal displacement is too short or invalid");
  }
  if (!std::isfinite(result.distance) || result.distance < 1.e-9) {return result;}
  result.vertical_ratio = std::abs(net.z()) / result.distance;
  if (result.vertical_ratio > c.max_straight_vertical_ratio) {
    result.reasons.emplace_back("straight run vertical slope is too large");
  }

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto & position : positions) {
    const Eigen::Vector2d offset = position.head<2>() - mean;
    covariance.noalias() += offset * offset.transpose();
  }
  covariance /= positions.size();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> pca(covariance);
  if (pca.info() != Eigen::Success || !pca.eigenvalues().allFinite()) {
    result.reasons.emplace_back("straight PCA failed");
    return result;
  }

  Eigen::Vector2d axis = pca.eigenvectors().col(1);
  if (axis.dot(net.head<2>()) < 0) {axis *= -1;}
  result.pca_angle_deg = degrees(unit_angle(axis.dot(net.head<2>() / result.distance)));
  result.lateral_rms = std::sqrt(std::max(0.0, pca.eigenvalues()[0]));
  if (result.pca_angle_deg > c.max_straight_pca_angle_deg) {
    result.reasons.emplace_back("straight PCA axis disagrees with net displacement");
  }
  if (result.lateral_rms > c.max_straight_lateral_rms) {
    result.reasons.emplace_back("straight run lateral RMS is too high");
  }

  const Eigen::Quaterniond average(quaternion_sum.normalized());
  const Eigen::Vector3d displacement = average.conjugate() *
    Eigen::Vector3d(axis.x() * result.distance, axis.y() * result.distance, net.z());
  if (!displacement.allFinite() || displacement.head<2>().norm() < 1.e-9) {
    result.reasons.emplace_back("straight displacement has no direction in the leveled frame");
    return result;
  }
  result.psi = wrap(std::atan2(-displacement.y(), displacement.x()) +
    (c.straight_direction == -1 ? calibration_pi : 0.0));
  return result;
}

SpinResult calibration_detail::evaluate_spin(
  const std::vector<OdomSample> & input, const Eigen::Matrix3d & base_from_imu,
  double tz, const CalibrationConfig & c, const std::optional<Eigen::Vector2d> & fixed_xy)
{
  SpinResult result;
  result.attempted = true;
  const auto samples = usable(input, valid_odom);
  result.sample_count = samples.size();
  if (samples.size() < static_cast<std::size_t>(c.min_spin_samples)) {
    result.reasons.emplace_back("too few valid spin odometry samples");
  }
  if (samples.size() < 3) {return result;}
  result.max_sample_gap = max_sample_gap(samples);
  if (result.max_sample_gap > c.auto_max_sample_gap) {
    result.reasons.emplace_back("spin odometry stream contains a sample gap");
  }

  const Eigen::Index rows = 2 * samples.size();
  Eigen::MatrixXd a(rows, 4);
  Eigen::VectorXd b(rows);
  Eigen::Vector2d raw_mean = Eigen::Vector2d::Zero();
  AngleSpan roll, pitch, yaw;
  double max_level_tilt_deg = 0.0;
  for (Eigen::Index row = 0; row < rows; row += 2) {
    const auto & sample = samples[row / 2];
    const Eigen::Matrix3d odom_from_base = odom_rotation(sample) * base_from_imu.transpose();
    max_level_tilt_deg = std::max(max_level_tilt_deg, degrees(unit_angle(odom_from_base(2, 2))));
    const Eigen::Vector3d rpy = rotation_to_rpy(odom_from_base);
    roll.add(rpy.x());
    pitch.add(rpy.y());
    yaw.add(rpy.z());
    a.block<2, 2>(row, 0).setIdentity();
    a.block<2, 2>(row, 2) = odom_from_base.topLeftCorner<2, 2>();
    b.segment<2>(row) = sample.position.head<2>() - odom_from_base.block<2, 1>(0, 2) * tz;
    raw_mean += sample.position.head<2>();
  }
  if (max_level_tilt_deg > c.max_odom_tilt_deg) {
    result.reasons.emplace_back(odom_tilt_reason);
  }
  result.yaw_span_deg = yaw.span_deg();
  result.roll_span_deg = roll.span_deg();
  result.pitch_span_deg = pitch.span_deg();
  if (result.yaw_span_deg + 1.e-8 < c.min_spin_yaw_span_deg) {
    result.reasons.emplace_back("unwrapped spin yaw span is too short");
  }
  if (std::max(result.roll_span_deg, result.pitch_span_deg) > c.max_spin_tilt_span_deg) {
    result.reasons.emplace_back("body roll or pitch fluctuates too much during spin");
  }
  raw_mean /= samples.size();
  double raw_square = 0.0;
  for (const auto & sample : samples) {
    raw_square += (sample.position.head<2>() - raw_mean).squaredNorm();
  }
  result.raw_radius_rms = std::sqrt(raw_square / samples.size());

  const std::array<std::optional<double>, 4> fixed{
    std::nullopt, std::nullopt,
    fixed_xy ? std::optional<double>(fixed_xy->x()) : c.manual_tx,
    fixed_xy ? std::optional<double>(fixed_xy->y()) : c.manual_ty};
  Eigen::Vector4d model = Eigen::Vector4d::Zero();
  std::vector<int> free;
  Eigen::VectorXd target = b;
  for (int column = 0; column < 4; ++column) {
    if (fixed[column]) {
      model[column] = *fixed[column];
      target -= a.col(column) * model[column];
    } else {
      free.push_back(column);
    }
  }

  Eigen::MatrixXd weighted = a;
  Eigen::MatrixXd reduced(rows, free.size());
  Eigen::VectorXd weighted_target = target;
  Eigen::VectorXd estimate;
  for (int iteration = 0; iteration < 5; ++iteration) {
    for (std::size_t column = 0; column < free.size(); ++column) {
      reduced.col(column) = weighted.col(free[column]);
    }
    const auto qr = reduced.colPivHouseholderQr();
    if (qr.rank() != reduced.cols()) {
      result.reasons.emplace_back("spin system is rank deficient");
      return result;
    }
    estimate = qr.solve(weighted_target);
    for (std::size_t column = 0; column < free.size(); ++column) {
      model[free[column]] = estimate[column];
    }
    if (iteration == 4) {break;}
    for (Eigen::Index row = 0; row < rows; row += 2) {
      const double error = (a.middleRows<2>(row) * model - b.segment<2>(row)).norm();
      const double weight = error > 0.02 ? std::sqrt(0.02 / error) : 1.0;
      weighted.middleRows<2>(row) = weight * a.middleRows<2>(row);
      weighted_target.segment<2>(row) = weight * target.segment<2>(row);
    }
  }
  if (!model.allFinite()) {
    result.reasons.emplace_back("spin solver returned nonfinite parameters");
    return result;
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> full_svd(weighted);
  result.rank = static_cast<int>(full_svd.rank());
  const auto & singular = full_svd.singularValues();
  if (singular[3] > 0) {result.condition = singular[0] / singular[3];}
  if (result.rank != 4 || !std::isfinite(result.condition) || result.condition >= c.max_spin_condition) {
    result.reasons.emplace_back("spin system is ill-conditioned");
  }

  result.tx = model[2];
  result.ty = model[3];
  result.center = model.head<2>();
  result.residual_rms = (a * model - b).norm() / std::sqrt(samples.size());
  if (!std::isfinite(result.residual_rms) || result.residual_rms > c.max_spin_residual_rms) {
    result.reasons.emplace_back("compensated base center drift RMS is too high");
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(reduced, Eigen::ComputeThinV);
  if (svd.rank() == reduced.cols()) {
    const double sigma_squared = (reduced * estimate - weighted_target).squaredNorm() /
      (rows - reduced.cols());
    const Eigen::MatrixXd covariance = sigma_squared * svd.matrixV() *
      svd.singularValues().cwiseInverse().cwiseAbs2().asDiagonal() * svd.matrixV().transpose();
    for (std::size_t row = 0; row < free.size(); ++row) {
      for (std::size_t column = 0; column < free.size(); ++column) {
        result.covariance(free[row], free[column]) = covariance(row, column);
      }
    }
    for (std::size_t column = 0; column < free.size(); ++column) {
      const double deviation = std::sqrt(std::max(0.0, covariance(column, column)));
      if (free[column] == 2) {result.std_tx = deviation;}
      if (free[column] == 3) {result.std_ty = deviation;}
    }
  }
  return result;
}

SpinResult solve_spin(
  const std::vector<OdomSample> & samples, const Eigen::Matrix3d & base_from_imu,
  double tz, const CalibrationConfig & c)
{
  return evaluate_spin(samples, base_from_imu, tz, c, {});
}

}  // namespace xxu_bringup
