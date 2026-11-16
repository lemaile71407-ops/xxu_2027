#pragma once

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

namespace xxu_bringup {

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d result;
  result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return result;
}

inline Eigen::Quaterniond rotation(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond result(q.w, q.x, q.y, q.z);
  const double norm = result.norm();
  if (!result.coeffs().allFinite() || !std::isfinite(norm) || norm < 1.e-12) {
    throw std::invalid_argument("invalid rotation quaternion");
  }
  return result.normalized();
}

inline nav_msgs::msg::Odometry convert_odometry(
  const nav_msgs::msg::Odometry & input, const Eigen::Isometry3d & base_from_sensor,
  const std::string & base_frame)
{
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using RowMatrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  const auto & pose = input.pose.pose;
  Eigen::Isometry3d odom_from_sensor = Eigen::Isometry3d::Identity();
  odom_from_sensor.linear() = rotation(pose.orientation).toRotationMatrix();
  odom_from_sensor.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Vector3d velocity(input.twist.twist.linear.x, input.twist.twist.linear.y,
    input.twist.twist.linear.z);
  const Eigen::Vector3d omega(input.twist.twist.angular.x, input.twist.twist.angular.y,
    input.twist.twist.angular.z);
  const Eigen::Map<const RowMatrix6> pose_cov(input.pose.covariance.data());
  const Eigen::Map<const RowMatrix6> twist_cov(input.twist.covariance.data());
  if (!odom_from_sensor.matrix().allFinite() || !base_from_sensor.matrix().allFinite() ||
    !velocity.allFinite() || !omega.allFinite() || !pose_cov.allFinite() || !twist_cov.allFinite())
  {
    throw std::invalid_argument("nonfinite odometry or extrinsics");
  }
  const Eigen::Isometry3d sensor_from_base = base_from_sensor.inverse();
  const Eigen::Isometry3d odom_from_base = odom_from_sensor * sensor_from_base;
  const auto & r_bs = base_from_sensor.linear();
  const Eigen::Vector3d lever_sensor = sensor_from_base.translation();
  const Eigen::Vector3d v_base = r_bs * (velocity + omega.cross(lever_sensor));
  const Eigen::Vector3d w_base = r_bs * omega;
  const Eigen::Quaterniond q(odom_from_base.linear());
  auto output = input;
  output.child_frame_id = base_frame;
  output.pose.pose.position.x = odom_from_base.translation().x();
  output.pose.pose.position.y = odom_from_base.translation().y();
  output.pose.pose.position.z = odom_from_base.translation().z();
  output.pose.pose.orientation.x = q.x();
  output.pose.pose.orientation.y = q.y();
  output.pose.pose.orientation.z = q.z();
  output.pose.pose.orientation.w = q.w();
  output.twist.twist.linear.x = v_base.x();
  output.twist.twist.linear.y = v_base.y();
  output.twist.twist.linear.z = v_base.z();
  output.twist.twist.angular.x = w_base.x();
  output.twist.twist.angular.y = w_base.y();
  output.twist.twist.angular.z = w_base.z();
  // ZLIO uses world position and left world rotation errors. Twist errors are
  // local to the sensor origin; the two covariance Jacobians are different.
  Matrix6 pose_j = Matrix6::Identity();
  pose_j.block<3, 3>(0, 3) = -skew(odom_from_sensor.linear() * lever_sensor);
  Matrix6 twist_j = Matrix6::Zero();
  twist_j.block<3, 3>(0, 0) = r_bs;
  twist_j.block<3, 3>(0, 3) = -r_bs * skew(lever_sensor);
  twist_j.block<3, 3>(3, 3) = r_bs;
  Eigen::Map<RowMatrix6>(output.pose.covariance.data()) = pose_j * pose_cov * pose_j.transpose();
  Eigen::Map<RowMatrix6>(output.twist.covariance.data()) = twist_j * twist_cov * twist_j.transpose();
  return output;
}
}  // namespace xxu_bringup
