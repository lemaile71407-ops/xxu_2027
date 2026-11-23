#pragma once

#include "xxu_bringup/localization_health.hpp"

#include <Eigen/Geometry>

#include <chrono>

namespace xxu_bringup
{

inline double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp_ns(stamp)) * 1.0e-9;
}

inline Eigen::Matrix3d rpy_rotation(const Eigen::Vector3d & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
    .toRotationMatrix();
}

}  // namespace xxu_bringup
