#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

#include <chrono>
#include <cmath>

namespace xxu_sim_bringup
{

inline double wrap_angle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

inline double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace xxu_sim_bringup
