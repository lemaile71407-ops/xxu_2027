#pragma once

#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/rigid_mount_calibration.hpp"

#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace xxu_bringup {

inline CalibrationPoints extract_calibration_points(
  const sensor_msgs::msg::PointCloud2 & cloud,
  std::size_t limit = std::numeric_limits<std::size_t>::max())
{
  const CloudLayout layout(cloud);
  const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
  const std::size_t count = std::min(total, limit);
  CalibrationPoints points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    // Sample uniformly across organized rows if the capture has a point cap.
    const std::size_t index = i * (total / count) + (i * (total % count)) / count;
    const auto * point = cloud.data.data() + (index / cloud.width) * cloud.row_step +
      (index % cloud.width) * cloud.point_step;
    const Eigen::Vector3d value(
      read_scalar<float>(point + layout.xyz[0], cloud.is_bigendian),
      read_scalar<float>(point + layout.xyz[1], cloud.is_bigendian),
      read_scalar<float>(point + layout.xyz[2], cloud.is_bigendian));
    if (value.allFinite()) {points.push_back(value);}
  }
  return points;
}

#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
inline CalibrationPoints extract_calibration_points(
  const livox_ros_driver2::msg::CustomMsg & cloud,
  std::size_t limit = std::numeric_limits<std::size_t>::max())
{
  if (cloud.point_num != cloud.points.size()) {
    throw std::invalid_argument("Livox point_num does not match data");
  }
  const std::size_t total = cloud.points.size();
  const std::size_t count = std::min(total, limit);
  CalibrationPoints points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & point = cloud.points[i * (total / count) + (i * (total % count)) / count];
    const Eigen::Vector3d value(point.x, point.y, point.z);
    if (value.allFinite()) {points.push_back(value);}
  }
  return points;
}
#endif

}  // namespace xxu_bringup
