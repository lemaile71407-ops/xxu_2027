#pragma once

#include <Eigen/Core>

#include <string>
#include <vector>

namespace xxu_bringup
{

/// Geometric robot self-filter using convex hull half-spaces and wheel cylinders
/// loaded from a YAML configuration file in the target sensor/chassis frame.
class RobotSelfFilter
{
public:
  void load(const std::string & yaml_path, const std::string & expected_frame);

  bool enabled() const;

  bool contains(const Eigen::Vector3d & point) const;

private:
  struct Wheel {
    Eigen::Vector3d center;
    Eigen::Vector3d axis;
    double half_width = 0.0;
    double radius_squared = 0.0;
  };

  double tolerance_ = 0.0;
  std::vector<std::vector<Eigen::Vector4d>> hulls_;
  std::vector<Wheel> wheels_;
};

}  // namespace xxu_bringup
