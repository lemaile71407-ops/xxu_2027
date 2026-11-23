#include "robot_self_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace xxu_bringup
{
namespace
{

Eigen::Vector3d read_vec3(const YAML::Node & node)
{
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

}  // namespace

void RobotSelfFilter::load(const std::string & yaml_path, const std::string & expected_frame)
{
  const auto data = YAML::LoadFile(yaml_path);
  const auto spec = data["self_filter"];
  if (spec["frame"].as<std::string>() != expected_frame) {
    throw std::invalid_argument("self-filter frame mismatch");
  }

  tolerance_ = spec["tolerance"].as<double>();
  hulls_.clear();
  for (const auto & planes : spec["hull_planes"]) {
    std::vector<Eigen::Vector4d> hull;
    for (const auto & plane : planes) {
      hull.emplace_back(plane[0].as<double>(), plane[1].as<double>(), plane[2].as<double>(),
                        plane[3].as<double>());
    }
    hulls_.push_back(std::move(hull));
  }

  wheels_.clear();
  for (const auto & wheel_node : data["wheels"]) {
    const double radius = wheel_node["radius"].as<double>() + tolerance_;
    const double half_width = wheel_node["width"].as<double>() / 2.0 + tolerance_;
    wheels_.push_back({read_vec3(wheel_node["center"]), read_vec3(wheel_node["axis"]), half_width,
                       radius * radius});
  }
}

bool RobotSelfFilter::enabled() const
{
  return !hulls_.empty() || !wheels_.empty();
}

bool RobotSelfFilter::contains(const Eigen::Vector3d & point) const
{
  for (const auto & hull : hulls_) {
    bool inside = true;
    for (const auto & plane : hull) {
      if (plane.head<3>().dot(point) + plane.w() > tolerance_) {
        inside = false;
        break;
      }
    }
    if (inside && point.allFinite()) {
      return true;
    }
  }

  for (const auto & wheel : wheels_) {
    const Eigen::Vector3d delta = point - wheel.center;
    const double axial = delta.dot(wheel.axis);
    if (std::abs(axial) <= wheel.half_width &&
        delta.squaredNorm() - axial * axial <= wheel.radius_squared) {
      return true;
    }
  }
  return false;
}

}  // namespace xxu_bringup
