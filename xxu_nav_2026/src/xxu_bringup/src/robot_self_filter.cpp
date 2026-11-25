#include "robot_self_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace xxu_bringup
{
namespace
{

Eigen::Vector3d read_vec3(const YAML::Node & node, const char * field_name)
{
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(std::string("self-filter ") + field_name + " must have 3 values");
  }
  const Eigen::Vector3d value(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string("self-filter ") + field_name + " must be finite");
  }
  return value;
}

}  // namespace

void RobotSelfFilter::load(const std::string & yaml_path, const std::string & expected_frame)
{
  const auto data = YAML::LoadFile(yaml_path);
  if (!data.IsMap()) {
    throw std::invalid_argument("self-filter geometry must be a mapping");
  }
  const auto spec = data["self_filter"];
  if (!spec.IsMap() || !spec["frame"].IsScalar()) {
    throw std::invalid_argument("self-filter section requires a frame");
  }
  if (spec["frame"].as<std::string>() != expected_frame) {
    throw std::invalid_argument("self-filter frame mismatch");
  }
  if (!spec["tolerance"].IsScalar()) {
    throw std::invalid_argument("self-filter tolerance must be specified");
  }

  const double tolerance = spec["tolerance"].as<double>();
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    throw std::invalid_argument("self-filter tolerance must be finite and non-negative");
  }

  std::vector<std::vector<Eigen::Vector4d>> hulls;
  const auto hull_planes = spec["hull_planes"];
  if (hull_planes && !hull_planes.IsNull()) {
    if (!hull_planes.IsSequence()) {
      throw std::invalid_argument("self-filter hull_planes must be a sequence");
    }
    for (const auto & planes : hull_planes) {
      if (!planes.IsSequence() || planes.size() == 0) {
        throw std::invalid_argument("self-filter hull must be a non-empty sequence of planes");
      }
      std::vector<Eigen::Vector4d> hull;
      hull.reserve(planes.size());
      for (const auto & plane : planes) {
        if (!plane.IsSequence() || plane.size() != 4) {
          throw std::invalid_argument("self-filter plane must have 4 coefficients");
        }
        const Eigen::Vector4d coeffs(plane[0].as<double>(), plane[1].as<double>(),
                                     plane[2].as<double>(), plane[3].as<double>());
        if (!coeffs.allFinite() || coeffs.head<3>().norm() < 1.0e-12) {
          throw std::invalid_argument("self-filter plane coefficients are invalid");
        }
        hull.push_back(coeffs);
      }
      hulls.push_back(std::move(hull));
    }
  }

  std::vector<Wheel> wheels;
  const auto wheel_nodes = data["wheels"];
  if (wheel_nodes && !wheel_nodes.IsNull()) {
    if (!wheel_nodes.IsSequence()) {
      throw std::invalid_argument("self-filter wheels must be a sequence");
    }
    for (const auto & wheel_node : wheel_nodes) {
      if (!wheel_node.IsMap()) {
        throw std::invalid_argument("self-filter wheel entry must be a mapping");
      }
      const Eigen::Vector3d center = read_vec3(wheel_node["center"], "wheel.center");
      Eigen::Vector3d axis = read_vec3(wheel_node["axis"], "wheel.axis");
      const double axis_norm = axis.norm();
      if (axis_norm < 1.0e-12) {
        throw std::invalid_argument("self-filter wheel.axis must be nonzero");
      }
      axis /= axis_norm;

      const double raw_radius = wheel_node["radius"].as<double>();
      const double raw_width = wheel_node["width"].as<double>();
      if (!std::isfinite(raw_radius) || raw_radius <= 0.0 || !std::isfinite(raw_width) ||
          raw_width <= 0.0) {
        throw std::invalid_argument("self-filter wheel radius and width must be positive");
      }
      const double radius = raw_radius + tolerance;
      const double half_width = raw_width / 2.0 + tolerance;
      wheels.push_back({center, axis, half_width, radius * radius});
    }
  }

  tolerance_ = tolerance;
  hulls_ = std::move(hulls);
  wheels_ = std::move(wheels);
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
