#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace xxu_bringup
{

inline double checked_positive_number(rclcpp::Node & node, const char * name, double initial,
                                      bool allow_zero = false)
{
  const double value = node.declare_parameter(name, initial);
  if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0)) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return value;
}

inline double checked_seconds(rclcpp::Node & node, const char * name, double initial,
                              bool allow_zero = false)
{
  return checked_positive_number(node, name, initial, allow_zero);
}

inline double checked_rate_hz(rclcpp::Node & node, const char * name, double initial)
{
  return checked_positive_number(node, name, initial, false);
}

inline Eigen::Vector3d vector_parameter(rclcpp::Node & node, const std::string & name,
                                        const std::vector<double> & initial)
{
  const auto value = node.declare_parameter(name, initial);
  if (value.size() != 3) {
    throw std::invalid_argument(name + " must contain three values");
  }
  const Eigen::Vector3d result(value[0], value[1], value[2]);
  if (!result.allFinite()) {
    throw std::invalid_argument(name + " must be finite");
  }
  return result;
}

}  // namespace xxu_bringup
