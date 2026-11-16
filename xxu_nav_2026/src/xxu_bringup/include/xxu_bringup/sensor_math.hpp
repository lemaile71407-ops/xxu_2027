#pragma once

#include "xxu_bringup/localization_health.hpp"
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <optional>

namespace xxu_bringup {
inline double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp_ns(stamp)) * 1.e-9;
}
inline Eigen::Vector3d vector_parameter(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & initial)
{
  const auto value = node.declare_parameter(name, initial);
  if (value.size() != 3) {throw std::invalid_argument(name + " must contain three values");}
  const Eigen::Vector3d result(value[0], value[1], value[2]);
  if (!result.allFinite()) {throw std::invalid_argument(name + " must be finite");}
  return result;
}
inline Eigen::Matrix3d rpy_rotation(const Eigen::Vector3d & rpy)
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}
inline double checked_seconds(rclcpp::Node & node, const char * name, double initial, bool zero = false)
{
  const double value = node.declare_parameter(name, initial);
  if (!std::isfinite(value) || value < 0 || (!zero && value == 0)) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return value;
}

// The original interpolator never extrapolates across absent joint samples.
class JointHistory
{
public:
  struct Sample {double time, position, velocity, acceleration;};
  double gap = .05, reset_threshold = .5, acceleration_tau = .05, maximum_acceleration = 100.;
  std::string name = "gimbal_joint";
  std::deque<Sample> samples;
  void reset_derivative() {derivative_valid_ = false; filtered_acceleration_ = 0.;}
  bool add(const sensor_msgs::msg::JointState & message, bool require_velocity)
  {
    const auto found = std::find(message.name.begin(), message.name.end(), name);
    const auto index = static_cast<std::size_t>(found - message.name.begin());
    if (found == message.name.end() || index >= message.position.size() ||
      (require_velocity && index >= message.velocity.size()) || stamp_ns(message.header.stamp) < 0)
    {return false;}
    const double raw = message.position[index];
    const double velocity = index < message.velocity.size() ? message.velocity[index] : 0.;
    const double time = seconds(message.header.stamp);
    if (!std::isfinite(raw) || !std::isfinite(velocity)) {return false;}
    bool reset = false;
    if (!samples.empty()) {
      const double dt = time - samples.back().time;
      if (dt <= 0 && -dt < reset_threshold) {return false;}
      if (dt <= 0 || dt > gap) {samples.clear(); raw_position_.reset(); reset_derivative(); reset = true;}
    }
    double position = raw, acceleration = 0.;
    if (!samples.empty()) {
      const double delta = raw - *raw_position_;
      position = samples.back().position + std::atan2(std::sin(delta), std::cos(delta));
      const double dt = time - samples.back().time;
      if (derivative_valid_) {
        const double raw_acc = std::clamp((velocity - samples.back().velocity) / dt,
          -maximum_acceleration, maximum_acceleration);
        filtered_acceleration_ += (acceleration_tau == 0 ? 1. : dt / (acceleration_tau + dt)) *
          (raw_acc - filtered_acceleration_);
        acceleration = filtered_acceleration_;
      }
    }
    raw_position_ = raw;
    derivative_valid_ = true;
    samples.push_back({time, position, velocity, acceleration});
    if (samples.size() > 4096) {samples.pop_front();}
    return reset;
  }
  std::optional<Sample> interpolate(double time, bool require_neighbor = true) const
  {
    if (samples.size() < 2 || time < samples.front().time - 1.e-9 || time > samples.back().time + 1.e-9) {
      return std::nullopt;
    }
    auto after = std::lower_bound(samples.begin(), samples.end(), time,
      [](const Sample & sample, double t) {return sample.time < t;});
    if (after == samples.end()) {after = std::prev(samples.end());}
    if (std::abs(after->time - time) <= 1.e-9) {
      if (!require_neighbor) {return *after;}
      const auto neighbor = after == samples.begin() ? std::next(after) : std::prev(after);
      return std::abs(after->time - neighbor->time) <= gap ? std::optional<Sample>(*after) : std::nullopt;
    }
    if (after == samples.begin()) {return std::nullopt;}
    const auto before = std::prev(after);
    const double dt = after->time - before->time;
    if (dt <= 0 || dt > gap) {return std::nullopt;}
    const double ratio = (time - before->time) / dt;
    return Sample{time, before->position + ratio * (after->position - before->position),
      before->velocity + ratio * (after->velocity - before->velocity),
      before->acceleration + ratio * (after->acceleration - before->acceleration)};
  }
private:
  std::optional<double> raw_position_;
  double filtered_acceleration_ = 0.;
  bool derivative_valid_ = false;
};
}  // namespace xxu_bringup
