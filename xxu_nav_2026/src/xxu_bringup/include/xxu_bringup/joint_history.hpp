#pragma once

#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <optional>
#include <string>

namespace xxu_bringup
{

// The original interpolator never extrapolates across absent joint samples.
class JointHistory
{
public:
  struct Sample {
    double time = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
  };

  double max_sample_gap_sec = 0.05;
  double time_reset_threshold_sec = 0.5;
  double acceleration_tau_sec = 0.05;
  double max_acceleration_rad_s2 = 100.0;
  std::string name = "gimbal_joint";
  std::deque<Sample> samples;

  void reset_derivative()
  {
    derivative_valid_ = false;
    filtered_acceleration_ = 0.0;
  }

  bool add(const sensor_msgs::msg::JointState & message, bool require_velocity)
  {
    const auto found = std::find(message.name.begin(), message.name.end(), name);
    const auto index = static_cast<std::size_t>(found - message.name.begin());
    if (found == message.name.end() || index >= message.position.size() ||
        (require_velocity && index >= message.velocity.size()) ||
        stamp_ns(message.header.stamp) < 0) {
      return false;
    }

    const double raw = message.position[index];
    const double velocity = index < message.velocity.size() ? message.velocity[index] : 0.0;
    const double time = seconds(message.header.stamp);
    if (!std::isfinite(raw) || !std::isfinite(velocity)) {
      return false;
    }

    bool reset = false;
    if (!samples.empty()) {
      const double dt = time - samples.back().time;
      if (dt <= 0.0 && -dt < time_reset_threshold_sec) {
        return false;
      }
      if (dt <= 0.0 || dt > max_sample_gap_sec) {
        samples.clear();
        raw_position_.reset();
        reset_derivative();
        reset = true;
      }
    }

    double position = raw;
    double acceleration = 0.0;
    if (!samples.empty()) {
      const double delta = raw - *raw_position_;
      position = samples.back().position + std::atan2(std::sin(delta), std::cos(delta));
      const double dt = time - samples.back().time;
      if (derivative_valid_) {
        const double raw_acc = std::clamp((velocity - samples.back().velocity) / dt,
                                          -max_acceleration_rad_s2, max_acceleration_rad_s2);
        const double alpha = (acceleration_tau_sec == 0.0) ? 1.0 : dt / (acceleration_tau_sec + dt);
        filtered_acceleration_ += alpha * (raw_acc - filtered_acceleration_);
        acceleration = filtered_acceleration_;
      }
    }

    raw_position_ = raw;
    derivative_valid_ = true;
    samples.push_back({time, position, velocity, acceleration});
    if (samples.size() > 4096) {
      samples.pop_front();
    }
    return reset;
  }

  std::optional<Sample> interpolate(double time, bool require_neighbor = true) const
  {
    if (samples.size() < 2 || time < samples.front().time - 1.0e-9 ||
        time > samples.back().time + 1.0e-9) {
      return std::nullopt;
    }

    auto after = std::lower_bound(
      samples.begin(), samples.end(), time,
      [](const Sample & sample, double target_time) { return sample.time < target_time; });
    if (after == samples.end()) {
      after = std::prev(samples.end());
    }

    if (std::abs(after->time - time) <= 1.0e-9) {
      if (!require_neighbor) {
        return *after;
      }
      const auto neighbor = (after == samples.begin()) ? std::next(after) : std::prev(after);
      return (std::abs(after->time - neighbor->time) <= max_sample_gap_sec)
               ? std::optional<Sample>(*after)
               : std::nullopt;
    }

    if (after == samples.begin()) {
      return std::nullopt;
    }
    const auto before = std::prev(after);
    const double dt = after->time - before->time;
    if (dt <= 0.0 || dt > max_sample_gap_sec) {
      return std::nullopt;
    }
    const double ratio = (time - before->time) / dt;
    return Sample{time, before->position + ratio * (after->position - before->position),
                  before->velocity + ratio * (after->velocity - before->velocity),
                  before->acceleration + ratio * (after->acceleration - before->acceleration)};
  }

private:
  std::optional<double> raw_position_;
  double filtered_acceleration_ = 0.0;
  bool derivative_valid_ = false;
};

}  // namespace xxu_bringup
