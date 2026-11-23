#pragma once

#include "rigid_mount_calibration_detail.hpp"
#include "xxu_bringup/calibration_point_extractor.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <filesystem>
#include <sstream>

namespace xxu_bringup::calibration_io {

inline constexpr std::size_t max_capture_samples = 100000;
inline constexpr std::size_t max_clouds = 240;
inline constexpr std::size_t max_points_per_cloud = 24000;

inline double stamp_time(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {return unmeasured;}
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.e-9;
}

inline Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & value)
{
  return {value.x, value.y, value.z};
}

inline ImuSample imu_sample(const sensor_msgs::msg::Imu & message)
{
  return {stamp_time(message.header.stamp), vector(message.linear_acceleration),
    vector(message.angular_velocity)};
}

inline OdomSample odom_sample(const nav_msgs::msg::Odometry & message)
{
  const auto & p = message.pose.pose.position;
  const auto & q = message.pose.pose.orientation;
  return {stamp_time(message.header.stamp), {p.x, p.y, p.z}, {q.w, q.x, q.y, q.z}};
}

inline void observe(std::set<std::string> & frames, const std::string & frame)
{
  // Two distinct names already prove the frame changed. Keep memory bounded.
  if (frames.size() < 2) {frames.insert(frame);}
}

inline void input_error(CalibrationSamples & data, const std::string & message)
{
  if (std::find(data.input_errors.begin(), data.input_errors.end(), message) ==
    data.input_errors.end())
  {
    data.input_errors.push_back(message);
  }
}

template<typename Sample>
bool append_sample(
  CalibrationSamples & data, std::vector<Sample> & buffer, Sample sample,
  std::size_t limit, const char * label, bool ordered = true)
{
  if (ordered && !buffer.empty() && sample.time <= buffer.back().time) {return false;}
  if (buffer.size() >= limit) {
    input_error(data, std::string(label) + " capture limit exceeded; recapture a shorter segment");
    return false;
  }
  buffer.push_back(std::move(sample));
  return true;
}

inline double parse_number(const std::string & text, const std::string & name)
{
  std::size_t length = 0;
  const double value = std::stod(text, &length);
  if (length != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(name + " must be a finite number");
  }
  return value;
}

inline int parse_count(const std::string & text, const std::string & name)
{
  std::size_t length = 0;
  const int value = std::stoi(text, &length);
  if (length != text.size()) {throw std::invalid_argument(name + " must be an integer");}
  return value;
}

struct OutputFiles
{
  std::string mount, backend, report;
};

inline auto output_paths(const OutputFiles & files)
{
  return std::array{files.mount, files.backend, files.report};
}

void validate_output_paths(const OutputFiles & files);
std::vector<std::string> write_outputs(
  const OutputFiles & files, const CalibrationSamples & data, const CalibrationConfig & config,
  const CalibrationResult & result, const std::optional<Segmentation> & segments = {},
  const std::optional<VerificationResult> & verification = {},
  const std::optional<RigidMount> & verified_mount = {});

inline std::string summarize(const std::vector<std::string> & reasons)
{
  std::ostringstream message;
  for (const auto & reason : reasons) {
    if (message.tellp() > 0) {message << " | ";}
    message << reason;
  }
  return message.str();
}

}  // namespace xxu_bringup::calibration_io
