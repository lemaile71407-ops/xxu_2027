#include "rigid_mount_calibration_detail.hpp"

namespace xxu_bringup {
using namespace calibration_detail;
namespace {

enum class Motion {unknown, stationary, straight, spin};

struct MotionRun
{
  Motion kind;
  std::size_t first, last;
};

std::vector<MotionRun> motion_runs(
  const std::vector<OdomSample> & odom, const CalibrationConfig & c)
{
  std::vector<MotionRun> runs;
  for (std::size_t i = 1; i < odom.size(); ++i) {
    const auto & first = odom[i - 1];
    const auto & last = odom[i];
    const double dt = last.time - first.time;
    if (dt > c.auto_max_sample_gap) {continue;}
    const double distance = (last.position - first.position).head<2>().norm();
    const Eigen::Matrix3d delta = odom_rotation(last) * odom_rotation(first).transpose();
    const double yaw_delta = std::abs(std::atan2(delta(1, 0), delta(0, 0)));
    const double speed = distance / dt;
    const double yaw_rate = yaw_delta / dt;
    Motion kind = Motion::unknown;
    if (yaw_rate <= c.auto_straight_max_yaw_rate) {
      if (speed <= c.auto_stationary_speed) {kind = Motion::stationary;}
      if (speed >= c.auto_straight_min_speed) {kind = Motion::straight;}
    } else if (yaw_rate >= c.auto_spin_min_yaw_rate &&
      distance <= c.auto_spin_max_translation_per_rad * yaw_delta)
    {
      kind = Motion::spin;
    }
    if (kind == Motion::unknown) {continue;}
    if (!runs.empty() && runs.back().kind == kind && runs.back().last == i - 1) {
      runs.back().last = i;
    } else {
      runs.push_back({kind, i - 1, i});
    }
  }
  return runs;
}

std::vector<ImuSample> imu_window(const std::vector<ImuSample> & imu, TimeWindow time)
{
  const auto first = std::lower_bound(imu.begin(), imu.end(), time.start,
    [](const auto & sample, double stamp) {return sample.time < stamp;});
  const auto end = std::upper_bound(first, imu.end(), time.end,
    [](double stamp, const auto & sample) {return stamp < sample.time;});
  return {first, end};
}

bool enough_clouds(const std::vector<double> & stamps, TimeWindow time, int minimum)
{
  const auto first = std::lower_bound(stamps.begin(), stamps.end(), time.start);
  return std::upper_bound(first, stamps.end(), time.end) - first >= minimum;
}

bool has_imu_gap(const std::vector<ImuSample> & imu, double maximum_gap)
{
  return std::adjacent_find(imu.begin(), imu.end(), [maximum_gap](const auto & a, const auto & b) {
      return b.time - a.time > maximum_gap;
    }) != imu.end();
}

std::optional<TimeWindow> find_stationary(
  const std::vector<MotionRun> & runs, const std::vector<OdomSample> & odom,
  const std::vector<ImuSample> & imu, const std::vector<double> & clouds, const CalibrationConfig & c)
{
  std::optional<TimeWindow> best_short;
  double best_noise = std::numeric_limits<double>::infinity();
  const double preferred = std::max(2.0, c.min_stationary_duration);
  const double length = std::max(3.0, preferred + 0.2);
  constexpr double step = 0.25;
  for (const auto & run : runs) {
    if (run.kind != Motion::stationary) {continue;}
    const double run_end = odom[run.last].time;
    for (double start = odom[run.first].time; start + c.min_stationary_duration <= run_end;
      start += step)
    {
      const double min_end = start + c.min_stationary_duration;
      // Odometry can still report a stop after the IMU has registered motion.
      for (double end = std::min(start + length, run_end); ; end = std::max(min_end, end - step)) {
        const TimeWindow window{start, end};
        if (!enough_clouds(clouds, window, c.min_ground_clouds)) {break;}
        const auto selected = imu_window(imu, window);
        if (!has_imu_gap(selected, c.auto_max_sample_gap)) {
          const auto quality = stationary_imu(selected, c);
          if (quality.passed()) {
            if (!enough_clouds(clouds, TimeWindow{selected.front().time, selected.back().time},
              c.min_ground_clouds))
            {
              break;
            }
            if (quality.duration >= preferred) {return window;}
            const double noise = quality.gyro_bias.norm() + quality.gyro_norm_std +
              quality.accel_norm_std / 9.81;
            if (noise < best_noise) {
              best_noise = noise;
              best_short = window;
            }
          }
        }
        if (end == min_end) {break;}
      }
    }
  }
  return best_short;
}

bool monotonic(const std::vector<OdomSample> & odom)
{
  const Eigen::Vector2d displacement = (odom.back().position - odom.front().position).head<2>();
  const Eigen::Vector2d axis = displacement.normalized();
  double reverse_distance = 0.0;
  for (std::size_t i = 1; i < odom.size(); ++i) {
    const Eigen::Vector2d step = (odom[i].position - odom[i - 1].position).head<2>();
    reverse_distance += std::max(0.0, -axis.dot(step));
  }
  return reverse_distance <= 0.03 * displacement.norm();
}

double spin_span(const std::vector<OdomSample> & odom, const Eigen::Matrix3d & level)
{
  double unwrapped = 0.0, low = 0.0, high = 0.0;
  const auto yaw = [&level](const OdomSample & sample) {
      const Eigen::Matrix3d leveled = odom_rotation(sample) * level.transpose();
      return std::atan2(leveled(1, 0), leveled(0, 0));
    };
  double previous = yaw(odom.front());
  for (const auto & sample : odom) {
    const double next = yaw(sample);
    unwrapped += wrap(next - previous);
    low = std::min(low, unwrapped);
    high = std::max(high, unwrapped);
    previous = next;
  }
  return degrees(high - low);
}

}  // namespace

Segmentation auto_segment_recording(
  const std::vector<ImuSample> & input_imu, const std::vector<OdomSample> & input_odom,
  const std::vector<double> & cloud_times, const CalibrationConfig & c)
{
  validate_calibration_config(c);
  Segmentation result;
  const auto imu = usable(input_imu, valid_imu);
  const auto odom = usable(input_odom, valid_odom);
  std::vector<double> clouds;
  std::copy_if(cloud_times.begin(), cloud_times.end(), std::back_inserter(clouds), valid_time);
  std::sort(clouds.begin(), clouds.end());
  clouds.erase(std::unique(clouds.begin(), clouds.end()), clouds.end());

  const auto runs = motion_runs(odom, c);
  result.stationary = find_stationary(runs, odom, imu, clouds, c);
  Eigen::Matrix3d level = Eigen::Matrix3d::Identity();
  if (result.stationary) {
    level = *stationary_imu(imu_window(imu, *result.stationary), c).leveling;
  } else {
    result.reasons.emplace_back("stationary window missing: require low odometry motion, stable IMU and LiDAR clouds");
  }

  double longest_straight = 0.0, widest_spin = 0.0;
  double longest_tilted_straight = 0.0;
  std::optional<TimeWindow> tilted_straight;
  for (const auto & run : runs) {
    const TimeWindow window{odom[run.first].time, odom[run.last].time};
    const std::vector<OdomSample> selected(odom.begin() + run.first, odom.begin() + run.last + 1);
    if (run.kind == Motion::straight) {
      const auto quality = solve_straight(selected, level, c);
      if (quality.passed() && quality.distance > longest_straight && monotonic(selected)) {
        result.straight = window;
        longest_straight = quality.distance;
      } else if (quality.reasons.size() == 1 && quality.reasons.front() == odom_tilt_reason &&
        quality.distance > longest_tilted_straight && monotonic(selected))
      {
        tilted_straight = window;
        longest_tilted_straight = quality.distance;
      }
    }
    if (run.kind == Motion::spin && selected.size() >= static_cast<std::size_t>(c.min_spin_samples)) {
      const double span = spin_span(selected, level);
      if (span + 1.e-8 >= c.min_spin_yaw_span_deg && span > widest_spin) {
        result.spin = window;
        widest_spin = span;
      }
    }
  }
  // Keep an otherwise usable run so the final solve can report the gravity mismatch.
  if (!result.straight) {result.straight = tilted_straight;}
  if (!result.straight) {
    result.reasons.emplace_back("straight window missing: require sustained monotonic travel, distance and low yaw rate");
  }
  if (!result.spin) {
    result.reasons.emplace_back("spin window missing: require sustained yaw, low translation per radian and enough yaw span");
  }
  return result;
}

}  // namespace xxu_bringup
