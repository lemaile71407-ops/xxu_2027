#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/overhead_clearance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<Eigen::Vector3d> & points,
                                         std::uint32_t row_padding = 0)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lio_base_sensor";
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  cloud.row_step = cloud.width * cloud.point_step + row_padding;
  cloud.is_dense = false;

  const char * names[3] = {"x", "y", "z"};
  for (std::uint32_t idx = 0; idx < 3; ++idx) {
    sensor_msgs::msg::PointField field;
    field.name = names[idx];
    field.offset = idx * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }

  cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::uint8_t * ptr = cloud.data.data() + i * cloud.point_step;
    xxu_bringup::write_scalar<float>(ptr + 0, static_cast<float>(points[i].x()), false);
    xxu_bringup::write_scalar<float>(ptr + 4, static_cast<float>(points[i].y()), false);
    xxu_bringup::write_scalar<float>(ptr + 8, static_cast<float>(points[i].z()), false);
  }
  return cloud;
}

nav_msgs::msg::Odometry make_odom(double x, double y, std::int64_t stamp_ns)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  odom.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

std::vector<Eigen::Vector3d> make_ring_points(double x, double y, double z, std::size_t count)
{
  std::vector<Eigen::Vector3d> points;
  points.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    points.emplace_back(x + 0.02 * static_cast<double>(i), y, z);
  }
  return points;
}

TEST(OverheadClearanceTest, DetectsOverheadWithExtrinsicsAndBaseHeightFiltering)
{
  xxu_bringup::OverheadClearanceConfig config;
  config.base_height_m = 0.175;
  config.min_overhead_height_m = 0.22;
  config.max_overhead_height_m = 0.55;
  config.min_xy_radius_m = 0.20;
  config.detection_radius_m = 1.50;
  config.min_points = 4;
  config.trigger_frames = 1;

  xxu_bringup::OverheadClearanceDetector detector(config);

  // Rotate sensor +90 deg around Z and shift +0.105 m in Z relative to base_link.
  Eigen::Isometry3d base_from_sensor = Eigen::Isometry3d::Identity();
  base_from_sensor.translation() = Eigen::Vector3d(-0.20, 0.0, 0.105);
  base_from_sensor.linear() =
    Eigen::AngleAxisd(std::acos(-1.0) * 0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Sensor local points mapped into base_link:
  // sensor (0.0, -0.8, 0.02) -> base (+0.60, 0.0, 0.125) -> z_ground = 0.30 (HIT)
  // sensor (0.0, -0.8, -0.25) -> z_ground = 0.03 (below min_overhead_height_m -> IGNORED)
  // sensor (0.0, -0.8, 0.40) -> z_ground = 0.68 (above max_overhead_height_m -> IGNORED)
  // sensor (0.0, -0.20, 0.02) -> base (0.0, 0.0, 0.125) -> r=0.0 < min_xy_radius_m -> IGNORED
  // sensor (0.0, -2.50, 0.02) -> base (2.3, 0.0, 0.125) -> r=2.3 > detection_radius_m -> IGNORED
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 4; ++i) {
    points.emplace_back(0.0, -0.80 - 0.02 * i, 0.02);
  }
  points.emplace_back(0.0, -0.80, -0.25);
  points.emplace_back(0.0, -0.80, 0.40);
  points.emplace_back(0.0, -0.20, 0.02);
  points.emplace_back(0.0, -2.50, 0.02);
  points.emplace_back(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);

  const auto cloud = make_cloud(points, 8);
  const auto status = detector.evaluate_cloud(cloud, base_from_sensor, 10.0);

  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_TRUE(status.overhead_detected_now);
  EXPECT_FALSE(status.cloud_stale);
  EXPECT_EQ(status.hit_point_count, 4u);
  EXPECT_NEAR(status.min_hit_xy_distance_m, 0.60, 1.0e-5);
  EXPECT_NEAR(status.min_hit_height_m, 0.30, 1.0e-5);
  EXPECT_EQ(status.reason, "overhead_detected_stowed");
}

TEST(OverheadClearanceTest, RejectsSparseNoiseAndSingleFrameGlitch)
{
  xxu_bringup::OverheadClearanceConfig config;
  config.min_points = 5;
  config.trigger_frames = 2;
  xxu_bringup::OverheadClearanceDetector detector(config);

  const auto identity = Eigen::Isometry3d::Identity();

  // Only 4 points (< min_points=5) should not even increment consecutive hit frames.
  const auto sparse = make_cloud(make_ring_points(0.8, 0.0, 0.30, 4));
  auto status = detector.evaluate_cloud(sparse, identity, 1.0);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_FALSE(status.overhead_detected_now);
  EXPECT_EQ(status.consecutive_hit_frames, 0);

  // One dense frame debounces without latching.
  const auto dense = make_cloud(make_ring_points(0.8, 0.0, 0.30, 6));
  status = detector.evaluate_cloud(dense, identity, 1.1);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_TRUE(status.overhead_detected_now);
  EXPECT_EQ(status.consecutive_hit_frames, 1);
  EXPECT_EQ(status.reason, "overhead_debouncing");

  // A clean gap resets consecutive hit count.
  const auto empty = make_cloud({});
  status = detector.evaluate_cloud(empty, identity, 1.2);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_EQ(status.consecutive_hit_frames, 0);

  // Two consecutive dense frames latch stow_gimbal = true.
  status = detector.evaluate_cloud(dense, identity, 1.3);
  EXPECT_FALSE(status.stow_gimbal);
  status = detector.evaluate_cloud(dense, identity, 1.4);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_EQ(status.consecutive_hit_frames, 2);
}

TEST(OverheadClearanceTest, HysteresisLatchPreventsTunnelBlindZonePopupUntilTimeAndTravelMet)
{
  xxu_bringup::OverheadClearanceConfig config;
  config.min_points = 4;
  config.trigger_frames = 2;
  config.release_hold_sec = 1.5;
  config.release_travel_distance_m = 0.80;
  xxu_bringup::OverheadClearanceDetector detector(config);

  const auto identity = Eigen::Isometry3d::Identity();
  const auto ceiling_cloud = make_cloud(make_ring_points(0.8, 0.0, 0.27, 6));
  const auto blind_cloud = make_cloud({});

  detector.observe_odometry(make_odom(0.0, 0.0, 1000000000LL));
  detector.evaluate_cloud(ceiling_cloud, identity, 1.0);
  auto status = detector.evaluate_cloud(ceiling_cloud, identity, 1.1);
  ASSERT_TRUE(status.stow_gimbal);

  // Inside tunnel blind zone: cloud shows 0 overhead points while robot is stationary.
  // Even long past release_hold_sec (3.0 s clear elapsed), stationary robot stays stowed.
  status = detector.evaluate_cloud(blind_cloud, identity, 1.2);
  EXPECT_TRUE(status.stow_gimbal);
  status = detector.evaluate_cloud(blind_cloud, identity, 4.2);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_GE(status.clear_elapsed_sec, 3.0);
  EXPECT_NEAR(status.clear_travel_m, 0.0, 1.0e-9);
  EXPECT_EQ(status.reason, "latched_waiting_travel");

  // Re-detecting ceiling resets both clear elapsed time and accumulated travel.
  detector.observe_odometry(make_odom(0.5, 0.0, 2000000000LL));
  status = detector.evaluate_cloud(ceiling_cloud, identity, 4.3);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_DOUBLE_EQ(status.clear_elapsed_sec, 0.0);
  EXPECT_DOUBLE_EQ(status.clear_travel_m, 0.0);

  // Now clear starts at t = 4.4. Move 0.90 m quickly, before release_hold_sec elapses.
  status = detector.evaluate_cloud(blind_cloud, identity, 4.4);
  detector.observe_odometry(make_odom(1.4, 0.0, 3000000000LL));
  status = detector.evaluate_cloud(blind_cloud, identity, 5.2);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_GE(status.clear_travel_m, 0.89);
  EXPECT_LT(status.clear_elapsed_sec, 1.5);
  EXPECT_EQ(status.reason, "latched_waiting_hold");

  // Once hold duration is also satisfied, gimbal stow releases cleanly.
  status = detector.evaluate_cloud(blind_cloud, identity, 6.0);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_EQ(status.reason, "clear_released");
  EXPECT_FALSE(xxu_bringup::format_overhead_clearance_status(status).empty());
}

TEST(OverheadClearanceTest, CloudStalenessNeverReleasesLatchOrCountsTowardHold)
{
  xxu_bringup::OverheadClearanceConfig config;
  config.min_points = 4;
  config.trigger_frames = 1;
  config.release_hold_sec = 1.0;
  config.release_travel_distance_m = 0.0;
  config.cloud_timeout_sec = 0.5;
  xxu_bringup::OverheadClearanceDetector detector(config);

  const auto identity = Eigen::Isometry3d::Identity();
  const auto ceiling_cloud = make_cloud(make_ring_points(0.8, 0.0, 0.27, 5));
  const auto blind_cloud = make_cloud({});

  detector.evaluate_cloud(ceiling_cloud, identity, 1.0);
  auto status = detector.evaluate_cloud(blind_cloud, identity, 1.1);
  ASSERT_TRUE(status.stow_gimbal);

  // Blackout beyond cloud_timeout_sec resets clear countdown and keeps stow latched.
  status = detector.check_staleness(2.5);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_TRUE(status.cloud_stale);
  EXPECT_DOUBLE_EQ(status.clear_elapsed_sec, 0.0);
  EXPECT_EQ(status.reason, "cloud_stale_latched");

  // When clouds resume at t=3.0, clear_start resets to 3.0 rather than counting blackout time.
  status = detector.evaluate_cloud(blind_cloud, identity, 3.0);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_FALSE(status.cloud_stale);
  EXPECT_DOUBLE_EQ(status.clear_elapsed_sec, 0.0);

  // Releases once full release_hold_sec passes with active non-stale clouds.
  status = detector.evaluate_cloud(blind_cloud, identity, 4.1);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_EQ(status.reason, "clear_released");
}

TEST(OverheadClearanceTest, OdometryRejectsJumpsAndTimestampRegressions)
{
  xxu_bringup::OverheadClearanceConfig config;
  config.min_points = 4;
  config.trigger_frames = 1;
  config.release_hold_sec = 0.5;
  config.release_travel_distance_m = 0.8;
  config.max_odom_step_m = 0.5;
  xxu_bringup::OverheadClearanceDetector detector(config);

  const auto identity = Eigen::Isometry3d::Identity();
  const auto ceiling_cloud = make_cloud(make_ring_points(0.8, 0.0, 0.27, 5));
  const auto blind_cloud = make_cloud({});

  detector.observe_odometry(make_odom(0.0, 0.0, 2000000000LL));
  detector.evaluate_cloud(ceiling_cloud, identity, 1.0);
  detector.evaluate_cloud(blind_cloud, identity, 1.1);

  // Teleport jump > max_odom_step_m resets baseline without adding clear_travel_m.
  detector.observe_odometry(make_odom(5.0, 0.0, 3000000000LL));
  // Backward non-zero timestamp regression also resets baseline without adding travel.
  detector.observe_odometry(make_odom(5.3, 0.0, 2500000000LL));

  auto status = detector.evaluate_cloud(blind_cloud, identity, 2.0);
  EXPECT_TRUE(status.stow_gimbal);
  EXPECT_DOUBLE_EQ(status.clear_travel_m, 0.0);

  // Valid small steps accumulate and release once >= 0.8 m.
  detector.observe_odometry(make_odom(5.7, 0.0, 3100000000LL));
  detector.observe_odometry(make_odom(6.15, 0.0, 3200000000LL));
  status = detector.evaluate_cloud(blind_cloud, identity, 2.2);
  EXPECT_FALSE(status.stow_gimbal);
  EXPECT_EQ(status.reason, "clear_released");
}

TEST(OverheadClearanceTest, RejectsInvalidConfigurationAndCloudInputs)
{
  xxu_bringup::OverheadClearanceConfig invalid;
  invalid.max_overhead_height_m = invalid.min_overhead_height_m;
  EXPECT_THROW(xxu_bringup::OverheadClearanceDetector{invalid}, std::invalid_argument);

  invalid = xxu_bringup::OverheadClearanceConfig{};
  invalid.detection_radius_m = invalid.min_xy_radius_m;
  EXPECT_THROW(xxu_bringup::OverheadClearanceDetector{invalid}, std::invalid_argument);

  invalid = xxu_bringup::OverheadClearanceConfig{};
  invalid.min_points = 0;
  EXPECT_THROW(xxu_bringup::OverheadClearanceDetector{invalid}, std::invalid_argument);

  invalid = xxu_bringup::OverheadClearanceConfig{};
  invalid.cloud_timeout_sec = 0.0;
  EXPECT_THROW(xxu_bringup::OverheadClearanceDetector{invalid}, std::invalid_argument);

  xxu_bringup::OverheadClearanceDetector detector(xxu_bringup::OverheadClearanceConfig{});
  auto broken_cloud = make_cloud(make_ring_points(0.5, 0.0, 0.3, 2));
  broken_cloud.data.pop_back();
  EXPECT_THROW(detector.evaluate_cloud(broken_cloud, Eigen::Isometry3d::Identity(), 1.0),
               std::invalid_argument);

  Eigen::Isometry3d nonfinite = Eigen::Isometry3d::Identity();
  nonfinite.translation().z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(detector.evaluate_cloud(make_cloud({}), nonfinite, 1.0), std::invalid_argument);
}

}  // namespace
