#include "xxu_bringup/calibration_point_extractor.hpp"
#include "xxu_bringup/odometry_conversion.hpp"
#include "xxu_bringup/sensor_math.hpp"
#include "../src/calibration/rigid_mount_calibration_io.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <unistd.h>

namespace xxu_bringup {
namespace {

constexpr double radians(double deg) {return deg * calibration_pi / 180.0;}
const Eigen::Vector3d lever(0.22, -0.11, 0.34);
const Eigen::Matrix3d base_from_imu = rpy_rotation({radians(12), radians(-8), radians(135)});
const double expected_psi = [] {
    const Eigen::Matrix3d level = leveling_rotation_from_up(base_from_imu.row(2).transpose());
    return rotation_to_rpy(base_from_imu * level.transpose()).z();
  }();

Eigen::Matrix3d yaw(double angle) {return rpy_rotation({0, 0, angle});}

OdomSample odom(double time, const Eigen::Vector3d & position, const Eigen::Matrix3d & orientation)
{
  return {time, position + orientation * lever, Eigen::Quaterniond(orientation * base_from_imu)};
}

std::vector<ImuSample> stationary_imu(double scale = 9.81)
{
  std::vector<ImuSample> result;
  for (int i = 0; i < 300; ++i) {
    const double jitter = std::sin(i) * 0.004;
    result.push_back({i * .01, base_from_imu.row(2).transpose() * (9.81 + jitter) / scale,
      Eigen::Vector3d(.006 + jitter, .004, -.001)});
  }
  return result;
}

std::vector<CloudSample> clouds(const CalibrationConfig & config, double slope = 0)
{
  std::mt19937 generator(7265);
  std::uniform_real_distribution<double> horizontal(-2.7, 2.7);
  std::normal_distribution<double> noise(0.0, .002);
  std::vector<CloudSample> result;
  const Eigen::Isometry3d lidar_from_imu = config.imu_from_lidar.inverse();
  for (int i = 0; i < 8; ++i) {
    CloudSample cloud{i * .3, {}};
    const auto add = [&](double x, double y, double z) {
        const Eigen::Vector3d base_point(x, y, z + noise(generator));
        cloud.points.push_back(lidar_from_imu * (base_from_imu.transpose() * (base_point - lever)));
      };
    for (int j = 0; j < 120; ++j) {
      const double x = horizontal(generator);
      add(x, horizontal(generator), -config.base_height_above_ground + slope * x);
      add(horizontal(generator), horizontal(generator), 1.9);
    }
    for (int j = 0; j < 230; ++j) {
      add(.70 + .17 * horizontal(generator), .17 * horizontal(generator), .20);
    }
    for (int j = 0; j < 30; ++j) {
      add(horizontal(generator), horizontal(generator), -.8 + .15 * horizontal(generator));
    }
    result.push_back(std::move(cloud));
  }
  return result;
}

std::vector<OdomSample> straight(int direction = 1, double span = 0, double length = 3.0)
{
  std::vector<OdomSample> result;
  const double heading = .63;
  for (int i = 0; i <= 100; ++i) {
    const double alpha = i / 100.0;
    const Eigen::Vector3d position = yaw(heading) * Eigen::Vector3d(direction * length * alpha, 0, 0);
    result.push_back(odom(i * .04, position, yaw(heading + span * (alpha - .5))));
  }
  return result;
}

std::vector<OdomSample> spin(double span = 2 * calibration_pi, bool tilt = true)
{
  std::vector<OdomSample> result;
  const Eigen::Vector3d center(3.6, -4.8, 0.1);
  for (int i = 0; i <= 180; ++i) {
    const double alpha = i / 180.0;
    const auto r = rpy_rotation({
      tilt ? .025 * std::sin(alpha * 6) : 0., tilt ? .015 * std::cos(alpha * 7) : 0.,
      -.51 + span * alpha});
    result.push_back(odom(i * .04, center, r));
  }
  return result;
}

CalibrationSamples full(const CalibrationConfig & config)
{
  CalibrationSamples result;
  result.imu = stationary_imu();
  result.clouds = clouds(config);
  result.straight = straight();
  result.spin = spin();
  return result;
}

class MountFile
{
public:
  MountFile()
  {
    std::string temp = (std::filesystem::temp_directory_path() / "xxu-mount-XXXXXX").string();
    const int fd = mkstemp(temp.data());
    if (fd < 0) {throw std::runtime_error("cannot create temporary mount");}
    close(fd);
    path = temp;
  }
  ~MountFile() {std::filesystem::remove(path);}
  const std::string & filename() const {return path;}
  RigidMount load(const std::string & yaml) const
  {
    std::ofstream(path) << yaml;
    return load_rigid_mount_cpp(path);
  }

private:
  std::string path;
};

nav_msgs::msg::Odometry pose_message(const Eigen::Isometry3d & pose)
{
  nav_msgs::msg::Odometry message;
  message.pose.pose.position.x = pose.translation().x();
  message.pose.pose.position.y = pose.translation().y();
  message.pose.pose.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  message.pose.pose.orientation.x = q.x();
  message.pose.pose.orientation.y = q.y();
  message.pose.pose.orientation.z = q.z();
  message.pose.pose.orientation.w = q.w();
  return message;
}

TEST(RigidMountCalibration, FixedAxisRpyRoundTripsIncludingGimbalLock)
{
  for (const double pitch : {0., -0.45, -calibration_pi / 2, calibration_pi / 2,
    calibration_pi / 2 - 3.e-7}) {
    const auto r = rpy_rotation({.72, pitch, -1.3});
    const Eigen::Vector3d angles = rotation_to_rpy(r);
    EXPECT_TRUE(angles.allFinite());
    EXPECT_TRUE(rpy_rotation(angles).isApprox(r, 1.e-12));
    if (std::abs(pitch) == calibration_pi / 2) {EXPECT_DOUBLE_EQ(angles.z(), 0.0);}
  }
  EXPECT_TRUE(rotation_to_rpy(Eigen::Matrix3d::Identity()).isZero());
}

TEST(RigidMountCalibration, LevelingHasNoSingularity)
{
  const std::array<Eigen::Vector3d, 7> cases{{
    {0., 0., 1.}, {0., 0., -1.}, {1., 0., 0.}, {0., -1., 0.},
    {.23, -.61, .3}, {.12, .33, -.84}, {1.e-13, 0., -1.}}};
  for (const auto & up : cases) {
    const auto r = leveling_rotation_from_up(up);
    EXPECT_TRUE((r * up.normalized()).isApprox(Eigen::Vector3d::UnitZ(), 1.e-12));
    EXPECT_TRUE((r * r.transpose()).isIdentity(1.e-12));
    EXPECT_NEAR(r.determinant(), 1, 1.e-12);
  }
  EXPECT_TRUE(leveling_rotation_from_up(Eigen::Vector3d::UnitZ()).isIdentity());
  EXPECT_THROW(leveling_rotation_from_up(Eigen::Vector3d::Zero()), std::invalid_argument);
}

TEST(RigidMountCalibration, PointExtractionHonorsEndiannessAndOrganizedPadding)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 2;
  cloud.width = 2;
  cloud.point_step = 16;
  cloud.row_step = 40;
  cloud.is_bigendian = true;
  cloud.data.resize(80, 0);
  for (const auto * name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = cloud.fields.size() * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  for (const int offset : {0, 16, 40, 56}) {
    write_scalar<float>(cloud.data.data() + offset, offset + .25F, true);
    write_scalar<float>(cloud.data.data() + offset + 4, 1.5F, true);
    write_scalar<float>(cloud.data.data() + offset + 8, -.5F, true);
  }
  write_scalar<float>(cloud.data.data() + 16, std::numeric_limits<float>::quiet_NaN(), true);
  const auto points = extract_calibration_points(cloud);
  ASSERT_EQ(points.size(), 3U);
  EXPECT_EQ(points[1], Eigen::Vector3d(40.25, 1.5, -.5));
  cloud.row_step = 31;
  EXPECT_THROW(extract_calibration_points(cloud), std::invalid_argument);
}

TEST(RigidMountCalibration, StationarySelectsFloorBelowDenserChassisAndCeiling)
{
  CalibrationConfig c;
  c.base_height_above_ground = .10;
  c.imu_from_lidar.linear() = rpy_rotation({.05, -.02, .03});
  for (const double scale : {9.81, 1.0}) {
    const auto result = solve_stationary(stationary_imu(scale), clouds(c), c);
    ASSERT_TRUE(result.passed()) << (result.reasons.empty() ? "" : result.reasons.front());
    ASSERT_TRUE(result.scale && result.tz && result.leveling);
    EXPECT_DOUBLE_EQ(*result.scale, scale);
    EXPECT_NEAR(*result.tz, lever.z(), .001);
    EXPECT_NEAR(result.floor_offset, lever.z() + c.base_height_above_ground, .001);
    EXPECT_GT(result.inlier_count, 800U);
    EXPECT_LT(result.floor_rms, .003);
    EXPECT_TRUE((yaw(expected_psi) * *result.leveling).isApprox(base_from_imu, 1.e-12));
    EXPECT_NEAR(result.gyro_bias.x(), .006, .0001);
    EXPECT_GT(result.accel_variance.sum(), 0);
  }
}

TEST(RigidMountCalibration, RejectsUnisolatedProductionOdomTopicsAndCompensatedTilt)
{
  CalibrationConfig c;
  EXPECT_EQ(c.odom_topic, "/calibration/lidar_odometry");
  for (const auto * topic : {"/lidar_odometry", "/odometry", "/frontend/odometry", "/odom"}) {
    SCOPED_TRACE(topic);
    auto shared = c;
    shared.odom_topic = topic;
    EXPECT_THROW(validate_calibration_config(shared), std::invalid_argument);
    shared.allow_unisolated_odom_topic = true;
    EXPECT_NO_THROW(validate_calibration_config(shared));
  }

  c.confirm = true;
  auto samples = full(c);
  samples.spin = spin(2 * calibration_pi, false);
  const Eigen::Quaterniond imu_from_base(base_from_imu.transpose());
  for (auto * motion : {&samples.straight, &samples.spin}) {
    for (auto & sample : *motion) {sample.orientation *= imu_from_base;}
  }
  const auto stationary = solve_stationary(samples.imu, samples.clouds, c);
  ASSERT_TRUE(stationary.passed());
  ASSERT_TRUE(stationary.leveling && stationary.tz);
  const std::string reason =
    "odometry orientation disagrees with stationary gravity leveling (check raw vs compensated odometry)";
  const auto straight_result = solve_straight(samples.straight, *stationary.leveling, c);
  EXPECT_FALSE(straight_result.passed());
  EXPECT_NE(std::find(straight_result.reasons.begin(), straight_result.reasons.end(), reason),
    straight_result.reasons.end());
  const auto spin_result = solve_spin(samples.spin, base_from_imu, *stationary.tz, c);
  EXPECT_FALSE(spin_result.passed());
  EXPECT_NE(std::find(spin_result.reasons.begin(), spin_result.reasons.end(), reason),
    spin_result.reasons.end());

  const auto rejected = solve_all(samples, c);
  EXPECT_FALSE(rejected.confirmed);
  for (const auto * stage : {"straight", "spin"}) {
    const auto stage_reason = std::string(stage) + ": " + reason;
    EXPECT_NE(std::find(rejected.reasons.begin(), rejected.reasons.end(), stage_reason),
      rejected.reasons.end());
  }
}

TEST(RigidMountCalibration, RejectsStationaryImuGapInSolveAndVerify)
{
  CalibrationConfig c;
  c.confirm = true;
  auto samples = full(c);
  const auto calibrated = solve_all(samples, c);
  ASSERT_TRUE(calibrated.confirmed);
  const auto mount = MountFile().load(render_rigid_mount_yaml(calibrated, c));

  samples.imu.resize(102);
  for (std::size_t i = 0; i < samples.imu.size(); ++i) {
    samples.imu[i].time = i <= 50 ? .01 * i : 2.5 + .01 * (i - 51);
  }
  auto shifted = c;
  shifted.base_height_above_ground -= .20;
  samples.clouds = clouds(shifted);
  for (std::size_t i = 0; i < samples.clouds.size(); ++i) {
    samples.clouds[i].time = 1.0 + .3 * i / (samples.clouds.size() - 1);
  }

  const auto stationary = solve_stationary(samples.imu, samples.clouds, c);
  c.manual_psi_deg = 135.;
  c.manual_tx = lever.x();
  c.manual_ty = lever.y();
  const auto rejected = solve_all(samples, c);
  const auto verification = verify_calibration(mount, samples, c);
  const std::string reason = "stationary IMU stream contains a sample gap";
  for (const auto * stage : {&stationary, &rejected.stationary, &verification.stationary}) {
    EXPECT_FALSE(stage->passed());
    EXPECT_FALSE(stage->leveling);
    EXPECT_FALSE(stage->tz);
    EXPECT_DOUBLE_EQ(stage->max_imu_gap, 2.0);
    EXPECT_NE(std::find(stage->reasons.begin(), stage->reasons.end(), reason), stage->reasons.end());
  }
  EXPECT_FALSE(rejected.confirmed);
  EXPECT_FALSE(rejected.tz);
  EXPECT_FALSE(verification.passed());
}

TEST(RigidMountCalibration, RejectsMotionOdometryGapsInSolveAndVerify)
{
  CalibrationConfig c;
  c.confirm = true;
  auto samples = full(c);
  const auto calibrated = solve_all(samples, c);
  ASSERT_TRUE(calibrated.confirmed);
  const auto mount = MountFile().load(render_rigid_mount_yaml(calibrated, c));

  for (std::size_t i = 50; i < samples.straight.size(); ++i) {
    samples.straight[i].time += 3.8 - .04;
  }
  samples.spin.clear();
  const Eigen::Vector3d center(3.6, -4.8, 0.1);
  for (int fragment = 0; fragment < 2; ++fragment) {
    for (int i = 0; i <= 30; ++i) {
      samples.spin.push_back(odom(
        fragment * 5.0 + i * .04, center, yaw(radians(fragment * 179.0 + i * 10.0 / 30.0))));
    }
  }

  const auto stationary = solve_stationary(samples.imu, samples.clouds, c);
  ASSERT_TRUE(stationary.passed());
  const auto straight_result = solve_straight(samples.straight, *stationary.leveling, c);
  const auto spin_result = solve_spin(samples.spin, base_from_imu, *stationary.tz, c);
  const auto rejected = solve_all(samples, c);
  const auto verification = verify_calibration(mount, samples, c);
  const std::vector<std::string> straight_reasons{"straight odometry stream contains a sample gap"};
  const std::vector<std::string> spin_reasons{"spin odometry stream contains a sample gap"};
  for (const auto * stage : {&straight_result, &rejected.straight, &verification.straight}) {
    EXPECT_FALSE(stage->passed());
    EXPECT_EQ(stage->reasons, straight_reasons);
    EXPECT_GT(stage->max_sample_gap, c.auto_max_sample_gap);
    EXPECT_NEAR(stage->max_sample_gap, 3.8, 1.e-9);
  }
  for (const auto * stage : {&spin_result, &rejected.spin, &verification.spin}) {
    EXPECT_FALSE(stage->passed());
    EXPECT_EQ(stage->reasons, spin_reasons);
    EXPECT_GT(stage->max_sample_gap, c.auto_max_sample_gap);
    EXPECT_NEAR(stage->max_sample_gap, 3.8, 1.e-9);
    EXPECT_GE(stage->sample_count, static_cast<std::size_t>(c.min_spin_samples));
    EXPECT_GT(stage->yaw_span_deg, c.min_spin_yaw_span_deg);
  }
  EXPECT_EQ(rejected.psi, std::nullopt);
  EXPECT_EQ(rejected.tx, std::nullopt);
  EXPECT_EQ(rejected.ty, std::nullopt);
  EXPECT_FALSE(rejected.confirmed);
  EXPECT_FALSE(verification.passed());
}

TEST(RigidMountCalibration, RejectsStationaryCloudsOutsideImuInterval)
{
  CalibrationConfig c;
  c.confirm = true;
  auto samples = full(c);
  for (auto & sample : samples.imu) {sample.time += 3.;}
  for (auto & cloud : samples.clouds) {cloud.time += 3.;}
  const auto calibrated = solve_all(samples, c);
  ASSERT_TRUE(calibrated.confirmed);
  const auto mount = MountFile().load(render_rigid_mount_yaml(calibrated, c));

  auto stale_config = c;
  stale_config.base_height_above_ground += .20;
  auto stale = samples;
  stale.clouds = clouds(stale_config);
  const auto stationary = solve_stationary(stale.imu, stale.clouds, stale_config);
  EXPECT_FALSE(stationary.passed());
  EXPECT_FALSE(stationary.tz);
  EXPECT_EQ(stationary.cloud_count, 0U);
  ASSERT_FALSE(stationary.reasons.empty());
  EXPECT_EQ(stationary.reasons.front(),
    "too few valid stationary LiDAR clouds within the stationary IMU interval");

  const auto rejected = solve_all(stale, stale_config);
  EXPECT_FALSE(rejected.confirmed);
  EXPECT_FALSE(rejected.tz);
  EXPECT_FALSE(rejected.stationary.passed());
  EXPECT_EQ(rejected.stationary.cloud_count, 0U);
  const auto stale_check = verify_calibration(mount, stale, stale_config);
  EXPECT_FALSE(stale_check.passed());
  EXPECT_FALSE(stale_check.stationary.passed());
  EXPECT_FALSE(stale_check.stationary.tz);
  EXPECT_EQ(stale_check.stationary.cloud_count, 0U);

  auto mixed = stale;
  mixed.clouds.insert(mixed.clouds.end(), samples.clouds.begin(), samples.clouds.end());
  for (auto cloud : stale.clouds) {
    cloud.time += 6.;
    mixed.clouds.push_back(std::move(cloud));
  }
  const auto solved = solve_stationary(mixed.imu, mixed.clouds, c);
  ASSERT_TRUE(solved.passed());
  EXPECT_EQ(solved.cloud_count, 8U);
  ASSERT_TRUE(solved.tz);
  EXPECT_NEAR(*solved.tz, lever.z(), .005);
  const auto mixed_check = verify_calibration(mount, mixed, c);
  ASSERT_TRUE(mixed_check.passed());
  EXPECT_EQ(mixed_check.stationary.cloud_count, 8U);
  EXPECT_NEAR(mount.base_from_sensor.translation().z() + mixed_check.stationary.floor_offset -
    c.base_height_above_ground, lever.z(), .005);
}

TEST(RigidMountCalibration, StationaryRejectsInvalidScaleAndMotion)
{
  CalibrationConfig c;
  auto imu = stationary_imu();
  const auto floor = clouds(c);
  c.imu_acceleration_scale = 1.0;
  EXPECT_FALSE(solve_stationary(imu, floor, c).passed());
  EXPECT_FALSE(detect_imu_acceleration_scale(3.0));
  EXPECT_FALSE(detect_imu_acceleration_scale(unmeasured));
  c.imu_acceleration_scale = 0;
  imu[10].gyro.x() += .14;
  EXPECT_FALSE(solve_stationary(imu, floor, c).passed());
  imu = stationary_imu();
  imu[10].acceleration.x() += .3;
  EXPECT_FALSE(solve_stationary(imu, floor, c).passed());
  imu = stationary_imu();
  imu[11].acceleration.x() = unmeasured;
  imu[12].time = -1;
  const auto clean = solve_stationary(imu, floor, c);
  EXPECT_TRUE(clean.passed());
  EXPECT_EQ(clean.imu_count, 298U);
}

TEST(RigidMountCalibration, StationaryRejectsFloorNormalMismatch)
{
  CalibrationConfig c;
  const auto result = solve_stationary(stationary_imu(), clouds(c, .085), c);
  EXPECT_FALSE(result.passed());
  EXPECT_GT(result.gravity_ground_angle_deg, 4.0);
  EXPECT_FALSE(result.tz);
}

TEST(RigidMountCalibration, StraightEstimatesForwardAndReverseYaw)
{
  CalibrationConfig c;
  const auto level = leveling_rotation_from_up(base_from_imu.row(2).transpose());
  for (const int direction : {1, -1}) {
    c.straight_direction = direction;
    auto samples = straight(direction);
    samples[25].orientation.coeffs() *= -1;
    const auto result = solve_straight(samples, level, c);
    ASSERT_TRUE(result.passed());
    ASSERT_TRUE(result.psi);
    EXPECT_NEAR(*result.psi, expected_psi, 1.e-10);
    EXPECT_NEAR(result.distance, 3.0, 1.e-10);
  }
}

TEST(RigidMountCalibration, StraightRejectsShortCurvedAndSteepTrajectories)
{
  CalibrationConfig c;
  const auto level = leveling_rotation_from_up(base_from_imu.row(2).transpose());
  EXPECT_FALSE(solve_straight(straight(1, 0, .8), level, c).passed());
  EXPECT_FALSE(solve_straight(straight(1, radians(15)), level, c).passed());
  auto samples = straight();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    samples[i].position.z() += .5 * i / (samples.size() - 1);
  }
  EXPECT_FALSE(solve_straight(samples, level, c).passed());
}

TEST(RigidMountCalibration, SpinHandlesTiltHeightAndOutliers)
{
  CalibrationConfig c;
  auto samples = spin();
  samples[50].position += Eigen::Vector3d(.17, -.11, .02);
  const auto result = solve_spin(samples, base_from_imu, lever.z(), c);
  ASSERT_TRUE(result.passed());
  ASSERT_TRUE(result.tx && result.ty);
  EXPECT_NEAR(*result.tx, lever.x(), .0005);
  EXPECT_NEAR(*result.ty, lever.y(), .0005);
  EXPECT_NEAR(result.yaw_span_deg, 360, 1.e-8);
  EXPECT_EQ(result.rank, 4);
  EXPECT_GT(result.std_tx, 0.0);
  EXPECT_NEAR(result.std_tx * result.std_tx, result.covariance(2, 2), 1.e-14);
  EXPECT_LT(result.residual_rms, .02);
  EXPECT_GT(result.raw_radius_rms, .20);
  EXPECT_FALSE(solve_spin(spin(radians(120)), base_from_imu, lever.z(), c).passed());
  EXPECT_TRUE(solve_spin(spin(radians(180)), base_from_imu, lever.z(), c).passed());
}

TEST(RigidMountCalibration, JointRefinementCorrectsYawFromLeverMotion)
{
  CalibrationConfig c;
  c.confirm = true;
  auto data = full(c);
  data.straight = straight(1, radians(8));
  const auto stationary = solve_stationary(data.imu, data.clouds, c);
  ASSERT_TRUE(stationary.leveling);
  const auto first = solve_straight(data.straight, *stationary.leveling, c);
  ASSERT_TRUE(first.passed() && first.psi);
  const auto result = solve_all(data, c);
  ASSERT_TRUE(result.confirmed) << render_report_yaml(data, c, result);
  ASSERT_EQ(result.refinement.size(), 3U);
  ASSERT_TRUE(result.psi && result.tx && result.ty && result.tz);
  EXPECT_GT(std::abs(*first.psi - expected_psi), radians(.3));
  EXPECT_NEAR(*result.psi, expected_psi, radians(.005));
  EXPECT_NEAR(*result.tx, lever.x(), .0001);
  EXPECT_NEAR(*result.ty, lever.y(), .0001);
  EXPECT_NEAR(*result.tz, lever.z(), .001);
}

TEST(RigidMountCalibration, VirtualModesProduceSameLidarOdometryAndBackendBody)
{
  CalibrationConfig c;
  c.confirm = true;
  c.imu_from_lidar.linear() = rpy_rotation({.08, .04, -.11});
  const auto data = full(c);
  Eigen::Matrix4d first_lidar = Eigen::Matrix4d::Zero();
  const MountFile file;
  Eigen::Isometry3d world_base = Eigen::Isometry3d::Identity();
  world_base.linear() = rpy_rotation({.09, -.17, 1.04});
  world_base.translation() = Eigen::Vector3d(7.1, -.9, 1.2);
  nav_msgs::msg::Odometry first_odom;
  for (const auto * mode : {"base_aligned", "imu_aligned"}) {
    c.sensor_frame_mode = mode;
    const auto result = solve_all(data, c);
    ASSERT_TRUE(result.confirmed);
    const auto mount = file.load(render_rigid_mount_yaml(result, c));
    const Eigen::Matrix4d lidar = (mount.base_from_sensor * mount.sensor_from_lidar).matrix();
    const Eigen::Isometry3d world_sensor = world_base * mount.base_from_sensor;
    const auto converted = convert_odometry(pose_message(world_sensor), mount.base_from_sensor, c.base_frame);
    EXPECT_NEAR(converted.pose.pose.position.x, world_base.translation().x(), 1.e-12);
    EXPECT_NEAR(converted.pose.pose.position.y, world_base.translation().y(), 1.e-12);
    EXPECT_NEAR(converted.pose.pose.position.z, world_base.translation().z(), 1.e-12);
    EXPECT_TRUE(rotation(converted.pose.pose.orientation).toRotationMatrix().isApprox(world_base.linear()));
    if (c.sensor_frame_mode == "base_aligned") {
      first_lidar = lidar;
      first_odom = converted;
    } else {
      EXPECT_TRUE(lidar.isApprox(first_lidar, 1.e-12));
      EXPECT_NEAR(converted.pose.pose.position.x, first_odom.pose.pose.position.x, 1.e-12);
      EXPECT_NEAR(converted.pose.pose.position.y, first_odom.pose.pose.position.y, 1.e-12);
      EXPECT_NEAR(converted.pose.pose.position.z, first_odom.pose.pose.position.z, 1.e-12);
    }

    const auto body = YAML::Load(render_backend_body_yaml(result, c))["common"]["body"];
    const auto r = body["rotation"];
    const auto offset = body["offset"];
    const auto body_from_sensor = rpy_rotation({radians(r["roll_deg"].as<double>()),
      radians(r["pitch_deg"].as<double>()), radians(r["yaw_deg"].as<double>())});
    const Eigen::Vector3d p(offset["x"].as<double>(), offset["y"].as<double>(), offset["z"].as<double>());
    // Same pose equations as backend::BodyTransform::bodyPoseFromSensorPose.
    const Eigen::Matrix3d parent_from_body = world_sensor.linear() * body_from_sensor.transpose();
    const Eigen::Vector3d parent_body_position = world_sensor.translation() + parent_from_body * p;
    EXPECT_TRUE(parent_from_body.isApprox(world_base.linear(), 1.e-12));
    EXPECT_TRUE(parent_body_position.isApprox(world_base.translation(), 1.e-12));
    EXPECT_EQ(body["frame"].as<std::string>(), "base_link");
  }
}

TEST(RigidMountCalibration, ConfirmationAndNullsPreserveUnmeasuredComponents)
{
  CalibrationConfig c;
  c.confirm = true;
  c.manual_tx = 0.0;
  CalibrationSamples data;
  data.imu = stationary_imu();
  data.clouds = clouds(c);
  const auto partial = solve_all(data, c);
  EXPECT_FALSE(partial.confirmed);
  const auto output = render_rigid_mount_yaml(partial, c);
  const auto yaml = YAML::Load(output);
  EXPECT_FALSE(yaml["calibration_confirmed"].as<bool>());
  EXPECT_DOUBLE_EQ(yaml["base_to_sensor"]["xyz"][0].as<double>(), 0.0);
  EXPECT_TRUE(yaml["base_to_sensor"]["xyz"][1].IsNull());
  EXPECT_NEAR(yaml["base_to_sensor"]["xyz"][2].as<double>(), lever.z(), .001);
  for (const auto & value : yaml["sensor_from_imu_rpy"]) {EXPECT_TRUE(value.IsNull());}
  for (const auto & value : yaml["sensor_to_lidar"]["rpy"]) {EXPECT_TRUE(value.IsNull());}
  const MountFile file;
  EXPECT_THROW(file.load(output), std::invalid_argument);

  c.manual_ty = lever.y();
  c.manual_psi_deg = expected_psi * 180.0 / calibration_pi;
  EXPECT_TRUE(solve_all(data, c).confirmed);
  data.spin_requested = true;
  EXPECT_FALSE(solve_all(data, c).confirmed);
  data.spin_requested = false;
  c.confirm = false;
  EXPECT_FALSE(solve_all(data, c).confirmed);
}

TEST(RigidMountCalibration, OverridesCannotBypassExecutedCrossChecks)
{
  CalibrationConfig c;
  c.confirm = true;
  auto data = full(c);
  c.manual_tz = .84;
  EXPECT_FALSE(solve_all(data, c).confirmed);
  c.manual_tz.reset();
  c.manual_tx = -.50;
  EXPECT_FALSE(solve_all(data, c).confirmed);
  c.manual_tx.reset();
  c.manual_psi_deg = 0;
  EXPECT_FALSE(solve_all(data, c).confirmed);
}

TEST(RigidMountCalibration, AutoSegmentsContinuousRecordingAndReportsMissingStages)
{
  CalibrationConfig c;
  std::vector<ImuSample> imu;
  std::vector<OdomSample> odometry;
  std::vector<double> stamps;
  for (int i = 0; i <= 1200; ++i) {
    const double t = i * .01;
    const double x = std::clamp(t - 3., 0.0, 3.0);
    const double spin_yaw = std::max(0., t - 6.) * calibration_pi / 3;
    imu.push_back({t, base_from_imu.row(2).transpose(),
      (t < 6. ? 0. : calibration_pi / 3) * base_from_imu.row(2).transpose()});
    if (i % 4 == 0) {odometry.push_back(odom(t, {x, 0., 0.}, yaw(spin_yaw)));}
    if (i % 10 == 0) {stamps.push_back(t);}
  }
  const auto selected = auto_segment_recording(imu, odometry, stamps, c);
  ASSERT_TRUE(selected.reasons.empty());
  ASSERT_TRUE(selected.stationary && selected.straight && selected.spin);
  EXPECT_NEAR(selected.stationary->start, 0, .01);
  EXPECT_NEAR(selected.straight->start, 3, .05);
  EXPECT_NEAR(selected.spin->start, 6, .05);
  odometry.resize(100);
  const auto partial = auto_segment_recording(imu, odometry, stamps, c);
  EXPECT_TRUE(partial.stationary);
  EXPECT_FALSE(partial.straight);
  EXPECT_FALSE(partial.spin);
  EXPECT_EQ(partial.reasons.size(), 2U);
}

void check_delayed_recording(double scale)
{
  CalibrationConfig c;
  c.imu_acceleration_scale = scale;
  c.confirm = true;
  std::vector<ImuSample> imu;
  std::vector<OdomSample> odometry;
  const auto recording_clouds = clouds(c);
  std::vector<double> stamps;
  for (const auto & cloud : recording_clouds) {stamps.push_back(cloud.time);}
  for (int i = 0; i <= 1300; ++i) {
    const double t = i * .01;
    const double x = std::clamp((t - 3.) * .75, 0.0, 3.0);
    const double spin_yaw = std::max(0., t - 7.) * calibration_pi / 3;
    const double accel_x = i >= 300 && i < 325 ? 2. : 0.;
    imu.push_back({t, base_from_imu.transpose() * Eigen::Vector3d(accel_x, 0., 9.81) / scale,
      (t < 7. ? 0. : calibration_pi / 3) * base_from_imu.row(2).transpose()});
    if (i % 4 == 0) {odometry.push_back(odom(t + .25, {x, 0., 0.}, yaw(spin_yaw)));}
  }
  const auto selected = auto_segment_recording(imu, odometry, stamps, c);
  ASSERT_TRUE(selected.reasons.empty());
  ASSERT_TRUE(selected.stationary && selected.straight && selected.spin);
  EXPECT_DOUBLE_EQ(selected.stationary->start, .25);
  EXPECT_LT(selected.stationary->end, 3.);
  EXPECT_GE(selected.stationary->end - selected.stationary->start, 2.);

  const auto capture = [](const auto & source, TimeWindow window, auto & dest) {
      std::copy_if(source.begin(), source.end(), std::back_inserter(dest),
        [window](const auto & sample) {return window.contains(sample.time);});
    };
  CalibrationSamples data;
  capture(imu, *selected.stationary, data.imu);
  capture(recording_clouds, *selected.stationary, data.clouds);
  capture(odometry, *selected.straight, data.straight);
  capture(odometry, *selected.spin, data.spin);
  const auto solved = solve_all(data, c);
  ASSERT_TRUE(solved.confirmed) << render_report_yaml(data, c, solved, selected);
  const auto mount = MountFile().load(render_rigid_mount_yaml(solved, c));
  EXPECT_DOUBLE_EQ(mount.imu_acceleration_scale, scale);
  const auto verified = verify_calibration(mount, data, CalibrationConfig{});
  EXPECT_TRUE(verified.passed()) << render_report_yaml(data, c, solved, selected, verified);
}

TEST(RigidMountCalibration, AutoSegmentsStationaryHoldWithDelayedOdometry)
{
  check_delayed_recording(9.81);
}

TEST(RigidMountCalibration, AutoSegmentsSolvesAndVerifiesMilligRecording)
{
  check_delayed_recording(.00981);
}

TEST(RigidMountCalibration, VerificationRequiresBothMotionStages)
{
  CalibrationConfig c;
  c.confirm = true;
  auto data = full(c);
  const auto solved = solve_all(data, c);
  ASSERT_TRUE(solved.confirmed);
  auto mount = MountFile().load(render_rigid_mount_yaml(solved, c));
  mount.base_from_sensor.linear() = yaw(calibration_pi / 2) * mount.base_from_sensor.linear();
  const auto spin_data = std::move(data.spin);
  data.spin.clear();
  data.straight.clear();
  // Requested stages with empty captures were not executed either.
  data.straight_requested = true;
  const auto checked = verify_calibration(mount, data, c);
  EXPECT_TRUE(checked.stationary.passed());
  EXPECT_FALSE(checked.passed());
  EXPECT_EQ(checked.reasons, (std::vector<std::string>{
      "straight motion verification was not executed", "spin motion verification was not executed"}));

  data.spin = spin_data;
  const auto partial = verify_calibration(mount, data, c);
  EXPECT_FALSE(partial.passed());
  EXPECT_TRUE(partial.spin.attempted);
  EXPECT_TRUE(std::isfinite(partial.spin.residual_rms));
  ASSERT_FALSE(partial.reasons.empty());
  EXPECT_EQ(partial.reasons.front(), "straight motion verification was not executed");
}

TEST(RigidMountCalibration, SolveAndVerifyRequireConfiguredOdometryFrames)
{
  CalibrationConfig c;
  c.confirm = true;
  auto data = full(c);
  data.odom_frames = {"odom"};
  data.odom_child_frames = {"imu"};
  const auto solved = solve_all(data, c);
  ASSERT_TRUE(solved.confirmed);
  const auto mount = MountFile().load(render_rigid_mount_yaml(solved, c));
  ASSERT_TRUE(verify_calibration(mount, data, c).passed());

  for (const auto * child : {"base_footprint", "", "lio_base_sensor"}) {
    SCOPED_TRACE(child);
    data.odom_child_frames = {child};
    const std::vector<std::string> reasons{
      "observed odometry child_frame_id does not match configured odom_child_frame"};
    const auto result = solve_all(data, c);
    EXPECT_FALSE(result.confirmed);
    EXPECT_EQ(result.reasons, reasons);
    const auto verified = verify_calibration(mount, data, c);
    EXPECT_FALSE(verified.passed());
    EXPECT_EQ(verified.reasons, reasons);
  }
  data.odom_child_frames = {"imu"};
  data.odom_frames = {"raw_odom"};
  const std::vector<std::string> reasons{
    "observed odometry frame does not match configured odom_frame"};
  const auto result = solve_all(data, c);
  EXPECT_FALSE(result.confirmed);
  EXPECT_EQ(result.reasons, reasons);
  const auto verified = verify_calibration(mount, data, c);
  EXPECT_FALSE(verified.passed());
  EXPECT_EQ(verified.reasons, reasons);

  c.odom_frame = "raw_odom";
  c.odom_child_frame = "raw_imu";
  data.odom_child_frames = {"raw_imu"};
  EXPECT_TRUE(solve_all(data, c).confirmed);
  EXPECT_TRUE(verify_calibration(mount, data, c).passed());
}

TEST(RigidMountCalibration, ConfigRejectsInvalidOdometryFrames)
{
  CalibrationConfig c;
  for (const auto * frame : {"", "/odom", "bad frame", "map", "base_footprint",
    "base_link", "lio_base_sensor", "livox_frame"})
  {
    SCOPED_TRACE(frame);
    c.odom_frame = frame;
    EXPECT_THROW(validate_calibration_config(c), std::invalid_argument);
  }
  c.odom_frame = "raw_odom";
  for (const auto * frame : {"", "/imu", "bad frame", "map", "odom",
    "base_footprint", "base_link", "raw_odom"})
  {
    SCOPED_TRACE(frame);
    c.odom_child_frame = frame;
    EXPECT_THROW(validate_calibration_config(c), std::invalid_argument);
  }
}

TEST(RigidMountCalibration, MountWriteFailureIsRecordedInUnconfirmedReport)
{
  CalibrationConfig c;
  c.confirm = true;
  const auto data = full(c);
  const auto result = solve_all(data, c);
  ASSERT_TRUE(result.confirmed);
  const MountFile report_file;
  const auto mount_path = report_file.filename() + ".missing/mount.yaml";
  calibration_io::OutputFiles outputs;
  outputs.mount = mount_path;
  outputs.report = report_file.filename();
  const auto errors = calibration_io::write_outputs(outputs, data, c, result);
  ASSERT_FALSE(errors.empty());
  EXPECT_NE(errors.front().find(mount_path), std::string::npos);
  const auto report = YAML::LoadFile(outputs.report);
  EXPECT_FALSE(report["calibration_confirmed"].as<bool>());
  EXPECT_EQ(report["reasons"].as<std::vector<std::string>>(), errors);
}

TEST(RigidMountCalibration, UnconfirmedResultDoesNotWriteOrOverwriteBackendBody)
{
  CalibrationConfig c;
  const auto data = full(c);
  const auto preview = solve_all(data, c);
  ASSERT_FALSE(preview.confirmed);
  ASSERT_TRUE(preview.reasons.empty());

  c.confirm = true;
  const auto confirmed = solve_all(data, c);
  ASSERT_TRUE(confirmed.confirmed);
  const MountFile confirmed_file, mount_file, backend_file, report_file;
  const auto mount = confirmed_file.load(render_rigid_mount_yaml(confirmed, c));
  auto incomplete = data;
  incomplete.spin.clear();
  incomplete.spin_requested = false;
  const auto verification = verify_calibration(mount, incomplete, c);
  ASSERT_FALSE(verification.passed());
  const calibration_io::OutputFiles outputs{
    mount_file.filename(), backend_file.filename(), report_file.filename()};
  const std::string original_backend = "# Existing backend\r\ncommon:\r\n  body: {}\r\n";

  for (const bool verify : {false, true}) {
    SCOPED_TRACE(verify ? "failed verification" : "unconfirmed solve");
    std::filesystem::remove(outputs.backend);
    for (const bool existing : {false, true}) {
      SCOPED_TRACE(existing ? "existing backend" : "missing backend");
      if (existing) {std::ofstream(outputs.backend, std::ios::binary) << original_backend;}
      std::ofstream(outputs.mount) << "calibration_confirmed: true\n";
      std::ofstream(outputs.report) << "calibration_confirmed: true\n";
      const auto errors = verify ?
        calibration_io::write_outputs(outputs, incomplete, c, CalibrationResult{}, {},
          verification, mount) :
        calibration_io::write_outputs(outputs, data, CalibrationConfig{}, preview);
      EXPECT_TRUE(errors.empty());
      EXPECT_FALSE(YAML::LoadFile(outputs.mount)["calibration_confirmed"].as<bool>());
      const auto report = YAML::LoadFile(outputs.report);
      EXPECT_FALSE(report["calibration_confirmed"].as<bool>());
      if (verify) {EXPECT_FALSE(report["verification_passed"].as<bool>());}
      if (existing) {
        std::ifstream backend(outputs.backend, std::ios::binary);
        const std::string contents((std::istreambuf_iterator<char>(backend)),
          std::istreambuf_iterator<char>());
        EXPECT_EQ(contents, original_backend);
      } else {
        EXPECT_FALSE(std::filesystem::exists(outputs.backend));
      }
    }
  }
}

TEST(RigidMountCalibration, ReportWriteFailureLeavesStagedExportsUnpublished)
{
  CalibrationConfig c;
  c.confirm = true;
  const auto data = full(c);
  const auto result = solve_all(data, c);
  ASSERT_TRUE(result.confirmed);
  const MountFile mount_file, backend_file;
  const calibration_io::OutputFiles outputs{mount_file.filename(), backend_file.filename(),
    backend_file.filename() + ".missing/report.yaml"};
  const auto errors = calibration_io::write_outputs(outputs, data, c, result);
  ASSERT_FALSE(errors.empty());
  EXPECT_NE(errors.front().find(outputs.report), std::string::npos);
  EXPECT_EQ(std::filesystem::file_size(outputs.mount), 0u);
  EXPECT_EQ(std::filesystem::file_size(outputs.backend), 0u);
}

TEST(RigidMountCalibration, ConfigAndMountRejectReservedNavigationFrames)
{
  CalibrationConfig c;
  c.confirm = true;
  EXPECT_NO_THROW(validate_calibration_config(c));
  const auto solved = solve_all(full(c), c);
  ASSERT_TRUE(solved.confirmed);
  const auto output = render_rigid_mount_yaml(solved, c);
  const MountFile file;
  const auto mount = file.load(output);
  EXPECT_EQ(mount.imu_source_frame, mount.lidar_frame);

  const std::pair<const char *, std::string CalibrationConfig::*> fields[] = {
    {"base", &CalibrationConfig::base_frame},
    {"sensor", &CalibrationConfig::sensor_frame},
    {"lidar", &CalibrationConfig::lidar_frame},
    {"imu_source", &CalibrationConfig::imu_source_frame},
  };
  for (const auto & [key, field] : fields) {
    for (const auto * frame : {"map", "odom", "base_footprint"}) {
      SCOPED_TRACE(std::string(key) + "=" + frame);
      auto invalid = c;
      invalid.*field = frame;
      EXPECT_THROW(validate_calibration_config(invalid), std::invalid_argument);
      auto yaml = YAML::Load(output);
      yaml["frames"][key] = frame;
      EXPECT_THROW(file.load(YAML::Dump(yaml)), std::invalid_argument);
    }
  }
}

TEST(RigidMountCalibration, VerificationUsesFixedMountInBothModesAndDetectsTampering)
{
  CalibrationConfig c;
  c.confirm = true;
  c.base_height_above_ground = .13;
  const auto data = full(c);
  const MountFile file;
  for (const auto * mode : {"base_aligned", "imu_aligned"}) {
    c.sensor_frame_mode = mode;
    const auto export_result = solve_all(data, c);
    ASSERT_TRUE(export_result.confirmed);
    auto mount = file.load(render_rigid_mount_yaml(export_result, c));
    const auto checked = verify_calibration(mount, data, c);
    ASSERT_TRUE(checked.passed()) << render_report_yaml(data, c, export_result, {}, checked);
    EXPECT_LT(checked.gravity_error, .01);
    EXPECT_LT(checked.floor_height_error, .001);
    EXPECT_LT(checked.straight.yaw_error_deg, .001);
    EXPECT_LT(checked.spin.residual_rms, .001);

    const auto original = mount;
    mount.base_from_sensor.translation().x() += .13;
    EXPECT_FALSE(verify_calibration(mount, data, c).passed());
    mount = original;
    mount.base_from_sensor.linear() = yaw(.15) * mount.base_from_sensor.linear();
    EXPECT_FALSE(verify_calibration(mount, data, c).passed());
    mount = original;
    mount.base_from_sensor.translation().z() += .12;
    EXPECT_FALSE(verify_calibration(mount, data, c).passed());
    mount = original;
    mount.sensor_from_imu = rpy_rotation({.13, 0, 0}) * mount.sensor_from_imu;
    EXPECT_FALSE(verify_calibration(mount, data, c).passed());
  }
}

TEST(RigidMountCalibration, LoaderRejectsQuotedNumericAndBooleanFields)
{
  CalibrationConfig c;
  c.confirm = true;
  const auto result = solve_all(full(c), c);
  ASSERT_TRUE(result.confirmed);
  const auto output = render_rigid_mount_yaml(result, c);
  const MountFile file;
  auto yaml = YAML::Load(output);
  yaml["calibration_confirmed"] = YAML::Load("'true'");
  yaml["calibration_confirmed"].SetTag("tag:yaml.org,2002:str");
  EXPECT_THROW(file.load(YAML::Dump(yaml)), std::invalid_argument);
  yaml = YAML::Load(output);
  yaml["imu_acceleration_scale"].SetTag("tag:yaml.org,2002:str");
  EXPECT_THROW(file.load(YAML::Dump(yaml)), std::invalid_argument);
}

}  // namespace
}  // namespace xxu_bringup
