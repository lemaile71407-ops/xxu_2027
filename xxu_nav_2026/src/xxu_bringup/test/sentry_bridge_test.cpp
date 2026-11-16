#include "xxu_bringup/odometry_conversion.hpp"
#include "xxu_bringup/localization_health.hpp"
#include "xxu_bringup/cloud_layout.hpp"
#include "xxu_bringup/sensor_math.hpp"
#include <gtest/gtest.h>

using xxu_bringup::LocalizationHealth;
namespace {
nav_msgs::msg::Odometry pose(std::int64_t ns)
{
  nav_msgs::msg::Odometry m;
  m.header.frame_id = "map"; m.child_frame_id = "imu";
  m.header.stamp.sec = ns / 1000000000; m.header.stamp.nanosec = ns % 1000000000;
  m.pose.pose.orientation.w = 1.; return m;
}
std::string state(std::int64_t ns, const std::string & status = "NORMAL", int epoch = 0, int generation = 2)
{
  const bool trusted = status == "NORMAL" || status == "OBS_DEGRADED";
  return "status=" + status + ";pose_trusted=" + (trusted ? "true" : "false") +
    ";tf_frozen=" + (trusted ? "false" : "true") + ";epoch=" + std::to_string(epoch) +
    ";generation=" + std::to_string(generation) + ";pose_revision=1;stamp_ns=" + std::to_string(ns);
}
void feed(LocalizationHealth & h, double wall, std::int64_t ns, const std::string & status = "NORMAL", int epoch = 0, int generation = 2)
{h.observe_state(state(ns, status, epoch, generation), wall); h.observe_pose(pose(ns), wall);}
}

TEST(Odometry, InverseMountRemovesRotationAndLeverVelocity)
{
  auto m = pose(1000000000);
  m.pose.pose.position.x = .2; m.pose.pose.position.y = -.1; m.pose.pose.position.z = .3;
  m.pose.pose.orientation.z = std::sqrt(.5); m.pose.pose.orientation.w = std::sqrt(.5);
  m.twist.twist.linear.x = .2; m.twist.twist.linear.y = -1.1; m.twist.twist.angular.z = 1.;
  Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
  mount.translation() = Eigen::Vector3d(.2, -.1, .3);
  mount.linear() = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto out = xxu_bringup::convert_odometry(m, mount, "base_link");
  EXPECT_NEAR(out.pose.pose.position.x, 0., 1.e-12);
  EXPECT_NEAR(out.pose.pose.position.y, 0., 1.e-12);
  EXPECT_NEAR(out.pose.pose.position.z, 0., 1.e-12);
  EXPECT_NEAR(out.pose.pose.orientation.w, 1., 1.e-12);
  EXPECT_NEAR(out.twist.twist.linear.x, 1., 1.e-12);
  EXPECT_NEAR(out.twist.twist.linear.y, 0., 1.e-12);
  EXPECT_NEAR(out.twist.twist.angular.z, 1., 1.e-12);
  EXPECT_EQ(out.child_frame_id, "base_link"); EXPECT_EQ(out.header, m.header);
}

TEST(Odometry, WorldLeftPoseAndLocalTwistCovarianceUseDifferentJacobians)
{
  auto m = pose(1000000000);
  m.pose.pose.position.x = 10.; m.pose.pose.position.y = 20.;
  m.pose.pose.orientation.z = m.pose.pose.orientation.w = std::sqrt(.5);
  m.twist.twist.linear.x = 1.; m.twist.twist.angular.z = 2.;
  for (int i = 0; i < 6; ++i) {m.pose.covariance[7 * i] = m.twist.covariance[7 * i] = 1.;}
  Eigen::Isometry3d sensor_from_base = Eigen::Isometry3d::Identity();
  sensor_from_base.translation().x() = 1.;
  sensor_from_base.linear() = Eigen::AngleAxisd(std::acos(-1.) / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto out = xxu_bringup::convert_odometry(m, sensor_from_base.inverse(), "base_link");
  EXPECT_NEAR(out.pose.pose.position.y, 21., 1.e-12);
  EXPECT_NEAR(out.twist.twist.linear.x, 2., 1.e-12);
  EXPECT_NEAR(out.twist.twist.linear.y, -1., 1.e-12);
  EXPECT_NEAR(out.pose.covariance[0], 2., 1.e-12);
  EXPECT_NEAR(out.pose.covariance[5], -1., 1.e-12);
  EXPECT_NEAR(out.twist.covariance[0], 2., 1.e-12);
  EXPECT_NEAR(out.twist.covariance[5], 1., 1.e-12);
}

TEST(Odometry, RejectsNonfiniteAndZeroQuaternions)
{
  auto m = pose(1000000000); m.pose.pose.orientation.w = 0.;
  EXPECT_THROW(xxu_bringup::convert_odometry(m, Eigen::Isometry3d::Identity(), "base_link"), std::invalid_argument);
  m.pose.pose.orientation.x = 1.e300;
  EXPECT_THROW(xxu_bringup::convert_odometry(m, Eigen::Isometry3d::Identity(), "base_link"), std::invalid_argument);
  m.pose.pose.orientation.x = 0.;
  m.pose.pose.orientation.w = 1.; m.twist.twist.angular.z = NAN;
  EXPECT_THROW(xxu_bringup::convert_odometry(m, Eigen::Isometry3d::Identity(), "base_link"), std::invalid_argument);
}

TEST(LocalizationHealth, RecoveryHoldAndUntrustedTransitions)
{
  LocalizationHealth h(.5, .2);
  feed(h, 0., 100000000000); EXPECT_FALSE(h.check(0., 100000000000).empty());
  feed(h, .21, 100210000000); EXPECT_TRUE(h.check(.21, 100210000000).empty());
  feed(h, .3, 100300000000, "SUSPECT", 0, 3); EXPECT_FALSE(h.check(.3, 100300000000).empty());
  h.observe_state(state(100400000000, "NORMAL", 0, 4), .4);
  EXPECT_FALSE(h.check(.4, 100400000000).empty());
  feed(h, .41, 100410000000, "NORMAL", 0, 4); EXPECT_FALSE(h.check(.41, 100410000000).empty());
  feed(h, .62, 100620000000, "NORMAL", 0, 4); EXPECT_TRUE(h.check(.62, 100620000000).empty());
}

TEST(LocalizationHealth, RepeatedMessagesCannotRenewPausedSourceClock)
{
  LocalizationHealth h(.5, 0.);
  feed(h, 0., 100000000000); EXPECT_TRUE(h.check(0., 100000000000).empty());
  feed(h, .6, 100000000000); EXPECT_FALSE(h.check(.6, 100000000000).empty());
}

TEST(LocalizationHealth, ZeroStampTrustedGenerationsDoNotExtendLifetime)
{
  LocalizationHealth h(.5, 0.);
  feed(h, 0., 100000000000); EXPECT_TRUE(h.check(0., 100000000000).empty());
  h.observe_state(state(0, "OBS_DEGRADED", 0, 3), .3);
  EXPECT_TRUE(h.check(.3, 100300000000).empty());
  h.observe_state(state(0, "NORMAL", 0, 4), .6);
  h.observe_pose(pose(100600000000), .6);
  EXPECT_FALSE(h.check(.6, 100600000000).empty());
}

TEST(LocalizationHealth, EpochChangeRequiresNewPoseAndRejectsRegression)
{
  LocalizationHealth h(.5, 0.);
  feed(h, 0., 100000000000); EXPECT_TRUE(h.check(0., 100000000000).empty());
  h.observe_state(state(100100000000, "NORMAL", 1, 3), .1);
  EXPECT_FALSE(h.check(.1, 100100000000).empty());
  feed(h, .2, 100200000000, "NORMAL", 1, 3); EXPECT_TRUE(h.check(.2, 100200000000).empty());
  feed(h, .3, 100300000000, "NORMAL", 0, 2); EXPECT_FALSE(h.check(.3, 100300000000).empty());
}

TEST(LocalizationHealth, MalformedStateAndPoseCloseGate)
{
  for (const auto & text : {std::string("status=NORMAL"), state(100000000000) + ";epoch=0", state(-1)}) {
    LocalizationHealth h(.5, 0.); feed(h, 0., 100000000000);
    h.observe_state(text, .1); EXPECT_FALSE(h.check(.1, 100100000000).empty());
  }
  LocalizationHealth h(.5, 0.); feed(h, 0., 100000000000);
  auto m = pose(100100000000); m.pose.pose.position.x = NAN;
  h.observe_pose(m, .1); EXPECT_FALSE(h.check(.1, 100100000000).empty());
}

TEST(CloudLayout, OrganizedPaddingEndianAndInvalidFieldBounds)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 1; cloud.height = 2; cloud.point_step = 24; cloud.row_step = 32; cloud.data.resize(64);
  for (int i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f; f.name = std::string(1, "xyz"[i]); f.offset = 4 * i;
    f.count = 1; f.datatype = sensor_msgs::msg::PointField::FLOAT32; cloud.fields.push_back(f);
  }
  xxu_bringup::CloudLayout layout(cloud);
  xxu_bringup::write_scalar<float>(cloud.data.data() + 32 + layout.xyz[2], 3.25f, true);
  EXPECT_FLOAT_EQ(xxu_bringup::read_scalar<float>(cloud.data.data() + 40, true), 3.25f);
  cloud.fields[0].offset = 23;
  EXPECT_THROW(xxu_bringup::CloudLayout bad(cloud), std::invalid_argument);
}

TEST(JointHistory, UnwrapsWithoutExtrapolatingAcrossGaps)
{
  xxu_bringup::JointHistory history;
  sensor_msgs::msg::JointState m; m.name = {"gimbal_joint"}; m.position = {3.1}; m.velocity = {1.};
  m.header.stamp.sec = 1; history.add(m, true);
  m.header.stamp.nanosec = 10000000; m.position = {-3.1}; history.add(m, true);
  ASSERT_TRUE(history.interpolate(1.005));
  EXPECT_NEAR(history.interpolate(1.005)->position, std::acos(-1.), 1.e-10);
  EXPECT_FALSE(history.interpolate(1.1));
  m.header.stamp.nanosec = 100000000; EXPECT_TRUE(history.add(m, true));
  EXPECT_FALSE(history.interpolate(1.05));
}
