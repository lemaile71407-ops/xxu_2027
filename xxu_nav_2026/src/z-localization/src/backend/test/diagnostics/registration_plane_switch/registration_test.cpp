#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <cstdlib>

#include "backend/geometry.hpp"
#include "backend/math.hpp"
#include "backend/registration.hpp"

namespace backend {
namespace {

Cloud::Ptr loadFixture(const char* name) {
    const auto path = std::filesystem::path(__FILE__).parent_path() / "data" / name;
    auto cloud = Cloud::Ptr(new Cloud);
    EXPECT_EQ(pcl::io::loadPCDFile(path.string(), *cloud), 0) << path;
    return cloud;
}

RegistrationConfig recordedConfig(int iterations = 16) {
    RegistrationConfig config;
    const char* experimental = std::getenv("XXU_REGISTRATION_EXPERIMENTAL");
    config.experimental_blended_adaptive = experimental && std::string(experimental) == "1";
    config.voxel_resolution = .1;
    config.max_iterations = iterations;
    config.max_correspondence_distance = 1.;
    config.min_correspondence_ratio = .5;
    config.coarse_trigger_min_correspondence_ratio = .35;
    config.max_rmse = .25;
    config.huber_delta = .25;
    config.measurement_translation_std_floor = .005;
    config.measurement_rotation_std_floor_rad = degreesToRadians(.02);
    return config;
}

}  // namespace

/// 真实失败帧中九个对应平面反复切换；不能靠增加迭代数或放宽最终判据通过。
TEST(Registration, BreaksRecordedTwoStatePlaneSwitchCycle) {
    const auto source = loadFixture("oscillating_source.pcd");
    const auto target = loadFixture("target.pcd");
    Registration registration(recordedConfig());
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto result = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_EQ(result.measurement_information.rank, 6);
    EXPECT_LT(result.quality.rmse, .0570287);
    EXPECT_LT((result.source_to_target.block<3, 1>(0, 3).norm()), .02);

    // 从返回状态实际执行一个未阻尼完整步，验证原来的 5 mm / 0.1° 条件。
    registration.reconfigure(recordedConfig(1));
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto checked = registration.align(*source, result.source_to_target, .2);
    const Eigen::Matrix4d full_step = checked.source_to_target * result.source_to_target.inverse();
    EXPECT_LE((full_step.block<3, 1>(0, 3).norm()), .005);
    EXPECT_LE(rotationAngleFromMatrix(full_step), degreesToRadians(.1));
}

TEST(Registration, StillRejectsRecordedUnstableFullStepWithoutCycleHistory) {
    const auto source = loadFixture("oscillating_source.pcd");
    Registration registration(recordedConfig(1));
    ASSERT_TRUE(registration.setTarget(loadFixture("target.pcd"), 0));
    const auto result = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    EXPECT_FALSE(result.converged);
    EXPECT_FALSE(result.likelihoodValid());
    EXPECT_TRUE(result.measurement_information.valid);
}

/// 第二次录包不精确回到上上次位姿，但更新方向反复切换，最终完整步仍约 28 mm。
TEST(Registration, BreaksRecordedReversingUpdatesWithoutExactPoseCycle) {
    const auto source = loadFixture("reversing_source.pcd");
    const auto target = loadFixture("reversing_target.pcd");
    Registration registration(recordedConfig());
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto result = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_LT(result.quality.rmse, .058255);
    EXPECT_LT((result.source_to_target.block<3, 1>(0, 3).norm()), .03);
    registration.reconfigure(recordedConfig(1));
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto checked = registration.align(*source, result.source_to_target, .2);
    const Eigen::Matrix4d full_step = checked.source_to_target * result.source_to_target.inverse();
    EXPECT_LE((full_step.block<3, 1>(0, 3).norm()), .005);
    EXPECT_LE(rotationAngleFromMatrix(full_step), degreesToRadians(.1));
}

TEST(Registration, PreservesRecordedStablePrecedingFrame) {
    const auto source = loadFixture("stable_source.pcd");
    Registration registration(recordedConfig());
    ASSERT_TRUE(registration.setTarget(loadFixture("target.pcd"), 0));
    const auto result = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_EQ(result.iterations, 1);
    EXPECT_EQ(result.quality.correspondences, 1188);
    EXPECT_LT((result.source_to_target - Eigen::Matrix4d::Identity()).norm(), 3e-6);
}

/// 已满足原最终完整步判据的阻尼状态不应因额外迭代再次离开接受范围。
TEST(Registration, RetainsValidDampedStateAtBudgetExhaustion) {
    const auto source = loadFixture("bounded_source.pcd");
    const auto target = loadFixture("bounded_target.pcd");
    Registration registration(recordedConfig());
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto result = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    ASSERT_TRUE(result.likelihoodValid());
    registration.reconfigure(recordedConfig(32));
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto extended = registration.align(*source, Eigen::Matrix4d::Identity(), .2);
    ASSERT_TRUE(extended.likelihoodValid());
    registration.reconfigure(recordedConfig(1));
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto checked = registration.align(*source, result.source_to_target, .2);
    const Eigen::Matrix4d full_step = checked.source_to_target * result.source_to_target.inverse();
    EXPECT_LE((full_step.block<3, 1>(0, 3).norm()), .005);
    EXPECT_LE(rotationAngleFromMatrix(full_step), degreesToRadians(.1));
}

/// 保留在线求解前的真实初值；最终已对齐点云的单位初值重算不能替代此路径。
TEST(Registration, ReplaysOriginalInitialTransformAtRecordedFailure) {
    const auto source = loadFixture("exact_source.pcd");
    const auto target = loadFixture("exact_target.pcd");
    const auto path = std::filesystem::path(__FILE__).parent_path() / "data/exact_initial.txt";
    std::ifstream seed(path);
    Eigen::Matrix4d initial;
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            ASSERT_TRUE(static_cast<bool>(seed >> initial(row, column)));
    Registration registration(recordedConfig());
    ASSERT_TRUE(registration.setTarget(target, 0));
    const auto result = registration.align(*source, initial, .2);
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_LT(result.quality.rmse, .06);
}

}  // namespace backend
