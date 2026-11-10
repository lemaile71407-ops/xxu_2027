// ROS 边界验证显式字段、物理单位及输入队列的关闭语义。
#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>

#include "apps/input.hpp"
#include "apps/ros.hpp"
#include "execution.hpp"
#include "zlio/odometry.hpp"
#include "zlio/state.hpp"

namespace zlio::ros {
namespace {
TEST(LidarDiagnosticMessage, PreservesRowsAnchorsCorrectionsAndFullCovariance) {
    FrameResult frame;
    frame.begin = nanoseconds(1);
    frame.estimate.state.time = nanoseconds(1.1);
    frame.estimate.state.pose.position = Vec3(3, 1, 7);
    frame.estimate.state.pose.rotation = expRotation(Vec3(0, 0, .2));
    frame.estimate.covariance = .3 * Mat15::Identity();
    frame.matches = 7;
    frame.lidar_updated = true;
    auto& diagnostic = frame.diagnostic.emplace();
    diagnostic.prediction.state.time = frame.estimate.state.time;
    diagnostic.prediction.state.pose.position = Vec3(2, 3, 4);
    diagnostic.prediction.covariance = 2 * Mat15::Identity();
    diagnostic.linearization_pose = diagnostic.prediction.state.pose;
    diagnostic.sampled_points = 23;
    diagnostic.linearizations = 3;
    // Deliberately asymmetric sentinels detect Eigen column-major memcpy mistakes.
    for (int row = 0; row < 6; ++row)
        for (int column = 0; column < 6; ++column) {
            diagnostic.solver_information(row, column) = 100 * row + column + .5;
            diagnostic.observation_information(row, column) = 100 * row + column + .25;
        }
    const auto message = makeLidarDiagnosticMessage(frame, {"world", "sensor"}, .01);
    EXPECT_EQ(message.prediction.header.frame_id, "world");
    EXPECT_EQ(message.corrected.child_frame_id, "sensor");
    EXPECT_EQ(messageTime(message.prediction.header.stamp), frame.estimate.state.time);
    EXPECT_EQ(messageTime(message.corrected.header.stamp), frame.estimate.state.time);
    EXPECT_EQ(message.matches, 7U);
    EXPECT_EQ(message.sampled_points, 23U);
    EXPECT_EQ(message.linearizations, 3U);
    EXPECT_TRUE(message.accepted);
    EXPECT_DOUBLE_EQ(message.linearization_pose.position.y, 3);
    EXPECT_DOUBLE_EQ(message.position_correction_world.x, 1);
    EXPECT_DOUBLE_EQ(message.position_correction_world.y, -2);
    EXPECT_DOUBLE_EQ(message.position_correction_world.z, 3);
    EXPECT_NEAR(message.rotation_correction_world.z, .2, 1e-12);
    for (int row = 0; row < 6; ++row)
        for (int column = 0; column < 6; ++column) {
            const auto index = static_cast<std::size_t>(row * 6 + column);
            EXPECT_DOUBLE_EQ(message.solver_information[index], 100 * row + column + .5);
            EXPECT_DOUBLE_EQ(message.observation_information[index], 100 * row + column + .25);
        }
    EXPECT_DOUBLE_EQ(message.predicted_covariance[16], 2);
    EXPECT_DOUBLE_EQ(message.corrected_covariance[16], .3);
    EXPECT_DOUBLE_EQ(message.prediction.pose.covariance[0], 52);
    EXPECT_DOUBLE_EQ(message.corrected.pose.covariance[0], 15.3);
    frame.diagnostic.reset();
    EXPECT_THROW(makeLidarDiagnosticMessage(frame, {}, .01), std::invalid_argument);
}

TEST(PointCloudDecoder, DecodesPointTimesAndRejectsTruncatedPayload) {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp.sec = 2;
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "offset_time", 1,
                                  sensor_msgs::msg::PointField::UINT32);
    modifier.resize(2);
    for (std::size_t index = 0; index < 2; ++index) {
        const float xyz[] = {1.F + static_cast<float>(index), 2.F, 3.F};
        const std::uint32_t offset = static_cast<std::uint32_t>(index) * 1000U;
        std::memcpy(message.data.data() + index * message.point_step, xyz, sizeof(xyz));
        std::memcpy(message.data.data() + index * message.point_step + 12, &offset, sizeof(offset));
    }
    PointCloudDecoder decoder({});
    const auto points = decoder.decode(message);
    ASSERT_EQ(points.size(), 2U);
    EXPECT_EQ(points[0].time, nanoseconds(2));
    EXPECT_EQ(points[1].time, nanoseconds(2) + 1000);
    EXPECT_DOUBLE_EQ(points[1].position.x(), 2);
    message.data.pop_back();
    EXPECT_THROW(decoder.decode(message), std::exception);
}

TEST(InputQueue, DrainAcceptsAdmittedWorkAndRejectsNewCallbacks) {
    InputQueue queue(4, 10);
    {
        InputQueue::Admission admitted(queue);
        ASSERT_TRUE(admitted);
        queue.close();
        InputQueue::Admission denied(queue);
        EXPECT_FALSE(denied);
        Imu imu;
        imu.time = 123;
        queue.push(imu);
    }
    const auto event = queue.take();
    ASSERT_TRUE(event);
    EXPECT_EQ(std::get<Imu>(*event).time, 123);
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(InputQueue, CancelDiscardsQueuedAndInFlightResults) {
    InputQueue queue(4, 10);
    {
        InputQueue::Admission admission(queue);
        queue.push(Imu{});
        queue.close(InputQueue::CloseMode::Cancel);
        queue.push(Imu{});
    }
    EXPECT_TRUE(queue.failed());
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(InputQueue, CapacityFailureDoesNotSilentlyDropAcceptedEvent) {
    InputQueue queue(1, 2);
    {
        InputQueue::Admission admission(queue);
        queue.push(Imu{});
        EXPECT_THROW(queue.push(Imu{}), std::exception);
    }
    queue.close();
    EXPECT_TRUE(queue.take());
    EXPECT_FALSE(queue.take());
    queue.finish();
}

TEST(BatchExecutor, JoinsAllPartitionsAndCanBeReusedAfterException) {
    BatchExecutor executor(4);
    std::array<int, 4> visits{};
    EXPECT_THROW(executor.run(4,
                              [&](std::size_t index) {
                                  ++visits[index];
                                  if (index == 1)
                                      throw std::runtime_error("partition failed");
                              }),
                 std::runtime_error);
    EXPECT_EQ(visits, (std::array<int, 4>{1, 1, 1, 1}));
    executor.run(4, [&](std::size_t index) { ++visits[index]; });
    EXPECT_EQ(visits, (std::array<int, 4>{2, 2, 2, 2}));
}
}  // namespace
}  // namespace zlio::ros
