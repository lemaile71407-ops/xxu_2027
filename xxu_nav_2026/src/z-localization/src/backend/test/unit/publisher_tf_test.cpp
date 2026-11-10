// 验证双出口 TF 的源时间、水位和信任边界，不依赖 ROS 调度时序。
#include <gtest/gtest.h>
#include <limits>

#include "apps/publisher.hpp"

namespace backend::ros {
namespace {
LocalizationProjection trusted(std::uint64_t epoch = 0, std::uint64_t generation = 2,
                               std::uint64_t revision = 4) {
    LocalizationProjection p;
    p.valid = p.pose_trusted = p.global_pose_available = p.should_publish_tf = true;
    p.status = LocalizationStatus::Normal;
    p.epoch = epoch;
    p.state_generation = generation;
    p.pose_revision = revision;
    return p;
}
constexpr std::int64_t t = 10000000000LL;
constexpr std::int64_t ms = 1000000LL;

TEST(PublisherTf, PredictionRequiresAFrameAndCannotRenewItsSourceTime) {
    TfPublicationState state;
    const auto p = trusted();
    EXPECT_FALSE(state.canPredict(p, t, .25));
    state.observeFrame(p, t);
    ASSERT_TRUE(state.canPredict(p, t + 10 * ms, .25));
    state.sent(p, t + 10 * ms);
    EXPECT_EQ(state.frame_stamp_ns, t);
    EXPECT_TRUE(state.canPredict(p, t + 240 * ms, .25));
    EXPECT_FALSE(state.canPredict(p, t + 260 * ms, .25));
    EXPECT_EQ(state.frame_stamp_ns, t);
}

TEST(PublisherTf, LateFrameRefreshesObservationWithoutRegressingPublishedTf) {
    TfPublicationState state;
    auto p = trusted();
    state.observeFrame(p, t);
    state.sent(p, t + 20 * ms);
    ++p.pose_revision;
    state.observeFrame(p, t + 10 * ms);
    EXPECT_EQ(state.frame_stamp_ns, t + 10 * ms);
    EXPECT_FALSE(state.canSend(p, t + 10 * ms));
    EXPECT_FALSE(state.canSend(p, t + 20 * ms));
    EXPECT_TRUE(state.canPredict(p, t + 30 * ms, .25));
    state.sent(p, t + 30 * ms);
    --p.pose_revision;
    EXPECT_FALSE(state.canSend(p, t + 40 * ms));
    EXPECT_FALSE(state.canPredict(p, t + 40 * ms, .25));
}

TEST(PublisherTf, UntrustedStatesStopPredictionsAndRecoveryNeedsItsOwnFrame) {
    TfPublicationState state;
    auto p = trusted();
    state.observeFrame(p, t);
    state.sent(p, t);
    for (const auto status : {LocalizationStatus::Initializing, LocalizationStatus::Suspect,
                              LocalizationStatus::Lost}) {
        auto untrusted = p;
        untrusted.status = status;
        // Even a contradictory pose_trusted flag cannot enable predictions.
        EXPECT_FALSE(state.canPredict(untrusted, t + 10 * ms, .25));
    }
    auto frozen = p;
    frozen.status = LocalizationStatus::Lost;
    frozen.pose_trusted = false;
    ++frozen.state_generation;
    state.observeFrame(frozen, t + 10 * ms);
    EXPECT_TRUE(state.canSend(frozen, t + 10 * ms));  // Existing synchronous freeze output.
    state.sent(frozen, t + 10 * ms);
    p.state_generation += 2;
    EXPECT_FALSE(state.canPredict(p, t + 20 * ms, .25));
    state.observeFrame(p, t + 20 * ms);
    EXPECT_TRUE(state.canPredict(p, t + 30 * ms, .25));
    auto old = trusted();
    state.observeFrame(old, t + 100 * ms);
    EXPECT_EQ(state.frame_generation, p.state_generation);
}

TEST(PublisherTf, NewEpochCanRestartTimeButOldEpochCannotPublishOrRefresh) {
    TfPublicationState state;
    const auto old = trusted();
    state.observeFrame(old, t);
    state.sent(old, t);
    const auto current = trusted(1, 0, 0);
    EXPECT_FALSE(state.canPredict(current, ms, .25));
    state.observeFrame(current, ms);
    EXPECT_TRUE(state.canSend(current, ms));
    state.sent(current, ms);
    state.observeFrame(old, t + ms);
    EXPECT_EQ(state.frame_epoch, 1U);
    EXPECT_EQ(state.frame_stamp_ns, ms);
    EXPECT_FALSE(state.canSend(old, t + ms));
    EXPECT_FALSE(state.canPredict(old, t + ms, .25));
    EXPECT_TRUE(state.canPredict(current, 2 * ms, .25));
}

TEST(PublisherTf, InvalidAndRegressingSamplesCannotAdvanceTheWatermark) {
    TfPublicationState state;
    auto p = trusted();
    state.observeFrame(p, t);
    state.sent(p, t);
    EXPECT_FALSE(state.canSend(p, 0));
    EXPECT_FALSE(state.canPredict(p, t - ms, .25));
    state.observeFrame(p, t - ms);
    EXPECT_EQ(state.frame_stamp_ns, t);
    p.valid = false;
    EXPECT_FALSE(state.canPredict(p, t + ms, .25));
    p = trusted();
    p.global_pose_available = false;
    EXPECT_FALSE(state.canSend(p, t + ms));
    p = trusted();
    p.T_map_odom(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(state.canPredict(p, t + ms, .25));
}
}  // namespace
}  // namespace backend::ros
