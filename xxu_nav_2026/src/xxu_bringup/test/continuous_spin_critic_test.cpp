#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>

#include "xxu_bringup/continuous_spin_critic.hpp"

class TestCritic : public mppi::critics::ContinuousSpinCritic
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    mppi::ParametersHandler & handler)
  {
    parent_ = node;
    parent_name_ = "FollowPath";
    name_ = "FollowPath.ContinuousSpinCritic";
    logger_ = node->get_logger();
    parameters_handler_ = &handler;
    enabled_ = true;
    initialize();
  }
  void disable() {enabled_ = false;}
};

class SpinCriticTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("spin_critic_test");
    handler = std::make_unique<mppi::ParametersHandler>(node);
    critic.configure(node, *handler);
    state.reset(3, 4);
    trajectories.reset(3, 4);
    goal.position.x = 4;
    for (size_t j = 0; j < 4; ++j) {
      state.wz(0, j) = 0;
      state.wz(1, j) = 0.5;
      state.wz(2, j) = -0.5;
    }
  }
  mppi::CriticData data()
  {
    return {state, trajectories, path, goal, costs, dt, false, nullptr,
      std::make_shared<mppi::OmniMotionModel>(), std::nullopt, std::nullopt};
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::unique_ptr<mppi::ParametersHandler> handler;
  TestCritic critic;
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  float dt = 0.0666667F;
};

TEST_F(SpinCriticTest, PrefersRequestedDirectionWithinPredictedMotion)
{
  auto d = data();
  critic.score(d);
  EXPECT_FLOAT_EQ(costs(1), 0);
  EXPECT_LT(costs(1), costs(0));
  EXPECT_LT(costs(0), costs(2));
}

TEST_F(SpinCriticTest, PreservesCollisionCostsInsteadOfOverridingSafety)
{
  costs(1) = 100000;
  auto d = data();
  critic.score(d);
  EXPECT_FLOAT_EQ(costs(1), 100000);
  EXPECT_LT(costs(0), costs(1));
}

TEST_F(SpinCriticTest, ReleasesForGoalHeadingAndStoppedGoalChecker)
{
  state.pose.pose.position.x = 3.8;
  costs.fill(12);
  auto d = data();
  critic.score(d);
  for (float cost : costs) {EXPECT_FLOAT_EQ(cost, 12);}
}

TEST_F(SpinCriticTest, PredictedApproachReleasesRotationBeforeArriving)
{
  trajectories.x.fill(3.8);
  auto d = data();
  critic.score(d);
  for (float cost : costs) {EXPECT_FLOAT_EQ(cost, 0);}
}

TEST_F(SpinCriticTest, DisabledOrFailedOptimizationLeavesCostsAlone)
{
  costs.fill(7);
  auto d = data();
  d.fail_flag = true;
  critic.score(d);
  d.fail_flag = false;
  critic.disable();
  critic.score(d);
  for (float cost : costs) {EXPECT_FLOAT_EQ(cost, 7);}
}

TEST_F(SpinCriticTest, RejectsNonHolonomicMotion)
{
  auto d = data();
  d.motion_model = std::make_shared<mppi::DiffDriveMotionModel>();
  EXPECT_THROW(critic.score(d), std::logic_error);
}

TEST_F(SpinCriticTest, RejectsUnboundedOrInvalidConfiguration)
{
  for (const auto & p : std::vector<rclcpp::Parameter>{
      {"FollowPath.ContinuousSpinCritic.target_angular_velocity", 0.6},
      {"FollowPath.ContinuousSpinCritic.cost_weight", -1.0},
      {"FollowPath.ContinuousSpinCritic.release_distance", 0.0},
      {"FollowPath.ContinuousSpinCritic.transition_distance", 0.0},
      {"FollowPath.motion_model", "DiffDrive"}})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({p});
    auto invalid_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("invalid_spin", options);
    mppi::ParametersHandler invalid_handler(invalid_node);
    TestCritic invalid;
    EXPECT_THROW(invalid.configure(invalid_node, invalid_handler), std::invalid_argument);
  }
}

TEST_F(SpinCriticTest, PluginIsDiscoverableUsingNav2CriticLoader)
{
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  EXPECT_NE(loader.createSharedInstance("mppi::critics::ContinuousSpinCritic"), nullptr);
}
