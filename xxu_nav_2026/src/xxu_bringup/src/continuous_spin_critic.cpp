#include "xxu_bringup/continuous_spin_critic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
void ContinuousSpinCritic::initialize()
{
  auto get = parameters_handler_->getParamGetter(name_);
  get(target_rate_, "target_angular_velocity", 0.5F, ParameterType::Static);
  get(weight_, "cost_weight", 64.0F, ParameterType::Static);
  get(release_distance_, "release_distance", 0.65F, ParameterType::Static);
  get(transition_distance_, "transition_distance", 0.35F, ParameterType::Static);
  auto controller = parameters_handler_->getParamGetter(parent_name_);
  std::string model;
  float limit;
  controller(model, "motion_model", std::string("Omni"), ParameterType::Static);
  controller(limit, "wz_max", 0.5F, ParameterType::Static);
  if (model != "Omni" || !std::isfinite(target_rate_) ||
    !std::isfinite(weight_) || weight_ < 0 || !std::isfinite(limit) || limit <= 0 ||
    std::abs(target_rate_) > limit || !std::isfinite(release_distance_) ||
    release_distance_ <= 0 || !std::isfinite(transition_distance_) || transition_distance_ <= 0)
  {
    throw std::invalid_argument("ContinuousSpinCritic requires Omni, a bounded finite rate, "
            "nonnegative weight and positive finite release/transition distances");
  }
  RCLCPP_INFO(logger_, "Continuous spin objective: rate=%.3f rad/s, release=%.3f m, ramp=%.3f m",
    target_rate_, release_distance_, transition_distance_);
}

void ContinuousSpinCritic::score(CriticData & data)
{
  if (!enabled_ || data.fail_flag) {return;}
  if (!data.motion_model || !data.motion_model->isHolonomic()) {
    throw std::logic_error("ContinuousSpinCritic requires a holonomic motion model");
  }
  const auto & pose = data.state.pose.pose.position;
  const auto & goal = data.goal.position;
  // Let the existing goal-angle objective and stopped goal checker take over.
  if (std::hypot(pose.x - goal.x, pose.y - goal.y) <= release_distance_) {return;}
  const auto steps = data.state.wz.shape()[1];
  if (steps == 0) {return;}
  for (size_t i = 0; i < data.costs.size(); ++i) {
    float cost = 0;
    for (size_t j = 0; j < steps; ++j) {
      const float distance = std::hypot(
        data.trajectories.x(i, j) - goal.x, data.trajectories.y(i, j) - goal.y);
      const float fraction = std::clamp(
        (distance - release_distance_) / transition_distance_, 0.0F, 1.0F);
      const float activation = fraction * fraction * (3.0F - 2.0F * fraction);
      cost += activation * std::abs(data.state.wz(i, j) - target_rate_ * activation);
    }
    // Add to, never replace, collision and constraint costs.
    data.costs(i) += weight_ * cost / static_cast<float>(steps);
  }
}
}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::ContinuousSpinCritic, mppi::critics::CriticFunction)
