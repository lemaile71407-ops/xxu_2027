#pragma once

#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{
// An opt-in MPPI objective: rotation is predicted with the translational motion,
// rather than added to the command after collision scoring.
class ContinuousSpinCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  float target_rate_{0.5F};
  float weight_{64.0F};
  float release_distance_{0.65F};
  float transition_distance_{0.35F};
};
}  // namespace mppi::critics
