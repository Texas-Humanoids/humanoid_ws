#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "th_policy_bridge/joint_mapping.hpp"
#include "th_policy_bridge/observation.hpp"

namespace tb = th_policy_bridge;

TEST(ProjectedGravity, UprightPointsDown)
{
  const auto g = tb::projected_gravity({1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(g[0], 0.0, 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], -1.0, 1e-12);
}

TEST(ProjectedGravity, PitchedForwardTiltsGravityTowardBodyX)
{
  // +90 degrees about body y (nose down): gravity lies along body +x.
  const double h = std::sqrt(0.5);
  const auto g = tb::projected_gravity({h, 0.0, h, 0.0});
  EXPECT_NEAR(g[0], 1.0, 1e-12);
  EXPECT_NEAR(g[1], 0.0, 1e-12);
  EXPECT_NEAR(g[2], 0.0, 1e-12);
}

TEST(ProjectedGravity, RolledLeftAndUnnormalized)
{
  // +90 degrees about body x, scaled by 2: gravity lies along body -y.
  const double h = 2.0 * std::sqrt(0.5);
  const auto g = tb::projected_gravity({h, h, 0.0, 0.0});
  EXPECT_NEAR(g[0], 0.0, 1e-12);
  EXPECT_NEAR(g[1], -1.0, 1e-12);
  EXPECT_NEAR(g[2], 0.0, 1e-12);
}

TEST(ProjectedGravity, YawDoesNotChangeGravity)
{
  const auto g = tb::projected_gravity({std::cos(0.4), 0.0, 0.0, std::sin(0.4)});
  EXPECT_NEAR(g[2], -1.0, 1e-12);
}

TEST(ProjectedGravity, RejectsInvalidQuaternions)
{
  EXPECT_THROW(tb::projected_gravity({0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(
    tb::projected_gravity({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0}),
    std::invalid_argument);
}

TEST(Actions, InvertsPolicyTargetScaling)
{
  const auto actions = tb::actions_from_targets({0.25, -0.2, 0.9}, {0.0, -0.2, 0.4}, 0.25);
  ASSERT_EQ(actions.size(), 3u);
  EXPECT_DOUBLE_EQ(actions[0], 1.0);
  EXPECT_DOUBLE_EQ(actions[1], 0.0);
  EXPECT_DOUBLE_EQ(actions[2], 2.0);
  EXPECT_THROW(tb::actions_from_targets({0.0}, {0.0, 0.0}, 0.25), std::invalid_argument);
  EXPECT_THROW(tb::actions_from_targets({0.0}, {0.0}, 0.0), std::invalid_argument);
}

TEST(Observation, FollowsBerkeleyOrder)
{
  tb::ObservationInputs in;
  in.command = {0.1, 0.2, 0.3};
  in.angular_velocity = {1.0, 2.0, 3.0};
  in.gravity = {0.0, 0.0, -1.0};
  in.joint_positions = {0.5, 0.4};
  in.joint_velocities = {-1.0, 1.0};
  in.previous_actions = {7.0, 8.0};
  const auto obs = tb::build_observation(in, {0.5, 0.1});
  const std::vector<float> expected{
    0.1F, 0.2F, 0.3F, 1.0F, 2.0F, 3.0F, 0.0F, 0.0F, -1.0F,
    0.0F, 0.3F, -1.0F, 1.0F, 7.0F, 8.0F};
  ASSERT_EQ(obs.size(), expected.size());
  for (size_t i = 0; i < obs.size(); ++i) {
    EXPECT_NEAR(obs[i], expected[i], 1e-6) << "index " << i;
  }
}

TEST(Observation, BerkeleyProfileHas75Values)
{
  tb::ObservationInputs in{};
  in.joint_positions.assign(22, 0.0);
  in.joint_velocities.assign(22, 0.0);
  in.previous_actions.assign(22, 0.0);
  EXPECT_EQ(tb::build_observation(in, std::vector<double>(22, 0.0)).size(), 75u);
}

TEST(Observation, RejectsWrongSizesAndNonFiniteValues)
{
  tb::ObservationInputs in{};
  in.joint_positions = {0.0};
  in.joint_velocities = {0.0};
  in.previous_actions = {0.0};
  EXPECT_THROW(tb::build_observation(in, {0.0, 0.0}), std::invalid_argument);
  in.joint_velocities = {std::numeric_limits<double>::infinity()};
  EXPECT_THROW(tb::build_observation(in, {0.0}), std::invalid_argument);
}

TEST(JointMapping, AppliesLimitsDirectionAndOffset)
{
  const tb::JointMotor m{-1.0, 0.5, -0.2, 1.0};
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, 0.3), 0.2);
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, 5.0), -0.5);  // clamped to upper = 1.0
  EXPECT_DOUBLE_EQ(tb::joint_to_motor(m, -5.0), 0.7);  // clamped to lower = -0.2
  EXPECT_DOUBLE_EQ(tb::motor_to_joint(m, 0.2), 0.3);
  EXPECT_THROW(
    tb::joint_to_motor(m, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(JointMapping, RejectsInvalidCalibration)
{
  EXPECT_NO_THROW(tb::validate({1.0, 0.0, -1.0, 1.0}, "j"));
  EXPECT_THROW(tb::validate({0.5, 0.0, -1.0, 1.0}, "j"), std::invalid_argument);
  EXPECT_THROW(tb::validate({1.0, 0.0, 1.0, 1.0}, "j"), std::invalid_argument);
  EXPECT_THROW(
    tb::validate({1.0, std::numeric_limits<double>::infinity(), -1.0, 1.0}, "j"),
    std::invalid_argument);
}
