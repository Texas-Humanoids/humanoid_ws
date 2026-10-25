#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include "th_policy_node/policy_profile.hpp"

TEST(PolicyProfile, AcceptsOneVectorOrOneDynamicBatch)
{
  EXPECT_EQ(
    th_policy_node::validate_vector_shape({75}, 75, "input"),
    (std::vector<int64_t>{75}));
  EXPECT_EQ(
    th_policy_node::validate_vector_shape({-1, 75}, 75, "input"),
    (std::vector<int64_t>{1, 75}));
}

TEST(PolicyProfile, RejectsWrongModelShapes)
{
  EXPECT_THROW(th_policy_node::validate_vector_shape({}, 75, "input"), std::invalid_argument);
  EXPECT_THROW(th_policy_node::validate_vector_shape({74}, 75, "input"), std::invalid_argument);
  EXPECT_THROW(th_policy_node::validate_vector_shape({2, 75}, 75, "input"), std::invalid_argument);
  EXPECT_THROW(th_policy_node::validate_vector_shape({1, 75, 1}, 75, "input"), std::invalid_argument);
}

TEST(PolicyProfile, ClipsScalesAndOffsetsActions)
{
  const float actions[] = {-2.0F, 0.5F, 3.0F};
  const auto targets = th_policy_node::scale_actions(
    actions, 3, {0.1, -0.2, 0.3}, 0.25, -1.0, 1.0);
  ASSERT_EQ(targets.size(), 3U);
  EXPECT_NEAR(targets[0], -0.15, 1e-6);
  EXPECT_NEAR(targets[1], -0.075, 1e-6);
  EXPECT_NEAR(targets[2], 0.55, 1e-6);
}

TEST(PolicyProfile, RejectsInvalidActionValuesAndProfiles)
{
  const float good[] = {0.0F};
  const float bad[] = {std::numeric_limits<float>::quiet_NaN()};
  EXPECT_THROW(
    th_policy_node::scale_actions(bad, 1, {0.0}, 0.25, -1.0, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    th_policy_node::scale_actions(good, 1, {0.0, 0.0}, 0.25, -1.0, 1.0),
    std::invalid_argument);
  EXPECT_THROW(
    th_policy_node::scale_actions(good, 1, {0.0}, 0.25, 1.0, -1.0),
    std::invalid_argument);
  EXPECT_THROW(
    th_policy_node::scale_actions(good, 1, {std::numeric_limits<double>::infinity()},
      0.25, -1.0, 1.0),
    std::invalid_argument);
}

TEST(PolicyProfile, RejectsTargetOverflow)
{
  const float action[] = {2.0F};
  EXPECT_THROW(
    th_policy_node::scale_actions(
      action, 1, {0.0}, std::numeric_limits<double>::max(), -1.0, 2.0),
    std::overflow_error);
}
