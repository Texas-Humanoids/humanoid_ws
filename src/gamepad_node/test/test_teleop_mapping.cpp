#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gamepad_node/teleop_mapping.hpp"

using gamepad_node::JoyAxisConfig;
using gamepad_node::KeyState;
using gamepad_node::ScaleConfig;
using gamepad_node::Velocity;
using gamepad_node::button_name;
using gamepad_node::is_enabled;
using gamepad_node::joy_to_velocity;
using gamepad_node::keys_to_velocity;
using gamepad_node::newly_pressed;
using gamepad_node::validate;

namespace
{

ScaleConfig test_scales()
{
  ScaleConfig s;
  s.linear_x = 0.5;
  s.linear_y = 0.4;
  s.angular_z = 1.0;
  s.keyboard_scaling = 0.8;
  return s;
}

// Xbox 360 layout from joy_node: 8 axes, all centred.
std::vector<float> centred_axes()
{
  return std::vector<float>(8, 0.0f);
}

void expect_velocity(const Velocity & v, double x, double y, double z)
{
  EXPECT_DOUBLE_EQ(v.linear_x, x);
  EXPECT_DOUBLE_EQ(v.linear_y, y);
  EXPECT_DOUBLE_EQ(v.angular_z, z);
}

}  // namespace

// ---------- Controller ----------

TEST(JoyToVelocity, CentredSticksGiveZero)
{
  expect_velocity(joy_to_velocity(centred_axes(), JoyAxisConfig{}, test_scales()), 0.0, 0.0, 0.0);
}

TEST(JoyToVelocity, LeftStickUpIsForward)
{
  auto axes = centred_axes();
  axes[1] = 1.0f;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.5, 0.0, 0.0);
}

TEST(JoyToVelocity, LeftStickLeftIsPositiveY)
{
  auto axes = centred_axes();
  axes[0] = 1.0f;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.0, 0.4, 0.0);
}

TEST(JoyToVelocity, LeftStickDiagonalCombinesXAndY)
{
  auto axes = centred_axes();
  axes[0] = -1.0f;  // right
  axes[1] = 1.0f;   // up
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.5, -0.4, 0.0);
}

TEST(JoyToVelocity, RightStickXIsTurningOnly)
{
  auto axes = centred_axes();
  axes[3] = -0.5f;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.0, 0.0, -0.5);
}

TEST(JoyToVelocity, RightStickYAndTriggersAreIgnored)
{
  auto axes = centred_axes();
  axes[2] = 1.0f;  // left trigger at rest
  axes[4] = 1.0f;  // right stick Y
  axes[5] = 1.0f;  // right trigger at rest
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.0, 0.0, 0.0);
}

TEST(JoyToVelocity, PartialDeflectionScalesLinearly)
{
  auto axes = centred_axes();
  axes[1] = 0.5f;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.25, 0.0, 0.0);
}

TEST(JoyToVelocity, OutOfRangeAxesAreClamped)
{
  auto axes = centred_axes();
  axes[1] = 2.0f;
  axes[3] = -3.0f;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.5, 0.0, -1.0);
}

TEST(JoyToVelocity, NonFiniteAxesAreZero)
{
  auto axes = centred_axes();
  axes[1] = std::numeric_limits<float>::quiet_NaN();
  axes[3] = std::numeric_limits<float>::infinity();
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, test_scales()), 0.0, 0.0, 0.0);
}

TEST(JoyToVelocity, MissingAxesAreZero)
{
  expect_velocity(joy_to_velocity({}, JoyAxisConfig{}, test_scales()), 0.0, 0.0, 0.0);
}

TEST(JoyToVelocity, CustomAxisIndicesAreUsed)
{
  auto axes = centred_axes();
  axes[6] = 1.0f;
  JoyAxisConfig a;
  a.forward = 6;
  expect_velocity(joy_to_velocity(axes, a, test_scales()), 0.5, 0.0, 0.0);
}

TEST(JoyToVelocity, ControllerIgnoresKeyboardScaling)
{
  auto axes = centred_axes();
  axes[1] = 1.0f;
  ScaleConfig s = test_scales();
  s.keyboard_scaling = 0.1;
  expect_velocity(joy_to_velocity(axes, JoyAxisConfig{}, s), 0.5, 0.0, 0.0);
}

// ---------- Keyboard ----------

TEST(KeysToVelocity, NoKeysGiveZero)
{
  expect_velocity(keys_to_velocity(KeyState{}, test_scales()), 0.0, 0.0, 0.0);
}

TEST(KeysToVelocity, EachKeyMatchesControllerDirection)
{
  const ScaleConfig s = test_scales();
  KeyState k;

  k = KeyState{};
  k.forward = true;
  expect_velocity(keys_to_velocity(k, s), 0.4, 0.0, 0.0);
  k = KeyState{};
  k.back = true;
  expect_velocity(keys_to_velocity(k, s), -0.4, 0.0, 0.0);
  k = KeyState{};
  k.left = true;
  expect_velocity(keys_to_velocity(k, s), 0.0, 0.32, 0.0);
  k = KeyState{};
  k.right = true;
  expect_velocity(keys_to_velocity(k, s), 0.0, -0.32, 0.0);
  k = KeyState{};
  k.turn_left = true;
  expect_velocity(keys_to_velocity(k, s), 0.0, 0.0, 0.8);
  k = KeyState{};
  k.turn_right = true;
  expect_velocity(keys_to_velocity(k, s), 0.0, 0.0, -0.8);
}

TEST(KeysToVelocity, OppositeKeysCancel)
{
  KeyState k;
  k.forward = true;
  k.back = true;
  k.left = true;
  k.right = true;
  k.turn_left = true;
  k.turn_right = true;
  expect_velocity(keys_to_velocity(k, test_scales()), 0.0, 0.0, 0.0);
}

TEST(KeysToVelocity, KeysOnDifferentAxesCombine)
{
  KeyState k;
  k.forward = true;
  k.left = true;
  k.turn_right = true;
  expect_velocity(keys_to_velocity(k, test_scales()), 0.4, 0.32, -0.8);
}

TEST(KeysToVelocity, KeyboardIsLimitedByKeyboardScaling)
{
  auto axes = centred_axes();
  axes[1] = 1.0f;
  const ScaleConfig s = test_scales();
  KeyState k;
  k.forward = true;
  const double stick = joy_to_velocity(axes, JoyAxisConfig{}, s).linear_x;
  const double key = keys_to_velocity(k, s).linear_x;
  EXPECT_DOUBLE_EQ(key, stick * s.keyboard_scaling);
}

TEST(KeysToVelocity, FullKeyboardScalingMatchesFullStick)
{
  auto axes = centred_axes();
  axes[0] = 1.0f;
  axes[1] = 1.0f;
  axes[3] = 1.0f;
  ScaleConfig s = test_scales();
  s.keyboard_scaling = 1.0;
  KeyState k;
  k.forward = true;
  k.left = true;
  k.turn_left = true;
  const Velocity stick = joy_to_velocity(axes, JoyAxisConfig{}, s);
  expect_velocity(keys_to_velocity(k, s), stick.linear_x, stick.linear_y, stick.angular_z);
}

// ---------- Enable button ----------

TEST(IsEnabled, HeldButtonEnables)
{
  EXPECT_TRUE(is_enabled({0, 0, 0, 0, 1}, 4));
}

TEST(IsEnabled, ReleasedButtonDisables)
{
  EXPECT_FALSE(is_enabled({0, 0, 0, 0, 0}, 4));
}

TEST(IsEnabled, MissingButtonDisables)
{
  EXPECT_FALSE(is_enabled({1, 1}, 4));
}

TEST(IsEnabled, NegativeIndexMeansAlwaysEnabled)
{
  EXPECT_TRUE(is_enabled({}, -1));
}

// ---------- Button events ----------

TEST(NewlyPressed, ReportsRisingEdgesOnly)
{
  const std::vector<int32_t> prev = {0, 1, 0, 1};
  const std::vector<int32_t> curr = {1, 1, 0, 0};
  const auto pressed = newly_pressed(prev, curr);
  ASSERT_EQ(pressed.size(), 1u);
  EXPECT_EQ(pressed[0], 0u);
}

TEST(NewlyPressed, HandlesEmptyPreviousState)
{
  const auto pressed = newly_pressed({}, {1, 0, 1});
  ASSERT_EQ(pressed.size(), 2u);
  EXPECT_EQ(pressed[0], 0u);
  EXPECT_EQ(pressed[1], 2u);
}

TEST(ButtonName, KnownAndUnknownButtons)
{
  EXPECT_EQ(button_name(0), "A");
  EXPECT_EQ(button_name(4), "LB");
  EXPECT_EQ(button_name(10), "R_STICK");
  EXPECT_EQ(button_name(15), "BUTTON_15");
}

// ---------- Validation ----------

TEST(Validate, AcceptsDefaults)
{
  EXPECT_NO_THROW(validate(ScaleConfig{}));
}

TEST(Validate, RejectsNonFiniteScale)
{
  ScaleConfig s;
  s.angular_z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate(s), std::invalid_argument);
}

TEST(Validate, RejectsKeyboardScalingOutsideZeroToOne)
{
  ScaleConfig s;
  s.keyboard_scaling = 1.5;
  EXPECT_THROW(validate(s), std::invalid_argument);
  s.keyboard_scaling = -0.1;
  EXPECT_THROW(validate(s), std::invalid_argument);
}

