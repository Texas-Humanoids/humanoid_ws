#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "gamepad_node/keyboard_input.hpp"

namespace
{

namespace gt = gamepad_node;
const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();

void expect_velocity(const gt::Velocity & actual, const gt::Velocity & expected)
{
  EXPECT_DOUBLE_EQ(actual.linear_x, expected.linear_x);
  EXPECT_DOUBLE_EQ(actual.linear_y, expected.linear_y);
  EXPECT_DOUBLE_EQ(actual.angular_z, expected.angular_z);
}

TEST(InputMapping, ControllerAxes)
{
  const struct Case
  {
    std::vector<float> axes;
    gt::Velocity expected;
  } cases[] = {
    {{}, {}},                                    // Missing axes.
    {{0, 0, 0, 0}, {}},                          // Centered sticks.
    {{0, 1, 0, 0}, {0.5, 0, 0}},                 // Forward.
    {{1, 0, 0, 0}, {0, 0.4, 0}},                 // Sidestep left.
    {{-1, 1, 0, 0}, {0.5, -0.4, 0}},             // Diagonal.
    {{0, 0, -0.5, 0}, {0, 0, -0.5}},             // Turn right.
    {{0, 0, 0, 1, 1, 1, 0, 0}, {}},             // Unmapped axes.
    {{0, 0.5, 0, 0}, {0.25, 0, 0}},              // Partial deflection.
    {{0, 2, -3, 0}, {0.5, 0, -1}},               // Clamping.
    {{0, kNaN, kInf, 0}, {}}                     // Invalid values.
  };
  const gt::ScaleConfig scales{0.5, 0.4, 1.0, 0.1};
  for (std::size_t i = 0; i < std::size(cases); ++i) {
    SCOPED_TRACE(i);
    expect_velocity(gt::joy_to_velocity(cases[i].axes, {}, scales), cases[i].expected);
  }
  expect_velocity(gt::joy_to_velocity({0, 0, 1}, {2, -1, 9}, scales), {0.5, 0, 0});
}

TEST(InputMapping, KeyboardDirectionsAndScaling)
{
  const struct Case
  {
    gt::KeyState keys;  // W, S, A, D, Left, Right.
    gt::Velocity full_speed;
  } cases[] = {
    {{}, {}},
    {{true}, {0.5, 0, 0}},
    {{false, true}, {-0.5, 0, 0}},
    {{false, false, true}, {0, 0.4, 0}},
    {{false, false, false, true}, {0, -0.4, 0}},
    {{false, false, false, false, true}, {0, 0, 1}},
    {{false, false, false, false, false, true}, {0, 0, -1}},
    {{true, true, true, true, true, true}, {}},
    {{true, false, true, false, false, true}, {0.5, 0.4, -1}}
  };
  for (double fraction : {0.0, 0.8, 1.0}) {
    SCOPED_TRACE(fraction);
    for (std::size_t i = 0; i < std::size(cases); ++i) {
      SCOPED_TRACE(i);
      const auto & v = cases[i].full_speed;
      expect_velocity(gt::keys_to_velocity(cases[i].keys, {0.5, 0.4, 1, fraction}),
        {v.linear_x * fraction, v.linear_y * fraction, v.angular_z * fraction});
    }
  }
}

TEST(InputMapping, ButtonNames)
{
  EXPECT_EQ(gt::button_name(0), "A");
  EXPECT_EQ(gt::button_name(4), "BACK");
  EXPECT_EQ(gt::button_name(6), "START");
  EXPECT_EQ(gt::button_name(9), "LB");
  EXPECT_EQ(gt::button_name(14), "DPAD_RIGHT");
  EXPECT_EQ(gt::button_name(15), "BUTTON_15");
}

TEST(InputMapping, ConfigurationValidation)
{
  EXPECT_NO_THROW(gt::validate(gt::ScaleConfig{}));
  for (double invalid : {double(kNaN), double(kInf)}) {
    EXPECT_THROW(gt::validate((gt::ScaleConfig{invalid, 0.4, 1, 0.8})), std::invalid_argument);
    EXPECT_THROW(gt::validate((gt::ScaleConfig{0.5, invalid, 1, 0.8})), std::invalid_argument);
    EXPECT_THROW(gt::validate((gt::ScaleConfig{0.5, 0.4, invalid, 0.8})), std::invalid_argument);
  }
  for (double invalid : {-0.1, 1.1, double(kNaN), double(kInf)}) {
    EXPECT_THROW(gt::validate((gt::ScaleConfig{0.5, 0.4, 1, invalid})), std::invalid_argument);
  }
}

input_event event(unsigned short type, unsigned short code, int value = 0)
{
  input_event ev{};
  ev.type = type;
  ev.code = code;
  ev.value = value;
  return ev;
}

TEST(KeyboardEvents, PressRepeatReleaseAndReset)
{
  gt::KeyboardInput input;
  const auto snapshot = [](gt::KeyState &) {return false;};
  const struct KeyCase
  {
    unsigned short code;
    const char * name;
    gt::Velocity expected;
  } cases[] = {
    {KEY_W, "w", {0.4, 0, 0}}, {KEY_S, "s", {-0.4, 0, 0}},
    {KEY_A, "a", {0, 0.4, 0}}, {KEY_D, "d", {0, -0.4, 0}},
    {KEY_LEFT, "left", {0, 0, 0.8}}, {KEY_RIGHT, "right", {0, 0, -0.8}}
  };
  for (const auto & key : cases) {
    SCOPED_TRACE(key.name);
    EXPECT_STREQ(input.handle_event(event(EV_KEY, key.code, 1), snapshot), key.name);
    EXPECT_EQ(input.handle_event(event(EV_KEY, key.code, 2), snapshot), nullptr);
    expect_velocity(gt::keys_to_velocity(input.keys(), {}), key.expected);
    EXPECT_EQ(input.handle_event(event(EV_KEY, key.code, 0), snapshot), nullptr);
    expect_velocity(gt::keys_to_velocity(input.keys(), {}), {});
  }
  EXPECT_EQ(input.handle_event(event(EV_KEY, KEY_SPACE, 1), snapshot), nullptr);
  EXPECT_EQ(input.handle_event(event(EV_MSC, KEY_W, 1), snapshot), nullptr);
  input.handle_event(event(EV_KEY, KEY_W, 1), snapshot);
  input.reset();
  EXPECT_FALSE(input.keys().forward);
}

TEST(KeyboardEvents, OverflowDiscardsStaleEventsAndResynchronizes)
{
  gt::KeyboardInput input;
  int reads = 0;
  bool readable = false;
  const auto snapshot = [&](gt::KeyState & state) {
      ++reads;
      state.turn_right = true;
      return readable;
    };
  input.handle_event(event(EV_KEY, KEY_W, 1), snapshot);
  for (int attempt = 0; attempt < 2; ++attempt) {
    input.handle_event(event(EV_SYN, SYN_DROPPED), snapshot);
    input.handle_event(event(EV_KEY, KEY_W, 1), snapshot);
    EXPECT_FALSE(input.keys().forward);
    EXPECT_EQ(reads, 0);
  }
  // Failed queries discard even partially filled snapshots and keep motion stopped.
  EXPECT_EQ(input.handle_event(event(EV_SYN, SYN_REPORT), snapshot), nullptr);
  EXPECT_FALSE(input.keys().turn_right);
  input.handle_event(event(EV_KEY, KEY_W, 1), snapshot);
  EXPECT_FALSE(input.keys().forward);
  readable = true;
  EXPECT_EQ(input.handle_event(event(EV_SYN, SYN_REPORT), snapshot), nullptr);
  EXPECT_FALSE(input.keys().forward);  // Dropped W release recovered.
  EXPECT_TRUE(input.keys().turn_right);  // A genuinely held key is restored.
  input.handle_event(event(EV_SYN, SYN_REPORT), snapshot);
  EXPECT_EQ(reads, 2);
  input.handle_event(event(EV_KEY, KEY_RIGHT, 0), snapshot);
  EXPECT_FALSE(input.keys().turn_right);
  input.handle_event(event(EV_SYN, SYN_DROPPED), snapshot);
  input.reset();
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_A, 1), snapshot), "a");
}

}  // namespace
