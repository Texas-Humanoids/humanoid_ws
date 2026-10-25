#ifndef GAMEPAD_NODE__TELEOP_MAPPING_HPP_
#define GAMEPAD_NODE__TELEOP_MAPPING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace gamepad_node
{

// Velocity command in the robot body frame (ROS REP 103).
struct Velocity
{
  double linear_x{0.0};   // forward (+) / backward (-), m/s
  double linear_y{0.0};   // left (+) / right (-), m/s
  double angular_z{0.0};  // turn left (+) / turn right (-), rad/s
};

// Speed at full input, shared by controller and keyboard.
struct ScaleConfig
{
  double linear_x{0.5};
  double linear_y{0.5};
  double angular_z{1.0};
  double keyboard_scaling{0.8};  // keyboard max speed = this fraction of full speed
};

// Joy axis indices (Xbox 360 layout from joy_node).
struct JoyAxisConfig
{
  int forward{1};  // left stick up/down (up = +)
  int lateral{0};  // left stick left/right (left = +)
  int turn{3};     // right stick left/right (left = +)
};

// Which keyboard keys are currently held.
struct KeyState
{
  bool forward{false};     // W
  bool back{false};        // S
  bool left{false};        // A
  bool right{false};       // D
  bool turn_left{false};   // Left arrow
  bool turn_right{false};  // Right arrow
};

// Throws std::invalid_argument if the configuration is unusable.
inline void validate(const ScaleConfig & s)
{
  if (!std::isfinite(s.linear_x) || !std::isfinite(s.linear_y) || !std::isfinite(s.angular_z)) {
    throw std::invalid_argument(
            "scale.linear.x, scale.linear.y and scale.angular.z must be finite numbers");
  }
  if (!std::isfinite(s.keyboard_scaling) || s.keyboard_scaling < 0.0 ||
    s.keyboard_scaling > 1.0)
  {
    throw std::invalid_argument("keyboard_scaling must be between 0.0 and 1.0");
  }
}

// Turns -0.0 into 0.0 so outputs never show as "-0.00".
inline double positive_zero(double v)
{
  return v + 0.0;
}

// Reads one axis safely: missing index -> 0, NaN/inf -> 0, otherwise clamped to [-1, 1].
inline double axis_value(const std::vector<float> & axes, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
    return 0.0;
  }
  const double v = axes[static_cast<std::size_t>(index)];
  if (!std::isfinite(v)) {
    return 0.0;
  }
  return std::clamp(v, -1.0, 1.0);
}

// Controller: left stick = translation (x, y), right stick X = turning only.
inline Velocity joy_to_velocity(
  const std::vector<float> & axes, const JoyAxisConfig & a, const ScaleConfig & s)
{
  Velocity v;
  v.linear_x = positive_zero(axis_value(axes, a.forward) * s.linear_x);
  v.linear_y = positive_zero(axis_value(axes, a.lateral) * s.linear_y);
  v.angular_z = positive_zero(axis_value(axes, a.turn) * s.angular_z);
  return v;
}

// +1 if only the positive key is held, -1 if only the negative one, 0 if neither or both.
inline int key_direction(bool positive, bool negative)
{
  return static_cast<int>(positive) - static_cast<int>(negative);
}

// Keyboard: same range as the controller ([-1, 1] per axis), times keyboard_scaling.
inline Velocity keys_to_velocity(const KeyState & k, const ScaleConfig & s)
{
  Velocity v;
  v.linear_x = positive_zero(
    key_direction(k.forward, k.back) * s.linear_x * s.keyboard_scaling);
  v.linear_y = positive_zero(
    key_direction(k.left, k.right) * s.linear_y * s.keyboard_scaling);
  v.angular_z = positive_zero(
    key_direction(k.turn_left, k.turn_right) * s.angular_z * s.keyboard_scaling);
  return v;
}

// True if the enable (deadman) button is held. enable_button < 0 means always enabled.
inline bool is_enabled(const std::vector<int32_t> & buttons, int enable_button)
{
  if (enable_button < 0) {
    return true;
  }
  const auto i = static_cast<std::size_t>(enable_button);
  return i < buttons.size() && buttons[i] == 1;
}

// Indices of buttons that went from released to pressed.
inline std::vector<std::size_t> newly_pressed(
  const std::vector<int32_t> & previous, const std::vector<int32_t> & current)
{
  std::vector<std::size_t> pressed;
  for (std::size_t i = 0; i < current.size(); ++i) {
    const bool was_down = i < previous.size() && previous[i] != 0;
    if (current[i] != 0 && !was_down) {
      pressed.push_back(i);
    }
  }
  return pressed;
}

// Xbox 360 button names as reported by joy_node.
inline std::string button_name(std::size_t index)
{
  static const std::array<const char *, 11> names = {
    "A", "B", "X", "Y", "LB", "RB", "BACK", "START", "GUIDE", "L_STICK", "R_STICK"};
  if (index < names.size()) {
    return names[index];
  }
  return "BUTTON_" + std::to_string(index);
}

}  // namespace gamepad_node

#endif  // GAMEPAD_NODE__TELEOP_MAPPING_HPP_
