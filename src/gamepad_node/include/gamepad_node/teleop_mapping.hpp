#ifndef GAMEPAD_NODE__TELEOP_MAPPING_HPP_
#define GAMEPAD_NODE__TELEOP_MAPPING_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

// Joy axis indices (SDL game controller layout from joy's game_controller_node).
struct JoyAxisConfig
{
  int forward{1};  // left stick up/down (up = +)
  int lateral{0};  // left stick left/right (left = +)
  int turn{2};     // right stick left/right (left = +)
};

struct KeyState
{
  bool forward{false};     // W
  bool back{false};        // S
  bool left{false};        // A
  bool right{false};       // D
  bool turn_left{false};   // Left arrow
  bool turn_right{false};  // Right arrow
};

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

// Reads one axis safely: missing index -> 0, NaN/inf -> 0, otherwise clamped to [-1, 1].
inline double axis_value(const std::vector<float> & axes, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= axes.size()) {
    return 0.0;
  }
  const double v = axes[static_cast<std::size_t>(index)];
  return std::isfinite(v) ? std::clamp(v, -1.0, 1.0) : 0.0;
}

// Controller: left stick = translation (x, y), right stick X = turning only.
inline Velocity joy_to_velocity(
  const std::vector<float> & axes, const JoyAxisConfig & a, const ScaleConfig & s)
{
  return {axis_value(axes, a.forward) * s.linear_x + 0.0,
    axis_value(axes, a.lateral) * s.linear_y + 0.0,
    axis_value(axes, a.turn) * s.angular_z + 0.0};
}

// Keyboard: same range as the controller ([-1, 1] per axis), times keyboard_scaling.
inline Velocity keys_to_velocity(const KeyState & k, const ScaleConfig & s)
{
  // Subtraction cancels opposite keys; +0.0 normalizes negative zero.
  return {(k.forward - k.back) * s.linear_x * s.keyboard_scaling + 0.0,
    (k.left - k.right) * s.linear_y * s.keyboard_scaling + 0.0,
    (k.turn_left - k.turn_right) * s.angular_z * s.keyboard_scaling + 0.0};
}

// Xbox button names in the SDL game controller layout from joy's game_controller_node.
// The layout is the same for any controller SDL knows, wired or Bluetooth.
inline std::string button_name(std::size_t index)
{
  static const std::array<const char *, 15> names = {
    "A", "B", "X", "Y", "BACK", "GUIDE", "START", "L_STICK", "R_STICK", "LB", "RB",
    "DPAD_UP", "DPAD_DOWN", "DPAD_LEFT", "DPAD_RIGHT"};
  return index < names.size() ? names[index] : "BUTTON_" + std::to_string(index);
}

}  // namespace gamepad_node

#endif  // GAMEPAD_NODE__TELEOP_MAPPING_HPP_
