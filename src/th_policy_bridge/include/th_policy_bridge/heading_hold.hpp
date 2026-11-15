#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace th_policy_bridge
{

// Keeps the robot's heading while the operator walks it without turning.
//
// A velocity-tracking policy only sees its yaw *rate*, never its heading, so small yaw errors
// add up and the robot curves. While the operator commands linear motion and no turn, this
// adds a yaw-rate correction toward the heading the robot had when that started:
//   wz = clamp(gain * (reference - heading), -max_rate, max_rate)
// Standing still or turning passes the command through and moves the reference along, so the
// robot never turns on its own while standing and keeps the heading the operator turned to.
//
// The heading is integrated from the gyro (base angular velocity projected on world vertical),
// not taken from the IMU's absolute yaw, which on a VN-100 comes from the magnetometer and is
// disturbed by the motor magnets. Gyro drift is negligible over a walk of a few seconds.
class HeadingHold
{
public:
  HeadingHold(double gain, double max_rate, double deadband = 0.05)
  : gain_(gain), max_rate_(max_rate), deadband_(deadband)
  {
    if (!std::isfinite(gain) || gain < 0.0 || !std::isfinite(max_rate) || max_rate < 0.0 ||
      !std::isfinite(deadband) || deadband < 0.0)
    {
      throw std::invalid_argument(
              "heading hold gain, max_rate and deadband must be finite and >= 0 (gain 0 disables)");
    }
  }

  bool enabled() const {return gain_ > 0.0;}

  // Forget the accumulated heading, e.g. when walking (re)starts.
  void reset()
  {
    heading_ = 0.0;
    reference_ = 0.0;
  }

  // command: operator (vx, vy, wz). gravity / angular_velocity: base frame, as in the
  // observation (gravity is world down, unit length). dt: seconds since the last call.
  // Returns the command to give the policy.
  std::array<double, 3> update(
    const std::array<double, 3> & command, const std::array<double, 3> & gravity,
    const std::array<double, 3> & angular_velocity, double dt)
  {
    // Yaw rate about world up = -(world down . angular velocity), both in the base frame.
    const double yaw_rate = -(gravity[0] * angular_velocity[0] + gravity[1] * angular_velocity[1] +
      gravity[2] * angular_velocity[2]);
    if (std::isfinite(yaw_rate) && std::isfinite(dt) && dt > 0.0) {
      // A stall (late tick) would otherwise stretch one gyro sample over the whole gap.
      heading_ = wrap(heading_ + yaw_rate * std::min(dt, kMaxStep));
    }
    const bool moving = std::hypot(command[0], command[1]) > deadband_;
    const bool turning = std::abs(command[2]) > deadband_;
    if (!enabled() || !moving || turning) {
      reference_ = heading_;
      return command;
    }
    const double correction =
      std::clamp(gain_ * wrap(reference_ - heading_), -max_rate_, max_rate_);
    return {command[0], command[1], correction};
  }

  double heading() const {return heading_;}
  double reference() const {return reference_;}

private:
  static double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
  static constexpr double kMaxStep = 0.1;  // s; ticks are 0.02 s at 50 Hz

  double gain_;
  double max_rate_;
  double deadband_;
  double heading_{0.0};
  double reference_{0.0};
};

}  // namespace th_policy_bridge
