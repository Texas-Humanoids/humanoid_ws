#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace th_policy_bridge
{

// Calibration between a policy joint angle and its motor's position:
// motor = direction * joint + offset. Limits are in joint space.
struct JointMotor
{
  double direction;
  double offset;
  double lower;
  double upper;
};

inline void validate(const JointMotor & m, const std::string & joint)
{
  if ((m.direction != 1.0 && m.direction != -1.0) || !std::isfinite(m.offset) ||
    !std::isfinite(m.lower) || !std::isfinite(m.upper) || m.lower >= m.upper)
  {
    throw std::invalid_argument(
            "Invalid motor calibration for " + joint +
            ": direction must be 1 or -1, offset and limits finite, lower < upper");
  }
}

inline double joint_to_motor(const JointMotor & m, double joint_target)
{
  if (!std::isfinite(joint_target)) {
    throw std::invalid_argument("Joint target is non-finite");
  }
  return m.direction * std::clamp(joint_target, m.lower, m.upper) + m.offset;
}

inline double motor_to_joint(const JointMotor & m, double motor_position)
{
  return m.direction * (motor_position - m.offset);
}

}  // namespace th_policy_bridge
