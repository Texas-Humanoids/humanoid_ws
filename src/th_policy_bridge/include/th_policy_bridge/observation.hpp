#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace th_policy_bridge
{

struct Quaternion
{
  double w;
  double x;
  double y;
  double z;
};

// World gravity direction (0, 0, -1) expressed in the body frame of orientation q.
inline std::array<double, 3> projected_gravity(const Quaternion & q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1e-6) {
    throw std::invalid_argument("IMU orientation is not a valid quaternion");
  }
  const double w = q.w / norm;
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  // Negated third row of the body-to-world rotation matrix.
  return {-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(1.0 - 2.0 * (x * x + y * y))};
}

// Recovers the clipped policy actions from published targets (target = default + scale * action).
inline std::vector<double> actions_from_targets(
  const std::vector<double> & targets, const std::vector<double> & defaults, double scale)
{
  if (targets.size() != defaults.size() || !std::isfinite(scale) || scale == 0.0) {
    throw std::invalid_argument("Invalid target profile");
  }
  std::vector<double> actions;
  actions.reserve(targets.size());
  for (size_t i = 0; i < targets.size(); ++i) {
    const double action = (targets[i] - defaults[i]) / scale;
    if (!std::isfinite(action)) {
      throw std::invalid_argument("Joint target is non-finite");
    }
    actions.push_back(action);
  }
  return actions;
}

struct ObservationInputs
{
  std::array<double, 3> command;  // forward m/s, lateral m/s, yaw rad/s
  std::array<double, 3> angular_velocity;  // body frame, rad/s
  std::array<double, 3> gravity;  // projected gravity, body frame
  std::vector<double> joint_positions;  // absolute, rad
  std::vector<double> joint_velocities;  // rad/s
  std::vector<double> previous_actions;
};

// Berkeley order: command, angular velocity, gravity, joint positions relative
// to the default pose, joint velocities, previous actions.
inline std::vector<float> build_observation(
  const ObservationInputs & in, const std::vector<double> & defaults)
{
  const size_t n = defaults.size();
  if (n == 0 || in.joint_positions.size() != n || in.joint_velocities.size() != n ||
    in.previous_actions.size() != n)
  {
    throw std::invalid_argument("Observation joint vectors have the wrong size");
  }
  std::vector<float> observation;
  observation.reserve(9 + 3 * n);
  const auto append = [&observation](double value) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Observation value is non-finite");
      }
      observation.push_back(static_cast<float>(value));
    };
  for (double value : in.command) {append(value);}
  for (double value : in.angular_velocity) {append(value);}
  for (double value : in.gravity) {append(value);}
  for (size_t i = 0; i < n; ++i) {append(in.joint_positions[i] - defaults[i]);}
  for (double value : in.joint_velocities) {append(value);}
  for (double value : in.previous_actions) {append(value);}
  return observation;
}

}  // namespace th_policy_bridge
