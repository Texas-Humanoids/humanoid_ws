#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
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

// Rotates v by unit quaternion q (q * v * q^-1).
inline std::array<double, 3> rotate(const Quaternion & q, const std::array<double, 3> & v)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1e-6) {
    throw std::invalid_argument("Rotation is not a valid quaternion");
  }
  const double w = q.w / norm, x = q.x / norm, y = q.y / norm, z = q.z / norm;
  // t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
  const double tx = 2.0 * (y * v[2] - z * v[1]);
  const double ty = 2.0 * (z * v[0] - x * v[2]);
  const double tz = 2.0 * (x * v[1] - y * v[0]);
  return {
    v[0] + w * tx + (y * tz - z * ty),
    v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)};
}

inline Quaternion conjugate(const Quaternion & q) {return {q.w, -q.x, -q.y, -q.z};}

// World gravity direction (0, 0, -1) expressed in the body frame of orientation q.
// For an IMU whose world frame is NED (z down), gravity is (0, 0, +1): pass
// world_z_down = true.
inline std::array<double, 3> projected_gravity(const Quaternion & q, bool world_z_down = false)
{
  return rotate(conjugate(q), {0.0, 0.0, world_z_down ? 1.0 : -1.0});
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
  double gait_phase{0.0};  // cycles, [0, 1)
};

// Observation terms; a policy's observation is a list of these in order.
enum class Term { kCommand, kAngularVelocity, kGravity, kJointPos, kJointVel, kLastAction,
  kGaitPhase };

inline Term parse_term(const std::string & name)
{
  if (name == "velocity_command") {return Term::kCommand;}
  if (name == "base_ang_vel") {return Term::kAngularVelocity;}
  if (name == "projected_gravity") {return Term::kGravity;}
  if (name == "joint_pos_rel") {return Term::kJointPos;}
  if (name == "joint_vel") {return Term::kJointVel;}
  if (name == "last_action") {return Term::kLastAction;}
  if (name == "gait_phase") {return Term::kGaitPhase;}
  throw std::invalid_argument(
          "Unknown observation term '" + name + "'; known: velocity_command, base_ang_vel, "
          "projected_gravity, joint_pos_rel, joint_vel, last_action, gait_phase");
}

inline size_t term_size(Term term, size_t joints)
{
  switch (term) {
    case Term::kCommand:
    case Term::kAngularVelocity:
    case Term::kGravity: return 3;
    case Term::kGaitPhase: return 2;
    default: return joints;
  }
}

struct ObservationLayout
{
  std::vector<Term> terms;
  std::vector<double> scales;  // one per term, multiplies every element of it

  size_t size(size_t joints) const
  {
    size_t total = 0;
    for (Term t : terms) {total += term_size(t, joints);}
    return total;
  }
};

// The Berkeley Humanoid layout: command, angular velocity, gravity, joint
// positions relative to default, joint velocities, previous actions; unscaled.
inline ObservationLayout berkeley_layout()
{
  return {{Term::kCommand, Term::kAngularVelocity, Term::kGravity, Term::kJointPos,
    Term::kJointVel, Term::kLastAction}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
}

inline ObservationLayout parse_layout(
  const std::vector<std::string> & names, const std::vector<double> & scales)
{
  if (names.empty()) {
    return berkeley_layout();
  }
  if (!scales.empty() && scales.size() != names.size()) {
    throw std::invalid_argument("observation_scales must match observation_terms in length");
  }
  ObservationLayout layout;
  for (size_t i = 0; i < names.size(); ++i) {
    layout.terms.push_back(parse_term(names[i]));
    const double scale = scales.empty() ? 1.0 : scales[i];
    if (!std::isfinite(scale)) {
      throw std::invalid_argument("observation scale is non-finite");
    }
    layout.scales.push_back(scale);
  }
  return layout;
}

// Concatenates the layout's terms, each multiplied by its scale. gait_phase is
// (sin 2*pi*phase, cos 2*pi*phase).
inline std::vector<float> build_observation(
  const ObservationInputs & in, const std::vector<double> & defaults,
  const ObservationLayout & layout)
{
  const size_t n = defaults.size();
  if (n == 0 || in.joint_positions.size() != n || in.joint_velocities.size() != n ||
    in.previous_actions.size() != n)
  {
    throw std::invalid_argument("Observation joint vectors have the wrong size");
  }
  if (layout.terms.size() != layout.scales.size()) {
    throw std::invalid_argument("Observation layout has mismatched scales");
  }
  std::vector<float> observation;
  observation.reserve(layout.size(n));
  for (size_t t = 0; t < layout.terms.size(); ++t) {
    const double scale = layout.scales[t];
    const auto append = [&observation, scale](double value) {
        if (!std::isfinite(value)) {
          throw std::invalid_argument("Observation value is non-finite");
        }
        observation.push_back(static_cast<float>(scale * value));
      };
    switch (layout.terms[t]) {
      case Term::kCommand: for (double v : in.command) {
          append(v);
      }
        break;
      case Term::kAngularVelocity: for (double v : in.angular_velocity) {
          append(v);
      }
        break;
      case Term::kGravity: for (double v : in.gravity) {
          append(v);
      }
        break;
      case Term::kJointPos:
        for (size_t i = 0; i < n; ++i) {
          append(in.joint_positions[i] - defaults[i]);
        }
        break;
      case Term::kJointVel: for (double v : in.joint_velocities) {
          append(v);
      }
        break;
      case Term::kLastAction: for (double v : in.previous_actions) {
          append(v);
      }
        break;
      case Term::kGaitPhase:
        append(std::sin(2.0 * M_PI * in.gait_phase));
        append(std::cos(2.0 * M_PI * in.gait_phase));
        break;
    }
  }
  return observation;
}

inline std::vector<float> build_observation(
  const ObservationInputs & in, const std::vector<double> & defaults)
{
  return build_observation(in, defaults, berkeley_layout());
}

// Linear blend from start to goal; alpha is clamped to [0, 1].
inline std::vector<double> interpolate(
  const std::vector<double> & start, const std::vector<double> & goal, double alpha)
{
  if (start.size() != goal.size()) {
    throw std::invalid_argument("interpolate: size mismatch");
  }
  alpha = std::clamp(alpha, 0.0, 1.0);
  std::vector<double> out(start.size());
  for (size_t i = 0; i < start.size(); ++i) {
    out[i] = start[i] + alpha * (goal[i] - start[i]);
  }
  return out;
}

}  // namespace th_policy_bridge
