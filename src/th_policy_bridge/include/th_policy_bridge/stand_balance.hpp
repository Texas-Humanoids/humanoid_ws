#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace th_policy_bridge
{

// STAND-mode COM centering (no policy): leans the robot on its ankles, slowly, until the mean
// ankle torque is ~0, i.e. the COM is over the ankle axes, which sit at mid-foot on the demo
// walker. A COM off-center (CAD vs. real, encoder offsets) costs push margin on one side; this
// gives it back. Same algorithm as th_training/stand_balance.py (CenteringTrim), where it was
// tuned and tested in MuJoCo:
//   filtered += (mean tau - filtered) * dt / filter_time
//   if both ankles are quiet (|tau - filtered| < freeze_above):  lean += gain * filtered * dt
//   lean clamped to +-max_lean, added to both ankle targets (+ = toes down = robot leans back)
// The torque needs no sensor: in MIT mode the drive applies kp (target - q) - kd v.
// Slow and frozen during pushes on purpose: a fast trim read push-recovery torque as a COM
// offset and threw the robot over the other way (sim).
// target_torque (auto-lean, 2026-11-19): the real robot balanced with the weight ~2 cm ahead of the
// ankles (mean ankle torque +1.4..+2.0 N m at lean -0.050): its usable sole is smaller than the CAD
// box and sits forward. So the loop can hold a set mean ankle torque instead of 0:
//   lean += gain * (filtered - target_torque) * dt
// target_torque 0 = the original COM-over-ankle centering.
struct CenteringTrim
{
  double gain{0.0};  // rad/s per N m; 0 = off
  double max_lean{0.06};  // rad
  double filter_time{1.0};  // s
  double freeze_above{2.5};  // N m
  double target_torque{0.0};  // N m, mean ankle torque to hold (+ = weight ahead of the ankles)
  double lean{0.0};
  double filtered{0.0};

  void reset()
  {
    lean = 0.0;
    filtered = target_torque;  // start "on target": no lean transient before the filter fills
  }

  double update(double tau_left, double tau_right, double dt)
  {
    const double raw = 0.5 * (tau_left + tau_right);
    filtered += (raw - filtered) * std::min(1.0, dt / filter_time);
    const bool quiet = std::abs(tau_left - filtered) < freeze_above &&
      std::abs(tau_right - filtered) < freeze_above;
    if (quiet) {
      lean = std::clamp(lean + gain * (filtered - target_torque) * dt, -max_lean, max_lean);
    }
    return lean;
  }
};

// Optional operator gate for the CenteringTrim (bridge param stand_balance_arm_event, e.g.
// "Y pressed"). While the robot is lowered on ropes the ankle torque doesn't show the final balance,
// so an always-on trim winds up to its limit and swings to the other one when the full weight lands
// (2026-11-19 floor tests: a 0.06 rad swing, wider than the ~0.045 rad balance window -> fell while
// being put down). With the gate the trim holds still until the operator presses the button once
// the robot carries its own weight; every new STAND (and every disarm) waits again.
struct TrimGate
{
  bool enabled{false};  // false = no gate: the trim runs right after the STAND ramp (old behavior)
  bool armed{false};

  void reset() {armed = false;}  // STAND entry, disarm
  void arm() {armed = true;}
  bool active() const {return !enabled || armed;}
};

// Operator-set STAND lean (D-pad): both ankle targets shift by `applied`, so the whole robot tilts
// as one piece over flat feet and its COM moves ~0.55 m x lean. + = toes down = leans back. Each
// nudge moves the target one step; the applied lean follows at `rate`, so nothing jumps.
struct ManualLean
{
  double step{0.01};  // rad per press
  double max{0.25};  // rad
  double rate{0.05};  // rad/s
  double target{0.0};
  double applied{0.0};

  void nudge(int direction)
  {
    target = std::clamp(target + direction * step, -max, max);
  }

  // At STAND entry: the ramp blends into the leaned pose, no separate catch-up.
  void snap() {applied = target;}

  double update(double dt)
  {
    applied += std::clamp(target - applied, -rate * dt, rate * dt);
    return applied;
  }
};

// Scripted balanced squat in STAND (no policy): one cycle per button press. The pelvis drops by
// `depth` (min-jerk over down_time), holds, and comes back the same way. Per leg, hip/knee/ankle
// pitch offsets from the stand pose come from an IK table solved in th_training/squat_study.py
// (MuJoCo, real-foot model, stand lean -0.055): feet flat, torso pitch unchanged and the whole-body
// COM x held over the same point relative to the ankles, so the balance point never moves.
// Sim (2026-11-19): passes 2 cm in every tested condition (15 ms delay, kp x0.9, ~1 deg encoder
// offsets, COM +-1 cm), 4 cm with the auto-lean trim on; peak joint speed 0.19 rad/s at 2 cm / 3 s.
struct SquatMotion
{
  struct Row
  {
    double depth, hip, knee, ankle;
  };
  static constexpr std::array<Row, 13> kTable{{
    {0.0000, 0.000000, 0.000000, 0.000000},
    {0.0050, -0.042257, 0.081702, -0.039396},
    {0.0100, -0.078143, 0.151221, -0.072987},
    {0.0150, -0.109903, 0.212848, -0.102818},
    {0.0200, -0.138711, 0.268828, -0.129959},
    {0.0250, -0.165274, 0.320514, -0.155056},
    {0.0300, -0.190055, 0.368795, -0.178530},
    {0.0350, -0.213379, 0.414290, -0.200679},
    {0.0400, -0.235482, 0.457454, -0.221719},
    {0.0450, -0.256545, 0.498630, -0.241813},
    {0.0500, -0.276707, 0.538085, -0.261090},
    {0.0550, -0.296079, 0.576034, -0.279651},
    {0.0600, -0.314751, 0.612649, -0.297580},
  }};
  static constexpr double kMaxDepth = 0.06;

  double depth{0.02};  // m of pelvis drop
  double down_time{3.0};  // s, also the time back up
  double hold_time{1.0};  // s at the bottom
  bool running{false};
  double start{0.0};

  static double min_jerk(double u)
  {
    u = std::clamp(u, 0.0, 1.0);
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  }

  double duration() const {return 2.0 * down_time + hold_time;}

  // Pelvis drop at time t (s) since the start of the cycle; 0 before and after it.
  double depth_at(double t) const
  {
    if (t <= 0.0) {return 0.0;}
    if (t < down_time) {return depth * min_jerk(t / down_time);}
    if (t < down_time + hold_time) {return depth;}
    if (t < duration()) {return depth * (1.0 - min_jerk((t - down_time - hold_time) / down_time));}
    return 0.0;
  }

  // {hip, knee, ankle} pitch offsets for a pelvis drop z (linear interpolation in the table).
  static std::array<double, 3> at_depth(double z)
  {
    z = std::clamp(z, 0.0, kMaxDepth);
    std::size_t i = 0;
    while (i + 2 < kTable.size() && kTable[i + 1].depth < z) {++i;}
    const Row & a = kTable[i];
    const Row & b = kTable[i + 1];
    const double w = (z - a.depth) / (b.depth - a.depth);
    return {a.hip + w * (b.hip - a.hip), a.knee + w * (b.knee - a.knee),
      a.ankle + w * (b.ankle - a.ankle)};
  }

  void begin(double now)
  {
    running = true;
    start = now;
  }

  void stop() {running = false;}

  // Offsets at `now`; returns zeros and ends the cycle (running = false) once it is over.
  std::array<double, 3> offsets(double now)
  {
    if (!running) {return {0.0, 0.0, 0.0};}
    const double t = now - start;
    if (t >= duration()) {
      running = false;
      return {0.0, 0.0, 0.0};
    }
    return at_depth(depth_at(t));
  }
};

}  // namespace th_policy_bridge
