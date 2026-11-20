#pragma once

#include <algorithm>
#include <cmath>
#include <cmath>
#include <vector>

namespace th_policy_bridge
{

// Detects a value that should keep changing but has stopped: motor feedback that stays
// bit-identical while its target moves (a stuck or stale drive, a frozen CAN stream). Feedback
// updates the value; the targets sent after the value last changed make up the target range.
// Bit-identical on purpose: a real encoder reading of a moving joint never repeats for long.
struct FreezeWatch
{
  bool seen{false};
  double value{};
  double since{};      // time (s) the value last changed
  bool has_ref{false};
  double ref_lo{};     // range of the targets sent since then (only those: a target from
  double ref_hi{};     // before, e.g. the last STAND target, would fake a big move)

  void reset() {seen = has_ref = false;}

  void update(double v, double t)
  {
    if (!seen || v != value) {
      seen = true;
      value = v;
      since = t;
      has_ref = false;
    }
  }

  void observe_ref(double ref)
  {
    if (!seen) {
      return;
    }
    if (!has_ref) {
      has_ref = true;
      ref_lo = ref_hi = ref;
    } else {
      ref_lo = std::min(ref_lo, ref);
      ref_hi = std::max(ref_hi, ref);
    }
  }

  // Unchanged for longer than timeout while the target moved by more than min_ref_change.
  // timeout <= 0 disables.
  bool frozen(double t, double timeout, double min_ref_change) const
  {
    return seen && has_ref && timeout > 0.0 && t - since > timeout &&
           ref_hi - ref_lo > min_ref_change;
  }
};

// The same for a whole vector that must never repeat for long: the policy's targets while
// walking (its gait clock makes every output different), so a repeat means a stuck policy.
struct VectorFreezeWatch
{
  bool seen{false};
  std::vector<double> value;
  double since{};

  void reset()
  {
    seen = false;
    value.clear();
  }

  void update(const std::vector<double> & v, double t)
  {
    if (!seen || v != value) {
      seen = true;
      value = v;
      since = t;
    }
  }

  bool frozen(double t, double timeout) const
  {
    return seen && timeout > 0.0 && t - since > timeout;
  }
};

// A value over its limit for longer than a hold time: a joint moving faster than it ever does in
// training (a policy thrashing, a runaway joint). The hold stops one noisy sample from tripping.
struct OverLimitWatch
{
  double since{-1.0};   // time (s) it went over, or < 0 while under
  double peak{};        // largest |value| since then

  void reset()
  {
    since = -1.0;
    peak = 0.0;
  }

  // limit <= 0 disables.
  void update(double value, double limit, double t)
  {
    const double v = std::abs(value);
    if (limit > 0.0 && v > limit) {
      if (since < 0.0) {
        since = t;
        peak = 0.0;
      }
      peak = std::max(peak, v);
    } else {
      since = -1.0;
    }
  }

  bool tripped(double t, double hold) const {return since >= 0.0 && t - since >= hold;}
};

}  // namespace th_policy_bridge
