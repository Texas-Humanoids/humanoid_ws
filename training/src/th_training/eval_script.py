"""Scripted command + push sequence shared by sim2sim_demo (MuJoCo) and isaac_check_demo (PhysX)."""
import math

import numpy as np

POLICY_DT = 0.02

SCRIPT = [  # (seconds, vx, vy, wz)
    (2.0, 0.0, 0.0, 0.0),
    (5.0, 0.2, 0.0, 0.0),
    (5.0, 0.4, 0.0, 0.0),
    (3.0, 0.0, 0.0, 0.0),
    (4.0, -0.2, 0.0, 0.0),
    (4.0, 0.0, 0.15, 0.0),
    (4.0, 0.0, -0.15, 0.0),
    (4.0, 0.0, 0.0, 0.6),
    (4.0, 0.0, 0.0, -0.6),
    (4.0, 0.3, 0.0, 0.4),
    (3.0, 0.04, 0.03, 0.0),   # real-stick drift while "standing" (no dead zone)
    (3.0, 0.0, 0.0, 0.0),
]
PUSHES = {int(1.5 / POLICY_DT): np.array([0.3, 0.0, 0.0]), int(9.0 / POLICY_DT): np.array([0.0, 0.25, 0.0]),
          int(16.5 / POLICY_DT): np.array([-0.3, 0.0, 0.0]), int(30.0 / POLICY_DT): np.array([0.0, -0.25, 0.0])}


class GaitClock:
    """The gait_phase observation as the bridge computes it, from policy_spec.json.
    "free" (v1): phase = (k * dt / T) mod 1, k = policy steps since WALK started.
    "held" (v2): phase = 0 at k = 0 or under a stand command, else phase += dt / T (mod 1)."""

    def __init__(self, spec: dict):
        self.period = float(spec.get("gait_period", 0.0))
        self.mode = spec.get("gait_clock", "free")
        self.threshold = float(spec.get("gait_stand_threshold", 0.1))
        self.phase, self.k = 0.0, None

    def __call__(self, k: int, cmd) -> np.ndarray:
        if self.mode == "held":
            if k != self.k:
                moving = math.hypot(float(cmd[0]), float(cmd[1])) + abs(float(cmd[2])) > self.threshold
                self.phase = 0.0 if (k == 0 or not moving) else (self.phase + POLICY_DT / self.period) % 1.0
                self.k = k
            ph = self.phase
        else:
            ph = (k * POLICY_DT / self.period) % 1.0
        return np.array([math.sin(2 * math.pi * ph), math.cos(2 * math.pi * ph)])


def target_limits(spec: dict, joint_order) -> tuple[np.ndarray, np.ndarray]:
    """The bridge clamps every target to the robot yaml's limits (= spec joint_limits)."""
    lim = spec.get("joint_limits") or {}
    lo = np.array([lim.get(j, [-1e9, 1e9])[0] for j in joint_order], float)
    hi = np.array([lim.get(j, [-1e9, 1e9])[1] for j in joint_order], float)
    return lo, hi
