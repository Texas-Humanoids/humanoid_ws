"""Stand-only demo controller (no policy): stiff joint PD on the stand pose + a slow CoM-centering
trim, and the MuJoCo harness that tests it.

First principles (numbers from the walker_demo MJCF in the stand pose):
  * The robot can't step without a policy, so it survives a push only while the capture point
    (COM + v / omega, omega = sqrt(g / h) = 4.24 1/s at h = 0.545 m) stays over the feet:
    v_max = omega * (COM-to-edge distance). Sole x -0.065 (heel) .. +0.087 (toe), so ~0.34 m/s
    forward, ~0.30 m/s back, ~0.9 m/s sideways (outer foot edges at +-0.22 m). Stiff PD already
    gets most of this; no non-stepping controller can beat the bound.
  * What a controller CAN fix is a COM that isn't over the middle of the feet (real COM ~3 cm
    behind the CAD, encoder offsets of a few degrees): each cm off costs ~0.04 m/s of push margin
    on one side. The ankle axis sits at mid-foot (x +0.011), so "COM over mid-foot" is "mean
    ankle torque = 0".
  * Ankle torque needs no sensor: in MIT mode the drive applies tau = kp (target - q) - kd v, all
    of which the bridge knows. Ankle +q is toes down; tau > 0 means the COM is forward of the
    ankles. Moving both ankle targets toes-down tilts the shins (and robot) back.

CenteringTrim: lean += gain * mean(ankle tau) * dt, clamped to +-max_lean, added to both ankle
targets. Slow on purpose (time constant ~1-2 s), so it centers the COM and leaves push
recovery to the PD. No IMU needed. Hanging in the air, the ankle torque is ~0, so it doesn't wind
up. Mirrored in C++ in th_policy_bridge (stand_balance.hpp) for the STAND mode.

  python -m th_training.stand_balance sweep           # PD vs PD+trim across COM offsets/pushes
  python -m th_training.stand_balance long            # 120 s hold, drift/windup check
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from th_training.robots import walker_demo as R
from th_training.sim2sim_demo import POLICY_DT, DemoSim, _quat2mat

J = R.JOINT_ORDER
ANKLES = [J.index("left_ankle_pitch_joint"), J.index("right_ankle_pitch_joint")]
# Stand gains on the Jetson (walker_demo_robot.yaml stand_kp / stand_kd, 2026-11-19)
STAND_GAINS = {"hip_pitch": (120, 5.0), "hip_roll": (150, 6.0), "hip_yaw": (40, 1.5), "knee_pitch": (180, 6.0),
               "ankle_pitch": (150, 3.5)}


class CenteringTrim:
    """Slow lean so the mean ankle torque goes to 0 (COM over mid-foot). Units: rad, N m, s.

    v1 of this (gain 0.012, raw torque) made things WORSE in sim: during a push the ankles hit the
    foot-edge limit (~5.7 N m each) and the trim read that as a COM offset, leaned ~0.03 rad and
    threw the robot over the other way; with ankle encoder offsets its lag against the ~1 s
    pendulum sway grew into a heel-toe rocking limit cycle. So: integrate a low-passed torque,
    slowly, and only while the ankles are quiet (|raw torque| < freeze_above)."""

    def __init__(self, gain: float = 0.0015, max_lean: float = 0.06, lpf_tau: float = 1.0, freeze_above: float = 2.5,
                 target_torque: float = 0.0):
        # target_torque (auto-lean): hold this mean ankle torque instead of 0. The real robot balanced
        # at +1.4..+2.0 N m (lean -0.050): weight ~2 cm ahead of the ankles. Mirrors the C++ (bridge).
        self.gain, self.max_lean, self.lpf_tau, self.freeze_above = gain, max_lean, lpf_tau, freeze_above
        self.target_torque = target_torque
        self.reset()

    def reset(self):
        self.lean, self.filtered = 0.0, self.target_torque

    def update(self, tau_left: float, tau_right: float, dt: float) -> float:
        raw = 0.5 * (tau_left + tau_right)
        self.filtered += (raw - self.filtered) * min(1.0, dt / self.lpf_tau)
        quiet = abs(tau_left - self.filtered) < self.freeze_above and abs(tau_right - self.filtered) < self.freeze_above
        if quiet:
            self.lean = float(np.clip(self.lean + self.gain * (self.filtered - self.target_torque) * dt,
                                      -self.max_lean, self.max_lean))
        return self.lean


def gains_array(gains=STAND_GAINS):
    kp = np.array([gains[R.joint_type(j)][0] for j in J], float)
    kd = np.array([gains[R.joint_type(j)][1] for j in J], float)
    return kp, kd


def run(trim: CenteringTrim | None, com_dx=0.0, com_dy=0.0, bias=None, delay=0, kp_scale=1.0, friction=0.8,
        push=None, push_t=20.0, seconds=26.0, gains=STAND_GAINS, seed=0, base_lean=0.0, foot_shift=0.0,
        foot_half_len=None):
    """One stand: PD on the stand pose (+ trim on the ankles). bias = encoder offsets (rad, measured -
    true): the drive servos the measured angle, so the true pose is off by -bias. base_lean = the saved
    D-pad lean (yaml stand_balance.lean), applied on both ankles under the trim."""
    sim = DemoSim(variant="cad", delay_steps=delay, friction=friction)
    kp, kd = gains_array(gains)
    sim.kp, sim.kd = kp * kp_scale, kd
    if com_dx or com_dy:
        b = sim.m.body("base_link").id
        sim.m.body_ipos[b, 0] += com_dx
        sim.m.body_ipos[b, 1] += com_dy
    sim.reset()
    if foot_shift or foot_half_len:
        # Emulate the real sole: support moved forward (the real robot balances with the weight
        # ~2 cm ahead of the ankles) and/or shorter than the CAD box.
        for side in ("left", "right"):
            g = sim.m.geom(f"{side}_foot_collision")
            body = g.bodyid[0]
            Rb = sim.d.xmat[body].reshape(3, 3)
            g.pos[:] = g.pos + Rb.T @ np.array([foot_shift, 0.0, 0.0])
            if foot_half_len:
                g.size[0] = foot_half_len
        sim.reset()
    bias = np.zeros(10) if bias is None else np.asarray(bias, float)
    q0 = sim.q0.copy()
    if trim:
        trim.reset()
    rng = np.random.default_rng(seed)
    fell, leans, tilts, taus, staus = None, [], [], [], []
    n = int(seconds / POLICY_DT)
    target = q0.copy()
    target[ANKLES] += base_lean
    for k in range(n):
        q_meas = sim.d.qpos[sim.qadr] + bias + rng.normal(0, 0.0005, 10)   # encoder noise
        v_meas = sim.d.qvel[sim.vadr] + rng.normal(0, 0.02, 10)
        tau_est = sim.kp * (target - q_meas) - sim.kd * v_meas            # what each drive applies
        lean = trim.update(tau_est[ANKLES[0]], tau_est[ANKLES[1]], POLICY_DT) if trim else 0.0
        target = q0.copy()
        target[ANKLES] += base_lean + lean
        # the drive servos the measured angle: true target = target - bias
        sim.step((target - bias - q0) / sim.scale,
                 push=np.array(push, float) if (push is not None and k == int(push_t / POLICY_DT)) else None)
        leans.append(lean)
        g = _quat2mat(sim.d.qpos[3:7]).T @ np.array([0, 0, -1.0])
        tilts.append(math.degrees(math.asin(np.clip(g[0], -1, 1))))
        taus.append(np.abs(tau_est))
        staus.append(float(tau_est[ANKLES].mean()))
        if sim.fallen():
            fell = round(k * POLICY_DT, 2)
            break
    tail = slice(max(0, len(tilts) - 50), None)
    return dict(fell=fell, lean_final=round(leans[-1], 4), pitch_final=round(float(np.mean(tilts[tail])), 2),
                tau_peak=np.max(taus, axis=0).round(1).tolist(),
                ankle_tau_final=round(float(np.mean(np.array(taus)[tail][:, ANKLES])), 2),
                ankle_tau_signed_final=round(float(np.mean(staus[-250:])), 2),
                lean_pp_last10s=round(float(np.ptp(leans[-500:])), 4) if leans else 0.0)


def max_push(trim_factory, direction, cond, lo=0.0, hi=0.8, iters=6):
    """Largest push (m/s, at t = 20 s, after the trim settled) survived, by bisection."""
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        r = run(trim_factory(), push=np.array(direction) * mid, **cond)
        lo, hi = (mid, hi) if r["fell"] is None else (lo, mid)
    return round(lo, 3)


def sweep():
    conds = {
        "nominal": {},
        "COM 3 cm back": dict(com_dx=-0.03),
        "COM 3 cm fwd": dict(com_dx=0.03),
        "ankle encoders 2 deg off": dict(bias=[0, 0, 0, 0, 0.035, 0, 0, 0, 0, 0.035]),
        "real-ish: COM -3cm, 15 ms, kp x0.9, mu 0.5, offsets": dict(
            com_dx=-0.03, delay=3, kp_scale=0.9, friction=0.5, bias=[0.02, -0.02, 0.01, -0.02, 0.03, -0.02, 0.02, -0.01, 0.02, 0.02]),
    }
    dirs = {"fwd": (1, 0, 0), "back": (-1, 0, 0), "side": (0, 1, 0)}
    out = {}
    for cname, cond in conds.items():
        row = {}
        for label, factory in (("PD", lambda: None), ("PD+trim", lambda: CenteringTrim())):
            settle = run(factory(), **cond)
            row[label] = dict(settle=settle, **{d: max_push(factory, v, cond) for d, v in dirs.items()})
        out[cname] = row
        a, b = row["PD"], row["PD+trim"]
        print(f"{cname}\n   PD      max push fwd {a['fwd']:.2f} back {a['back']:.2f} side {a['side']:.2f} m/s | "
              f"standing pitch {a['settle']['pitch_final']:+.1f} deg, mean ankle torque {a['settle']['ankle_tau_final']:.1f} Nm")
        print(f"   PD+trim max push fwd {b['fwd']:.2f} back {b['back']:.2f} side {b['side']:.2f} m/s | "
              f"standing pitch {b['settle']['pitch_final']:+.1f} deg, mean ankle torque {b['settle']['ankle_tau_final']:.1f} Nm, "
              f"lean {b['settle']['lean_final']:+.3f} rad", flush=True)
    return out


def long_hold():
    for cname, cond in (("nominal", {}), ("COM -3 cm, 15 ms, kp x0.9", dict(com_dx=-0.03, delay=3, kp_scale=0.9))):
        for label, trim in (("PD", None), ("PD+trim", CenteringTrim())):
            r = run(trim, seconds=120.0, **cond)
            print(f"{cname:26s} {label:8s} 120 s: {'FELL at ' + str(r['fell']) if r['fell'] else 'stood'}, pitch {r['pitch_final']:+.2f} deg, "
                  f"lean {r['lean_final']:+.4f}, peak tau {r['tau_peak'][:5]}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["sweep", "long"])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = sweep() if a.what == "sweep" else long_hold()
    if a.out and res:
        json.dump(res, open(a.out, "w"), indent=1)
