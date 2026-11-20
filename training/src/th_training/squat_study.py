"""Balanced squat for the stand-only demo (no policy): first-principles IK + MuJoCo robustness sweep.

A squat lowers the pelvis by bending hip, knee and ankle pitch together (both legs equal). To stay
balanced on the tiny real foot (usable sole ~1.5 cm behind / ~6 cm ahead of the ankle) three things
must hold at every depth:
  (a) feet flat,
  (b) torso pitch unchanged (same as the stand),
  (c) the whole-body COM's x unchanged relative to the ankles (= the balanced point of the stand).
With the torso orientation fixed, (a)-(c) plus the pelvis-height target are 3 equations in the 3
per-leg unknowns (d_hip, d_knee, d_ankle), solved with Newton on MuJoCo forward kinematics. The
result is a table depth -> joint offsets from the stand pose. The bridge (C++ SquatMotion in
th_policy_bridge/stand_balance.hpp) carries the same table and the same minimum-jerk timing.

  python -m th_training.squat_study table           # print the IK table (C++ initializer too)
  python -m th_training.squat_study sweep           # robustness sweep, depths x conditions
  python -m th_training.squat_study traj --depth 0.03 --out traj.csv   # reference trajectory
"""
from __future__ import annotations

import argparse
import math

import mujoco
import numpy as np

from th_training import stand_balance as SB
from th_training.robots import walker_demo as R
from th_training.sim2sim_demo import POLICY_DT, DemoSim, _quat2mat

J = R.JOINT_ORDER
LEAN = -0.055                     # saved stand lean (robot yaml stand_balance.lean, 2026-11-19 21:45)
SAG = ("hip_pitch", "knee_pitch", "ankle_pitch")
IDX = {s: [J.index(f"{side}_{s}_joint") for side in ("left", "right")] for s in SAG}
REAL_FOOT = dict(com_dx=-0.025, foot_shift=0.0225, foot_half=0.0375)   # matches today's 3 real observations
CAPS = {"hip_pitch": 15.0, "hip_roll": 15.0, "hip_yaw": 6.0, "knee_pitch": 20.0, "ankle_pitch": 10.0}
DEPTHS = np.round(np.arange(0.0, 0.0601, 0.005), 4)                    # table rows: 0..6 cm, 0.5 cm steps
T_DOWN, T_HOLD = 3.0, 1.0


# ----------------------------------------------------------------------------- kinematics / IK
def _stand_q(model_q0):
    q = model_q0.copy()
    q[IDX["ankle_pitch"]] += LEAN
    return q


def _fk(m, d, pitch, q):
    """Foot toe-down angle (rad), pelvis height above the left sole, COM x - left ankle x."""
    d.qpos[:] = 0.0
    d.qpos[2] = 1.0
    d.qpos[3:7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]
    for i, j in enumerate(J):
        d.qpos[m.joint(j).qposadr[0]] = q[i]
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    g = m.geom("left_foot_collision").id
    Rf = d.geom_xmat[g].reshape(3, 3)
    toe_down = -math.asin(float(np.clip(Rf[2, 0], -1, 1)))
    height = d.qpos[2] - d.geom_xpos[g][2]
    ankle_x = d.xanchor[m.joint("left_ankle_pitch_joint").id][0]
    com_x = d.subtree_com[1][0]
    return toe_down, height, com_x - ankle_x


def solve_table(m, q0, depths=DEPTHS):
    """IK table: rows of (depth, d_hip, d_knee, d_ankle), offsets from the leaned stand pose."""
    d = mujoco.MjData(m)
    qs = _stand_q(q0)
    # torso pitch that puts the feet flat in the stand pose
    pitch = 0.0
    for _ in range(30):
        f, _, _ = _fk(m, d, pitch, qs)
        f2, _, _ = _fk(m, d, pitch + 1e-6, qs)
        pitch -= f / ((f2 - f) / 1e-6)
    _, h0, cx0 = _fk(m, d, pitch, qs)

    def resid(x, depth):
        q = qs.copy()
        for k, s in enumerate(SAG):
            q[IDX[s]] += x[k]
        f, h, cx = _fk(m, d, pitch, q)
        return np.array([f, h - (h0 - depth), cx - cx0])

    rows, x = [], np.zeros(3)
    for depth in depths:
        for _ in range(40):
            r = resid(x, depth)
            if np.max(np.abs(r)) < 1e-9:
                break
            Jm = np.zeros((3, 3))
            for k in range(3):
                dx = np.zeros(3)
                dx[k] = 1e-6
                Jm[:, k] = (resid(x + dx, depth) - r) / 1e-6
            x = x - np.linalg.solve(Jm, r)
        rows.append((float(depth), *[float(v) for v in x]))
    return np.array(rows), pitch


# ----------------------------------------------------------------------------- trajectory (mirrors C++)
def min_jerk(u: float) -> float:
    u = min(max(u, 0.0), 1.0)
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def depth_at(t: float, depth: float, t_down=T_DOWN, t_hold=T_HOLD) -> float:
    """0 -> depth (min-jerk, t_down), hold, depth -> 0 (min-jerk, t_down). 0 before/after."""
    if t <= 0.0:
        return 0.0
    if t < t_down:
        return depth * min_jerk(t / t_down)
    if t < t_down + t_hold:
        return depth
    if t < 2 * t_down + t_hold:
        return depth * (1.0 - min_jerk((t - t_down - t_hold) / t_down))
    return 0.0


def offsets_at(table, t, depth, t_down=T_DOWN, t_hold=T_HOLD):
    """(d_hip, d_knee, d_ankle) at time t: linear interpolation of the table at depth_at(t)."""
    z = depth_at(t, depth, t_down, t_hold)
    return tuple(float(np.interp(z, table[:, 0], table[:, k])) for k in (1, 2, 3))


# ----------------------------------------------------------------------------- simulation
def make_sim(real_foot=True, com_extra=0.0, delay=0, kp_scale=1.0):
    sim = DemoSim(variant="cad", delay_steps=delay)
    kp, kd = SB.gains_array()
    sim.kp, sim.kd = kp * kp_scale, kd
    sim.limit = np.minimum(sim.limit, np.array([CAPS[R.joint_type(j)] for j in J]))
    dx = (REAL_FOOT["com_dx"] if real_foot else 0.0) + com_extra
    if dx:
        sim.m.body_ipos[sim.m.body("base_link").id, 0] += dx
    sim.reset()
    if real_foot:
        for side in ("left", "right"):
            g = sim.m.geom(f"{side}_foot_collision")
            Rb = sim.d.xmat[g.bodyid[0]].reshape(3, 3)
            g.pos[:] = g.pos + Rb.T @ np.array([REAL_FOOT["foot_shift"], 0.0, 0.0])
            g.size[0] = REAL_FOOT["foot_half"]
        sim.reset()
    return sim


def run(table, depth, real_foot=True, com_extra=0.0, delay=0, kp_scale=1.0, bias=None, trim=False,
        settle=3.0, after=2.0):
    """Stand (leaned) for `settle` s, then one squat, then `after` s. Bias = encoder offsets
    (measured - true): the drive servos the measured angle."""
    sim = make_sim(real_foot, com_extra, delay, kp_scale)
    q0 = sim.q0.copy()
    bias = np.zeros(10) if bias is None else np.asarray(bias, float)
    tr = SB.CenteringTrim(gain=0.0015, max_lean=0.015, target_torque=1.5) if trim else None
    rng = np.random.default_rng(0)
    target = _stand_q(q0)
    t_sq = 2 * T_DOWN + T_HOLD
    n = int((settle + t_sq + after) / POLICY_DT)
    fell, taus, ank_mean, vmax, pitch = None, [], [], np.zeros(10), []
    lean = 0.0
    for k in range(n):
        t = k * POLICY_DT
        q_meas = sim.d.qpos[sim.qadr] + bias + rng.normal(0, 0.0005, 10)
        v_meas = sim.d.qvel[sim.vadr] + rng.normal(0, 0.02, 10)
        tau_est = sim.kp * (target - q_meas) - sim.kd * v_meas
        if tr is not None and t >= 1.0:          # armed (Y) once standing
            lean = tr.update(tau_est[SB.ANKLES[0]], tau_est[SB.ANKLES[1]], POLICY_DT)
        target = _stand_q(q0)
        target[SB.ANKLES] += lean
        dh, dk, da = offsets_at(table, t - settle, depth)
        target[IDX["hip_pitch"]] += dh
        target[IDX["knee_pitch"]] += dk
        target[IDX["ankle_pitch"]] += da
        sim.step((target - bias - q0) / sim.scale)
        if settle - 0.5 <= t <= settle + t_sq + after:
            ank_mean.append(0.5 * (tau_est[SB.ANKLES[0]] + tau_est[SB.ANKLES[1]]))
            taus.append(np.abs(np.array(sim.tau_log[-4:])).max(0))
            vmax = np.maximum(vmax, np.abs(sim.d.qvel[sim.vadr]))
            g = _quat2mat(sim.d.qpos[3:7]).T @ np.array([0, 0, -1.0])
            pitch.append(math.degrees(math.asin(np.clip(g[0], -1, 1))))
        if sim.fallen():
            fell = round(t, 2)
            break
    taus = np.array(taus).max(0) if taus else np.zeros(10)
    cap = np.array([CAPS[R.joint_type(j)] for j in J])
    am = np.array(ank_mean) if ank_mean else np.zeros(1)
    return dict(fell=fell, ank_min=float(am.min()), ank_max=float(am.max()), ank_start=float(am[:20].mean()),
                vpeak=float(vmax.max()), vpeak_joint=R.joint_type(J[int(np.argmax(vmax))]),
                tau_frac=float((taus / cap).max()), pitch_range=(float(min(pitch or [0])), float(max(pitch or [0]))))


BIAS1 = [0.017, -0.017, 0.0, -0.017, 0.017, -0.017, 0.017, 0.0, 0.017, -0.017]   # ~1 deg calibration errors
CONDS = {
    "real-foot nominal": dict(),
    "real-foot 15ms, kp x0.9": dict(delay=3, kp_scale=0.9),
    "real-foot enc ~1deg": dict(bias=BIAS1),
    "real-foot COM +1cm": dict(com_extra=0.01),
    "real-foot COM -1cm": dict(com_extra=-0.01),
    "real-foot all combined": dict(delay=3, kp_scale=0.9, bias=BIAS1, com_extra=-0.01),
    "CAD-size foot, real COM": dict(real_foot=False, com_extra=-0.025),   # bigger foot (heel plate)
}


def passes(r):
    return r["fell"] is None and r["ank_min"] > -0.2 and r["ank_max"] < 4.0 and r["vpeak"] < 0.5 and r["tau_frac"] < 0.8


def sweep(table, depths=(0.01, 0.02, 0.03, 0.04, 0.05, 0.06)):
    print(f"timing: down {T_DOWN} s, hold {T_HOLD} s, up {T_DOWN} s; lean {LEAN}; stand gains + torque caps")
    print("pass = no fall, mean ankle torque within -0.2..+4 N m, peak joint speed < 0.5 rad/s, torque < 80% cap")
    summary = {}
    for trim in (False, True):
        print(f"\n=== CenteringTrim {'ON (target 1.5, +-0.015)' if trim else 'OFF'}")
        print(f"{'condition':26s} {'depth':>6s}  result  {'ankle mean min..max (start)':>28s} {'peak v (joint)':>20s} {'tau/cap':>7s} {'pitch deg':>13s}")
        for cname, cond in CONDS.items():
            for dep in depths:
                r = run(table, dep, trim=trim, **cond)
                ok = passes(r)
                summary.setdefault((trim, dep), []).append(ok)
                print(f"{cname:26s} {dep * 100:5.1f}cm  {'PASS' if ok else ('FELL' if r['fell'] else 'fail')}  "
                      f"{r['ank_min']:+6.2f}..{r['ank_max']:+5.2f} ({r['ank_start']:+5.2f})   "
                      f"{r['vpeak']:5.2f} ({r['vpeak_joint']:11s}) {r['tau_frac']:6.0%} "
                      f"{r['pitch_range'][0]:+5.1f}..{r['pitch_range'][1]:+4.1f}", flush=True)
    print("\nall-condition pass by depth:")
    for (trim, dep), oks in sorted(summary.items()):
        print(f"  trim {'on ' if trim else 'off'} depth {dep * 100:.1f} cm: {sum(oks)}/{len(oks)}")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["table", "sweep", "traj"])
    ap.add_argument("--depth", type=float, default=0.03)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    sim = make_sim(real_foot=True)
    table, pitch = solve_table(sim.m, sim.q0)
    if a.what == "table":
        print(f"stand torso pitch {math.degrees(pitch):+.2f} deg (feet flat, lean {LEAN}); real-foot COM model")
        print(f"{'depth m':>8s} {'d_hip':>9s} {'d_knee':>9s} {'d_ankle':>9s}")
        for row in table:
            print(f"{row[0]:8.4f} {row[1]:+9.5f} {row[2]:+9.5f} {row[3]:+9.5f}")
        print("\nC++ initializer (depth, hip, knee, ankle):")
        print(",\n".join(f"  {{{row[0]:.4f}, {row[1]:.6f}, {row[2]:.6f}, {row[3]:.6f}}}" for row in table))
    elif a.what == "sweep":
        sweep(table)
    else:
        rows = []
        t = 0.0
        while t <= 2 * T_DOWN + T_HOLD + 0.5:
            rows.append((t, *offsets_at(np.round(table, 6), t, a.depth)))
            t = round(t + POLICY_DT, 6)
        if a.out:
            np.savetxt(a.out, rows, fmt="%.9f", delimiter=",")
        print(f"{len(rows)} samples; peak offsets hip {min(r[1] for r in rows):+.4f} knee {max(r[2] for r in rows):+.4f} "
              f"ankle {min(r[3] for r in rows):+.4f}")


if __name__ == "__main__":
    main()
