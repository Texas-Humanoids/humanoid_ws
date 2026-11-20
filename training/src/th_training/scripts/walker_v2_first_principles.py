"""Phase 0: first-principles numbers for WalkerV2, computed from the MJCF.

Everything the walking task has to pick (gait period, swing height, command ranges, PD gains,
action scale) is derived here from the robot's mass distribution and the Robstride ratings.
That way the choices are traceable, not copied from G1/BHL.

usage:  python -m th_training.scripts.walker_v2_first_principles [--md out.md]
"""
from __future__ import annotations

import argparse
import math

import mujoco
import numpy as np

from th_training.paths import WALKER_V2_XML
from th_training.robots.walker_v2 import (
    DEFAULT_JOINT_POS,
    JOINT_ORDER,
    MOTOR_OF_JOINT,
    MOTORS,
    gains_of,
    joint_type,
)

G = 9.81
DEFL, F_ARM, ERR_SAT, ZETA = 0.2, 10.0, 0.4, 0.5  # gain-design constants, explained in section 5
DEFL_OF = {"knee_pitch": 0.1}  # knee sag maps 1:1 into torso pitch (found in phase 2, see section 5)


def set_pose(m, d, q: dict[str, float], base_z: float = 0.0):
    d.qpos[:] = 0
    d.qpos[3] = 1.0
    d.qpos[2] = base_z
    for name, val in q.items():
        d.qpos[m.joint(name).qposadr[0]] = val
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)


def stand_height(m, d, q):
    set_pose(m, d, q, 0.0)
    return -min(d.site_xpos[m.site(f"{s}_foot").id][2] for s in ("left", "right"))


def place_foot(m, d, side: str, dx: float, base_z: float):
    """Sagittal IK: move `side` foot dx along x from its ready-pose spot at the same height by
    solving hip pitch + knee. The ankle keeps the sole flat (ankle = -(hip + knee))."""
    q = dict(DEFAULT_JOINT_POS)
    set_pose(m, d, q, base_z)
    sid = m.site(f"{side}_foot").id
    target = d.site_xpos[sid].copy()
    target[0] += dx
    hp, kn, an = (f"{side}_hip_pitch_joint", f"{side}_knee_pitch_joint", f"{side}_ankle_pitch_joint")
    jacp = np.zeros((3, m.nv))
    for _ in range(50):
        q[an] = -(q[hp] + q[kn])
        set_pose(m, d, q, base_z)
        err = target - d.site_xpos[sid]
        if np.linalg.norm(err[[0, 2]]) < 1e-5:
            break
        mujoco.mj_jacSite(m, d, jacp, None, sid)
        cols = [m.joint(hp).dofadr[0], m.joint(kn).dofadr[0], m.joint(an).dofadr[0]]
        # ankle follows -(hip+knee), so fold its column in
        J = np.stack([jacp[[0, 2], cols[0]] - jacp[[0, 2], cols[2]], jacp[[0, 2], cols[1]] - jacp[[0, 2], cols[2]]], 1)
        dq = np.linalg.lstsq(J, err[[0, 2]], rcond=None)[0]
        dq *= min(1.0, 0.1 / (np.linalg.norm(dq) + 1e-9))  # damped step
        q[hp] += dq[0]
        q[kn] += dq[1]
    return q


def leg_torques(m, d, side: str, fz: float, zc: float, cop_dx: float = 0.0,
                com_dxy: tuple[float, float] = (0.0, 0.0)) -> dict[str, float]:
    """|tau| on one leg to carry a LIPM-consistent ground reaction, plus that leg's own gravity.

    The GRF acts at the COP (sole centre shifted by cop_dx along the foot) and points from the
    COP to the COM: f = fz * [dx/zc, dy/zc, 1], with (dx, dy) = COM - COP horizontally.
    com_dxy places the COM relative to the sole centre (the gait phase being checked)."""
    sid = m.site(f"{side}_foot").id
    cop = d.site_xpos[sid].copy()
    cop[0] += cop_dx
    f = fz * np.array([(com_dxy[0] - cop_dx) / zc, com_dxy[1] / zc, 1.0])
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jacp, None, cop, m.site_bodyid[sid])
    tau_grf = jacp.T @ f
    out = {}
    for j in JOINT_ORDER:
        if j.startswith(side):
            dof = m.joint(j).dofadr[0]
            out[j] = abs(d.qfrc_bias[dof] - tau_grf[dof])  # static: tau = gravity bias - J^T f
    return out


def analyse() -> tuple[list[str], dict]:
    m = mujoco.MjModel.from_xml_path(str(WALKER_V2_XML))
    d = mujoco.MjData(m)
    L: list[str] = []
    P = L.append

    mass = float(m.body_mass.sum())
    h0 = stand_height(m, d, {})
    h = stand_height(m, d, DEFAULT_JOINT_POS)
    set_pose(m, d, DEFAULT_JOINT_POS, h)
    com = d.subtree_com[m.body("base_link").id].copy()
    pl = d.site_xpos[m.site("left_foot").id].copy()
    pr = d.site_xpos[m.site("right_foot").id].copy()
    width = pl[1] - pr[1]
    half = width / 2
    leg_len = h

    P("## 1. Mass and geometry (ready pose)")
    P(f"- total mass **{mass:.2f} kg**, weight {mass*G:.0f} N")
    P(f"- base (hip-pitch axis) height: straight legs {h0:.3f} m, ready pose **{h:.3f} m**")
    P(f"- COM at ready pose: ({com[0]:+.3f}, {com[1]:+.3f}, **{com[2]:.3f}**) m; "
      f"{com[0]-pl[0]:+.3f} m ahead of the sole centres (sole half-length 0.075)")
    P(f"- sole centres at y = {pl[1]:+.3f} / {pr[1]:+.3f} m: **stance width {width:.3f} m**. Sole 0.150 x 0.102 m")

    zc = com[2]
    w = math.sqrt(G / zc)
    ts_raw = 1.5 / w
    period = round(2 * ts_raw / 0.04) * 0.04  # even number of 20 ms policy steps
    ts = period / 2
    A = half / math.cosh(w * ts / 2)
    sway = half - A
    vy_sw = half * w * math.tanh(w * ts / 2)
    P("\n## 2. Gait timing from the linear inverted pendulum (LIPM)")
    P(f"- z_c = {zc:.3f} m, so omega = sqrt(g/z_c) = **{w:.2f} rad/s** (time constant {1/w:.3f} s)")
    P("- Pick omega * T_step ~= 1.5: G1 (z_c ~0.70 m, T_step 0.4 s) sits at 1.5, Cassie at ~1.4. The capture point grows")
    P("  e^1.5 = 4.5x per step: short enough to stay recoverable, long enough to keep joint speeds and torques low.")
    P(f"- T_step = {ts_raw:.3f} s. Rounded to whole policy steps: **gait period {period:.2f} s, T_step {ts:.2f} s** "
      f"({1/ts:.1f} steps/s). Matches the ~0.7 s in the teammate's video.")
    P(f"- Lateral LIPM orbit at {width:.2f} m stance width: COM sways **+/-{sway*100:.1f} cm**, peak lateral COM speed "
      f"{vy_sw:.2f} m/s at foot switch. **No ankle roll**, so lateral balance is done entirely by step timing and hip-roll foot placement.")

    v_fr = math.sqrt(0.25 * G * leg_len)
    P("\n## 3. Speed range (dynamic similarity)")
    P(f"- Walking tops out near Froude v^2/(g l) ~ 0.25-0.5. l = {leg_len:.2f} m: "
      f"**v(Fr=0.25) = {v_fr:.2f} m/s**, v(Fr=0.5) = {math.sqrt(0.5*G*leg_len):.2f} m/s")
    for v in (0.3, 0.5, 1.0):
        step = v * ts
        P(f"- v = {v:.1f} m/s: step {step*100:.0f} cm, hip pitch swing ~+/-{math.atan2(step/2, leg_len):.2f} rad (limit +/-0.785)")
    P("- Command box for the first policy: vx [-0.5, 1.0] m/s, vy [-0.3, 0.3] m/s, yaw rate [-1.0, 1.0] rad/s. That's under Fr 0.25,")
    P("  with margin for the unknown actuators.")
    P(f"- Swing apex ~10% of leg length = {0.1*leg_len*100:.1f} cm: **0.07 m** (G1's 0.1 m at 0.79 m is the same ratio)")

    mg = mass * G
    step = 1.0 * ts
    cases = {
        "mid-stance (COM over foot)": dict(fz=mg),
        "foot switch, lateral (COM on midline)": dict(fz=mg, com_dxy=(0.0, -half)),
        "push-off @1 m/s (COM ahead, COP at toe)": dict(fz=mg, com_dxy=(step / 2, 0.0), cop_dx=0.06),
        "heel strike @1 m/s, 1.5 mg, COP at heel": dict(fz=1.5 * mg, com_dxy=(-step / 2, 0.0), cop_dx=-0.06),
    }
    # The sagittal cases put the foot where the step puts it (behind the hip at push-off, ahead at
    # heel strike), so the leg lines up with the GRF the way it does in real walking.
    foot_dx = {"push-off": -step / 2, "heel strike": step / 2}
    # Reach: knee kept >= 0.2 rad bent. How far can the foot get from under the hip?
    bent = {f"{s}_{j}_joint": v for s in ("left", "right")
            for j, v in (("hip_pitch", -0.1), ("knee_pitch", 0.2), ("ankle_pitch", -0.1))}
    h_reach = stand_height(m, d, bent)
    reach = math.sqrt(max(h_reach**2 - h**2, 0.0))
    res, poses, base_for = {}, {}, {}
    for k, v in cases.items():
        dx = next((x for key, x in foot_dx.items() if k.startswith(key)), 0.0)
        bz = min(h, math.sqrt(h_reach**2 - dx**2))  # drop the base if the step is out of reach
        base_for[k] = bz
        q = place_foot(m, d, "left", dx, bz)
        poses[k] = (q["left_hip_pitch_joint"], q["left_knee_pitch_joint"], q["left_ankle_pitch_joint"])
        res[k] = leg_torques(m, d, "left", zc=zc, **v)
    set_pose(m, d, DEFAULT_JOINT_POS, h)
    legj = [j for j in JOINT_ORDER if j.startswith("left")]
    worst = {j: max(r[j] for r in res.values()) for j in legj}
    P("\n## 4. Joint torques vs Robstride ratings (Nm)")
    P("Quasi-static, ready pose, one foot carrying the load. The ground reaction runs from the COP to the COM (the")
    P("LIPM force). That's what loads hip roll and ankle pitch whenever the COM isn't directly over the foot.\n")
    P("| case | leg pose (hip, knee, ankle) | " + " | ".join(joint_type(j) for j in legj) + " |")
    P("|---|---|" + "---|" * len(legj))
    for k, r in res.items():
        P(f"| {k} | ({poses[k][0]:+.2f}, {poses[k][1]:+.2f}, {poses[k][2]:+.2f}) | " + " | ".join(f"{r[j]:.1f}" for j in legj) + " |")
    P("| **worst case** | | " + " | ".join(f"**{worst[j]:.1f}**" for j in legj) + " |")
    P("| motor | | " + " | ".join(MOTOR_OF_JOINT[j] for j in legj) + " |")
    P("| rated / peak | | " + " | ".join(f"{MOTORS[MOTOR_OF_JOINT[j]].rated_torque:.0f} / {MOTORS[MOTOR_OF_JOINT[j]].peak_torque:.0f}" for j in legj) + " |")
    P("| worst / peak | | " + " | ".join(f"{100*worst[j]/MOTORS[MOTOR_OF_JOINT[j]].peak_torque:.0f}%" for j in legj) + " |")

    P(f"\n- Reach: with the knee kept >= 0.2 rad bent and the base at the ready height {h:.3f} m, the foot gets "
      f"**{reach*100:.1f} cm** ahead of / behind the hip.")
    P(f"  That's a {2*reach*100:.0f} cm step, so **v_max ~ {2*reach/ts:.2f} m/s at {ts:.2f} s steps** without crouching. For 1 m/s the base has to drop to "
      f"{min(base_for.values()):.3f} m (the push-off and heel-strike rows use that height). There's deliberately no base-height reward, so the policy can crouch.")

    Mfull = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, Mfull)
    P("\n## 5. PD gains")
    P(f"- **kp** = clip(worst-case torque / {DEFL} rad, lower, upper). The {DEFL} rad is the PD sag the policy has to")
    P("  pre-compensate. With action scale ~0.25 rad, that keeps actions around +/-1.")
    P(f"  - lower = armature * (2 pi * {F_ARM:.0f} Hz)^2 (mjlab's rule), so the rotor's own dynamics are tracked")
    P(f"  - upper = peak / {ERR_SAT} rad, so a {ERR_SAT} rad tracking error saturates the motor and no more")
    P(f"- **kd** = 2 zeta sqrt(kp I_eff), zeta = {ZETA}, on the swing-leg inertia I_eff = M_ii + armature, clipped to")
    P("  kp * [0.02, 0.1] s. The PD time constant stays within 1-5 policy steps.")
    P(f"- **Knee exception: sag budget {DEFL_OF['knee_pitch']} rad.** With the feet flat, the sagittal sags add straight into torso pitch")
    P("  (pitch = -(hip + knee + ankle)). The knee carries the most stance torque, so its sag dominates. Phase 2 (PD-only stand")
    P("  in `sim2sim stand`) found that at knee kp 60, the 0.087 rad sag tilts the base back 7 deg and the stand diverges at")
    P("  0.8x kp (inside the DR range). With knee kp >= 100 it stands at 0.8x kp, 10 ms delay, tilt <= 7 deg.")
    P("- Action scale = 0.25 * peak / kp (mjlab convention): action 1 asks for 25% of peak torque at zero speed.\n")
    P("| joint | I_eff kg m^2 | kp lower | kp from load | kp upper | **kp** | **kd** | swing f_n Hz | sag @ worst rad | action scale rad |")
    P("|---|---|---|---|---|---|---|---|---|---|")
    rec = {}
    for j in legj:
        dof = m.joint(j).dofadr[0]
        mot = MOTORS[MOTOR_OF_JOINT[j]]
        ieff = Mfull[dof, dof] + mot.armature
        lo = mot.armature * (2 * math.pi * F_ARM) ** 2
        hi = mot.peak_torque / ERR_SAT
        kl = worst[j] / DEFL_OF.get(joint_type(j), DEFL)
        kp = float(5 * round(min(max(kl, lo), hi) / 5))
        kd = min(max(2 * ZETA * math.sqrt(kp * ieff), 0.02 * kp), 0.1 * kp)
        kd = round(kd * 2) / 2
        rec[joint_type(j)] = (kp, kd)
        P(f"| {joint_type(j)} | {ieff:.3f} | {lo:.0f} | {kl:.0f} | {hi:.0f} | **{kp:.0f}** | **{kd:.1f}** | "
          f"{math.sqrt(kp/ieff)/(2*math.pi):.1f} | {worst[j]/kp:.2f} | {0.25*mot.peak_torque/kp:.3f} |")
    cur = {joint_type(j): gains_of(j) for j in legj}
    P(f"\nRecommended `GAINS = {rec}`  \nIn `robots/walker_v2.py`: **{'MATCH' if cur == rec else 'DIFFERENT: ' + str(cur)}**")
    return L, dict(period=period, swing_height=0.07, gains=rec, stand_height=h)


def main(md_path: str | None):
    lines, _ = analyse()
    text = "\n".join(lines)
    print(text)
    if md_path:
        with open(md_path, "w", encoding="utf-8") as f:
            f.write("# WalkerV2: first-principles numbers\n\n")
            f.write("Generated by `python -m th_training.scripts.walker_v2_first_principles --md <this file>`. Don't hand-edit it.\n\n")
            f.write(text + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", default=None)
    main(ap.parse_args().md)
