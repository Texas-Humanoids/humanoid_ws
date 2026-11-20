#!/usr/bin/env python3
"""Non-interactive bring-up steps: one command = one test, prints a report, exits (0 = pass).

Same controller, safety and conventions as jog.py (it reuses jog.Bringup), so it can be run over
ssh by a person or by Claude Code. Every motion ends with 0.4 s of damping, then the motors go limp.
Robot HANGING in the gantry for everything that moves. robot.launch.py must be stopped.

  python3 tools/walker/bringup/bringup.py scan                       # step 1: who answers, where, VBUS, faults
  python3 tools/walker/bringup/bringup.py wiggle [--ids 15,14] [--amp 0.1]   # step 2: each motor +-amp, one at a time
  python3 tools/walker/bringup/bringup.py pulse left_knee_pitch_joint         # step 3: one +0.15 rad pulse (motor +)
  python3 tools/walker/bringup/bringup.py pulse-all [--amp 0.2]                # every joint +amp in the joint frame, one at a time
  python3 tools/walker/bringup/bringup.py set-direction left_knee_pitch_joint -1
  python3 tools/walker/bringup/bringup.py calibrate [--pose zero|default] [--ids 11-15]  # offsets (robot held in the pose)
  python3 tools/walker/bringup/bringup.py check-cal [--pose zero]             # step 4: re-check after a power cycle
  python3 tools/walker/bringup/bringup.py ranges --seconds 60 [--write-limits] # step 4: move joints by hand to the stops
  python3 tools/walker/bringup/bringup.py dircheck --seconds 45                 # directions by hand, motors limp
  python3 tools/walker/bringup/bringup.py read                                # positions, limp (any time)
  python3 tools/walker/bringup/bringup.py pose default --hold 5 [--policy-gains]  # step 5
  python3 tools/walker/bringup/bringup.py squat [--depth 0.25] [--policy-gains]   # step 5
  python3 tools/walker/bringup/bringup.py estop                               # stop frames to every motor

Defaults: --robot src/th_bringup/config/walker_demo_robot.yaml, --policy cad (policies/walker_demo/cad_v1).
State shared with jog.py: directions/offsets/limits in the robot yaml; which directions were checked,
measured ranges and calibration time in bringup_session.json next to it. --sim runs on simulated drives.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
import jog  # noqa: E402
import chirp  # noqa: E402  (jog put sysid on the path)
import rsproto as rs  # noqa: E402

PERIOD = 1.0 / jog.RATE
ORDER = ["ankle_pitch", "knee_pitch", "hip_yaw", "hip_roll", "hip_pitch"]   # lightest load first
MARGIN = 0.05   # rad: a real stop must clear the policy's used range by this much


# ============================================================================= session / yaml
def session_path(a):
    return os.path.join(os.path.dirname(os.path.abspath(a.robot)), "bringup_session.json")


def load_session(a):
    try:
        return json.load(open(session_path(a)))
    except (OSError, ValueError):
        return {}


def save_session(a, s):
    s["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    json.dump(s, open(session_path(a), "w"), indent=1)


def yaml_set(a, joint, key, value, fmt="{:.4f}"):
    text = open(a.robot).read()
    pat = rf"(- joint: {re.escape(joint)}\n(?:    .*\n)*?    {key}: )\S+"
    text, n = re.subn(pat, lambda m: m.group(1) + fmt.format(value), text)
    if n != 1:
        sys.exit(f"could not find '{key}:' for {joint} in {a.robot}; nothing written")
    open(a.robot, "w").write(text)


def spec_of(a):
    return json.load(open(a.spec))


def used_ranges(a):
    p = os.path.join(os.path.dirname(os.path.abspath(a.spec)), "used_ranges.json")
    return json.load(open(p)) if os.path.exists(p) else {}


# ============================================================================= motor plumbing
def open_all(a, need_offsets=False):
    argv = ["--robot", a.robot, "--spec", a.spec, "--window", str(a.window)]
    if a.sim:
        argv.append("--sim")
    if getattr(a, "policy_gains", False):
        argv.append("--policy-gains")
    if a.torque_limit is not None:
        argv += ["--torque-limit", str(a.torque_limit)]
    ja = jog.parse_args(argv)
    ja.trust_directions = True
    motors, buses, clock, missing = jog.build(ja)
    b = jog.Bringup(motors, buses, clock, ja)
    b._t = clock.now() + PERIOD
    b.stats = {}
    run(b, 0.3)
    if need_offsets:
        bad = [m.joint for m in motors if not m.calibrated]
        if bad:
            close(b)
            sys.exit(f"offsets missing for {bad}: run `calibrate` first")
    return b, missing


def run(b, seconds, until=None):
    end = b.clock.now() + seconds
    while b.clock.now() < end:
        b._t = jog.loop_once(b, b._t, PERIOD, b.a.sim)
        for m in b.motors:
            if m.enabled and m.st is not None:
                s = b.stats.setdefault(m.id, dict(err=0.0, tau=0.0, qmin=m.st.q, qmax=m.st.q, n=0, tau_sum=0.0))
                s["err"] = max(s["err"], abs(m.q_ref - m.st.q))
                s["tau"] = max(s["tau"], abs(m.st.tau))
                s["qmin"], s["qmax"] = min(s["qmin"], m.st.q), max(s["qmax"], m.st.q)
                s["n"] += 1
                s["tau_sum"] += abs(m.st.tau)
        if b.estop_reason:
            return False
        if until is not None and until():
            return True
    return until is None


def finish(b):
    """0.4 s damping on everything enabled, then limp. Not a safety trip."""
    if any(m.enabled for m in b.motors):
        for m in b.motors:
            if m.enabled:
                m.state, m.ramp, m.pulse = "damp", None, None
        reason, b.estop_reason = b.estop_reason, None
        run(b, 0.4)
        b.estop_reason = reason
        for m in b.motors:
            b.disable(m)


def close(b):
    try:
        finish(b)
    finally:
        for bus in b.buses.values():
            bus.close()


def by_name(b, key):
    for m in b.motors:
        if key in (m.joint, str(m.id), m.joint.replace("_joint", "")):
            return m
    sys.exit(f"no motor '{key}' (known: {', '.join(m.joint for m in b.motors)})")


def fq(x, p=3):
    return "    --" if x is None else f"{x:+.{p}f}"


def table(rows, head):
    w = [max(len(str(r[i])) for r in rows + [head]) for i in range(len(head))]
    print("  ".join(str(h).ljust(w[i]) for i, h in enumerate(head)))
    for r in rows:
        print("  ".join(str(c).ljust(w[i]) for i, c in enumerate(r)))


# ============================================================================= steps
def cmd_scan(a):
    """Step 1."""
    import yaml
    cfg = {m["joint"]: m for m in (yaml.safe_load(open(a.robot)) or {}).get("motors", [])}
    if not a.sim:
        print("CAN interfaces up:", ", ".join(chirp.can_interfaces()) or "NONE")
    b, missing = open_all(a)
    ok = not missing
    rows = []
    for m in b.motors:
        st = m.st
        mech = m.link.read_param(rs.P_MECH_POS, timeout=0.1)
        notes = []
        if st is None:
            notes.append("NO STATUS"); ok = False
        else:
            if st.faults:
                notes.append(f"FAULT {st.fault_names()}"); ok = False
            if st.temp > 55:
                notes.append(f"HOT {st.temp:.0f}C")
            if mech is not None and abs(st.q - mech) > 0.05:
                notes.append(f"MIT q != mechPos ({mech:+.2f}): run sysid/mit_all.py home")
            if abs(st.q) > 4 * math.pi - 1.0:
                notes.append("near the +-4pi MIT wrap: run mit_all.py home")
        want = cfg.get(m.joint, {}).get("can_interface")
        if want and want != m.iface and not a.sim:
            notes.append(f"yaml says {want}")
        if m.vbus is not None and not 34.0 <= m.vbus <= 40.5:
            notes.append(f"VBUS {m.vbus:.1f} V out of 34-40.5"); ok = False
        rows.append((m.id, m.joint.replace("_joint", ""), m.model, m.iface, f"{m.vbus:.1f}V" if m.vbus else "--",
                     f"{st.temp:.0f}C" if st else "--", fq(st.q if st else None), "; ".join(notes) or "ok"))
    close(b)
    table(rows, ("id", "joint", "model", "iface", "VBUS", "temp", "q_motor", "notes"))
    ifaces = {}
    for r in rows:
        ifaces.setdefault(r[3], []).append(r[0])
    print("per adapter:", "; ".join(f"{k}: {v}" for k, v in sorted(ifaces.items())))
    if missing:
        print(f"MISSING (no reply): {missing}")
    print("SCAN", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cmd_wiggle(a):
    """Step 2: each motor alone, +amp, back, -amp, back (motor frame)."""
    b, _ = open_all(a)
    sel = [b.by_id[int(i)] if i.isdigit() else by_name(b, i) for i in a.ids.split(",")] if a.ids else \
        sorted(b.motors, key=lambda m: (ORDER.index(jog.joint_type(m.joint)), m.id))
    rows, ok = [], True
    try:
        for m in sel:
            b.stats.pop(m.id, None)
            b.enable(m)
            if not m.enabled:
                rows.append((m.id, m.joint, "", "", "", "", "FAIL: " + b.msg)); ok = False
                continue
            q0, T, amp = m.st.q, a.seconds, a.amp
            t0 = b.clock.now()

            def f(t, q0=q0, t0=t0):
                x = (t - t0) / T
                seg, u = int(x), jog.smooth(x - int(x))
                return q0 + amp * {0: u, 1: 1 - u, 2: -u, 3: -(1 - u)}.get(seg, 0.0)
            m.ramp, m.ramp_end = f, t0 + 4 * T
            m.q_en = q0
            run(b, 4 * T + 0.5, until=lambda: m.ramp is None and b.clock.now() > t0 + 4 * T + 0.3)
            s = b.stats.get(m.id, {})
            up, down = s.get("qmax", q0) - q0, q0 - s.get("qmin", q0)
            fails = []
            if b.estop_reason:
                fails.append(f"SAFETY STOP: {b.estop_reason}")
            if m.flag == "STALL":
                fails.append("STALL (hit something / binding)")
            if up < 0.8 * amp or down < 0.8 * amp:
                fails.append(f"did not reach +-{amp:g}")
            if s.get("err", 1) > 0.06:
                fails.append(f"tracking error {s.get('err', 0):.3f}")
            rows.append((m.id, m.joint.replace("_joint", ""), f"+{up:.3f}/-{down:.3f}", f"{s.get('err', 0):.3f}",
                         f"{s.get('tau', 0):.2f}", f"{s.get('tau_sum', 0) / max(s.get('n', 1), 1):.2f}",
                         "PASS" if not fails else "FAIL: " + "; ".join(fails)))
            ok &= not fails
            b.disable(m)
            if b.estop_reason:
                break
            run(b, 0.2)
    finally:
        close(b)
    table(rows, ("id", "joint", "reached", "max err", "peak tau", "mean |tau|", "result"))
    print("(mean |tau| while moving ~ friction + gravity: RS03 ~0.3-0.5 Nm unloaded; big constant values = rubbing)")
    print("WIGGLE", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cmd_pulse(a):
    """Step 3: one pulse, motor-positive. A human says whether the joint did '+' (jog.EXPECT)."""
    b, _ = open_all(a)
    m = by_name(b, a.joint)
    b.sel = b.motors.index(m)           # start_pulse acts on the selected motor
    b.stats.pop(m.id, None)
    b.start_pulse()
    q0 = m.q_en
    run(b, 4.0, until=lambda: b.asking is m)
    s = b.stats.get(m.id, {})
    close(b)
    moved = s.get("qmax", q0) - q0
    print(f"{m.id} {m.joint}: motor moved +{moved:.3f} rad and back (target +{jog.PULSE_AMP}).")
    print(f"QUESTION: did the joint go '{jog.EXPECT[jog.joint_type(m.joint)]}'?")
    print(f"  yes -> set-direction {m.joint} +1      no -> set-direction {m.joint} -1")
    ok = moved > 0.1 and not b.estop_reason
    print("PULSE", "DONE" if ok else f"FAIL ({b.estop_reason or 'did not move'})")
    return 0 if ok else 1


def cmd_set_direction(a):
    d = float(a.direction)
    assert d in (1.0, -1.0), "direction is +1 or -1"
    yaml_set(a, a.joint, "direction", d, "{:.1f}")
    s = load_session(a)
    s.setdefault("directions_checked", {})[a.joint] = d
    if s.get("calibration", {}).get("pose") != "zero":
        s.pop("calibration", None)      # offsets from the default pose depend on direction; zero-pose ones don't
    save_session(a, s)
    left = [j for j in jog_joints() if j not in s["directions_checked"]]
    print(f"{a.joint}: direction {d:+.0f} written. Checked {len(s['directions_checked'])}/10."
          + (f" Still to do: {', '.join(left)}" if left else " All directions checked: next is calibrate."))
    return 0


def jog_joints():
    return [j for _, j, _ in jog.DEFAULT_MAP]


def read_limp(b, seconds):
    samples = {m.id: [] for m in b.motors}
    end = b.clock.now() + seconds
    while b.clock.now() < end:
        run(b, 0.05)
        for m in b.motors:
            if m.st is not None and m.last_fb is not None and b.clock.now() - m.last_fb < 0.2:
                samples[m.id].append(m.st.q)
    return samples


def pose_of(a, name):
    if name == "zero":
        return {j: 0.0 for j in jog_joints()}
    sp = spec_of(a)
    return dict(zip(sp["joint_order"], sp["default_joint_pos"]))


def cmd_read(a):
    b, _ = open_all(a)
    samples = read_limp(b, a.seconds)
    rows = []
    for m in b.motors:
        s = samples[m.id]
        q = sum(s) / len(s) if s else None
        rows.append((m.id, m.joint.replace("_joint", ""), fq(q), fq(m.to_joint(q)), f"{m.direction:+.0f}", fq(m.offset),
                     f"[{fq(m.lower, 2)}, {fq(m.upper, 2)}]"))
    close(b)
    table(rows, ("id", "joint", "q_motor", "q_joint", "dir", "offset", "yaml limits"))
    return 0


def cmd_calibrate(a):
    s = load_session(a)
    checked = s.get("directions_checked", {})
    todo = [j for j in jog_joints() if j not in checked]
    # At the zero pose offset = q_motor, whatever the direction, so only the default pose needs them.
    if a.pose != "zero" and todo and not a.trust_directions:
        sys.exit(f"directions not checked yet for {todo} (pulse + set-direction), or pass --trust-directions")
    b, _ = open_all(a)
    samples = read_limp(b, 1.0)
    pose = pose_of(a, a.pose)
    rows, ok, offs = [], True, {}
    only = None
    if a.ids:
        only = set()
        for part in a.ids.split(","):
            lo, _, hi = part.partition("-")
            only.update(range(int(lo), int(hi or lo) + 1))
    for m in b.motors:
        if only is not None and m.id not in only:
            continue
        q = samples[m.id]
        if not q:
            rows.append((m.id, m.joint, "", "", "NO FEEDBACK")); ok = False
            continue
        spread = max(q) - min(q)
        off = sum(q) / len(q) - m.direction * pose[m.joint]
        offs[m.joint] = off
        flag = "MOVING - hold still" if spread > 0.01 else "ok"
        ok &= spread <= 0.01
        rows.append((m.id, m.joint.replace("_joint", ""), f"{sum(q) / len(q):+.4f}", f"{spread:.4f}", f"{off:+.4f}", flag))
    close(b)
    table(rows, ("id", "joint", "q_motor", "spread", "offset", ""))
    if not ok:
        print("CALIBRATE FAIL: nothing written")
        return 1
    for j, off in offs.items():
        yaml_set(a, j, "offset", off)
    s["calibration"] = dict(pose=a.pose, time=time.strftime("%Y-%m-%d %H:%M:%S"), offsets=offs)
    save_session(a, s)
    print(f"CALIBRATE PASS: {len(offs)} offsets written to {a.robot} (pose '{a.pose}')")
    return 0


def cmd_check_cal(a):
    b, _ = open_all(a, need_offsets=True)
    samples = read_limp(b, 1.0)
    pose = pose_of(a, a.pose)
    rows, worst = [], 0.0
    for m in b.motors:
        q = samples[m.id]
        qj = m.to_joint(sum(q) / len(q)) if q else None
        err = None if qj is None else qj - pose[m.joint]
        worst = max(worst, abs(err) if err is not None else 9)
        rows.append((m.id, m.joint.replace("_joint", ""), fq(qj), fq(err), "ok" if err is not None and abs(err) < 0.03 else "OFF"))
    close(b)
    table(rows, ("id", "joint", "q_joint", "error vs pose", ""))
    print(f"CHECK-CAL {'PASS' if worst < 0.03 else 'FAIL'} (worst {worst:.3f} rad). A jump of ~0.70 (RS03) / ~0.81 (RS02) "
          "means the drive lost its absolute position: calibrate at every power-up.")
    return 0 if worst < 0.03 else 1


def cmd_ranges(a):
    """Step 4: motors limp; a person moves each joint by hand to both hard stops."""
    b, _ = open_all(a, need_offsets=True)
    for m in b.motors:
        m.seen = None
    print(f"Recording for {a.seconds:g} s: move each joint slowly to BOTH hard stops...", flush=True)
    t0 = b.clock.now()
    while b.clock.now() - t0 < a.seconds:
        run(b, 1.0)
    used = used_ranges(a)
    sp = spec_of(a)
    plim = sp.get("joint_limits", {})
    rows, ok, meas = [], True, {}
    for m in b.motors:
        if m.seen is None:
            rows.append((m.id, m.joint, "", "", "", "NO DATA")); ok = False
            continue
        lo, hi = sorted([m.to_joint(m.seen[0]), m.to_joint(m.seen[1])])
        meas[m.joint] = [round(lo, 3), round(hi, 3)]
        u = used.get(m.joint)
        verdict = "ok"
        if u and (lo > u[0] - MARGIN or hi < u[1] + MARGIN):
            verdict = f"TOO SMALL for the policy (needs [{u[0] - MARGIN:+.2f}, {u[1] + MARGIN:+.2f}])"
            ok = False
        p = plim.get(m.joint, [m.lower, m.upper])
        rec = [round(max(p[0], lo + 0.03), 3), round(min(p[1], hi - 0.03), 3)]
        meas[m.joint + "_suggested_limits"] = rec
        rows.append((m.id, m.joint.replace("_joint", ""), f"[{lo:+.3f}, {hi:+.3f}]",
                     f"[{u[0]:+.2f}, {u[1]:+.2f}]" if u else "--", f"[{rec[0]:+.3f}, {rec[1]:+.3f}]", verdict))
    close(b)
    table(rows, ("id", "joint", "measured (joint rad)", f"{sp.get('limits_variant', '?')} policy uses", "suggested yaml limits", ""))
    s = load_session(a)
    s["ranges"] = dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), policy=a.policy, measured=meas)
    save_session(a, s)
    if a.write_limits and ok:
        for m in b.motors:
            lo, hi = meas[m.joint + "_suggested_limits"]
            yaml_set(a, m.joint, "lower", lo)
            yaml_set(a, m.joint, "upper", hi)
        print(f"wrote suggested limits to {a.robot}")
    print("RANGES", "PASS" if ok else "FAIL", f"for policy {a.policy}. (Hand-moved ranges only count if you really reached each stop.)")
    return 0 if ok else 1


def cmd_dircheck(a):
    """Directions by hand, motors limp: move each joint in its '+' direction (jog.EXPECT) and back.
    Per joint, the larger excursion from the start says the sign. Needs offsets (calibrate)."""
    b, _ = open_all(a, need_offsets=True)
    start = {m.id: m.to_joint(m.st.q) for m in b.motors}
    for m in b.motors:
        m.seen = None
    print(f"Recording {a.seconds:g} s: move each joint by hand in its + direction (about 0.3 rad / 20 deg) and back.", flush=True)
    t0 = b.clock.now()
    while b.clock.now() - t0 < a.seconds:
        run(b, 1.0)
    rows, bad, unmoved = [], [], []
    for m in b.motors:
        if m.seen is None:
            rows.append((m.id, m.joint, "", "", "NO DATA")); unmoved.append(m.joint)
            continue
        js = sorted([m.to_joint(m.seen[0]), m.to_joint(m.seen[1])])
        up, down = js[1] - start[m.id], start[m.id] - js[0]
        if max(up, down) < 0.1:
            verdict = "not moved (< 0.1 rad)"; unmoved.append(m.joint)
        elif up >= 2 * down:
            verdict = "+ OK"
        elif down >= 2 * up:
            verdict = "REVERSED: flip direction"; bad.append(m.joint)
        else:
            verdict = "moved both ways: redo"; unmoved.append(m.joint)
        rows.append((m.id, m.joint.replace("_joint", ""), f"+{up:.2f}", f"-{down:.2f}", verdict,
                     jog.EXPECT[jog.joint_type(m.joint)]))
    close(b)
    table(rows, ("id", "joint", "max +", "max -", "verdict", "+ means"))
    if bad:
        print("REVERSED:", ", ".join(bad), "-> set-direction <joint> <opposite>, then calibrate again")
    if unmoved:
        print("redo:", ", ".join(unmoved))
    ok = not bad and not unmoved
    print("DIRCHECK", "PASS" if ok else "INCOMPLETE" if not bad else "FAIL")
    return 0 if ok else 1


def cmd_pulse_all(a):
    """Powered direction check: each joint in turn moves +amp in the JOINT frame (direction from the
    yaml), holds, returns, goes limp. A joint whose + side is at a yaml limit (right hip roll, upper 0)
    moves -amp instead. A person watches that each motion matches jog.EXPECT."""
    b, _ = open_all(a, need_offsets=True)
    sel = sorted(b.motors, key=lambda m: (m.id // 10, ORDER.index(jog.joint_type(m.joint))))
    if a.ids:
        order = [int(x) for x in a.ids.split(",")]
        sel = [b.by_id[i] for i in order if i in b.by_id]
    rows, ok = [], True
    try:
        for m in sel:
            sign = -1.0 if (m.upper is not None and m.upper < 0.1) else 1.0
            jog.PULSE_AMP = sign * a.amp * m.direction          # motor frame
            b.sel = b.motors.index(m)
            b.stats.pop(m.id, None)
            b.asking = None
            b.start_pulse()
            if not m.enabled:
                rows.append((m.id, m.joint, "", "", "", "FAIL: " + b.msg)); ok = False
                continue
            q0 = m.to_joint(m.q_en)
            run(b, 4.0, until=lambda: b.asking is m)
            s = b.stats.get(m.id, {})
            ja, jb = m.to_joint(s.get("qmin", m.q_en)), m.to_joint(s.get("qmax", m.q_en))
            moved = (max(ja, jb) - q0) if sign > 0 else (min(ja, jb) - q0)
            fails = []
            if b.estop_reason:
                fails.append(f"SAFETY STOP: {b.estop_reason}")
            if abs(moved) < 0.6 * a.amp or moved * sign < 0:
                fails.append("blocked / did not follow")
            want = jog.EXPECT[jog.joint_type(m.joint)] if sign > 0 else "OPPOSITE of: " + jog.EXPECT[jog.joint_type(m.joint)]
            rows.append((m.id, m.joint.replace("_joint", ""), f"{sign * a.amp:+.2f}", f"{moved:+.3f}",
                         f"{s.get('tau', 0):.1f}", "ok" if not fails else "FAIL: " + "; ".join(fails), want))
            ok &= not fails
            b.disable(m)
            if b.estop_reason:
                break
            run(b, a.gap)
    finally:
        close(b)
    table(rows, ("id", "joint", "cmd (joint rad)", "moved", "peak tau", "", "it should have looked like"))
    print("PULSE-ALL", "PASS (encoders agree; directions confirmed only if every motion LOOKED right)" if ok else "FAIL")
    return 0 if ok else 1


def enable_all(b):
    for m in b.motors:
        b.enable(m)
    bad = [m.id for m in b.motors if not m.enabled]
    if bad:
        print(f"could not enable {bad}: {b.msg}")
    return not bad


def report_motion(b, title):
    rows = []
    for m in b.motors:
        s = b.stats.get(m.id, {})
        rows.append((m.id, m.joint.replace("_joint", ""), fq(m.to_joint(m.st.q) if m.st else None),
                     f"{s.get('err', 0):.3f}", f"{s.get('tau', 0):.1f}", m.flag))
    table(rows, ("id", "joint", "q_joint end", "max err", "peak tau", "flag"))
    ok = not b.estop_reason and all(m.flag != "STALL" for m in b.motors)
    print(title, "PASS" if ok else f"FAIL ({b.estop_reason or 'stall'})")
    return 0 if ok else 1


def cmd_pose(a):
    b, _ = open_all(a, need_offsets=True)
    try:
        if not enable_all(b):
            return 1
        pose = pose_of(a, a.name)
        for item in a.set or []:            # e.g. hip_pitch=-0.44 (both legs) or left_knee_pitch=0.5
            key, _, val = item.partition("=")
            hits = [j for j in pose if j.replace("_joint", "") == key or j.replace("_joint", "").endswith("_" + key)]
            if not hits:
                sys.exit(f"--set {item}: no joint matches '{key}'")
            for j in hits:
                pose[j] = float(val)
        print("target pose:", {j.replace("_joint", ""): round(v, 3) for j, v in pose.items() if v})
        b.go_pose(pose, duration=a.ramp)
        run(b, a.ramp + a.hold)
        return report_motion(b, f"POSE {a.name}")
    finally:
        close(b)


def cmd_squat(a):
    b, _ = open_all(a, need_offsets=True)
    try:
        if not enable_all(b):
            return 1
        pose = pose_of(a, "default")
        b.go_pose(pose, duration=3.0)
        if run(b, 3.5) is False:
            return report_motion(b, "SQUAT")
        b.squat(pose, depth=a.depth, period=a.period, cycles=a.cycles)
        run(b, a.period * a.cycles + 1.0)
        return report_motion(b, "SQUAT")
    finally:
        close(b)


def cmd_estop(a):
    import yaml
    ids = [int(m["motor_id"]) for m in (yaml.safe_load(open(a.robot)) or {}).get("motors", []) if m.get("motor_id")]
    if a.sim:
        print("estop (sim): nothing to do")
        return 0
    clock = chirp.RealClock()
    sent = []
    for name in chirp.can_interfaces():
        try:
            bus = chirp.SocketCanBus(name, clock)
        except Exception as e:  # noqa: BLE001
            print(f"{name}: {e}")
            continue
        for _ in range(3):
            for i in ids:
                try:
                    bus.send(*rs.stop(i, clear_faults=False))
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.0005)
        sent.append(name)
        bus.close()
    print(f"ESTOP: stop frames x3 to {ids} on {', '.join(sent) or 'no interface'}")
    return 0


# ============================================================================= cli
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--robot", default=os.path.join(REPO, "src", "th_bringup", "config", "walker_demo_robot.yaml"))
    ap.add_argument("--policy", default="cad", choices=("cad", "cad_safe", "wide"))
    ap.add_argument("--spec", default=None, help="default: policies/walker_demo/<policy>_v1/policy_spec.json")
    ap.add_argument("--window", type=float, default=0.6)
    ap.add_argument("--torque-limit", type=float, default=None)
    ap.add_argument("--sim", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    p = sub.add_parser("wiggle")
    p.add_argument("--ids", default=None, help="comma list of IDs or joint names (default: all, ankles first)")
    p.add_argument("--amp", type=float, default=0.1)
    p.add_argument("--seconds", type=float, default=1.5, help="per quarter of the wiggle")
    p = sub.add_parser("pulse")
    p.add_argument("joint")
    p = sub.add_parser("pulse-all")
    p.add_argument("--amp", type=float, default=0.2, help="joint rad")
    p.add_argument("--gap", type=float, default=1.0, help="s between joints")
    p.add_argument("--ids", default=None, help="comma list of motor IDs, in this order (default: all)")
    p = sub.add_parser("set-direction")
    p.add_argument("joint")
    p.add_argument("direction")
    p = sub.add_parser("read")
    p.add_argument("--seconds", type=float, default=1.0)
    p = sub.add_parser("calibrate")
    p.add_argument("--pose", choices=("zero", "default"), default="zero")
    p.add_argument("--ids", default=None, help="only these motors (e.g. 11-15 while the other leg is down)")
    p.add_argument("--trust-directions", action="store_true")
    p = sub.add_parser("check-cal")
    p.add_argument("--pose", choices=("zero", "default"), default="zero")
    p = sub.add_parser("ranges")
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--write-limits", action="store_true")
    p = sub.add_parser("dircheck")
    p.add_argument("--seconds", type=float, default=45.0)
    p = sub.add_parser("pose")
    p.add_argument("name", choices=("default", "zero"))
    p.add_argument("--hold", type=float, default=5.0)
    p.add_argument("--ramp", type=float, default=3.0)
    p.add_argument("--policy-gains", action="store_true")
    p.add_argument("--set", action="append", help="override a joint: hip_pitch=-0.44 (both legs) or left_knee_pitch=0.5")
    p = sub.add_parser("squat")
    p.add_argument("--depth", type=float, default=0.25)
    p.add_argument("--period", type=float, default=4.0)
    p.add_argument("--cycles", type=int, default=3)
    p.add_argument("--policy-gains", action="store_true")
    sub.add_parser("estop")
    a = ap.parse_args(argv)
    a.spec = a.spec or os.path.join(REPO, "policies", "walker_demo", f"{a.policy}_v1", "policy_spec.json")
    fn = {"scan": cmd_scan, "wiggle": cmd_wiggle, "pulse": cmd_pulse, "pulse-all": cmd_pulse_all, "set-direction": cmd_set_direction,
          "read": cmd_read, "calibrate": cmd_calibrate, "check-cal": cmd_check_cal, "ranges": cmd_ranges,
          "dircheck": cmd_dircheck, "pose": cmd_pose, "squat": cmd_squat, "estop": cmd_estop}[a.cmd]
    return fn(a)


if __name__ == "__main__":
    sys.exit(main())
