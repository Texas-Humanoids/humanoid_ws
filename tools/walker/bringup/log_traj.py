#!/usr/bin/env python3
"""Joint trajectories from a robot.launch.py log: the motor nodes' 2 s "Actual position ...
target ... Nm" lines, converted to joint angles with the robot yaml (direction, offset).

  python3 tools/walker/bringup/log_traj.py bringup_logs/<launch>.log [--robot <yaml>] [--from <t>]
"""
import argparse
import re
import sys

import yaml

LINE = re.compile(r"\[(\d+\.\d+)\] \[(\w+)\.robstride_motor_node\]: Actual position: (-?[\d.]+) rad, velocity: "
                  r"(-?[\d.]+) rad/s, target: (-?[\d.]+) rad \((\w+)\), ([\d.]+) C, (-?[\d.]+) Nm")
EVENT = re.compile(r"\[(\d+\.\d+)\] \[policy_bridge\]: (STAND|WALK|Disarmed)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--robot", default="src/th_bringup/config/walker_demo_robot.yaml")
    ap.add_argument("--from", dest="t_from", type=float, default=None, help="start time (default: first WALK)")
    a = ap.parse_args()
    cal = {m["joint"].removesuffix("_joint"): m for m in yaml.safe_load(open(a.robot))["motors"]}
    rows, events = [], []
    for raw in open(a.log, "rb"):
        line = raw.decode(errors="ignore")
        if (m := LINE.search(line)):
            t, j, q, v, tgt, state, temp, tau = m.groups()
            c = cal[j]
            d, off = float(c["direction"]), float(c["offset"])
            rows.append((float(t), j, (float(q) - off) / d, (float(tgt) - off) / d, float(v) * d, state, float(temp), float(tau)))
        elif (e := EVENT.search(line)):
            events.append((float(e.group(1)), e.group(2)))
    if not rows:
        sys.exit("no motor lines")
    walk = [t for t, e in events if e == "WALK"]
    t0 = a.t_from if a.t_from is not None else (walk[-1] if walk else rows[0][0])
    print("events:", ", ".join(f"{e}@{t - t0:+.1f}s" for t, e in events if t >= t0 - 10))
    joints = list(cal)
    print(f"{'t(s)':>6s} " + " ".join(f"{j.replace('_pitch', '').replace('left_', 'L').replace('right_', 'R'):>14s}" for j in joints))
    by_slot = {}
    for t, j, qj, tj, vj, state, temp, tau in rows:
        if t < t0 - 4:
            continue
        slot = round((t - t0) / 2.0) * 2
        by_slot.setdefault(slot, {})[j] = (qj, tj, tau, state)
    for slot in sorted(by_slot):
        cells = []
        for j in joints:
            if j in by_slot[slot]:
                qj, tj, tau, state = by_slot[slot][j]
                cells.append(f"{qj:+.2f}>{tj:+.2f}{'' if state == 'PD' else '*'}")
            else:
                cells.append("--")
        print(f"{slot:+6.0f} " + " ".join(f"{c:>14s}" for c in cells))
    print("cell = joint angle > target (rad, joint frame); * = damping (not tracking targets)")
    torques = {}
    for t, j, qj, tj, vj, state, temp, tau in rows:
        if t >= t0:
            torques[j] = max(torques.get(j, 0.0), abs(tau))
    print("peak |torque| sampled after t0 (Nm):", {j: round(v, 1) for j, v in torques.items()})


if __name__ == "__main__":
    main()
