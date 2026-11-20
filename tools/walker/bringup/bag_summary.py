#!/usr/bin/env python3
"""Summarize a bring-up bag: bridge modes; per joint during WALK the range, peak speed, dominant
motion frequency and tracking; and a per-second timeline of body tilt, joint speed and torque, to
see how a run developed. Copes with a bag that was never closed (power cut): reads the raw .mcap
until the data stops. Runs in the robot container:
  python3 tools/walker/bringup/bag_summary.py bringup_logs/bag_<...> [--spec <policy_spec.json>]
"""
import argparse
import glob
import json
import math
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PEAK = {"hip_pitch": 60.0, "hip_roll": 60.0, "hip_yaw": 17.0, "knee_pitch": 60.0, "ankle_pitch": 36.0}


def jtype(name):
    return name.replace("left_", "").replace("right_", "").replace("_joint", "")


def dominant_freq(x, dt):
    if len(x) < 32:
        return float("nan")
    spec = np.abs(np.fft.rfft(x - x.mean()))
    freqs = np.fft.rfftfreq(len(x), dt)
    return float(freqs[1 + int(np.argmax(spec[1:]))])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--spec", default="policies/walker_demo/cad_v1/policy_spec.json")
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    default = dict(zip(spec["joint_order"], spec["default_joint_pos"]))
    uri, sid = a.bag, ""
    if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
        uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"  # never closed: no index
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    modes, js, tg, cmd, grav, tau = [], [], [], [], [], {}
    n_read, broken = 0, None
    while True:
        try:
            if not r.has_next():
                break
            topic, data, t = r.read_next()
        except Exception as e:  # noqa: BLE001  truncated file: stop at the last good message
            broken = str(e)[:120]
            break
        n_read += 1
        if topic not in ("/policy_bridge/mode", "/joint_states", "/policy/joint_targets", "/cmd_vel",
                         "/policy_bridge/projected_gravity") and not topic.endswith("/motor/actual_torque"):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        ts = t * 1e-9
        if topic == "/policy_bridge/mode":
            modes.append((ts, msg.data))
        elif topic == "/joint_states":
            js.append((ts, list(msg.name), np.array(msg.position), np.array(msg.velocity)))
        elif topic == "/policy/joint_targets":
            tg.append((ts, np.array(msg.position)))
        elif topic == "/cmd_vel":
            cmd.append((ts, msg.linear.x, msg.angular.z))
        elif topic == "/policy_bridge/projected_gravity":
            grav.append((ts, msg.x, msg.y, msg.z))
        else:
            tau.setdefault(topic.split("/")[1] + "_joint", []).append((ts, msg.data))
    print(f"read {n_read} messages" + (f" (file ends early: {broken})" if broken else ""))
    if not js:
        print("no joint states")
        return
    t0 = js[0][0]
    print("modes:", ", ".join(f"{m}@{t - t0:.1f}s" for t, m in modes))
    walk = [t for t, m in modes if m == "WALK"]
    if not walk:
        print("no WALK in this bag")
        return
    w0 = walk[-1]
    after = [t for t, m in modes if t > w0]
    w1 = after[0] if after else js[-1][0]
    print(f"WALK {w0 - t0:.1f}s .. {w1 - t0:.1f}s ({w1 - w0:.1f} s){'' if after else ' (no disarm recorded: power cut?)'}")
    c = [(x, z) for t, x, z in cmd if w0 <= t <= w1]
    if c:
        print(f"stick during WALK: vx {min(x for x, _ in c):+.2f}..{max(x for x, _ in c):+.2f}  "
              f"wz {min(z for _, z in c):+.2f}..{max(z for _, z in c):+.2f}")
    names = js[0][1]
    sel = [i for i, (t, *_) in enumerate(js) if w0 <= t <= w1]
    T = np.array([js[i][0] for i in sel])
    Q = np.array([js[i][2] for i in sel])
    V = np.array([js[i][3] for i in sel])
    dt = float(np.median(np.diff(T))) if len(T) > 2 else 0.02
    tgt = [(t, p) for t, p in tg if w0 <= t <= w1]
    print(f"\n{'joint':24s} {'range rel. default':>20s} {'peak|v|':>8s} {'freq':>7s} {'track rms':>9s} {'peak|tau|':>9s} {'of peak':>7s}")
    for i, n in enumerate(names):
        q = Q[:, i] - default.get(n, 0.0)
        err = float("nan")
        if len(tgt) > 2:
            tq = np.interp(T, [t for t, _ in tgt], [p[i] for _, p in tgt])
            err = float(np.sqrt(np.mean((Q[:, i] - tq) ** 2)))
        tt = [v for t, v in tau.get(n, []) if w0 <= t <= w1]
        pt = max((abs(v) for v in tt), default=float("nan"))
        print(f"{n:24s} {q.min():+8.3f}..{q.max():+8.3f}  {np.abs(V[:, i]).max():8.2f} {dominant_freq(q, dt):6.2f}Hz {err:9.3f} "
              f"{pt:9.1f} {pt / PEAK[jtype(n)]:6.0%}")
    g = [(t, x, y, zz) for t, x, y, zz in grav if w0 <= t <= w1]
    if g:
        GT = np.array([t for t, *_ in g])
        pitch = np.degrees(np.arcsin(np.clip([x for _, x, _, _ in g], -1, 1)))
        roll = np.degrees(np.arcsin(np.clip([y for _, _, y, _ in g], -1, 1)))
        gdt = float(np.median(np.diff(GT))) if len(GT) > 2 else 0.02
        print(f"\nbody tilt during WALK (from IMU gravity, + = nose down / left side down):")
        print(f"  pitch {pitch.min():+.1f}..{pitch.max():+.1f} deg, main oscillation {dominant_freq(pitch, gdt):.2f} Hz")
        print(f"  roll  {roll.min():+.1f}..{roll.max():+.1f} deg, main oscillation {dominant_freq(roll, gdt):.2f} Hz")
    print("\nper-second timeline during WALK:")
    print(f"{'t':>5s} {'pitch':>12s} {'roll':>12s} {'max|v| joint':>24s} {'max|tau| joint':>26s}")
    for k in range(int(math.ceil(w1 - w0))):
        a_, b_ = w0 + k, w0 + k + 1
        m = (T >= a_) & (T < b_)
        line = f"{k:4d}s "
        if g:
            gm = (GT >= a_) & (GT < b_)
            line += (f"{pitch[gm].min():+5.1f}..{pitch[gm].max():+5.1f} {roll[gm].min():+5.1f}..{roll[gm].max():+5.1f} "
                     if gm.any() else f"{'':12s} {'':12s} ")
        if m.any():
            j = int(np.argmax(np.abs(V[m]).max(axis=0)))
            line += f"{np.abs(V[m]).max():6.1f} {names[j].replace('_joint', ''):>17s} "
        best = (0.0, "")
        for n, series in tau.items():
            vals = [abs(v) for t, v in series if a_ <= t < b_]
            if vals and max(vals) > best[0]:
                best = (max(vals), n)
        if best[1]:
            line += f"{best[0]:6.1f} Nm {best[1].replace('_joint', ''):>17s}"
        print(line)


if __name__ == "__main__":
    main()
