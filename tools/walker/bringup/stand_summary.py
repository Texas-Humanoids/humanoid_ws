#!/usr/bin/env python3
"""Summarize the STAND phases of a bring-up bag (stand-only demo): per joint the hold error vs the
stand pose, motion, peak speed (vs the speed trip), torque (vs the torque cap) and buzz (torque
power above 8 Hz), plus body tilt. Reads an open (still recording) or unclosed bag. In the robot
container:
  python3 tools/walker/bringup/stand_summary.py bringup_logs/bag_<...> [--spec ...] [--robot ...]
"""
import argparse
import glob
import json
import math
import os

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--spec", default="policies/walker_demo/cad_v1/policy_spec.json")
ap.add_argument("--robot", default="src/th_bringup/config/walker_demo_robot.yaml")
ap.add_argument("--torque-caps", default="hip_pitch=15,hip_roll=15,hip_yaw=6,knee_pitch=20,ankle_pitch=10")
ap.add_argument("--speed-trip", type=float, default=2.0)
a = ap.parse_args()
spec = json.load(open(a.spec))
default = dict(zip(spec["joint_order"], spec["default_joint_pos"]))
caps = {k: float(v) for k, v in (x.split("=") for x in a.torque_caps.split(","))}
direction = {m["joint"]: float(m["direction"]) for m in yaml.safe_load(open(a.robot))["motors"]}

uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
modes, js, grav, tau = [], [], [], {}
while True:
    try:
        if not r.has_next():
            break
        topic, data, t = r.read_next()
    except Exception:  # noqa: BLE001  truncated file
        break
    if topic not in ("/policy_bridge/mode", "/joint_states", "/policy_bridge/projected_gravity") and \
            not topic.endswith("/motor/actual_torque"):
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    ts = t * 1e-9
    if topic == "/policy_bridge/mode":
        modes.append((ts, msg.data))
    elif topic == "/joint_states":
        js.append((ts, list(msg.name), np.array(msg.position), np.array(msg.velocity)))
    elif topic == "/policy_bridge/projected_gravity":
        grav.append((ts, msg.x, msg.y))
    else:
        tau.setdefault(topic.split("/")[1] + "_joint", []).append((ts, msg.data))
if not js:
    raise SystemExit("no joint states")
t0, t_end = js[0][0], js[-1][0]
print("modes:", ", ".join(f"{m}@{t - t0:.1f}s" for t, m in modes))
spans = []
for i, (t, m) in enumerate(modes):
    if m == "STAND":
        spans.append((t, modes[i + 1][0] if i + 1 < len(modes) else t_end))
if not spans:
    raise SystemExit("no STAND in this bag")
for s0, s1 in spans:
    h0 = s0 + 2.5  # skip the 2 s ramp
    print(f"\n== STAND {s0 - t0:.1f}s .. {s1 - t0:.1f}s ({s1 - s0:.1f} s; holding from +2.5 s)")
    if s1 - h0 < 1.0:
        print("   (too short to analyze the hold)")
        continue
    names = js[0][1]
    sel = [x for x in js if h0 <= x[0] <= s1]
    T = np.array([x[0] for x in sel])
    Q = np.array([x[2] for x in sel])
    V = np.array([x[3] for x in sel])
    print(f"{'joint':24s} {'mean err':>8s} {'p-p':>6s} {'peak|v|':>8s} {'of trip':>7s} {'peak|tau|':>9s} {'of cap':>6s} {'mean tau':>8s} {'buzz':>6s}")
    for i, n in enumerate(names):
        jt = n.removesuffix("_joint").removeprefix("left_").removeprefix("right_")
        q = Q[:, i]
        tt = np.array([(t, v) for t, v in tau.get(n, []) if h0 <= t <= s1])
        if len(tt):
            tv = tt[:, 1] * direction.get(n, 1.0)   # joint frame
            dt = np.median(np.diff(tt[:, 0])) if len(tt) > 2 else 0.01
            spec_ = np.abs(np.fft.rfft(tv - tv.mean())) ** 2
            f = np.fft.rfftfreq(len(tv), dt)
            buzz = math.sqrt(spec_[f > 8].sum() / max(spec_[f > 0].sum(), 1e-12)) if len(tv) > 32 else float("nan")
            pk, mt = float(np.abs(tv).max()), float(tv.mean())
        else:
            pk = mt = buzz = float("nan")
        print(f"{n:24s} {q.mean() - default.get(n, 0.0):+8.3f} {q.max() - q.min():6.3f} {np.abs(V[:, i]).max():8.2f} "
              f"{np.abs(V[:, i]).max() / a.speed_trip:6.0%} {pk:9.1f} {pk / caps.get(jt, 1e9):6.0%} {mt:+8.2f} {buzz:6.0%}")
    g = np.array([(x, y) for t, x, y in grav if h0 <= t <= s1])
    if len(g):
        pitch = np.degrees(np.arcsin(np.clip(g[:, 0], -1, 1)))
        roll = np.degrees(np.arcsin(np.clip(g[:, 1], -1, 1)))
        print(f"body tilt: pitch {pitch.mean():+.1f} deg (range {pitch.min():+.1f}..{pitch.max():+.1f}), "
              f"roll {roll.mean():+.1f} deg (range {roll.min():+.1f}..{roll.max():+.1f})   (+pitch = nose down)")
print("\nmean err = joint - stand pose (rad); buzz = share of torque fluctuation above 8 Hz (high = chatter)")
