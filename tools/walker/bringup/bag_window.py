#!/usr/bin/env python3
"""Print joint angle, speed and policy target for chosen joints in a time window of a bag (also an
unclosed one). For "what happened in the first steps after A?":
  python3 tools/walker/bringup/bag_window.py bringup_logs/bag_<...> --t0 <unix s> --before 0.3 --after 0.4 \
      --joints right_ankle_pitch_joint,right_knee_pitch_joint
"""
import argparse
import glob
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--t0", type=float, required=True, help="event time, unix seconds (e.g. the WALK log line)")
ap.add_argument("--before", type=float, default=0.3)
ap.add_argument("--after", type=float, default=0.4)
ap.add_argument("--joints", default="right_ankle_pitch_joint,right_knee_pitch_joint,right_hip_yaw_joint")
a = ap.parse_args()
want = a.joints.split(",")
uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
rows, last_t = [], None
lo, hi = a.t0 - a.before, a.t0 + a.after
while True:
    try:
        if not r.has_next():
            break
        topic, data, t = r.read_next()
    except Exception:  # noqa: BLE001
        break
    ts = t * 1e-9
    last_t = ts
    if not (lo <= ts <= hi) or topic not in ("/joint_states", "/policy/joint_targets", "/policy_bridge/projected_gravity"):
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    rows.append((ts, topic, msg))
print(f"bag data ends at {last_t - a.t0:+.2f} s relative to t0" if last_t else "empty bag")
tgt = {}
for ts, topic, msg in rows:
    if topic == "/policy/joint_targets":
        tgt = dict(zip(msg.name, msg.position))
    elif topic == "/projected_gravity" or topic == "/policy_bridge/projected_gravity":
        continue
    else:
        d = dict(zip(msg.name, zip(msg.position, msg.velocity)))
        cells = []
        for j in want:
            q, v = d.get(j, (float("nan"), float("nan")))
            cells.append(f"{q:+.3f} {v:+6.2f} {tgt.get(j, float('nan')):+.3f}")
        print(f"{ts - a.t0:+.3f}s  " + " | ".join(cells))
print("columns per joint: angle (rad), speed (rad/s), latest policy target (rad);  joints:", ", ".join(want))
