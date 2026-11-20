#!/usr/bin/env python3
"""Mean and spread of the policy observation (/policy/observation) in a window, labeled with the
walker_demo spec layout, to compare the robot's view against sim. In the robot container:
  python3 tools/walker/bringup/obs_dump.py bringup_logs/bag_<...> --t0 <unix s> --before 1.0 --after 0.0
"""
import argparse
import glob
import json
import os

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--t0", type=float, required=True)
ap.add_argument("--before", type=float, default=1.0)
ap.add_argument("--after", type=float, default=0.0)
ap.add_argument("--spec", default="policies/walker_demo/cad_v3/policy_spec.json")
a = ap.parse_args()
spec = json.load(open(a.spec))
J = [j.replace("_joint", "").replace("left_", "L_").replace("right_", "R_") for j in spec["joint_order"]]
labels = []
for t in spec["observations"]:
    n = t["name"]
    if n in ("base_ang_vel", "projected_gravity", "velocity_command"):
        labels += [f"{n}[{c}]" for c in "xyz"]
    elif n == "gait_phase":
        labels += ["gait_sin", "gait_cos"]
    else:
        labels += [f"{n}:{j}" for j in J]
uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/policy/observation"]))
O = []
while True:
    try:
        if not r.has_next():
            break
        topic, data, t = r.read_next()
    except Exception:  # noqa: BLE001
        break
    ts = t * 1e-9
    if a.t0 - a.before <= ts <= a.t0 + a.after:
        O.append(deserialize_message(data, get_message(types[topic])).data)
O = np.array(O)
print(json.dumps({"n": len(O), "labels": labels, "mean": O.mean(0).round(4).tolist(), "std": O.std(0).round(4).tolist()}))
