#!/usr/bin/env python3
"""Export STAND-phase samples (time, projected gravity, joint angles) from a bag to CSV, ~5 Hz, for
offline kinematics on the PC (foot flatness, stance). Works on unclosed bags. In the robot container:
  python3 tools/walker/bringup/bag_export.py bringup_logs/bag_<...> out.csv
"""
import glob
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
uri, sid = bag, ""
if os.path.isdir(bag) and not os.path.exists(os.path.join(bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
mode, grav, last_t, names = "DISARMED", None, 0.0, None
with open(out, "w") as f:
    while True:
        try:
            if not r.has_next():
                break
            topic, data, t = r.read_next()
        except Exception:  # noqa: BLE001
            break
        ts = t * 1e-9
        if topic == "/policy_bridge/mode":
            mode = deserialize_message(data, get_message(types[topic])).data
        elif topic == "/policy_bridge/projected_gravity":
            m = deserialize_message(data, get_message(types[topic]))
            grav = (m.x, m.y, m.z)
        elif topic == "/joint_states" and mode == "STAND" and grav and ts - last_t >= 0.2:
            m = deserialize_message(data, get_message(types[topic]))
            if names is None:
                names = list(m.name)
                f.write("t,gx,gy,gz," + ",".join(names) + "\n")
            f.write(f"{ts:.3f},{grav[0]:.5f},{grav[1]:.5f},{grav[2]:.5f}," + ",".join(f"{q:.5f}" for q in m.position) + "\n")
            last_t = ts
print("wrote", out)
