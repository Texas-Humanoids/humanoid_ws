#!/usr/bin/env python3
"""Export joint states + policy targets + gravity in a window around an event to CSV (all joints),
for replaying the real targets in sim. In the robot container:
  python3 tools/walker/bringup/bag_traj.py bringup_logs/bag_<...> --t0 <unix s> --before 0.5 --after 0.5 out.csv
"""
import argparse
import glob
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("out")
ap.add_argument("--t0", type=float, required=True)
ap.add_argument("--before", type=float, default=0.5)
ap.add_argument("--after", type=float, default=0.5)
a = ap.parse_args()
uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/joint_states", "/policy/joint_targets", "/policy_bridge/projected_gravity"]))
names, tgt, grav = None, None, (0.0, 0.0, -1.0)
with open(a.out, "w") as f:
    while True:
        try:
            if not r.has_next():
                break
            topic, data, t = r.read_next()
        except Exception:  # noqa: BLE001
            break
        ts = t * 1e-9
        if not (a.t0 - a.before <= ts <= a.t0 + a.after):
            continue
        m = deserialize_message(data, get_message(types[topic]))
        if topic == "/policy/joint_targets":
            tgt = dict(zip(m.name, m.position))
        elif topic == "/policy_bridge/projected_gravity":
            grav = (m.x, m.y, m.z)
        else:
            if names is None:
                names = list(m.name)
                f.write("t,gx,gy,gz," + ",".join(f"q_{n}" for n in names) + "," + ",".join(f"v_{n}" for n in names) + ","
                        + ",".join(f"tgt_{n}" for n in names) + "\n")
            tg = [tgt.get(n, float("nan")) if tgt else float("nan") for n in names]
            f.write(f"{ts - a.t0:.4f},{grav[0]:.5f},{grav[1]:.5f},{grav[2]:.5f}," + ",".join(f"{x:.5f}" for x in m.position) + ","
                    + ",".join(f"{x:.4f}" for x in m.velocity) + "," + ",".join(f"{x:.5f}" for x in tg) + "\n")
print("wrote", a.out)
