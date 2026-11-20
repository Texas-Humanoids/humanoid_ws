#!/usr/bin/env python3
"""Print chosen policy-observation terms step by step around an event (/policy/observation).
  python3 tools/walker/bringup/obs_series.py bringup_logs/bag_<...> --t0 <unix s> --before 0.06 --after 0.3
"""
import argparse
import glob
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--t0", type=float, required=True)
ap.add_argument("--before", type=float, default=0.06)
ap.add_argument("--after", type=float, default=0.3)
a = ap.parse_args()
uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/policy/observation"]))
# walker_demo layout: ang vel 0-2, gravity 3-5, joint_pos_rel 6-15, joint_vel 16-25, last_action 26-35, cmd 36-38, clock 39-40
print("   t      gyro x  gyro y |  grav x  grav y | qR_hipP qR_knee qR_ankle | vR_hipP vR_knee vL_knee | aR_hipP aR_knee | clock sin,cos")
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
    o = deserialize_message(data, get_message(types[topic])).data
    print(f"{ts - a.t0:+.3f}  {o[0]:+6.3f} {o[1]:+6.3f} | {o[3]:+6.3f} {o[4]:+6.3f} | {o[11]:+6.3f} {o[14]:+6.3f} {o[15]:+6.3f} | "
          f"{o[21]:+6.2f} {o[24]:+6.2f} {o[19]:+6.2f} | {o[31]:+6.2f} {o[34]:+6.2f} | {o[39]:+.2f},{o[40]:+.2f}")
