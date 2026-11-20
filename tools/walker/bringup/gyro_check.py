#!/usr/bin/env python3
"""Is the gyro in the policy observation consistent with the gravity vector in the same observation?
For a body frame x fwd / y left / z up (what training uses), with gravity g = R^T (0,0,-1):
    d(g)/dt = -omega x g   ->   dgx/dt ~ +omega_y * (-gz),  dgy/dt ~ -omega_x * (-gz)   (gz ~ -1)
so over any motion, corr(dgx/dt, omega_y) and corr(dgy/dt, -omega_x) must be strongly POSITIVE.
A negative correlation means that gyro axis has the wrong sign (e.g. a VectorNav FRD rate passed
through as FLU). Reads /policy/observation (first 3 = base_ang_vel, next 3 = projected_gravity,
per the walker_demo spec). In the robot container:
  python3 tools/walker/bringup/gyro_check.py bringup_logs/bag_<...>
"""
import glob
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = sys.argv[1]
uri, sid = bag, ""
if os.path.isdir(bag) and not os.path.exists(os.path.join(bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/policy/observation"]))
T, W, G = [], [], []
while True:
    try:
        if not r.has_next():
            break
        topic, data, t = r.read_next()
    except Exception:  # noqa: BLE001
        break
    o = deserialize_message(data, get_message(types[topic])).data
    T.append(t * 1e-9)
    W.append(o[0:3])
    G.append(o[3:6])
T, W, G = np.array(T), np.array(W), np.array(G)
print(f"{len(T)} observations over {T[-1] - T[0]:.0f} s")
dt = np.gradient(T)
dG = np.gradient(G, axis=0) / dt[:, None]
moving = np.linalg.norm(W[:, :2], axis=1) > 0.05   # only samples with real rotation
print(f"{moving.sum()} samples with |omega_xy| > 0.05 rad/s")
for name, a, b in (("pitch: d(gx)/dt vs +omega_y", dG[:, 0], W[:, 1] * -G[:, 2]),
                   ("roll:  d(gy)/dt vs -omega_x", dG[:, 1], -W[:, 0] * -G[:, 2])):
    c = np.corrcoef(a[moving], b[moving])[0, 1]
    k = np.dot(a[moving], b[moving]) / max(np.dot(b[moving], b[moving]), 1e-9)
    print(f"{name}: correlation {c:+.2f}, slope {k:+.2f}  -> {'OK' if c > 0.5 else 'SIGN WRONG' if c < -0.5 else 'unclear'}")
print("Expect correlation ~ +1 and slope ~ +1 for a consistent gyro.")
