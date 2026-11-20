#!/usr/bin/env python3
"""How fresh is the IMU the policy sees? Compares /imu/data (arrival times at the recorder, header
stamps, how often the value actually changes) with how often the IMU part of /policy/observation
changes. In the robot container:
  python3 tools/walker/bringup/imu_timing.py bringup_logs/bag_<...> [--t0 <unix s> --window 20]
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
ap.add_argument("--t0", type=float, default=None, help="centre of the window (default: whole bag)")
ap.add_argument("--window", type=float, default=20.0)
a = ap.parse_args()
uri, sid = a.bag, ""
if os.path.isdir(a.bag) and not os.path.exists(os.path.join(a.bag, "metadata.yaml")):
    uri, sid = sorted(glob.glob(os.path.join(a.bag, "*.mcap")))[0], "mcap"
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=["/imu/data", "/policy/observation"]))
imu_rx, imu_stamp, imu_val, obs_rx, obs_val = [], [], [], [], []
while True:
    try:
        if not r.has_next():
            break
        topic, data, t = r.read_next()
    except Exception:  # noqa: BLE001
        break
    ts = t * 1e-9
    if a.t0 is not None and abs(ts - a.t0) > a.window / 2:
        continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == "/imu/data":
        imu_rx.append(ts)
        imu_stamp.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        imu_val.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.orientation.x, msg.orientation.y))
    else:
        obs_rx.append(ts)
        obs_val.append(tuple(msg.data[0:6]))


def changes(times, vals):
    times, vals = np.array(times), np.array(vals)
    ch = np.r_[True, np.any(np.abs(np.diff(vals, axis=0)) > 0, axis=1)]
    return np.diff(times[ch])


print(f"window: {'whole bag' if a.t0 is None else f'{a.window:.0f} s around t0'}")
if imu_rx:
    rx = np.diff(imu_rx)
    st = np.diff(imu_stamp)
    print(f"/imu/data: {len(imu_rx)} msgs, {len(imu_rx) / (imu_rx[-1] - imu_rx[0]):.0f} Hz at the recorder; "
          f"arrival gaps p50 {np.median(rx) * 1000:.1f} ms, p99 {np.percentile(rx, 99) * 1000:.1f} ms, max {rx.max() * 1000:.0f} ms")
    print(f"           header stamp gaps p50 {np.median(st) * 1000:.1f} ms, p99 {np.percentile(st, 99) * 1000:.1f} ms, max {st.max() * 1000:.0f} ms")
    lag = np.array(imu_rx) - np.array(imu_stamp)
    print(f"           arrival - header stamp: p50 {np.median(lag) * 1000:+.1f} ms, p99 {np.percentile(lag, 99) * 1000:+.1f} ms (clock offset included)")
    c = changes(imu_rx, imu_val)
    print(f"           value actually changes every p50 {np.median(c) * 1000:.1f} ms, p90 {np.percentile(c, 90) * 1000:.1f} ms")
if obs_rx:
    c = changes(obs_rx, obs_val)
    print(f"observation IMU terms (gyro+gravity) change every p50 {np.median(c) * 1000:.1f} ms, p90 {np.percentile(c, 90) * 1000:.1f} ms, "
          f"max {c.max() * 1000:.0f} ms ({len(obs_rx)} observations)")
