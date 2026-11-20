#!/usr/bin/env python3
"""Record bridge mode, joint deflection from the default pose and motor torque for N seconds,
then print per joint: max deflection, max |torque| and the apparent stiffness (torque / deflection).
Runs in the robot container, read-only:
  python3 tools/walker/bringup/ros_watch.py --spec <policy_spec.json> --seconds 30
"""
import argparse
import json
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    joints = spec["joint_order"]
    default = dict(zip(joints, spec["default_joint_pos"]))
    kp = dict(zip(joints, spec["kp"]))
    rclpy.init()
    n = rclpy.create_node("ros_watch")
    dev = {j: 0.0 for j in joints}
    tau = {j: 0.0 for j in joints}
    modes = []
    pos_now = {}
    stand_since = [None]            # ignore the STAND ramp (2 s) + 0.5 s settle

    def holding():
        if not modes:
            return False
        if modes[-1] == "WALK":
            return True
        return modes[-1] == "STAND" and (
            stand_since[0] is None or time.monotonic() - stand_since[0] > 2.5)

    def on_js(msg):
        for name, q in zip(msg.name, msg.position):
            pos_now[name] = q
            if holding():
                dev[name] = max(dev[name], abs(q - default[name]))

    def on_tau(j):
        def cb(msg):
            if holding():
                tau[j] = max(tau[j], abs(msg.data))
        return cb

    def on_mode(msg):
        if not modes or modes[-1] != msg.data:
            modes.append(msg.data)
            # Already standing when we start (latched mode): no ramp to skip.
            stand_since[0] = time.monotonic() if (msg.data == "STAND" and len(modes) > 1) else None
            print(f"{time.strftime('%H:%M:%S')} mode -> {msg.data}", flush=True)

    latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, "/policy_bridge/mode", on_mode, latched)
    n.create_subscription(JointState, "/joint_states", on_js, 10)
    for j in joints:
        n.create_subscription(Float64, "/" + j.removesuffix("_joint") + "/motor/actual_torque", on_tau(j), 10)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.01)
    print(f"modes seen: {' -> '.join(modes) or 'none'}")
    print(f"{'joint':26s} {'now-default':>11s} {'max defl':>9s} {'max |tau|':>9s} {'tau/defl':>9s} {'kp':>5s}   (STAND after the ramp, and WALK)")
    for j in joints:
        now = pos_now.get(j)
        k = tau[j] / dev[j] if dev[j] > 0.01 else float("nan")
        print(f"{j:26s} {(now - default[j]) if now is not None else float('nan'):+11.3f} {dev[j]:9.3f} {tau[j]:9.2f} {k:9.0f} {kp[j]:5.0f}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
