#!/usr/bin/env python3
"""Print what the policy sees, by name: bridge mode, projected gravity, gyro, /cmd_vel and the
observation vector split by policy_spec.json terms (joint_pos_rel per joint). Runs in the robot
container: python3 tools/walker/bringup/ros_obs.py --spec <policy_spec.json> [--seconds 3]

Use it for step 6: tilt the robot nose-down (gravity x should go +), left side down (y +), yaw it
left (gyro z +), hold the zero pose (joint_pos_rel = minus the default pose).
"""
import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

SIZES = {"base_ang_vel": 3, "projected_gravity": 3, "velocity_command": 3, "gait_phase": 2}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", required=True)
    ap.add_argument("--seconds", type=float, default=3.0)
    a = ap.parse_args()
    spec = json.load(open(a.spec))
    joints = spec["joint_order"]
    rclpy.init()
    n = rclpy.create_node("ros_obs")
    got = {}
    counts = {}

    def keep(name):
        def cb(msg):
            got[name] = msg
            counts[name] = counts.get(name, 0) + 1
        return cb
    qos_latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, "/policy_bridge/mode", keep("mode"), qos_latched)
    n.create_subscription(Vector3, "/policy_bridge/projected_gravity", keep("gravity"), 10)
    n.create_subscription(Imu, "/imu/data", keep("imu"), rclpy.qos.qos_profile_sensor_data)
    n.create_subscription(Float32MultiArray, "/policy/observation", keep("obs"), 10)
    n.create_subscription(Twist, "/cmd_vel", keep("cmd"), 10)
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    rate = {k: round(v / a.seconds, 1) for k, v in counts.items()}
    print(f"rates (msgs/s over {a.seconds:g} s): {rate}")
    print("bridge mode:", got["mode"].data if "mode" in got else "NO MESSAGE (bridge not running?)")
    if "gravity" in got:
        g = got["gravity"]
        print(f"projected gravity: x {g.x:+.3f}  y {g.y:+.3f}  z {g.z:+.3f}   (upright 0 0 -1; nose down x+; left side down y+)")
    else:
        print("projected gravity: NO MESSAGE (IMU missing?)")
    if "imu" in got:
        w = got["imu"].angular_velocity
        print(f"gyro: x {w.x:+.3f}  y {w.y:+.3f}  z {w.z:+.3f} rad/s   (yaw left/CCW from above -> z+)")
    else:
        print("imu: NO MESSAGE on /imu/data")
    if "cmd" in got:
        c = got["cmd"]
        print(f"cmd_vel: vx {c.linear.x:+.3f}  vy {c.linear.y:+.3f}  wz {c.angular.z:+.3f}   (stick centred: ~0)")
    else:
        print("cmd_vel: NO MESSAGE (gamepad node / controller?)")
    if "obs" in got:
        o = list(got["obs"].data)
        i = 0
        for t in spec["observations"]:
            k = SIZES.get(t["name"], len(joints))
            seg = o[i:i + k]
            i += k
            if k == len(joints):
                print(f"{t['name']}:")
                for j, v in zip(joints, seg):
                    print(f"    {j:26s} {v:+.3f}")
            else:
                print(f"{t['name']}: " + " ".join(f"{v:+.3f}" for v in seg))
        if i != len(o):
            print(f"WARNING: observation has {len(o)} values, spec adds up to {i}")
        dj = dict(zip(joints, spec["default_joint_pos"]))
        print("(zero pose -> joint_pos_rel should read minus the default: "
              + ", ".join(f"{j.replace('_joint', '')} {-v:+.2f}" for j, v in dj.items() if v) + ")")
    else:
        print("observation: NO MESSAGE")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
