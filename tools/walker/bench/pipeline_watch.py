# Watches the bench pipeline: bridge mode, gamepad button events, joint angles (joint
# space, via the robot yaml calibration) and IMU tilt. Usage: pipeline_watch.py <robot.yaml> <seconds>
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
import yaml

robot = yaml.safe_load(open(sys.argv[1]))
duration = float(sys.argv[2])
rclpy.init()
n = rclpy.create_node("pipeline_watch")
t0 = time.time()
joints = {}
state = {"mode": None, "g": None}


def stamp():
    return "%6.1fs" % (time.time() - t0)


def joint_cb(m, motor):
    joints[motor["joint"]] = float(motor["direction"]) * (m.data - float(motor["offset"]))


for motor in robot["motors"]:
    ns = "/" + motor["joint"].removesuffix("_joint")
    n.create_subscription(Float64, ns + "/motor/actual_position",
                          lambda m, motor=motor: joint_cb(m, motor), 20)


def on_mode(m):
    if m.data != state["mode"]:
        state["mode"] = m.data
        print("%s  MODE -> %s" % (stamp(), m.data), flush=True)


n.create_subscription(String, "/policy_bridge/mode", on_mode,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String, "/gamepad/button_event",
                      lambda m: print("%s  button: %s" % (stamp(), m.data), flush=True), 10)
n.create_subscription(Vector3, "/policy_bridge/projected_gravity",
                      lambda m: state.update(g=(m.x, m.y, m.z)), 10)
next_print = time.time() + 1.0
while time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.02)
    if time.time() >= next_print:
        next_print += 2.0
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -state["g"][2])))) if state["g"] else float("nan")
        js = "  ".join("%s %+.2f" % (k.replace("_joint", "").replace("left_", "L_"), v)
                       for k, v in sorted(joints.items()))
        print("%s  [%s] %s  imu tilt %.0f deg" % (stamp(), state["mode"], js, tilt), flush=True)
