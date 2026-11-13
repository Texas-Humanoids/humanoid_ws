# Watches a policy on the bench: mode, command, and for each motored joint the target and
# actual range, tracking error and swing period over each window.
# Usage: policy_watch.py <robot.yaml> <seconds>
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
import yaml

robot = yaml.safe_load(open(sys.argv[1]))
duration = float(sys.argv[2])
motors = {m["joint"]: m for m in robot["motors"]}
rclpy.init()
n = rclpy.create_node("policy_watch")
t0 = time.time()
st = {"mode": None, "cmd": (0.0, 0.0, 0.0), "cmd_t": 0.0}
actual = {j: None for j in motors}
win = {j: [] for j in motors}  # (t, target, actual)
target_times = []
rev = {j: [0, None] for j in motors}  # direction reversals in the window, last sign


def on_vel(m, j):
    v = float(motors[j]["direction"]) * m.data
    if abs(v) > 0.05:
        sign = v > 0
        if rev[j][1] is not None and sign != rev[j][1]:
            rev[j][0] += 1
        rev[j][1] = sign


def stamp():
    return "%6.1fs" % (time.time() - t0)


def on_mode(m):
    if m.data != st["mode"]:
        st["mode"] = m.data
        print("%s  MODE -> %s" % (stamp(), m.data), flush=True)


def on_targets(m):
    now = time.time()
    target_times.append(now)
    for name, pos in zip(m.name, m.position):
        if name in motors and actual[name] is not None:
            win[name].append((now, pos, actual[name]))


def on_pos(m, j):
    mm = motors[j]
    actual[j] = float(mm["direction"]) * (m.data - float(mm["offset"]))


for j in motors:
    n.create_subscription(Float64, "/" + j.removesuffix("_joint") + "/motor/actual_position",
                          lambda m, j=j: on_pos(m, j), 20)
for j in motors:
    n.create_subscription(Float64, "/" + j.removesuffix("_joint") + "/motor/actual_velocity",
                          lambda m, j=j: on_vel(m, j), 50)
n.create_subscription(String, "/policy_bridge/mode", on_mode,
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(JointState, "/policy/joint_targets", on_targets, 20)
n.create_subscription(Twist, "/cmd_vel", lambda m: st.update(
    cmd=(m.linear.x, m.linear.y, m.angular.z), cmd_t=time.time()), 10)
n.create_subscription(String, "/gamepad/button_event",
                      lambda m: print("%s  button: %s" % (stamp(), m.data), flush=True), 10)


def period(samples):
    # mean time between upward crossings of the window mean
    if len(samples) < 20:
        return float("nan")
    mean = sum(s[1] for s in samples) / len(samples)
    ups = [samples[i][0] for i in range(1, len(samples))
           if samples[i - 1][1] < mean <= samples[i][1]]
    return (ups[-1] - ups[0]) / (len(ups) - 1) if len(ups) >= 3 else float("nan")


next_print = time.time() + 2.0
while time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.005)
    if time.time() >= next_print:
        next_print += 2.0
        cmd = st["cmd"] if time.time() - st["cmd_t"] < 0.5 else (0.0, 0.0, 0.0)
        rate = len([t for t in target_times if t > time.time() - 2.0]) / 2.0
        parts = []
        for j, s in win.items():
            if not s:
                continue
            tg = [x[1] for x in s]
            ac = [x[2] for x in s]
            rms = math.sqrt(sum((a - b) ** 2 for _, b, a in s) / len(s))
            parts.append("%s tgt %+.2f..%+.2f act %+.2f..%+.2f err %.3f T %.2fs rev/s %.1f" % (
                j.replace("left_", "L_").replace("_joint", ""), min(tg), max(tg), min(ac), max(ac),
                rms, period(s), rev[j][0] / 2.0))
            rev[j][0] = 0
            win[j] = []
        print("%s  [%s] cmd vx %+.2f wz %+.2f | policy %.0f Hz | %s" % (
            stamp(), st["mode"], cmd[0], cmd[2], rate, " | ".join(parts)), flush=True)
