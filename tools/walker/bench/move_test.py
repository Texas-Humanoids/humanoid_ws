# Slow visible move on one MIT-mode motor node: ramp +amp over ramp s, hold, ramp back.
# Usage: move_test.py <ns> <amp_rad> [ramp_s] [hold_s]
import math
import sys
import time

import rclpy
from std_msgs.msg import Float64

ns, amp = sys.argv[1], float(sys.argv[2])
ramp = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
hold = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
rclpy.init()
n = rclpy.create_node("move_test")
st = {}
n.create_subscription(Float64, ns + "/motor/actual_position", lambda m: st.update(p=m.data), 50)
n.create_subscription(Float64, ns + "/motor/actual_torque", lambda m: st.update(t=m.data), 50)
pub = n.create_publisher(Float64, ns + "/motor/target_position", 10)
end = time.time() + 3.0
while time.time() < end or "p" not in st:
    rclpy.spin_once(n, timeout_sec=0.01)
p0 = st["p"]
print("start pos %.4f rad" % p0, flush=True)
t0 = time.time()
total = 2 * ramp + hold
max_err = 0.0
marks = {}
while (t := time.time() - t0) < total:
    if t < ramp:
        off = amp * 0.5 * (1 - math.cos(math.pi * t / ramp))
    elif t < ramp + hold:
        off = amp
    else:
        off = amp * 0.5 * (1 + math.cos(math.pi * (t - ramp - hold) / ramp))
    pub.publish(Float64(data=p0 + off))
    rclpy.spin_once(n, timeout_sec=0.004)
    max_err = max(max_err, abs(p0 + off - st["p"]))
    if t > ramp + hold / 2 and "mid" not in marks:
        marks["mid"] = (st["p"] - p0, st.get("t", float("nan")))
        print("HOLDING: moved %.3f rad (%.1f deg), torque %.2f Nm" % (
            marks["mid"][0], math.degrees(marks["mid"][0]), marks["mid"][1]), flush=True)
print("back: final offset %.4f rad, max tracking error %.3f rad" % (st["p"] - p0, max_err), flush=True)
end = time.time() + 0.5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.01)
