# Hold test: ramp the target `off` rad away over `ramp` s, hold `hold` s, ramp back.
# While a hand holds the shaft, torque = kp * error. Usage: hold_test.py <ns> <kp> <off> [ramp] [hold]
import sys
import time

import rclpy
from std_msgs.msg import Float64

ns, kp, amp = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
ramp = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5
hold = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
rclpy.init()
n = rclpy.create_node("hold_test")
st = {}
n.create_subscription(Float64, ns + "/motor/actual_position", lambda m: st.update(p=m.data), 50)
n.create_subscription(Float64, ns + "/motor/actual_torque", lambda m: st.update(t=m.data), 50)
pub = n.create_publisher(Float64, ns + "/motor/target_position", 10)
end = time.time() + 3.0
while time.time() < end or "p" not in st:
    rclpy.spin_once(n, timeout_sec=0.01)
p0 = st["p"]
rows = []
t0 = time.time()
total = 2 * ramp + hold
while (t := time.time() - t0) < total:
    if t < ramp:
        off = amp * t / ramp
    elif t < ramp + hold:
        off = amp
    else:
        off = amp * (total - t) / ramp
    pub.publish(Float64(data=p0 + off))
    rclpy.spin_once(n, timeout_sec=0.004)
    if ramp + 0.3 < t < ramp + hold - 0.2 and "t" in st:
        rows.append((p0 + off - st["p"], st["t"]))
err = [r[0] for r in rows]
tq = [r[1] for r in rows]
e, q = sum(err) / len(err), sum(tq) / len(tq)
print("hold: error %.3f rad (min %.3f, max %.3f), torque %.2f Nm (max %.2f)" % (
    e, min(err), max(err), q, max(tq, key=abs)))
print("expected torque kp*error = %.2f Nm; kp_measured = %s (commanded %.1f)" % (
    kp * e, "%.2f" % (q / e) if abs(e) > 0.05 else "n/a (shaft not held)", kp))
print("shaft turned %.3f rad of %.3f during the hold" % (amp - e, amp))
end = time.time() + 0.5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.01)
