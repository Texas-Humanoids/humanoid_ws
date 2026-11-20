#!/usr/bin/env python3
"""Re-enable every drive that left run mode (CAN timeout, power dip) without restarting the stack:
calls each motor node's motor/clear_faults in ONE ROS process (each `ros2 service call` is a new
DDS participant, and a burst of those stalls the Jetson). Run disarmed, in the robot container:
  python3 tools/walker/bringup/clear_all.py [--all]     # default: only motors not 'running'
"""
import argparse
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from std_srvs.srv import Trigger

MOTORS = ["left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee_pitch", "left_ankle_pitch",
          "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee_pitch", "right_ankle_pitch"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true", help="clear every motor, not only the ones not running")
    a = ap.parse_args()
    rclpy.init()
    n = rclpy.create_node("clear_all")
    status = {}
    for m in MOTORS:
        n.create_subscription(DiagnosticStatus, f"/{m}/motor/status",
                              lambda msg, m=m: status.__setitem__(m, (msg.level, msg.message)), 10)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    todo = [m for m in MOTORS if a.all or status.get(m, (None,))[0] != DiagnosticStatus.OK]
    for m in MOTORS:
        print(f"{m:18s} {status.get(m, ('-', 'NO STATUS (CAN down or node gone?)'))[1]}")
    if not todo:
        print("all running: nothing to clear")
    for m in todo:
        cli = n.create_client(Trigger, f"/{m}/motor/clear_faults")
        if not cli.wait_for_service(timeout_sec=2.0):
            print(f"{m}: no clear_faults service (motor node not running?)")
            continue
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
        res = fut.result()
        print(f"{m}: {'OK ' + res.message if res and res.success else 'FAILED ' + (res.message if res else 'timeout')}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
