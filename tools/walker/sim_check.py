#!/usr/bin/env python3
"""Drive the full deploy stack against th_sim and check that it runs.

Start the stack first (see the WalkerV2 sim section of the th_bringup README):

    ros2 launch th_bringup robot.launch.py sim:=true \\
      motors_config:=src/th_bringup/config/walker_v2_robot.yaml \\
      policy_spec:=policies/walker_v2/v6/policy_spec.json \\
      model_path:=policies/walker_v2/v6/policy.onnx walk_event:='A pressed'

then, in a second shell:

    python3 tools/walker/sim_check.py policies/walker_v2/v6/policy_spec.json [--vx 0.3]

It presses START (stand ramp), then A (walk), optionally commands a forward
velocity, and reports the observation/target/IMU/motor rates, the bridge mode,
and the simulated base height. Exits non-zero if a check fails.
"""

import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Float64, String

from common import namespace


class Probe:

    def __init__(self, node, msg_type, topic, qos=10):
        self.stamps = []
        self.last = None
        node.create_subscription(msg_type, topic, self.on_msg, qos)

    def on_msg(self, msg):
        self.stamps.append(time.monotonic())
        self.last = msg

    def rate(self, since):
        stamps = [t for t in self.stamps if t >= since]
        if len(stamps) < 2:
            return 0.0
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('policy_spec')
    parser.add_argument('--vx', type=float, default=0.0, help='forward command while walking')
    parser.add_argument('--stand', type=float, default=3.0, help='seconds in STAND')
    parser.add_argument('--walk', type=float, default=10.0, help='seconds in WALK')
    parser.add_argument('--min-height', type=float, default=0.45,
                        help='base height (m) below which the robot counts as fallen')
    args = parser.parse_args()
    with open(args.policy_spec) as f:
        spec = json.load(f)
    joints = spec['joint_order']
    rate = 1.0 / spec['policy_dt']

    rclpy.init()
    node = rclpy.create_node('sim_check')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    probes = {
        'observation': Probe(node, Float32MultiArray, '/policy/observation'),
        'joint_targets': Probe(node, JointState, '/policy/joint_targets'),
        'imu': Probe(node, Imu, '/imu/data', qos_profile_sensor_data),
        'motor_command': Probe(node, Float64, namespace(joints[0]) + '/motor/target_position'),
        'motor_feedback': Probe(node, Float64, namespace(joints[0]) + '/motor/actual_position'),
    }
    mode = Probe(node, String, '/policy_bridge/mode', latched)
    pose = Probe(node, PoseStamped, '/mujoco_sim/base_pose')
    buttons = node.create_publisher(String, '/gamepad/button_event', 10)
    cmd_vel = node.create_publisher(Twist, '/cmd_vel', 10)
    failures = []
    heights = []

    def spin_for(seconds, vx=0.0):
        end = time.monotonic() + seconds
        next_cmd = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if vx and time.monotonic() >= next_cmd:
                twist = Twist()
                twist.linear.x = vx
                cmd_vel.publish(twist)
                next_cmd = time.monotonic() + 0.05
            if pose.last is not None:
                heights.append(pose.last.pose.position.z)

    def press(button):
        buttons.publish(String(data=button + ' pressed'))
        spin_for(0.3)

    def current_mode():
        return mode.last.data if mode.last else None

    def report(phase, since):
        line = ', '.join(f'{k} {p.rate(since):5.1f} Hz' for k, p in probes.items())
        z = pose.last.pose.position.z if pose.last else float('nan')
        print(f'[{phase:8s}] mode {current_mode()}, base z {z:.3f} m | {line}')

    print('Waiting for the stack (observations and sim)...')
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline and (
            probes['observation'].last is None or pose.last is None or mode.last is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if probes['observation'].last is None or pose.last is None:
        print('FAIL: no /policy/observation or /mujoco_sim/base_pose after 30 s')
        return 1
    size = len(probes['observation'].last.data)
    expected = spec.get('observation_size')
    print(f'Observation size {size}' + (f' (spec {expected})' if expected else ''))
    if expected and size != expected:
        failures.append(f'observation size {size} != {expected}')

    t = time.monotonic()
    spin_for(2.0)
    report('DISARMED', t)

    press('START')
    t = time.monotonic()
    spin_for(args.stand)
    report('STAND', t)
    if current_mode() != 'STAND':
        failures.append(f'START did not reach STAND (mode {current_mode()})')

    press('A')
    t = time.monotonic()
    heights.clear()
    spin_for(args.walk, args.vx)
    report('WALK', t)
    if current_mode() != 'WALK':
        failures.append(f'A did not reach WALK, or it disarmed (mode {current_mode()})')
    for key in ('observation', 'joint_targets', 'motor_command'):
        measured = probes[key].rate(t)
        if abs(measured - rate) > 0.1 * rate:
            failures.append(f'{key} at {measured:.1f} Hz, expected {rate:.0f}')
    if heights and min(heights) < args.min_height:
        failures.append(f'base fell to {min(heights):.3f} m while walking')
    if pose.last is not None:
        p = pose.last.pose.position
        print(f'Base moved to x {p.x:+.2f} y {p.y:+.2f} m; min height while walking '
              f'{min(heights) if heights else float("nan"):.3f} m')

    press('BACK')
    node.destroy_node()
    rclpy.shutdown()
    for failure in failures:
        print('FAIL:', failure)
    print('PASS' if not failures else f'{len(failures)} check(s) failed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
