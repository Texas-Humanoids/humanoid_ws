#!/usr/bin/env python3
"""Step one joint at a time by +delta and back, to check directions and log step responses.

Run with the robot HANGING (feet off the ground), launched with control_mode: mit and the
bridge DISARMED (do not press START). Every motor holds where it is while one joint steps.
For each joint it says which way the foot must move; if it moves the other way, flip that
joint's direction in the robot yaml (and re-run calibrate_offsets.py).

  python3 tools/walker/wiggle_test.py <robot.yaml> [--joint left_knee_pitch_joint]
      [--delta 0.1] [--csv steps.csv]

The CSV (time, joint, target, position, velocity) is the step response for armature tuning:
compare it against the same step in sim.
"""

import argparse
import csv
import sys
import time

import rclpy
from std_msgs.msg import Float64, String

from common import load_robot, namespace

EXPECT = {
    'hip_pitch': 'foot swings BACKWARD (leg moves behind the body)',
    'knee_pitch': 'knee FLEXES, foot moves backward',
    'ankle_pitch': 'toes go DOWN',
    'hip_roll': 'foot moves toward +y (LEFT, for both legs)',
    'hip_yaw': 'toes rotate toward +y (LEFT, counter-clockwise seen from above)',
}

RATE = 50.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('robot_yaml')
    parser.add_argument('--joint', help='only this joint')
    parser.add_argument('--delta', type=float, default=0.1, help='joint-space step, rad')
    parser.add_argument('--hold', type=float, default=1.5, help='seconds per step')
    parser.add_argument('--csv')
    args = parser.parse_args()
    if not 0.0 < abs(args.delta) <= 0.3:
        sys.exit('--delta must be within 0.3 rad')

    robot, motors = load_robot(args.robot_yaml)
    if robot.get('control_mode') != 'mit':
        print('Warning: not in MIT mode; CSP motors move at their 1 rad/s limit.')
    rclpy.init()
    node = rclpy.create_node('wiggle_test')
    state = {m['joint']: None for m in motors}
    mode = {'value': None}
    pubs = {}
    for m in motors:
        ns = namespace(m['joint'])
        pubs[m['joint']] = node.create_publisher(Float64, ns + '/motor/target_position', 10)
        node.create_subscription(
            Float64, ns + '/motor/actual_position',
            lambda msg, j=m['joint']: state.__setitem__(j, (msg.data, state[j][1] if state[j] else 0.0)), 10)
        node.create_subscription(
            Float64, ns + '/motor/actual_velocity',
            lambda msg, j=m['joint']: state.__setitem__(j, (state[j][0], msg.data)) if state[j] else None, 10)
    from rclpy.qos import QoSProfile, DurabilityPolicy
    node.create_subscription(
        String, '/policy_bridge/mode', lambda msg: mode.__setitem__('value', msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)

    spin(1.0)
    missing = [j for j, s in state.items() if s is None]
    if missing:
        sys.exit(f'No feedback from {missing}; are the motor nodes running?')
    if mode['value'] not in (None, 'DISARMED'):
        sys.exit(f'policy_bridge is {mode["value"]}; press BACK first.')

    hold = {j: s[0] for j, s in state.items()}  # motor space
    rows = []
    t0 = time.monotonic()
    by_joint = {m['joint']: m for m in motors}
    joints = [args.joint] if args.joint else list(by_joint)
    for joint in joints:
        m = by_joint[joint]
        kind = joint.split('_', 1)[1].removesuffix('_joint')
        input(f'\n{joint}: +{args.delta} rad -> {EXPECT[kind]}. Enter to step (Ctrl-C quits) ')
        step = float(m['direction']) * args.delta  # joint delta -> motor delta
        for target in (hold[joint] + step, hold[joint]):
            end = time.monotonic() + args.hold
            while time.monotonic() < end:
                for j, pub in pubs.items():
                    pub.publish(Float64(data=target if j == joint else hold[j]))
                spin(1.0 / RATE)
                pos, vel = state[joint]
                rows.append((time.monotonic() - t0, joint, target, pos, vel))
        answer = input('Did it move as described? [y/n] ').strip().lower()
        if answer != 'y':
            print(f'  -> flip direction for {joint} in {args.robot_yaml}')
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'joint', 'target_motor', 'position_motor', 'velocity_motor'])
            w.writerows(rows)
        print(f'Wrote {len(rows)} samples to {args.csv}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
