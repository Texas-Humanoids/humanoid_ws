#!/usr/bin/env python3
"""Measure every motor's offset with the robot held in the policy's default pose.

motor = direction * joint + offset, so offset = motor_position - direction * default.

1. Launch the robot disarmed (motor nodes damp, nothing moves):
     ros2 launch th_bringup robot.launch.py motors_config:=<robot.yaml> policy_spec:=<spec>
   With offsets still null the launch refuses, so for the very first run copy the yaml and
   put 0.0 in every offset.
2. Hold the robot in the default pose (jig or by hand: knees ~23 deg, feet flat, legs straight
   down from the hips, no yaw).
3. python3 tools/walker/calibrate_offsets.py <robot.yaml> <policy_spec.json> [--write]
"""

import argparse
import re
import statistics
import sys
import time

import rclpy
from std_msgs.msg import Float64

from common import load_defaults, load_robot, namespace


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('robot_yaml')
    parser.add_argument('policy_spec')
    parser.add_argument('--seconds', type=float, default=1.0)
    parser.add_argument('--write', action='store_true', help='update offsets in robot_yaml')
    args = parser.parse_args()

    _, motors = load_robot(args.robot_yaml)
    defaults = load_defaults(args.policy_spec)
    rclpy.init()
    node = rclpy.create_node('calibrate_offsets')
    samples = {m['joint']: [] for m in motors}
    for m in motors:
        node.create_subscription(
            Float64, namespace(m['joint']) + '/motor/actual_position',
            lambda msg, j=m['joint']: samples[j].append(msg.data), 10)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    offsets = {}
    ok = True
    print(f'{"joint":28s} {"motor pos":>10s} {"spread":>8s} {"offset":>9s}')
    for m in motors:
        joint, s = m['joint'], samples[m['joint']]
        if not s:
            print(f'{joint:28s} NO FEEDBACK')
            ok = False
            continue
        pos = statistics.fmean(s)
        spread = max(s) - min(s)
        offsets[joint] = pos - float(m['direction']) * defaults[joint]
        flag = '  <- moving? hold still' if spread > 0.01 else ''
        print(f'{joint:28s} {pos:10.4f} {spread:8.4f} {offsets[joint]:9.4f}{flag}')
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        sys.exit('Some motors sent no feedback; nothing written.')
    if args.write:
        text = open(args.robot_yaml).read()
        for joint, offset in offsets.items():
            pattern = rf'(- joint: {re.escape(joint)}\n(?:    .*\n)*?    offset: )\S+'
            text, n = re.subn(pattern, rf'\g<1>{offset:.4f}', text)
            if n != 1:
                sys.exit(f'Could not find the offset line for {joint}; nothing written.')
        open(args.robot_yaml, 'w').write(text)
        print(f'Wrote {len(offsets)} offsets to {args.robot_yaml}. Relaunch to use them.')


if __name__ == '__main__':
    main()
