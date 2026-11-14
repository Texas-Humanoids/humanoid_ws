#!/usr/bin/env python3
"""Set up one new RobStride motor for a WalkerV2 joint: find it, check it, give it its ID.

Plug in ONE new motor (motors already set up can stay on the bus), then run on the Jetson,
with no motor nodes running (they share the bus):

  python3 tools/walker/init_motor.py right_hip_roll [--robot-yaml PATH] [--iface can_usb]
      [--no-move] [--yes]

Steps, each checked before the next:
  1. Scan IDs 0-127 and find the one motor that is not yet in the robot yaml.
  2. Health: no fault bits, sane bus voltage and temperature, torque range matches the model.
  3. Change its CAN ID to the joint's ID and confirm it answers there (and not at the old ID).
  4. Small test move: 0.3 rad out and back at 0.5 rad/s with a 3 A cap, then disable.
     Run it before mounting, with the shaft free (or pass --no-move).
  5. Write motor_id and the motor's UUID into the robot yaml.

IDs follow each leg's joint order (hip pitch, hip roll, hip yaw, knee, ankle):
left 1-5, right 11-15. Offsets and directions are not touched: measure them on the
mounted robot with calibrate_offsets.py and wiggle_test.py.
"""

import argparse
import errno
import math
import os
import re
import socket
import struct
import sys
import time

import yaml

HOST = 0xFF
LEG_ORDER = ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch']
LEG_BASE = {'left': 1, 'right': 11}
# Factory torque limit (param 0x700B) equals the model's peak torque.
MODEL_TORQUE = {'RS02': 17.0, 'RS03': 60.0, 'RS06': 36.0}
FAULT_BITS = ['undervoltage', 'overcurrent', 'overtemperature', 'magnetic encoder',
              'HALL encoder', 'not calibrated']
DEFAULT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '../../src/th_bringup/config/walker_v2_robot.yaml')

COMM_GET_ID, COMM_FEEDBACK, COMM_ENABLE, COMM_STOP, COMM_SET_ID = 0, 2, 3, 4, 7
COMM_READ, COMM_WRITE = 0x11, 0x12
P_RUN_MODE, P_TORQUE_LIMIT, P_LOC_REF = 0x7005, 0x700B, 0x7016
P_SPEED_LIMIT, P_CURRENT_LIMIT, P_MECH_POS, P_VBUS = 0x7017, 0x7018, 0x7019, 0x701C
RUN_MODE_CSP = 5

MOVE_DELTA, MOVE_SPEED, MOVE_CURRENT, MOVE_TOL = 0.3, 0.5, 3.0, 0.05


class InitError(Exception):
    pass


def joint_name(arg):
    joint = arg if arg.endswith('_joint') else arg + '_joint'
    side, _, rest = joint.removesuffix('_joint').partition('_')
    if side not in LEG_BASE or rest not in LEG_ORDER:
        raise InitError(f'Unknown joint {arg!r}; expected e.g. right_hip_roll '
                        f'(left/right + {", ".join(LEG_ORDER)})')
    return joint


def scheme_id(joint):
    side, _, rest = joint.removesuffix('_joint').partition('_')
    return LEG_BASE[side] + LEG_ORDER.index(rest)


class SocketBus:
    def __init__(self, iface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))

    def send(self, cid, data=b''):
        frame = struct.pack('=IB3x8s', cid | socket.CAN_EFF_FLAG, len(data),
                            bytes(data).ljust(8, b'\0'))
        for _ in range(50):  # the adapter's queue is short; give it ~0.5 s to drain
            try:
                self.sock.send(frame)
                return
            except OSError as e:
                if e.errno != errno.ENOBUFS:
                    raise
                time.sleep(0.01)
        raise InitError('CAN transmit queue is stuck: nothing on the bus acknowledges frames. '
                        'Check motor power, the ground wire and CAN_H/CAN_L, then reset the '
                        'interface (ip link set can_usb down; ip link set can_usb up type can '
                        'bitrate 1000000).')

    def recv(self, timeout):
        self.sock.settimeout(max(timeout, 0.001))
        try:
            frame = self.sock.recv(16)
        except socket.timeout:
            return None
        cid, dlc, data = struct.unpack('=IB3x8s', frame)
        if not cid & socket.CAN_EFF_FLAG:
            return None
        return cid & socket.CAN_EFF_MASK, data[:dlc]


class Motors:
    """RobStride private protocol: 29-bit ID = comm type (28-24) | data (23-8) | target (7-0)."""

    def __init__(self, bus):
        self.bus = bus

    def _request(self, ctype, mid, data=b'', middle=HOST, match=None, timeout=0.3):
        self.bus.send((ctype << 24) | (middle << 8) | mid, data)
        end = time.monotonic() + timeout
        while (left := end - time.monotonic()) > 0:
            frame = self.bus.recv(left)
            if frame and match(*frame):
                return frame
        return None

    def scan(self):
        """{id: set(uuid hex)}; more than one UUID at an ID means two motors share it."""
        for mid in range(128):
            self.bus.send((COMM_GET_ID << 24) | (HOST << 8) | mid)
            time.sleep(0.005)
        found = {}
        end = time.monotonic() + 0.5
        while (left := end - time.monotonic()) > 0:
            frame = self.bus.recv(left)
            if not frame:
                continue
            cid, data = frame
            if cid >> 24 == COMM_GET_ID and cid & 0xFF == 0xFE and any(data):
                found.setdefault((cid >> 8) & 0xFF, set()).add(data.hex())
        return found

    def feedback(self, mid, ctype=COMM_STOP):
        frame = self._request(ctype, mid, match=lambda c, d: c >> 24 == COMM_FEEDBACK
                              and (c >> 8) & 0xFF == mid)
        if not frame:
            return None
        cid, data = frame
        faults = [n for i, n in enumerate(FAULT_BITS) if (cid >> (16 + i)) & 1]
        return {'mode': (cid >> 22) & 3, 'faults': faults,
                'temp': int.from_bytes(data[6:8], 'big') / 10.0}

    def read(self, mid, index, kind='f'):
        frame = self._request(
            COMM_READ, mid, struct.pack('<H', index),
            match=lambda c, d: c >> 24 == COMM_READ and (c >> 8) & 0xFF == mid
            and len(d) >= 8 and struct.unpack('<H', d[:2])[0] == index)
        if not frame:
            return None
        data = frame[1]
        return data[4] if kind == 'u8' else struct.unpack('<f', data[4:8])[0]

    def write(self, mid, index, value, kind='f'):
        raw = bytes([int(value), 0, 0, 0]) if kind == 'u8' else struct.pack('<f', value)
        self.bus.send((COMM_WRITE << 24) | (HOST << 8) | mid,
                      struct.pack('<HH', index, 0) + raw)
        time.sleep(0.02)

    def set_id(self, old, new):
        self.bus.send((COMM_SET_ID << 24) | (new << 16) | (HOST << 8) | old)
        time.sleep(0.3)


def configured(robot_text):
    """{joint: (motor_id, uuid or None)} for every yaml entry that has a motor_id."""
    robot = yaml.safe_load(robot_text) or {}
    uuids = dict(re.findall(r'- joint: (\S+)\n(?:    .*\n)*?    motor_id: \d+[ \t]*#[ \t]*UUID (\w+)',
                            robot_text))
    return {m['joint']: (int(m['motor_id']), uuids.get(m['joint']))
            for m in robot.get('motors') or [] if m.get('motor_id') is not None}


def model_of(robot_text, joint):
    for m in (yaml.safe_load(robot_text) or {}).get('motors') or []:
        if m['joint'] == joint:
            return m.get('motor_model')
    raise InitError(f'{joint} is not in the robot yaml')


def write_id(robot_text, joint, mid, uuid):
    pattern = rf'(- joint: {re.escape(joint)}\n(?:    .*\n)*?    motor_id: ).*'
    text, n = re.subn(pattern, rf'\g<1>{mid}  # UUID {uuid}', robot_text)
    if n != 1:
        raise InitError(f'Could not find the motor_id line for {joint}; nothing written.')
    return text


def pick_new_motor(found, known, joint, target):
    """Return (current_id, uuid) of the one motor that is not set up yet."""
    shared = {mid: u for mid, u in found.items() if len(u) > 1}
    if shared:
        mid = next(iter(shared))
        raise InitError(f'{len(shared[mid])} motors answer at ID {mid}. Unplug all but one of '
                        'them, set it up, then plug the next one back in.')
    by_id = {mid: (j, u) for j, (mid, u) in known.items()}
    new = []
    for mid, uuids in sorted(found.items()):
        uuid = next(iter(uuids))
        if mid in by_id:
            other, recorded = by_id[mid]
            if recorded is None or recorded == uuid:
                continue
            raise InitError(f'ID {mid} answers with UUID {uuid}, but the yaml gives ID {mid} to '
                            f'{other} (UUID {recorded}). A new motor is sitting on that ID: '
                            f'unplug the {other} motor, run this again, then plug it back in.')
        new.append((mid, uuid))
    if joint in known:
        if new:
            raise InitError(f'{joint} already has motor {known[joint][0]} in the yaml. Set its '
                            'motor_id back to null to give the joint a different motor.')
        if known[joint][0] not in found:
            raise InitError(f'{joint} is set to ID {known[joint][0]} but nothing answers there.')
        return known[joint][0], next(iter(found[known[joint][0]]))
    if not new:
        raise InitError('No new motor answers. Check its power, ground and CAN wiring '
                        '(motors already in the yaml are ignored).')
    if len(new) > 1:
        raise InitError('More than one motor not yet in the yaml answers: '
                        + ', '.join(f'ID {m} (UUID {u})' for m, u in new)
                        + '. Plug in one new motor at a time.')
    mid, uuid = new[0]
    if target in found and target != mid:
        raise InitError(f'ID {target} is already taken by UUID {next(iter(found[target]))}, '
                        'which is not in the yaml. Unplug it or record it first.')
    return mid, uuid


def check_health(motors, mid, model):
    fb = motors.feedback(mid)
    if fb is None:
        raise InitError(f'Motor {mid} did not answer the stop request.')
    vbus = motors.read(mid, P_VBUS)
    torque = motors.read(mid, P_TORQUE_LIMIT)
    if vbus is None or torque is None:
        raise InitError(f'Motor {mid} did not answer parameter reads.')
    print(f'  bus {vbus:.1f} V, {fb["temp"]:.1f} C, torque limit {torque:.0f} Nm, '
          f'faults: {", ".join(fb["faults"]) or "none"}')
    if fb['faults']:
        raise InitError('Motor reports faults; fix them before using it.')
    if not 20.0 <= vbus <= 60.0:
        raise InitError(f'Bus voltage {vbus:.1f} V is outside 20-60 V.')
    if fb['temp'] > 60.0:
        raise InitError(f'Motor is at {fb["temp"]:.0f} C; let it cool down.')
    expected = MODEL_TORQUE.get(model)
    if expected and abs(torque - expected) > 1.0:
        print(f'  WARNING: torque limit {torque:.0f} Nm does not match a {model} '
              f'({expected:.0f} Nm). Is this the right motor for this joint?')


def check_move(motors, mid):
    saved = {i: motors.read(mid, i) for i in (P_SPEED_LIMIT, P_CURRENT_LIMIT)}
    run_mode = motors.read(mid, P_RUN_MODE, 'u8')
    try:
        motors.feedback(mid, COMM_STOP)
        motors.write(mid, P_RUN_MODE, RUN_MODE_CSP, 'u8')
        motors.write(mid, P_SPEED_LIMIT, MOVE_SPEED)
        motors.write(mid, P_CURRENT_LIMIT, MOVE_CURRENT)
        start = motors.read(mid, P_MECH_POS)
        if start is None or not math.isfinite(start):
            raise InitError('Could not read the motor position.')
        motors.write(mid, P_LOC_REF, start)  # hold where it is before enabling
        motors.feedback(mid, COMM_ENABLE)
        results = []
        for target in (start + MOVE_DELTA, start):
            motors.write(mid, P_LOC_REF, target)
            time.sleep(MOVE_DELTA / MOVE_SPEED + 0.9)
            reached = motors.read(mid, P_MECH_POS)
            results.append((target, reached))
    finally:
        motors.feedback(mid, COMM_STOP)
        if run_mode is not None:
            motors.write(mid, P_RUN_MODE, run_mode, 'u8')
        for index, value in saved.items():
            if value is not None:
                motors.write(mid, index, value)
    for target, reached in results:
        err = float('nan') if reached is None else reached - target
        print(f'  target {target:+.3f}  reached {reached if reached is None else f"{reached:+.3f}"}'
              f'  error {err:+.3f} rad')
        if not abs(err) <= MOVE_TOL:
            raise InitError('Motor did not track the test move. Is the shaft blocked?')


def confirm(question, assume_yes):
    if assume_yes:
        return
    if input(f'{question} [y/N] ').strip().lower() != 'y':
        raise InitError('Stopped; nothing more was changed.')


def run(args, motors):
    joint = joint_name(args.joint)
    target = scheme_id(joint)
    text = open(args.robot_yaml).read()
    model = model_of(text, joint)
    known = configured(text)
    print(f'{joint}: {model}, ID {target}')

    print('1. Scanning the bus...')
    found = motors.scan()
    print('  answering: ' + (', '.join(f'{m}' for m in sorted(found)) or 'nothing'))
    mid, uuid = pick_new_motor(found, known, joint, target)
    print(f'  motor at ID {mid}, UUID {uuid}')

    print('2. Health')
    check_health(motors, mid, model)

    if mid != target:
        print(f'3. Changing CAN ID {mid} -> {target}')
        confirm(f'Change motor UUID {uuid} from ID {mid} to {target}?', args.yes)
        motors.set_id(mid, target)
        after = motors.scan()
        if after.get(target) != {uuid} or uuid in after.get(mid, set()):
            raise InitError(f'ID change not confirmed; bus now shows {after}.')
        print(f'  confirmed: UUID {uuid} answers at ID {target}')
    else:
        print(f'3. Already at ID {target}')

    if args.no_move:
        print('4. Test move skipped (--no-move)')
    else:
        print(f'4. Test move: {MOVE_DELTA} rad out and back at {MOVE_SPEED} rad/s, '
              f'{MOVE_CURRENT} A cap')
        confirm('Is the shaft free to turn and are you watching it?', args.yes)
        check_move(motors, target)

    if known.get(joint) == (target, uuid):
        print(f'5. {args.robot_yaml} already lists it')
    else:
        open(args.robot_yaml, 'w').write(write_id(text, joint, target, uuid))
        print(f'5. Wrote motor_id {target} (UUID {uuid}) for {joint} to {args.robot_yaml}')
    print('Done. Plug in the next motor, or commit the yaml.')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('joint', help='e.g. right_hip_roll or right_hip_roll_joint')
    parser.add_argument('--robot-yaml', default=os.path.normpath(DEFAULT_YAML))
    parser.add_argument('--iface', default='can_usb')
    parser.add_argument('--no-move', action='store_true', help='skip the test move')
    parser.add_argument('--yes', action='store_true', help='do not ask before changes')
    args = parser.parse_args()
    try:
        run(args, Motors(SocketBus(args.iface)))
    except (InitError, OSError) as e:
        sys.exit(f'ERROR: {e}')


if __name__ == '__main__':
    main()
