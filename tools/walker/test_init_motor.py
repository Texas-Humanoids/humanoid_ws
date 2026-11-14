"""Tests for init_motor.py against a simulated RobStride bus. Run: python3 -m pytest tools/walker"""

import os
import re
import shutil
import struct
import sys
import tempfile
import unittest
from collections import deque
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))
import init_motor as im  # noqa: E402

ROBOT_YAML = os.path.join(os.path.dirname(__file__),
                          '../../src/th_bringup/config/walker_v2_robot.yaml')


class FakeMotor:
    def __init__(self, uuid, torque=60.0, faults=0, stuck=False):
        self.uuid = bytes.fromhex(uuid)
        self.params = {im.P_RUN_MODE: 0, im.P_TORQUE_LIMIT: torque, im.P_SPEED_LIMIT: 10.0,
                       im.P_CURRENT_LIMIT: 43.0, im.P_MECH_POS: -2.0, im.P_VBUS: 48.9,
                       im.P_LOC_REF: -2.0}
        self.faults, self.stuck, self.enabled = faults, stuck, False


class FakeBus:
    """Motors keyed by CAN ID; several motors may share one ID, as on a real bus."""

    def __init__(self, motors):
        self.motors = motors  # list of [id, FakeMotor]
        self.rx = deque()

    def at(self, mid):
        return [m for i, m in self.motors if i == mid]

    def send(self, cid, data=b''):
        ctype, middle, mid = cid >> 24, (cid >> 8) & 0xFFFF, cid & 0xFF
        data = bytes(data).ljust(8, b'\0')
        for m in self.at(mid):
            if ctype == im.COMM_GET_ID:
                self.rx.append(((mid << 8) | 0xFE, m.uuid))
            elif ctype == im.COMM_SET_ID:
                new = middle >> 8
                for entry in self.motors:
                    if entry[1] is m:
                        entry[0] = new
                self.rx.append(((new << 8) | 0xFE, m.uuid))
            elif ctype in (im.COMM_STOP, im.COMM_ENABLE):
                m.enabled = ctype == im.COMM_ENABLE
                self.rx.append(((im.COMM_FEEDBACK << 24) | (m.faults << 16) | (mid << 8) | 0xFF,
                                bytes([0, 0, 0, 0, 0, 0, 0, 240])))
            elif ctype == im.COMM_READ:
                index = struct.unpack('<H', data[:2])[0]
                value = m.params[index]
                raw = bytes([value, 0, 0, 0]) if index == im.P_RUN_MODE else struct.pack('<f', value)
                self.rx.append(((im.COMM_READ << 24) | (mid << 8) | 0xFF, data[:4] + raw))
            elif ctype == im.COMM_WRITE:
                index = struct.unpack('<H', data[:2])[0]
                value = data[4] if index == im.P_RUN_MODE else struct.unpack('<f', data[4:8])[0]
                m.params[index] = value
                if index == im.P_LOC_REF and m.enabled and not m.stuck:
                    m.params[im.P_MECH_POS] = value

    def recv(self, timeout):
        return self.rx.popleft() if self.rx else None


class InitMotorTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.yaml = os.path.join(self.dir, 'robot.yaml')
        text = re.sub(r'motor_id: .*', 'motor_id: null', open(ROBOT_YAML).read())
        open(self.yaml, 'w').write(text)
        patcher = mock.patch.object(im.time, 'sleep')
        patcher.start()
        self.addCleanup(patcher.stop)
        self.addCleanup(shutil.rmtree, self.dir)

    def run_tool(self, joint, motors, **flags):
        args = SimpleNamespace(joint=joint, robot_yaml=self.yaml, no_move=False, yes=True)
        vars(args).update(flags)
        bus = FakeBus(motors)
        im.run(args, im.Motors(bus))
        return bus

    def ids(self):
        return im.configured(open(self.yaml).read())

    def test_scheme(self):
        self.assertEqual(im.scheme_id('left_hip_pitch_joint'), 11)
        self.assertEqual(im.scheme_id('left_ankle_pitch_joint'), 15)
        self.assertEqual(im.scheme_id(im.joint_name('right_hip_pitch')), 21)
        self.assertEqual(im.scheme_id(im.joint_name('right_knee_pitch')), 24)
        with self.assertRaises(im.InitError):
            im.joint_name('right_elbow')

    def test_new_motor_gets_id_moves_and_is_recorded(self):
        m = FakeMotor('3f39300290613713')
        bus = self.run_tool('right_hip_pitch', [[12, m]])
        self.assertEqual(bus.motors[0][0], 21)
        self.assertEqual(self.ids()['right_hip_pitch_joint'], (21, '3f39300290613713'))
        self.assertFalse(m.enabled)
        self.assertEqual(m.params[im.P_SPEED_LIMIT], 10.0)  # limits restored after the move
        self.assertEqual(m.params[im.P_RUN_MODE], 0)
        self.assertEqual(len(self.ids()), 1)

    def test_set_up_motors_stay_on_the_bus(self):
        self.run_tool('right_hip_pitch', [[12, FakeMotor('aa' * 8)]])
        bus = self.run_tool('right_hip_roll', [[21, FakeMotor('aa' * 8)],
                                               [127, FakeMotor('bb' * 8)]])
        self.assertEqual(sorted(i for i, _ in bus.motors), [21, 22])
        self.assertEqual(self.ids()['right_hip_roll_joint'], (22, 'bb' * 8))

    def test_two_motors_on_one_id_is_refused(self):
        with self.assertRaisesRegex(im.InitError, '2 motors answer at ID 127'):
            self.run_tool('right_hip_roll', [[127, FakeMotor('aa' * 8)],
                                             [127, FakeMotor('bb' * 8)]])
        self.assertEqual(self.ids(), {})

    def test_new_motor_on_an_assigned_id_is_refused(self):
        self.run_tool('right_hip_roll', [[12, FakeMotor('aa' * 8)]])
        with self.assertRaisesRegex(im.InitError, 'unplug the right_hip_roll_joint motor'):
            self.run_tool('right_knee_pitch', [[22, FakeMotor('bb' * 8)]])

    def test_two_new_motors_are_refused(self):
        with self.assertRaisesRegex(im.InitError, 'one new motor at a time'):
            self.run_tool('right_knee_pitch', [[127, FakeMotor('aa' * 8)],
                                               [100, FakeMotor('bb' * 8)]])

    def test_faults_stop_before_any_change(self):
        m = FakeMotor('aa' * 8, faults=0b000100)
        with self.assertRaisesRegex(im.InitError, 'faults'):
            self.run_tool('right_hip_pitch', [[127, m]])
        self.assertEqual(self.ids(), {})

    def test_blocked_shaft_fails_the_move_and_disables(self):
        m = FakeMotor('aa' * 8, stuck=True)
        with self.assertRaisesRegex(im.InitError, 'did not track'):
            self.run_tool('right_hip_pitch', [[127, m]])
        self.assertFalse(m.enabled)
        self.assertEqual(m.params[im.P_CURRENT_LIMIT], 43.0)
        self.assertEqual(self.ids(), {})

    def test_rerun_is_a_no_op(self):
        self.run_tool('right_hip_pitch', [[12, FakeMotor('aa' * 8)]])
        before = open(self.yaml).read()
        self.run_tool('right_hip_pitch', [[21, FakeMotor('aa' * 8)]], no_move=True)
        self.assertEqual(open(self.yaml).read(), before)

    def test_no_move_leaves_motor_untouched(self):
        m = FakeMotor('aa' * 8)
        self.run_tool('left_knee_pitch', [[127, m]], no_move=True)
        self.assertEqual(m.params[im.P_RUN_MODE], 0)
        self.assertEqual(self.ids()['left_knee_pitch_joint'], (14, 'aa' * 8))

    def test_declining_the_id_change_changes_nothing(self):
        m = FakeMotor('aa' * 8)
        with mock.patch('builtins.input', return_value='n'):
            with self.assertRaises(im.InitError):
                self.run_tool('right_hip_pitch', [[127, m]], yes=False)
        self.assertEqual(self.ids(), {})


if __name__ == '__main__':
    unittest.main()
