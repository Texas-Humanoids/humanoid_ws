"""Tests for the sysid tools against a simulated RobStride motor. Run: python3 -m pytest tools/walker

The protocol vectors are the ones in src/robstride_motor/test/test_robstride_motor.cpp, so this
Python port and the C++ driver can't drift apart silently. The fit tests need numpy + scipy and
are skipped without them (the Jetson only needs chirp.py).
"""
import json
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import chirp  # noqa: E402
import rsproto as rs  # noqa: E402

try:
    import numpy  # noqa: F401
    import scipy  # noqa: F401
    import fit
    HAVE_FIT = True
except ImportError:
    HAVE_FIT = False

TRUTH = 'J=0.02,b=0.01,Fc=0.15,delay=0.003'


class Protocol(unittest.TestCase):
    def test_motion_encodes_id_and_big_endian_fields(self):
        cid, data = rs.motion(7, 0.0, 0.0, 5000.0, 0.0, 0.0, rs.RANGES['RS03'])
        self.assertEqual(cid & 0xFF, 7)
        self.assertEqual(cid >> 24 & 0x1F, rs.T_MOTION)
        self.assertEqual(cid >> 8 & 0xFFFF, 32768)  # zero torque is mid-range
        self.assertEqual(data[0:2], b'\x80\x00')
        self.assertEqual(data[4:6], b'\xff\xff')
        self.assertEqual(data[6:8], b'\x00\x00')

    def test_rs02_gains_scale_with_its_smaller_range(self):
        _, data = rs.motion(1, 0.0, 0.0, 250.0, 2.5, 0.0, rs.RANGES['RS02'])
        self.assertEqual(struct.unpack('>H', data[4:6])[0], 32768)
        self.assertEqual(struct.unpack('>H', data[6:8])[0], 32768)

    def test_out_of_range_saturates(self):
        _, data = rs.motion(1, 100.0, -100.0, -5.0, 1e6, 0.0, rs.RANGES['RS06'])
        self.assertEqual(struct.unpack('>4H', data), (65535, 0, 0, 65535))

    def test_param_frames_match_captured_bytes(self):
        _, data = rs.write_f32(1, 0x7017, 1.0)
        self.assertEqual(data, bytes([0x17, 0x70, 0, 0, 0, 0, 0x80, 0x3F]))
        _, data = rs.write_f32(1, 0x7018, 3.0)
        self.assertEqual(data, bytes([0x18, 0x70, 0, 0, 0, 0, 0x40, 0x40]))
        cid, data = rs.param_reply(1, rs.P_MECH_POS, 0.0)
        _, idx, val = rs.decode_param_reply(cid, bytes([0x19, 0x70, 0, 0, 0xA5, 0xEE, 0x85, 0x3E]))
        self.assertEqual(idx, rs.P_MECH_POS)
        self.assertAlmostEqual(val, 0.2616, places=4)

    def test_status_round_trip(self):
        r = rs.RANGES['RS03']
        st = rs.Status(rs.MODE_RUNNING, 0b000100, 1.234, -3.21, 7.5, 41.3)
        cid, data = rs.status_frame(14, st, r)
        self.assertEqual(rs.frame_motor(cid), 14)
        back = rs.decode_status(cid, data, r)
        self.assertEqual((back.mode, back.faults), (rs.MODE_RUNNING, 0b000100))
        self.assertEqual(back.fault_names(), ['overtemperature'])
        self.assertAlmostEqual(back.q, 1.234, delta=1e-3)
        self.assertAlmostEqual(back.v, -3.21, delta=1e-3)
        self.assertAlmostEqual(back.tau, 7.5, delta=3e-3)
        self.assertAlmostEqual(back.temp, 41.3, places=5)

    def test_position_round_trips_through_motion_and_status(self):
        r = rs.RANGES['RS06']
        _, data = rs.motion(1, 1.234, 0, 0, 0, 0, r)
        self.assertAlmostEqual(rs.u16_decode(struct.unpack('>H', data[:2])[0], r.position_max), 1.234,
                               delta=1e-3)


def sim_run(test, *extra, hook=None, motor_id=14):
    out = tempfile.mkdtemp()
    motors = []

    def capture(m):
        motors.append(m)
        if hook:
            hook(m)
    code = chirp.main([test, '--id', str(motor_id), '--sim', '--sim-params', TRUTH, '--out', out, *extra],
                      sim_motor_hook=capture)
    runs = sorted(os.path.join(out, d) for d in os.listdir(out))
    metas = []
    for d in runs:
        with open(os.path.join(d, 'meta.json')) as f:
            metas.append(json.load(f))
    return code, runs, metas, motors


SHORT = {
    'friction': ['--speeds', '0.5', '2', '--hold', '1.0'],
    'torque-chirp': ['--duration', '6'],
    'pos-chirp': ['--duration', '6', '--f0', '0.5', '--f1', '10'],
    'step': ['--amps', '0.1', '--hold', '0.5'],
    'top-speed': [],
}


class Runs(unittest.TestCase):
    def test_every_profile_completes_and_logs(self):
        for test, extra in SHORT.items():
            with self.subTest(test=test):
                code, runs, metas, motors = sim_run(test, *extra)
                self.assertEqual(code, 0)
                meta = metas[0]
                self.assertTrue(meta['completed'])
                self.assertEqual(meta['model'], 'RS03')
                self.assertGreater(meta['n_cmd'], 0.95 * meta['duration'] * 500)
                self.assertGreater(meta['n_fb'], 0.95 * meta['n_cmd'])
                self.assertAlmostEqual(meta['vbus_start'], 36.4, places=3)
                m = motors[0]
                self.assertEqual(m.torque_limit, chirp.TORQUE_CAP['RS03'])  # written to the drive
                self.assertEqual(m.received[-2:], [rs.T_STOP, rs.T_READ])   # stopped, then vbus read
                self.assertFalse(m.enabled)
                with open(os.path.join(runs[0], 'cmd.csv')) as f:
                    self.assertEqual(f.readline().strip(), 't,q_ref,v_ref,kp,kd,tau_ff')

    def test_position_tests_start_where_the_motor_is(self):
        code, _, metas, _ = sim_run('step', *SHORT['step'], hook=lambda m: setattr(m, 'q', -2.0))
        self.assertEqual(code, 0)
        self.assertAlmostEqual(metas[0]['q_start'], -2.0, delta=1e-3)

    def test_spinning_tests_rewind_to_zero(self):
        code, _, metas, motors = sim_run('top-speed')
        self.assertEqual(code, 0)
        self.assertTrue(metas[0]['rewound'])
        self.assertLess(abs(motors[0].q), 0.15)

    def test_top_speed_target_stays_inside_the_wire_range(self):
        a = chirp.parse_args(['top-speed', '--id', '14'])
        p = chirp.build_profile('top-speed', 'RS03', a)
        self.assertLessEqual(p.params['vmax'], 0.98 * rs.RANGES['RS03'].velocity_max)

    def test_model_comes_from_the_id_scheme(self):
        self.assertEqual(chirp.parse_args(['step', '--id', '13']).model, 'RS02')
        self.assertEqual(chirp.parse_args(['step', '--id', '25']).model, 'RS06')
        self.assertEqual(chirp.parse_args(['step', '--id', '25']).torque_limit, 8.0)
        with self.assertRaises(SystemExit):
            chirp.parse_args(['step', '--id', '2'])  # bench IDs need --model
        with self.assertRaises(SystemExit):
            chirp.parse_args(['step', '--id', '13', '--torque-limit', '20'])  # RS02 max is 17

    def test_hold_rate_holds_targets_like_the_policy(self):
        a = chirp.parse_args(['pos-chirp', '--id', '14', '--hold-rate', '50'])
        p = chirp.build_profile('pos-chirp', 'RS03', a)
        self.assertEqual(p(3.001).dq, p(3.019).dq)
        self.assertNotEqual(p(3.019).dq, p(3.021).dq)

    def test_profiles_start_and_end_without_jumps(self):
        a = chirp.parse_args(['all', '--id', '14'])
        for t in ('torque-chirp', 'pos-chirp', 'step'):
            p = chirp.build_profile(t, 'RS03', a)
            for c in (p(0.0), p(p.duration - 1e-6)):
                self.assertAlmostEqual(c.dq, 0.0, places=6)
                self.assertAlmostEqual(c.tau, 0.0, places=6)


class Aborts(unittest.TestCase):
    def assert_aborted(self, code, metas, motors, text):
        self.assertEqual(code, 2)
        self.assertIn(text, metas[0]['aborted'])
        self.assertIn(rs.T_STOP, motors[0].received[-3:])
        self.assertFalse(motors[0].enabled)

    def test_fault_aborts_and_stops(self):
        code, _, metas, motors = sim_run('torque-chirp', '--duration', '6',
                                         hook=lambda m: setattr(m, 'fault_at', (SIM_T0 + 1.0, 0b100)))
        self.assert_aborted(code, metas, motors, 'overtemperature')

    def test_silence_aborts(self):
        code, _, metas, motors = sim_run('step', *SHORT['step'],
                                         hook=lambda m: setattr(m, 'mute_at', SIM_T0 + 1.0))
        self.assertEqual(code, 2)
        self.assertIn('no status reply', metas[0]['aborted'])

    def test_runaway_aborts_on_excursion(self):
        # A 10x too-light rotor makes the torque chirp move ~10x too far: the guard trips.
        code, _, metas, motors = sim_run('torque-chirp', '--duration', '6', '--amp', '0.3',
                                         hook=lambda m: setattr(m, 'J', 0.0005))
        self.assert_aborted(code, metas, motors, 'from the start')

    def test_refuses_when_mit_angle_disagrees_with_mechpos(self):
        code, runs, _, motors = sim_run('step', hook=lambda m: setattr(m, 'q', 14.0))  # > 4 pi
        self.assertEqual(code, 2)
        self.assertEqual(runs, [])  # nothing ran
        self.assertTrue(all(kp == 0.0 for _, _, kp, _, _ in motors[0].commands))  # damping only
        self.assertEqual(motors[0].received[-2:], [rs.T_STOP, rs.T_READ])


SIM_T0 = chirp.SimClock().t


@unittest.skipUnless(HAVE_FIT, 'fit.py needs numpy + scipy')
class FitRecovery(unittest.TestCase):
    """Fit the simulated motor and get its true parameters back."""

    @classmethod
    def setUpClass(cls):
        _, fr, _, _ = sim_run('friction', '--speeds', '0.5', '1', '2', '4', '--hold', '1.5')
        _, tc, _, _ = sim_run('torque-chirp', '--duration', '12')
        _, pc, _, _ = sim_run('pos-chirp', '--duration', '12', '--f0', '0.5', '--f1', '15')
        out = os.path.join(tempfile.mkdtemp(), 'r.json')
        fit.main([*fr, *tc, *pc, '--out', out])
        with open(out) as f:
            cls.res = json.load(f)['motors'][0]

    def test_friction(self):
        self.assertAlmostEqual(self.res['Fc'], 0.15, delta=0.015)
        self.assertAlmostEqual(self.res['b'], 0.01, delta=0.003)
        self.assertAlmostEqual(self.res['torque_estimate_ratio'], 1.0, delta=0.05)

    def test_inertia_and_delay(self):
        self.assertAlmostEqual(self.res['J'], 0.02, delta=0.001)
        self.assertAlmostEqual(self.res['delay'], 0.003, delta=0.0006)

    def test_closed_loop_validates(self):
        v = self.res['validation']['pos_chirp']
        self.assertLess(v['bode_err_to_10Hz']['gain_db'], 1.0)
        self.assertLess(v['bode_err_to_10Hz']['phase_deg'], 10.0)
        self.assertAlmostEqual(v['refit']['kp_scale'], 1.0, delta=0.06)


if __name__ == '__main__':
    unittest.main()
