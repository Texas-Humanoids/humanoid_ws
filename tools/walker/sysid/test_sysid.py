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
import spin  # noqa: E402
import mit_all  # noqa: E402
import monitor  # noqa: E402

try:
    import numpy  # noqa: F401
    import scipy  # noqa: F401
    import fit
    HAVE_FIT = True
except ImportError:
    HAVE_FIT = False

TRUTH = 'J=0.07,b=0.01,Fc=0.15,delay=0.003'


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

    def test_spin_turns_the_commanded_way_and_stays_there(self):
        for d, sign in (('+', 1), ('-', -1)):
            with self.subTest(dir=d):
                code, _, metas, motors = sim_run('spin', '--speed', '3', '--dir', d, '--duration', '2')
                self.assertEqual(code, 0)
                self.assertNotIn('rewound', metas[0])          # spin leaves the output where it ended
                self.assertGreater(sign * (motors[0].q - metas[0]['q_start']), 2.0)  # >6 rad of 3x2 s

    def test_spin_rejects_bad_input(self):
        with self.assertRaises(SystemExit):
            chirp.parse_args(['spin', '--id', '11', '--speed', '2'])               # no direction
        with self.assertRaises(SystemExit):
            chirp.parse_args(['spin', '--id', '11', '--speed', '40', '--dir', '+'])  # RS03 cap ~16.8
        self.assertEqual(spin.to_chirp_args(['13', '2', 'neg', '4'], [])[:8],
                         ['spin', '--id', '13', '--speed', '2.0', '--dir', '-', '--duration'])
        with self.assertRaises(ValueError):
            spin.to_chirp_args(['13', '2', 'up'], [])

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

    def test_interface_down_aborts_cleanly_and_keeps_the_log(self):
        code, runs, metas, _ = sim_run('step', *SHORT['step'],
                                       hook=lambda m: setattr(m, 'bus_down_at', SIM_T0 + 1.5))
        self.assertEqual(code, 2)
        self.assertIn('CAN interface error', metas[0]['aborted'])
        self.assertGreater(metas[0]['n_cmd'], 0)

    def test_refuses_when_a_safety_write_is_lost(self):
        for index, text in ((rs.P_TORQUE_LIMIT, 'torque limit'), (rs.P_CAN_TIMEOUT, 'CAN timeout')):
            with self.subTest(param=text):
                code, runs, _, motors = sim_run('step', hook=lambda m: m.drop_writes.add(index))
                self.assertEqual(code, 2)
                self.assertEqual(runs, [])
                # never more than damping (the timeout behaviour check enables with kd only)
                self.assertTrue(all(kp == 0.0 and abs(tau) < 0.01  # u16 zero decodes to ~0.002 Nm
                                    for _, _, kp, _, tau in motors[0].commands))
                self.assertFalse(motors[0].enabled)

    def test_unreadable_timeout_is_checked_by_behaviour(self):
        # RS06 firmware reads 0x7028 back as 0 though the write works: run, after checking it trips.
        code, _, metas, motors = sim_run('step', *SHORT['step'],
                                         hook=lambda m: m.unreadable.add(rs.P_CAN_TIMEOUT))
        self.assertEqual(code, 0)
        self.assertTrue(metas[0]['completed'])
        self.assertEqual(motors[0].received.count(rs.T_ENABLE), 2)
        # ...but refuse when the write didn't land either.
        def broken(m):
            m.unreadable.add(rs.P_CAN_TIMEOUT)
            m.drop_writes.add(rs.P_CAN_TIMEOUT)
        code, runs, _, motors = sim_run('step', hook=broken)
        self.assertEqual(code, 2)
        self.assertEqual(runs, [])
        self.assertFalse(motors[0].enabled)

    def test_sim_motor_stops_on_can_timeout(self):
        clock = chirp.SimClock()
        m = chirp.SimMotor(1, 'RS03')
        bus = chirp.SimBus(m, clock)
        bus.send(*rs.write_u32(1, rs.P_CAN_TIMEOUT, rs.can_timeout_counts(0.2)))
        bus.send(*rs.enable(1))
        clock.sleep(0.1)
        bus.send(*rs.motion(1, 0, 0, 0, 1, 0, m.r))
        self.assertTrue(m.enabled)
        clock.sleep(0.3)
        bus.send(*rs.motion(1, 0, 0, 0, 1, 0, m.r))
        self.assertFalse(m.enabled)

    def test_refuses_when_mit_angle_disagrees_with_mechpos(self):
        code, runs, _, motors = sim_run('step', hook=lambda m: setattr(m, 'q', 14.0))  # > 4 pi
        self.assertEqual(code, 2)
        self.assertEqual(runs, [])  # nothing ran
        self.assertTrue(all(kp == 0.0 for _, _, kp, _, _ in motors[0].commands))  # damping only
        self.assertEqual(motors[0].received[-2:], [rs.T_STOP, rs.T_READ])


SIM_T0 = chirp.SimClock().t
STALL = ['--arm', '0.15', '--dir', '+', '--levels', '4']


class FakeReader:
    """Scripted operator: answers[i] are the lines typed during hold number i."""

    def __init__(self, answers):
        self.answers, self.holds = list(answers), -1

    def poll(self, prof, t):
        if not prof.waiting():
            return []
        if prof.i != self.holds and t - prof.t_phase > 0.5:
            self.holds = prof.i
            return self.answers.pop(0) if self.answers else []
        return []


def drive(prof, until=200.0, dt=0.002):
    t = 0.0
    while not prof.done and t < until:
        prof.tick(t)
        t += dt
    return t


class Stall(unittest.TestCase):
    def test_steps_up_and_back_down_reading_every_step(self):
        code, runs, metas, motors = sim_run('stall', *STALL, motor_id=21)
        self.assertEqual(code, 0)
        meta, m = metas[0], motors[0]
        self.assertEqual([r['tau_cmd'] for r in meta['readings']], [3, 6, 9, 12, 9, 6, 3])
        self.assertEqual([r['leg'] for r in meta['readings']], ['up'] * 4 + ['down'] * 3)
        for r in meta['readings']:                       # perfect drive: scale torque = command
            self.assertAlmostEqual(r['tau_true'] / r['tau_cmd'], 1.0, delta=0.02)
        run = [c for c in m.commands if abs(c[4]) > 0.05]
        self.assertTrue(all(c[2] == 0.0 for c in m.commands))             # never a position gain
        self.assertTrue(all(c[4] > 0 for c in run))                       # always pressing +
        self.assertAlmostEqual(max(c[4] for c in run), 12.0, delta=0.01)
        self.assertAlmostEqual(m.torque_limit, 13.2, places=3)            # 1.1 x --stall-max
        self.assertFalse(m.enabled)
        self.assertTrue(os.path.exists(os.path.join(runs[0], 'iq.csv')))
        self.assertGreater(min(r['iq'] for r in meta['readings']), 0.0)

    def test_minus_direction_presses_negative(self):
        code, _, metas, motors = sim_run('stall', '--arm', '0.15', '--dir', '-', '--levels', '2', motor_id=21)
        self.assertEqual(code, 0)
        self.assertTrue(all(c[4] < 0.01 for c in motors[0].commands))
        self.assertEqual(len(metas[0]['readings']), 3)

    def test_rod_slipping_off_the_scale_aborts(self):
        code, _, metas, motors = sim_run('stall', *STALL, motor_id=21,
                                         hook=lambda m: setattr(m, 'wall_off_at', SIM_T0 + 4.0))
        self.assertEqual(code, 2)
        self.assertIn('rod slipped', metas[0]['aborted'])
        self.assertFalse(motors[0].enabled)
        self.assertIn(rs.T_STOP, motors[0].received[-3:])
        self.assertIn('rad/s', metas[0]['aborted'])                       # the speed check caught it
        self.assertLess(abs(motors[0].q - metas[0]['q_start']), 0.2)     # braked within ~10 degrees

    def test_pressing_the_wrong_way_aborts_on_the_first_step(self):
        def wrong_side(m):
            m.wall_q, m.wall_dir, m.wall_k = m.q, -1.0, 2000.0
        code, _, metas, _ = sim_run('stall', *STALL, motor_id=21, hook=wrong_side)
        self.assertEqual(code, 2)
        self.assertIn('rod slipped', metas[0]['aborted'])
        self.assertEqual(metas[0]['readings'], [])

    def test_hold_takes_readings_for_the_duration(self):
        code, _, metas, _ = sim_run('stall-hold', '--arm', '0.2', '--dir', '+', '--duration', '20',
                                    '--prompt-every', '5', '--stall-max', '10', motor_id=21)
        self.assertEqual(code, 0)
        self.assertEqual(len(metas[0]['readings']), 3)                    # at 5, 10, 15 s
        self.assertTrue(all(r['tau_cmd'] == 10.0 for r in metas[0]['readings']))

    def test_args(self):
        with self.assertRaises(SystemExit):
            chirp.parse_args(['stall', '--id', '21', '--dir', '+'])                      # no arm
        with self.assertRaises(SystemExit):
            chirp.parse_args(['stall', '--id', '21', '--arm', '0.15'])                   # no dir
        with self.assertRaises(SystemExit):
            chirp.parse_args(['stall', '--id', '13', *STALL[:4], '--stall-max', '20'])   # RS02 peak 17
        with self.assertRaises(SystemExit):
            chirp.parse_args(['stall', '--id', '21', *STALL[:4], '--stall-max', '30', '--torque-limit', '20'])
        a = chirp.parse_args(['stall', '--id', '21', *STALL[:4], '--stall-max', '58'])
        self.assertEqual(a.torque_limit, 60.0)                                           # capped at peak
        self.assertNotIn('stall', chirp.ALL_SEQUENCE)                                     # opposite setup

    def test_operator_can_skip_stop_and_mistype(self):
        a = chirp.parse_args(['stall', '--id', '21', *STALL])
        prof = chirp.build_profile('stall', 'RS03', a)
        log = {'fb': []}
        prof.attach(FakeReader([['2.0'], ['oops', '4.1'], ['\n'], ['q']]), log, None)
        drive(prof)
        self.assertTrue(prof.done)
        self.assertEqual([(r['tau_cmd'], r['reading']) for r in log['readings']], [(3, 2.0), (6, 4.1)])
        self.assertLess(prof.i, len(prof.levels) - 1)      # stopped before the last step

    def test_no_reading_moves_on_after_max_hold(self):
        a = chirp.parse_args(['stall', '--id', '21', *STALL, '--max-hold', '2'])
        prof = chirp.build_profile('stall', 'RS03', a)
        log = {'fb': []}
        prof.attach(FakeReader([]), log, None)
        t = drive(prof)
        self.assertTrue(prof.done)
        self.assertEqual(log['readings'], [])
        self.assertLess(t, 7 * (2 + 0.3) + 1)


def mit_all_run(*args, hook=None):
    out = tempfile.mkdtemp()
    sims = []

    def capture(s):
        sims.extend(s)
        if hook:
            hook(s)
    code = mit_all.main([*args, '--sim', '--out', out], sim_hook=capture)
    runs = [os.path.join(out, d) for d in os.listdir(out)]
    meta = None
    if runs:
        with open(os.path.join(runs[0], 'meta.json')) as f:
            meta = json.load(f)
    return code, meta, sims


class MitAll(unittest.TestCase):
    def test_ids_parse(self):
        self.assertEqual(mit_all.parse_ids('11-13,21'), [11, 12, 13, 21])
        with self.assertRaises(ValueError):
            mit_all.parse_ids('0-3')

    def test_sine_drives_every_motor_with_full_mit_frames(self):
        code, meta, sims = mit_all_run('sine', '--seconds', '6', '--ids', '11-15')
        self.assertEqual(code, 0)
        self.assertTrue(meta['completed'])
        for s, m in zip(sims, meta['motors']):
            kp, kd = chirp.DEPLOY_GAINS[m['model']]
            run = [c for c in s.commands if c[2] > 1.0]           # the sine, not the damping
            self.assertGreater(len(run), 1150)                     # 200 Hz for 6 s
            self.assertAlmostEqual(run[0][2], kp, delta=0.1)
            self.assertAlmostEqual(run[0][3], kd, delta=0.01)
            dq = [c[0] - m['q_start'] for c in run]
            self.assertAlmostEqual(max(dq), 0.2, delta=0.01)
            self.assertAlmostEqual(min(dq), -0.2, delta=0.01)
            self.assertFalse(s.enabled)                            # stopped at the end

    def test_one_motor_fault_stops_all(self):
        code, meta, sims = mit_all_run('sine', '--seconds', '5', '--ids', '11-15',
                                       hook=lambda s: setattr(s[2], 'fault_at', (SIM_T0 + 2.5, 0b100)))
        self.assertEqual(code, 2)
        self.assertIn('motor 13', meta['aborted'])
        self.assertTrue(all(not s.enabled for s in sims))
        self.assertTrue(all(rs.T_STOP in s.received[-3:] for s in sims))

    def test_position_motion_refuses_near_the_wire_limit(self):
        code, meta, sims = mit_all_run('hold', '--ids', '11-12', hook=lambda s: setattr(s[1], 'q', 14.0))
        self.assertEqual(code, 2)
        self.assertTrue(all(not s.enabled for s in sims))
        self.assertTrue(all(c[2] == 0.0 for s in sims for c in s.commands))   # damping only

    def test_home_brings_everyone_to_zero(self):
        def spread(s):
            for i, m in enumerate(s):
                m.q = 3.0 * (i - 2)
        code, meta, sims = mit_all_run('home', '--ids', '11-15', hook=spread)
        self.assertEqual(code, 0)
        self.assertTrue(all(abs(s.q) < 0.15 for s in sims))

    def test_spin_is_velocity_only(self):
        code, meta, sims = mit_all_run('spin', '--seconds', '2', '--dir', '-', '--ids', '21-22')
        self.assertEqual(code, 0)
        for s in sims:
            self.assertTrue(all(c[2] == 0.0 for c in s.commands))
            self.assertLess(min(c[1] for c in s.commands), -1.9)


class MonitorTests(unittest.TestCase):
    def make(self, *argv):
        a = monitor.parse_args(['--sim', *argv])
        clock = chirp.SimClock()
        sims = [chirp.SimMotor(mid, chirp.MODEL_BY_SLOT[mid % 10], seed=mid) for mid in (11, 12, 13)]
        bus = chirp.SimMultiBus(sims, clock)
        return monitor.Monitor({'sim': monitor.TaggedBus('sim', bus)}, clock, a), clock, bus, sims

    def test_finds_motors_and_reads_registers_and_status(self):
        mon, clock, bus, sims = self.make()
        end = clock.now() + 4.0
        while clock.now() < end:
            mon.step(0.002)
        self.assertEqual(sorted(mon.motors), [11, 12, 13])
        for m in mon.motors.values():
            self.assertAlmostEqual(m.reg['vbus'], 36.4, places=3)
            self.assertIsNotNone(m.st)                      # from the stop ping
        self.assertTrue(all(not s.enabled for s in sims))   # nothing was moved
        self.assertNotIn(rs.T_MOTION, [rs.comm_type(c) for _, c, _ in bus.sent])
        self.assertNotIn(rs.T_ENABLE, [rs.comm_type(c) for _, c, _ in bus.sent])

    def test_passive_sends_nothing(self):
        mon, clock, bus, _ = self.make('--passive')
        end = clock.now() + 3.0
        while clock.now() < end:
            mon.step(0.002)
        self.assertEqual(bus.sent, [])

    def test_never_pings_a_motor_something_else_drives(self):
        mon, clock, bus, sims = self.make()
        r = sims[0].r
        end = clock.now() + 6.0
        while clock.now() < end:
            # another program commanding motor 11 at 100 Hz: its commands and the replies
            st = rs.Status(rs.MODE_RUNNING, 0, 0.1, 0.0, 0.0, 30.0)
            mon.on_frame('sim', (*rs.motion(11, 0.1, 0, 60, 3, 0, r), clock.now()))
            mon.on_frame('sim', (*rs.status_frame(11, st, r), clock.now()))
            stop_at = clock.now() + 0.01
            while clock.now() < stop_at:
                mon.step(0.002)
        stops = [c & 0xFF for _, c, _ in bus.sent if rs.comm_type(c) == rs.T_STOP]
        self.assertNotIn(11, stops)
        self.assertIn(12, stops)
        self.assertGreater(mon.motors[11].cmd_rate.get(clock.now()), 80)
        self.assertAlmostEqual(mon.motors[11].cmd[2], 60.0, delta=0.1)


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
        self.assertAlmostEqual(self.res['J'], 0.07, delta=0.0035)
        self.assertAlmostEqual(self.res['delay'], 0.003, delta=0.0006)

    def test_closed_loop_validates(self):
        v = self.res['validation']['pos_chirp']
        self.assertLess(v['bode_err_to_10Hz']['gain_db'], 1.0)
        self.assertLess(v['bode_err_to_10Hz']['phase_deg'], 10.0)
        self.assertAlmostEqual(v['refit']['kp_scale'], 1.0, delta=0.06)


@unittest.skipUnless(HAVE_FIT, 'fit.py needs numpy + scipy')
class StallFit(unittest.TestCase):
    """A drive that gives 1.1x the commanded torque, Kt 2.0 Nm/A: the stall fit gets both back, and
    turns the torque chirp's commanded-torque inertia (J / 1.1) back into the true one."""

    @classmethod
    def setUpClass(cls):
        truth = ['--sim-params', TRUTH + ',tau_gain=1.1,kt=2.0']
        _, st, _, _ = sim_run('stall', '--arm', '0.15', '--dir', '+', '--levels', '5', *truth, motor_id=21)
        _, tc, _, _ = sim_run('torque-chirp', '--duration', '12', *truth, motor_id=21)
        out = os.path.join(tempfile.mkdtemp(), 'r.json')
        fit.main([*st, *tc, '--out', out])
        with open(out) as f:
            cls.res = json.load(f)['motors'][0]

    def test_torque_gain_and_constant(self):
        self.assertAlmostEqual(self.res['torque_gain'], 1.1, delta=0.02)
        self.assertAlmostEqual(self.res['Kt'], 2.0, delta=0.04)
        self.assertAlmostEqual(self.res['stall']['drive_estimate_ratio'], 1 / 1.1, delta=0.02)
        self.assertGreater(self.res['stall']['top_step_vs_linear'], 0.97)   # no saturation at 12 Nm

    def test_inertia_in_true_torque_units(self):
        self.assertAlmostEqual(self.res['J'], 0.07 / 1.1, delta=0.0035)       # commanded units
        self.assertAlmostEqual(self.res['J_true_torque'], 0.07, delta=0.004)


if __name__ == '__main__':
    unittest.main()
