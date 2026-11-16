#!/usr/bin/env python3
"""Drive every motor at once in MIT mode from one process: hold, sine, spin or home.

  python3 tools/walker/sysid/mit_all.py sine                    # every motor found, +/-0.2 rad at 0.5 Hz, 10 s
  python3 tools/walker/sysid/mit_all.py sine --amp 0.3 --freq 1 --seconds 20
  python3 tools/walker/sysid/mit_all.py hold --seconds 30       # stiff at the policy gains: push on it
  python3 tools/walker/sysid/mit_all.py spin --speed 2 --dir +  # velocity only (kp 0), like spin.py
  python3 tools/walker/sysid/mit_all.py home                    # turn every output back to 0 rad
  python3 tools/walker/sysid/mit_all.py sine --ids 11-13,21     # just these

Every command is a full MIT frame (q_ref, v_ref, kp, kd, tau_ff), the same frame the ROS motor node
sends. hold and sine use the v6 policy gains (RS03 60/3, RS02 25/1, RS06 30/1.5) around the angle
each motor starts at. Motors are found on every CAN adapter that is up (can_left, can_right, can_usb).

Each motor gets chirp.py's setup: the drive's torque limit and 0.2 s CAN timeout, both checked
before enabling. The run stops ALL motors (0.5 s of damping, then stop) if any motor faults, leaves
run mode, goes over 70 C, misses replies for 50 ms or moves further than the motion allows; and on
Ctrl-C. Logs: sysid_logs/<date>_mit_all_<motion>/ (cmd.csv, fb.csv, meta.json).

hold and sine need each MIT angle to match the true angle (mechPos) and to be clear of the +/-4 pi
wire limit. After spins it may not be: run `home` first (it says so). Nothing attached to the
outputs, housings clamped, robot.launch.py and every motor node stopped. Add --sim to preview.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import signal
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import chirp  # noqa: E402
import rsproto as rs  # noqa: E402
from chirp import Abort, SetupError  # noqa: E402

DEFAULT_IDS = '11-15,21-25'
ENABLE_GAP = 0.005   # s between an enable and the next frame to that drive (RS03 drops them otherwise)


def parse_ids(text: str):
    """'11-15,21,23' -> [11, 12, 13, 14, 15, 21, 23]"""
    ids = []
    for part in text.split(','):
        part = part.strip()
        if not part:
            continue
        lo, _, hi = part.partition('-')
        ids += range(int(lo), int(hi or lo) + 1)
    if not ids or any(not 1 <= i <= 127 for i in ids):
        raise ValueError(f'bad --ids {text!r}: IDs 1-127, e.g. 11-15,21-25')
    return sorted(set(ids))


def model_for(mid: int, override: str | None):
    slot = mid % 10
    if mid // 10 in (1, 2) and slot in chirp.MODEL_BY_SLOT:
        return chirp.MODEL_BY_SLOT[slot]
    return override


class Motor:
    def __init__(self, mid, model, iface, bus, clock, vbus):
        self.id, self.model, self.iface, self.vbus = mid, model, iface, vbus
        self.r = rs.RANGES[model]
        self.link = chirp.Link(bus, clock, mid, model)
        self.q0 = self.mech = self.temp_start = self.temp = None
        self.st = None
        self.last_fb = None
        self.n_cmd = self.n_fb = 0
        self.enabled = False
        self.ref = (0.0, 0.0)          # last (q_ref, v_ref) sent
        self.err2 = 0.0                # sum of squared tracking error (position motions)
        self.err_max = 0.0
        self.hold_v, self.hold_tau = [], []
        self.homed = False


# ============================================================================= motions
class Motion:
    """What every motor is told at time t. `gains(model)` -> (kp, kd)."""
    name = ''
    needs_position = False
    reads_mech = False
    max_excursion = None     # rad from the start angle, or None

    def __init__(self, a):
        self.a = a

    def gains(self, model):
        raise NotImplementedError

    def command(self, m: Motor, t: float):
        raise NotImplementedError

    def finished(self, motors):
        return False


class Hold(Motion):
    name, needs_position = 'hold', True
    max_excursion = 0.5

    def __init__(self, a):
        super().__init__(a)
        self.duration = a.seconds

    def gains(self, model):
        kp, kd = chirp.DEPLOY_GAINS[model]
        return (self.a.kp if self.a.kp is not None else kp, self.a.kd if self.a.kd is not None else kd)

    def command(self, m, t):
        kp, kd = self.gains(m.model)
        return m.q0, 0.0, kp, kd, 0.0

    def describe(self):
        return f'hold the start angle for {self.duration:g} s'


class Sine(Hold):
    name = 'sine'

    def __init__(self, a):
        super().__init__(a)
        self.max_excursion = a.amp + 0.3
        self.order = {}

    def command(self, m, t):
        kp, kd = self.gains(m.model)
        a = self.a
        w = 2 * math.pi * a.freq
        ph = w * t + math.radians(a.stagger) * self.order.get(m.id, 0)
        env = chirp.taper(t, self.duration, a.ramp)
        return m.q0 + a.amp * env * math.sin(ph), a.amp * env * w * math.cos(ph), kp, kd, 0.0

    def describe(self):
        st = f', each motor {self.a.stagger:g} deg behind the previous' if self.a.stagger else ''
        return (f'+/-{self.a.amp:g} rad at {self.a.freq:g} Hz around the start angle for '
                f'{self.duration:g} s ({self.a.ramp:g} s fade in/out){st}')


class Spin(Motion):
    name = 'spin'

    def __init__(self, a):
        super().__init__(a)
        sign = 1.0 if a.dir in ('+', 'pos', 'positive') else -1.0
        self.v = sign * a.speed
        self.prof = {}
        self.duration = 2 * a.ramp + a.seconds + 0.5

    def gains(self, model):
        return 0.0, self.a.kd if self.a.kd is not None else chirp.TOP_SPEED_KD[model]

    def command(self, m, t):
        p = self.prof.get(m.id)
        if p is None:
            p = self.prof[m.id] = chirp.spin_profile(self.v, self.gains(m.model)[1], self.a.seconds,
                                                     self.a.ramp)
        c = p(t)
        return 0.0, c.v, 0.0, c.kd, 0.0

    def in_hold(self, t):
        a = self.a
        return a.ramp + 0.25 * a.seconds <= t <= a.ramp + 0.9 * a.seconds

    def describe(self):
        return (f'{self.v:+g} rad/s for {self.a.seconds:g} s, {self.a.ramp:g} s ramps, kp 0 '
                f'(velocity only). The outputs SPIN.')


class Home(Motion):
    """chirp.rewind() for every motor at once: kd-only velocity towards mechPos 0."""
    name, reads_mech = 'home', True

    def __init__(self, a):
        super().__init__(a)
        self.duration = 60.0

    def gains(self, model):
        return 0.0, self.a.kd if self.a.kd is not None else chirp.FRICTION_KD[model]

    def command(self, m, t):
        kd = self.gains(m.model)[1]
        err = 0.0 - (m.mech if m.mech is not None else 0.0)
        if m.homed or abs(err) < 0.1:
            m.homed = True
            return 0.0, 0.0, 0.0, kd, 0.0   # damping: stays put
        v = math.copysign(min(2.0, max(abs(3.0 * err), 1.0 / kd)), err)
        return 0.0, v, 0.0, kd, 0.0

    def finished(self, motors):
        return all(m.homed for m in motors)

    def describe(self):
        return 'turn every output back to 0 rad (mechPos) at up to 2 rad/s, kp 0. The outputs SPIN.'


MOTIONS = {'hold': Hold, 'sine': Sine, 'spin': Spin, 'home': Home}


# ============================================================================= setup
def discover(ids, clock, a):
    """-> ({id: (iface, vbus)}, {iface: bus}). Sends a VBUS read to every ID on every adapter."""
    found, buses = {}, {}
    for name in chirp.can_interfaces():
        try:
            bus = chirp.SocketCanBus(name, clock)
        except SetupError:
            continue
        here = {}
        try:
            while bus.recv(0.0):
                pass
            for mid in ids:
                if mid not in found:
                    bus.send(*rs.read_param(mid, rs.P_VBUS))
                    time.sleep(0.0003)
            end = clock.now() + 0.15
            while (left := end - clock.now()) > 0:
                fr = bus.recv(left)
                if fr and rs.comm_type(fr[0]) == rs.T_READ and fr[0] & 0xFF == rs.HOST:
                    mid, idx, val = rs.decode_param_reply(fr[0], fr[1])
                    if idx == rs.P_VBUS and mid in ids and mid not in found:
                        here[mid] = (name, val)
        except (Abort, OSError):   # nothing on this bus acknowledges frames
            pass
        if here:
            found.update(here)
            buses[name] = bus
        else:
            bus.close()
    return found, buses


def settings_for(m: Motor, a):
    """The per-motor options chirp.configure() and timeout_trips() read."""
    tl = a.torque_limit if a.torque_limit is not None else chirp.TORQUE_CAP[m.model]
    return types.SimpleNamespace(torque_limit=min(tl, m.r.torque_max), can_timeout=a.can_timeout,
                                 damping_kd=a.damping_kd)


def damp(m: Motor, a):
    m.link.command(0.0, 0.0, 0.0, a.damping_kd, 0.0)


def enable_all(motors, clock, a, motion):
    """Enable one motor after another, keeping the ones already running fed with damping commands
    (their CAN timeout is 0.2 s), and check each one answers in run mode. One retry per motor: a
    drive can miss an enable sent right after its setup frames."""
    for i, m in enumerate(motors):
        for attempt in (1, 2):
            m.link.send(rs.enable(m.id))
            m.enabled = True
            clock.sleep(ENABLE_GAP)
            for other in motors[:i]:
                damp(other, a)
            st = m.link.fresh_exchange(0.0, 0.0, 0.0, a.damping_kd, 0.0)
            if st is not None and not st.faults and st.mode == rs.MODE_RUNNING:
                break
            if attempt == 2:
                raise SetupError(f'motor {m.id} not running after enable: '
                                 + ('no reply' if st is None else f'mode {st.mode}, faults {st.fault_names()}'))
        m.st, m.q0, m.temp_start = st, st.q, st.temp
        if motion.needs_position:
            margin = (motion.max_excursion or 0.0) + 0.3
            if why := chirp.position_problem(st.q, m.mech, m.r, margin=margin):
                raise SetupError(f'motor {m.id}: {why}. Run `mit_all.py home` first (or power-cycle it)')


def finish_all(motors, buses, clock, a):
    """0.5 s of damping for every enabled motor, then stop them all. Never raises."""
    live = [m for m in motors if m.enabled]
    end = clock.now() + 0.5
    try:
        while clock.now() < end and live:
            for m in live:
                damp(m, a)
            stop_at = clock.now() + 0.01
            while (left := stop_at - clock.now()) > 0:
                for fr in chirp.wait_frames(buses, left):
                    m = by_id(live).get(rs.frame_motor(fr[0]))
                    cl = m and m.link.classify(fr)
                    if cl and cl[0] == 'status':
                        m.temp = cl[1].temp
    except (Abort, OSError):
        pass
    finally:
        for m in live:
            try:
                m.link.send(rs.stop(m.id, clear_faults=False))
                m.enabled = False
            except Exception:  # noqa: BLE001  the CAN timeout still stops it
                pass
            clock.sleep(0.002)


def by_id(motors):
    return {m.id: m for m in motors}


# ============================================================================= run
def run(motors, buses, clock, a, motion, log):
    """Fixed-rate loop over every motor. Returns the abort reason, or None."""
    index = by_id(motors)
    period = 1.0 / a.rate
    for b in buses:
        while b.recv(0.0):
            pass
    t0 = clock.now()
    log['t0'] = t0
    next_tick = t0
    for m in motors:
        m.last_fb = t0
    last_send = None
    try:
        while True:
            t = clock.now() - t0
            if t >= motion.duration or motion.finished(motors):
                break
            for m in motors:
                q, v, kp, kd, tau = motion.command(m, t)
                m.link.command(q, v, kp, kd, tau)
                m.ref = (q, v)
                m.n_cmd += 1
                log['cmd'].append((clock.now() - t0, m.id, q, v, kp, kd, tau))
                if motion.reads_mech:
                    m.link.send(rs.read_param(m.id, rs.P_MECH_POS))
            now = clock.now()
            if last_send is not None:
                log['periods'].append(now - last_send)
            last_send = now
            next_tick += period
            while (left := next_tick - clock.now()) > 0:
                for fr in chirp.wait_frames(buses, left):
                    handle(fr, index, clock.now() - t0, t0, a, motion, log)
            now = clock.now()
            for m in motors:
                if now - m.last_fb > a.feedback_timeout:
                    raise Abort(f'motor {m.id}: no status reply for {a.feedback_timeout * 1e3:.0f} ms')
            if now - next_tick > period:
                log['overruns'] += 1
                next_tick = now
        if not motion.finished(motors) and isinstance(motion, Home):
            left = [m.id for m in motors if not m.homed]
            return f'home: motors {left} did not reach 0 within {motion.duration:g} s'
    except Abort as e:
        return str(e)
    except KeyboardInterrupt:
        return 'interrupted (Ctrl-C / SIGTERM)'
    return None


def handle(fr, index, t, t0, a, motion, log):
    cid, data, t_rx = fr
    m = index.get(rs.frame_motor(cid))
    if m is None:
        return
    if rs.comm_type(cid) == rs.T_READ:
        _, idx, val = rs.decode_param_reply(cid, data)
        if idx == rs.P_MECH_POS:
            m.mech = val
        return
    cl = m.link.classify(fr)
    if not cl:
        return
    if cl[0] == 'fault':
        raise Abort(f'motor {m.id}: fault report {cl[1].hex()}')
    _, st, _, ct = cl
    m.st, m.last_fb, m.temp = st, t_rx, st.temp
    m.n_fb += 1
    log['fb'].append((t_rx - t0, m.id, ct, st.q, st.v, st.tau, st.temp, st.mode, st.faults))
    if st.faults:
        raise Abort(f'motor {m.id}: faults {st.fault_names()}')
    if st.mode != rs.MODE_RUNNING:
        raise Abort(f'motor {m.id} left run mode (mode {st.mode}): CAN timeout or a fault')
    if st.temp > a.max_temp:
        raise Abort(f'motor {m.id}: {st.temp:.1f} C > {a.max_temp:g} C')
    if motion.max_excursion is not None and abs(st.q - m.q0) > motion.max_excursion:
        raise Abort(f'motor {m.id} moved {st.q - m.q0:+.2f} rad from its start (limit '
                    f'{motion.max_excursion:g})')
    if motion.needs_position:
        e = abs(st.q - m.ref[0])
        m.err2 += e * e
        m.err_max = max(m.err_max, e)
    if isinstance(motion, Spin) and motion.in_hold(t_rx - t0):
        m.hold_v.append(st.v)
        m.hold_tau.append(st.tau)


# ============================================================================= main
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('motion', choices=sorted(MOTIONS), nargs='?', default='sine')
    ap.add_argument('--ids', help=f'e.g. 11-15,21. Default: whichever of {DEFAULT_IDS} answer')
    ap.add_argument('--model', choices=sorted(rs.RANGES), help='for IDs outside 11-15 / 21-25')
    ap.add_argument('--seconds', type=float, default=10.0, help='hold / sine: run time; spin: time at speed')
    ap.add_argument('--amp', type=float, default=0.2, help='sine amplitude, rad (max 1.0)')
    ap.add_argument('--freq', type=float, default=0.5, help='sine frequency, Hz (max 3)')
    ap.add_argument('--stagger', type=float, default=0.0,
                    help='sine: phase lag between consecutive motors, degrees (a wave along the leg)')
    ap.add_argument('--speed', type=float, default=2.0, help='spin: rad/s')
    ap.add_argument('--dir', default='+', choices=['+', '-', 'pos', 'neg', 'positive', 'negative'])
    ap.add_argument('--ramp', type=float, default=1.0, help='s of fade in / ramp up and out')
    ap.add_argument('--kp', type=float, help='override every motor (default: v6 gains; spin/home 0)')
    ap.add_argument('--kd', type=float, help='override every motor')
    ap.add_argument('--rate', type=float, default=200.0, help='commands per second to EACH motor')
    ap.add_argument('--torque-limit', type=float, help='Nm, every motor. Default chirp.py caps (12/4/8)')
    ap.add_argument('--damping-kd', type=float, default=0.3)
    ap.add_argument('--can-timeout', type=float, default=0.2)
    ap.add_argument('--max-temp', type=float, default=70.0)
    ap.add_argument('--feedback-timeout', type=float, default=0.05)
    ap.add_argument('--out', default='sysid_logs')
    ap.add_argument('--note', default='')
    ap.add_argument('--yes', action='store_true', help='skip the confirmation prompt')
    ap.add_argument('--sim', action='store_true', help='simulated motors instead of CAN')
    a = ap.parse_args(argv)
    try:
        a.id_list = parse_ids(a.ids or DEFAULT_IDS)
    except ValueError as e:
        ap.error(str(e))
    for mid in a.id_list:
        if model_for(mid, a.model) is None:
            ap.error(f'ID {mid} is not in the 11-15 / 21-25 scheme; pass --model')
    if not 0 < a.amp <= 1.0 or not 0 < a.freq <= 3.0:
        ap.error('--amp must be in (0, 1] rad and --freq in (0, 3] Hz')
    if not 0 < a.seconds <= 600 or not 0.1 <= a.ramp <= 10:
        ap.error('--seconds in (0, 600], --ramp in [0.1, 10]')
    if not 20 <= a.rate <= 500:
        ap.error('--rate in [20, 500] Hz')
    if a.motion == 'spin' and not 0 < a.speed:
        ap.error('--speed must be > 0')
    for name, val, idx in (('--kp', a.kp, 'kp_max'), ('--kd', a.kd, 'kd_max')):
        if val is not None:
            top = min(getattr(rs.RANGES[model_for(i, a.model)], idx) for i in a.id_list)
            if not 0 <= val <= top:
                ap.error(f'{name} must be in [0, {top:g}] for these motors')
    if a.motion == 'home' and a.rate > 200:
        a.rate = 200.0   # two frames per motor per tick
    return a


def setup_sim(a):
    clock = chirp.SimClock()
    sims = [chirp.SimMotor(mid, model_for(mid, a.model), q0=0.1 * (i - 4), seed=i)
            for i, mid in enumerate(a.id_list)]
    bus = chirp.SimMultiBus(sims, clock)
    found = {mid: ('sim', 36.4) for mid in a.id_list}
    return clock, {'sim': bus}, found, sims


def main(argv=None, sim_hook=None) -> int:
    a = parse_args(argv)
    motion = MOTIONS[a.motion](a)
    sims = []
    if a.sim:
        clock, buses, found, sims = setup_sim(a)
        if sim_hook:
            sim_hook(sims)
    else:
        clock = chirp.RealClock()
        found, buses = discover(a.id_list, clock, a)
    missing = [i for i in a.id_list if i not in found]
    if not found:
        print(f'no motor answered (looked for {a.ids or DEFAULT_IDS} on '
              f'{", ".join(chirp.can_interfaces()) or "no CAN interface that is up"}). '
              f'Run tools/jetson/can_status.sh.')
        return 2
    if missing and a.ids:
        print(f'not found: {missing}. Drop them from --ids or check the wiring (can_status.sh).')
        for b in buses.values():
            b.close()
        return 2
    motors = [Motor(mid, model_for(mid, a.model), found[mid][0], buses[found[mid][0]], clock,
                    found[mid][1]) for mid in sorted(found)]
    if isinstance(motion, Sine):
        motion.order = {m.id: i for i, m in enumerate(motors)}
    if isinstance(motion, Spin):
        for m in motors:
            cap = chirp.spin_speed_cap(m.model, m.vbus)
            if a.speed > cap:
                print(f'--speed {a.speed:g} is above motor {m.id} ({m.model}) cap {cap:.1f} rad/s at {m.vbus:.1f} V')
                return 2

    print(f'MIT mode, {len(motors)} motors, {a.rate:g} Hz each: {motion.describe()}')
    print('   ID  model  bus        VBUS   kp     kd     torque limit')
    for m in motors:
        kp, kd = motion.gains(m.model)
        print(f'   {m.id:<3d} {m.model:6s} {m.iface:10s} {m.vbus:5.1f}V {kp:<6g} {kd:<6g} '
              f'{settings_for(m, a).torque_limit:g} Nm')
    if missing:
        print(f'   (not found, skipped: {", ".join(map(str, missing))})')
    if not a.sim and not a.yes:
        print('Nothing attached to the outputs, housings clamped, hand on the power switch, '
              'motor nodes stopped?')
        if input('Run? [y/N] ').strip().lower() != 'y':
            print('Not run.')
            return 1

    def on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    bus_list = list(buses.values())
    log = {'cmd': [], 'fb': [], 'periods': [], 'overruns': 0}
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    out_dir = os.path.join(a.out, f'{stamp}_mit_all_{motion.name}')
    reason = None
    try:
        for m in motors:
            print(f'  setting up motor {m.id} ({m.model}, {m.iface}) ...', flush=True)
            m.vbus, m.mech = chirp.configure(m.link, settings_for(m, a))
        enable_all(motors, clock, a, motion)
        print(f'Running {motion.duration:.0f} s' + (' (until every motor is home)' if isinstance(motion, Home) else '')
              + ' ... Ctrl-C stops every motor.', flush=True)
        reason = run(motors, bus_list, clock, a, motion, log)
    except SetupError as e:
        reason = f'setup: {e}'
    except Abort as e:
        reason = str(e)
    except KeyboardInterrupt:
        reason = 'interrupted (Ctrl-C / SIGTERM)'
    except OSError as e:
        reason = (f'CAN interface error: {e}. If "Network is down", the adapter reconnected: bring it '
                  f'up again (can_status.sh shows each one). The motors stop themselves after '
                  f'{a.can_timeout:g} s.')
    finally:
        finish_all(motors, bus_list, clock, a)
        for b in bus_list:
            b.close()
    report(motors, motion, log, reason, a)
    if log['cmd']:
        write_logs(out_dir, motors, motion, log, reason, a, stamp)
        print(f'  logs in {out_dir}')
    return 0 if reason is None else 2


def report(motors, motion, log, reason, a):
    per = log['periods']
    print(f'[{motion.name}] {"DONE" if reason is None else "ABORTED: " + reason}')
    if per:
        print(f'  tick period p50 {chirp.percentile(per, 50) * 1e3:.2f} ms, p99 '
              f'{chirp.percentile(per, 99) * 1e3:.2f} ms, max {max(per) * 1e3:.2f} ms, '
              f'{log["overruns"]} overruns')
    for m in motors:
        if not m.n_cmd:
            continue
        line = (f'  m{m.id:<3d} {m.model} {m.iface:10s} {m.n_fb}/{m.n_cmd} replies '
                f'({m.n_fb / m.n_cmd:.0%})')
        if motion.needs_position and m.n_fb:
            line += (f', tracking error rms {1e3 * math.sqrt(m.err2 / m.n_fb):.1f} mrad, '
                     f'max {1e3 * m.err_max:.1f} mrad')
        if isinstance(motion, Spin) and m.hold_v:
            v = sum(m.hold_v) / len(m.hold_v)
            tau = sum(m.hold_tau) / len(m.hold_tau)
            line += f', {v:+.2f} rad/s ({100 * v / motion.v:.0f}%), friction {tau:+.3f} Nm'
        if isinstance(motion, Home):
            line += ', home' if m.homed else f', NOT home (mechPos {m.mech})'
        if m.temp_start is not None and m.temp is not None:
            line += f', {m.temp_start:.0f} -> {m.temp:.0f} C'
        print(line)
    bad = [m.id for m in motors if m.n_cmd and m.n_fb / m.n_cmd < 0.9]
    if reason is None and bad:
        print(f'  WARNING: under 90% replies from {bad}: lower --rate, run with sudo chrt -f 50, '
              f'or check the bus load (canbusload).')


def write_logs(out_dir, motors, motion, log, reason, a, stamp):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'cmd.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'id', 'q_ref', 'v_ref', 'kp', 'kd', 'tau_ff'])
        w.writerows([[f'{x:.6f}' if isinstance(x, float) else x for x in row] for row in log['cmd']])
    with open(os.path.join(out_dir, 'fb.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'id', 'type', 'q', 'v', 'tau', 'temp', 'mode', 'faults'])
        w.writerows([[f'{x:.6f}' if isinstance(x, float) else x for x in row] for row in log['fb']])
    meta = dict(motion=motion.name, describe=motion.describe(), args={k: v for k, v in vars(a).items()},
                motors=[dict(id=m.id, model=m.model, iface=m.iface, vbus=m.vbus, q_start=m.q0,
                             mech_pos_start=m.mech, gains=motion.gains(m.model), n_cmd=m.n_cmd,
                             n_fb=m.n_fb, temp_start=m.temp_start, temp_end=m.temp) for m in motors],
                aborted=reason, completed=reason is None, overruns=log['overruns'], date=stamp,
                host=platform.node(), git=chirp.git_rev(), note=a.note, sim=a.sim)
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1, default=str)


if __name__ == '__main__':
    sys.exit(main())
