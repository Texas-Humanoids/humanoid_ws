#!/usr/bin/env python3
"""Live dashboard of every RobStride motor on every CAN adapter: position, velocity, torque,
temperature, bus voltage, current, mode, faults, limits and gains. Never moves anything.

  python3 tools/walker/sysid/monitor.py              # live table (q quits)
  python3 tools/walker/sysid/monitor.py --once       # print one snapshot and exit
  python3 tools/walker/sysid/monitor.py --csv m.csv  # also log every motor 10x a second
  python3 tools/walker/sysid/monitor.py --passive    # send nothing at all: only listen

Where each value comes from:
- Status frames (position, velocity, torque estimate, temperature, mode, faults): the reply a drive
  sends to every command. While something else drives the motors (ROS motor node, chirp.py,
  mit_all.py) the monitor just listens to them, and also shows the commands that program sends
  (rate, kp, kd). A motor nobody has commanded for 2 s gets a stop frame once a second instead, which
  a stopped drive answers with a status frame and otherwise ignores.
- Registers, read with parameter reads: bus voltage, q-axis current (iq), mechPos/mechVel (true
  multi-turn angle), run mode, torque/current/speed limits, the drive's own loop gains. Safe while
  a motor runs; adds about 1000 small frames/s per bus.
- The MCU UID, once per motor.

New motors are found every 5 s (any ID 1-127), so you can plug things in while it runs.
--passive sends no frames: only motors that something else is talking to show up.
Runs anywhere the CAN adapters are (the Jetson host, or the robot container); --sim to preview.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import os
import signal
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import chirp  # noqa: E402
import rsproto as rs  # noqa: E402
from chirp import Abort, SetupError  # noqa: E402

# (index, name, kind): kind f = float, u8, i16
FAST = [(rs.P_VBUS, 'vbus', 'f'), (0x701A, 'iq', 'f'), (rs.P_MECH_POS, 'mech_pos', 'f'),
        (rs.P_MECH_VEL, 'mech_vel', 'f')]
SLOW = [(rs.P_RUN_MODE, 'run_mode', 'u8'), (rs.P_TORQUE_LIMIT, 'limit_torque', 'f'),
        (0x7018, 'limit_cur', 'f'), (0x7017, 'limit_spd', 'f'), (0x701E, 'loc_kp', 'f'),
        (0x701F, 'spd_kp', 'f'), (0x7020, 'spd_ki', 'f'), (0x7010, 'cur_kp', 'f'),
        (0x7011, 'cur_ki', 'f')]
KIND = {i: k for i, _, k in FAST + SLOW}
NAME = {i: n for i, n, _ in FAST + SLOW}
RUN_MODES = {0: 'MIT', 1: 'CSP-pos', 2: 'speed', 3: 'current', 5: 'CSP', 6: 'speed'}
MODES = {rs.MODE_RESET: 'reset', rs.MODE_CALIBRATION: 'calib', rs.MODE_RUNNING: 'RUN'}

POLL_GAP = 0.002     # s between parameter reads on one bus
PING_AFTER = 2.0     # s without a status frame before the monitor asks for one
SCAN_EVERY = 5.0


def decode_value(index, raw: bytes):
    kind = KIND.get(index, 'f')
    if kind == 'u8':
        return raw[0]
    if kind == 'i16':
        return struct.unpack('<h', raw[:2])[0]
    return struct.unpack('<f', raw)[0]


class Rate:
    """Events per second over the last second."""

    def __init__(self):
        self.ts = collections.deque()

    def add(self, t):
        self.ts.append(t)

    def get(self, now):
        while self.ts and now - self.ts[0] > 1.0:
            self.ts.popleft()
        return len(self.ts)


class MotorView:
    def __init__(self, mid, iface, model_override, now):
        self.id, self.iface, self.found = mid, iface, now
        slot = mid % 10
        guess = chirp.MODEL_BY_SLOT.get(slot) if mid // 10 in (1, 2) else None
        self.model = guess or model_override
        self.model_known = self.model is not None
        self.r = rs.RANGES[self.model or 'RS03']
        self.st = None              # last rs.Status
        self.st_t = None            # when it arrived
        self.status_rate = Rate()   # status frames per second (all sources)
        self.cmd_rate = Rate()      # MIT commands per second from other programs on this host
        self.cmd = None             # last (q, v, kp, kd, tau) another program sent
        self.reg = {}               # register name -> value
        self.reg_t = None
        self.uid = None
        self.fault_frame = None
        self.last_ping = 0.0
        self.seen = None            # last frame of any kind from this motor

    def quiet(self, now):
        """Nothing has commanded this motor for PING_AFTER s (and we have watched it that long),
        so it is stopped and a stop frame only asks it for a status reply."""
        return (now - self.found > PING_AFTER and (self.st_t is None or now - self.st_t > PING_AFTER)
                and not self.cmd_rate.get(now))

    def decode_cmd(self, cid, data):
        r = self.r
        raw = struct.unpack('>4H', bytes(data[:8]))
        return (rs.u16_decode(raw[0], r.position_max), rs.u16_decode(raw[1], r.velocity_max),
                raw[2] * r.kp_max / 65535.0, raw[3] * r.kd_max / 65535.0,
                rs.u16_decode(cid >> 8 & 0xFFFF, r.torque_max))


class Monitor:
    def __init__(self, buses, clock, a):
        self.buses, self.clock, self.a = buses, clock, a   # {iface: bus}
        self.motors = {}                                    # id -> MotorView
        self.queue = {name: collections.deque() for name in buses}
        self.next_poll = {name: 0.0 for name in buses}
        self.slow_i = {name: 0 for name in buses}
        self.frames = {name: Rate() for name in buses}      # frames seen + sent per bus
        self.sent = {name: Rate() for name in buses}
        self.errors = {name: '' for name in buses}
        self.next_scan = 0.0
        self.messages = collections.deque(maxlen=4)
        self.state = {}
        self.next_state = 0.0

    # --------------------------------------------------------------- transmit
    def send(self, iface, frame):
        if self.a.passive:
            return
        try:
            self.buses[iface].send(*frame)
            self.sent[iface].add(self.clock.now())
            self.errors[iface] = ''
        except (Abort, OSError) as e:
            self.errors[iface] = 'no ACK (nothing powered?)' if isinstance(e, Abort) else str(e)

    def scan(self, now):
        """A VBUS read to every ID not seen yet, spread over the poll slots."""
        for name in self.buses:
            known = {m.id for m in self.motors.values() if m.iface == name}
            other = {m.id for m in self.motors.values() if m.iface != name}
            self.queue[name].extend(('scan', i) for i in range(1, 128) if i not in known | other)
        self.next_scan = now + SCAN_EVERY

    def poll(self, now):
        for name, bus in self.buses.items():
            if now < self.next_poll[name]:
                continue
            self.next_poll[name] = now + POLL_GAP
            q = self.queue[name]
            if not q:
                mine = sorted((m for m in self.motors.values() if m.iface == name), key=lambda m: m.id)
                for m in mine:
                    q.extend(('read', m.id, idx) for idx, _, _ in FAST)
                    idx = SLOW[self.slow_i[name] % len(SLOW)][0]
                    q.append(('read', m.id, idx))
                    if m.uid is None:
                        q.append(('uid', m.id))
                    if m.quiet(now) and now - m.last_ping > 1.0:
                        q.append(('ping', m.id))
                        m.last_ping = now
                self.slow_i[name] += 1
                if not q:
                    continue
            job = q.popleft()
            if job[0] == 'scan':
                self.send(name, rs.read_param(job[1], rs.P_VBUS))
            elif job[0] == 'read':
                self.send(name, rs.read_param(job[1], job[2]))
            elif job[0] == 'uid':
                self.send(name, (rs.can_id(rs.T_GET_ID, rs.HOST, job[1]), bytes(8)))
            elif job[0] == 'ping':
                m = self.motors.get(job[1])
                # Re-check right before sending: never stop a motor something is driving.
                if m and m.quiet(now):
                    self.send(name, rs.stop(job[1], clear_faults=False))

    # --------------------------------------------------------------- receive
    def motor(self, mid, iface):
        m = self.motors.get(mid)
        if m is None:
            m = self.motors[mid] = MotorView(mid, iface, self.a.model, self.clock.now())
            self.messages.append(f'found motor {mid} on {iface}')
        return m

    def on_frame(self, iface, fr):
        cid, data, t = fr[:3]
        self.frames[iface].add(t)
        ct = rs.comm_type(cid)
        src, dst = cid >> 8 & 0xFF, cid & 0xFF
        if ct == rs.T_MOTION:                    # another program on this host commanding a motor
            m = self.motors.get(dst)
            if m:
                m.cmd = m.decode_cmd(cid, data)
                m.cmd_rate.add(t)
            return
        if not 1 <= src <= 127:                  # host-originated frames (reads, stops, enables)
            return
        if ct in (rs.T_STATUS, rs.T_ACTIVE):
            m = self.motor(src, iface)
            m.st = rs.decode_status(cid, data, m.r)
            m.st_t = m.seen = t
            m.status_rate.add(t)
        elif ct == rs.T_READ:
            m = self.motor(src, iface)
            index = struct.unpack('<H', bytes(data[:2]))[0]
            if index in NAME:
                m.reg[NAME[index]] = decode_value(index, bytes(data[4:8]))
                m.reg_t = m.seen = t
        elif ct == rs.T_GET_ID and dst == 0xFE:
            m = self.motor(src, iface)
            m.uid = bytes(data[:8]).hex()
            m.seen = t
        elif ct == rs.T_FAULT:
            m = self.motor(src, iface)
            m.fault_frame = bytes(data[:8]).hex()
            m.seen = t
            self.messages.append(f'motor {src}: fault frame {m.fault_frame}')

    def step(self, timeout):
        now = self.clock.now()
        if not self.a.passive and now >= self.next_scan:
            self.scan(now)
        if not self.a.passive:
            self.poll(now)
        names = list(self.buses)
        bus_list = [self.buses[n] for n in names]
        for fr in chirp.wait_frames(bus_list, timeout):
            self.on_frame(fr[3], fr)   # TaggedBus adds the interface name

    def bus_states(self, now):
        if now < self.next_state:
            return self.state
        self.next_state = now + 1.0
        for name in self.buses:
            try:
                out = subprocess.run(['ip', '-details', '-json', 'link', 'show', name], capture_output=True,
                                     text=True, timeout=1).stdout
                info = json.loads(out)[0]
                st = info.get('linkinfo', {}).get('info_data', {}).get('state', '?')
                self.state[name] = f'{info.get("operstate", "?")} {st}'
            except Exception:  # noqa: BLE001
                self.state[name] = ''
        return self.state


# ============================================================================= display
def fmt(x, spec, width):
    return (format(x, spec) if isinstance(x, (int, float)) else '-').rjust(width)


def rows(mon, now):
    """-> (main table lines, register table lines), each with a header, plus colour hints."""
    head1 = (' ID  model bus        mode   pos rad  vel r/s  torq Nm  temp C  VBUS V    iq A   '
             'status/s  cmd/s  faults           seen')
    head2 = (' ID  run mode  mechPos  mechVel  lim Nm  lim A  lim r/s   loc_kp  spd_kp  spd_ki   '
             'last cmd (q_ref kp kd tau)       UID')
    t1, t2 = [], []
    for m in sorted(mon.motors.values(), key=lambda m: (m.iface, m.id)):
        st, reg = m.st, m.reg
        fresh = st is not None and now - m.st_t < 1.5
        age = now - m.seen if m.seen else None
        mode = MODES.get(st.mode, '?') if st else '-'
        faults = ','.join(st.fault_names()) if st and st.faults else ('none' if st else '-')
        model = m.model if m.model_known else 'RS03?'
        hint = 'lost' if age is None or age > 3.0 else 'fault' if st and st.faults else (
            'run' if st and st.mode == rs.MODE_RUNNING and fresh else 'idle')
        temp = st.temp if st else None
        line = (f' {m.id:<3d} {model:5s} {m.iface:10s} {mode:5s} '
                f'{fmt(st.q if st else None, "+.3f", 8)} {fmt(st.v if st else None, "+.2f", 8)} '
                f'{fmt(st.tau if st else None, "+.2f", 8)} {fmt(temp, ".1f", 7)} '
                f'{fmt(reg.get("vbus"), ".2f", 7)} {fmt(reg.get("iq"), "+.2f", 7)} '
                f'{m.status_rate.get(now):9d} {m.cmd_rate.get(now):6d}  {faults:15s} '
                f'{fmt(age, ".1f", 6)}s')
        t1.append((line, hint, temp))
        rm = reg.get('run_mode')
        c = m.cmd
        cmd = f'{c[0]:+.3f} {c[2]:.1f} {c[3]:.2f} {c[4]:+.2f}' if c and m.cmd_rate.get(now) else '-'
        line2 = (f' {m.id:<3d} {RUN_MODES.get(rm, rm if rm is not None else "-")!s:9s} '
                 f'{fmt(reg.get("mech_pos"), "+.3f", 8)} {fmt(reg.get("mech_vel"), "+.2f", 8)} '
                 f'{fmt(reg.get("limit_torque"), ".1f", 7)} {fmt(reg.get("limit_cur"), ".1f", 6)} '
                 f'{fmt(reg.get("limit_spd"), ".1f", 8)} {fmt(reg.get("loc_kp"), "g", 8)} '
                 f'{fmt(reg.get("spd_kp"), "g", 7)} {fmt(reg.get("spd_ki"), "g", 7)}   '
                 f'{cmd:32s} {m.uid or "-"}')
        t2.append((line2, hint, None))
    return head1, t1, head2, t2


def header(mon, now):
    parts = []
    states = mon.bus_states(now)
    for name in mon.buses:
        n = sum(1 for m in mon.motors.values() if m.iface == name)
        err = f' {mon.errors[name]}' if mon.errors[name] else ''
        parts.append(f'{name}: {states.get(name, "")} {n} motors, {mon.frames[name].get(now) + mon.sent[name].get(now)} '
                     f'frames/s{err}')
    return parts


def snapshot_text(mon, now):
    out = ['RobStride monitor  ' + time.strftime('%H:%M:%S')] + ['  ' + p for p in header(mon, now)]
    h1, t1, h2, t2 = rows(mon, now)
    out += ['', h1] + [r[0] for r in t1] + ['', h2] + [r[0] for r in t2]
    if not mon.motors:
        out.append('  (no motors yet: powered? adapters up? see tools/jetson/can_status.sh)')
    return '\n'.join(out)


def run_curses(mon, a):
    import curses

    def ui(scr):
        curses.curs_set(0)
        scr.nodelay(True)
        try:
            curses.start_color()
            curses.use_default_colors()
            for i, c in enumerate((curses.COLOR_GREEN, curses.COLOR_RED, curses.COLOR_YELLOW,
                                   curses.COLOR_CYAN), 1):
                curses.init_pair(i, c, -1)
        except curses.error:
            pass   # no colours on this terminal
        colour = {'run': curses.color_pair(1), 'fault': curses.color_pair(2) | curses.A_BOLD,
                  'lost': curses.A_DIM, 'idle': 0}
        next_draw = 0.0
        paused = False
        while True:
            mon.a.passive = a.passive or paused
            mon.step(0.002)
            log_row(mon, a)
            now = mon.clock.now()
            if now < next_draw:
                continue
            next_draw = now + 0.1
            key = scr.getch()
            if key in (ord('q'), 27):
                return
            if key == ord('p'):
                paused = not paused
            scr.erase()
            h, w = scr.getmaxyx()
            y = 0

            def put(text, attr=0):
                nonlocal y
                if y < h - 1:
                    scr.addnstr(y, 0, text, w - 1, attr)
                y += 1
            put('RobStride monitor  ' + time.strftime('%H:%M:%S') + '   q quit   p '
                + ('resume polling' if paused else 'pause polling (listen only)')
                + ('   [listening only]' if mon.a.passive else ''), curses.A_BOLD)
            for p in header(mon, now):
                put('  ' + p, curses.color_pair(4))
            h1, t1, h2, t2 = rows(mon, now)
            put('')
            put(h1, curses.A_UNDERLINE)
            for line, hint, temp in t1:
                attr = colour[hint]
                if temp is not None and temp >= a.warn_temp and hint != 'lost':
                    attr = curses.color_pair(2 if temp >= a.max_temp else 3) | curses.A_BOLD
                put(line, attr)
            put('')
            put(h2, curses.A_UNDERLINE)
            for line, hint, _ in t2:
                put(line, colour[hint])
            if not mon.motors:
                put('  (no motors yet: powered? adapters up? see tools/jetson/can_status.sh)')
            put('')
            for msg in mon.messages:
                put('  ' + msg, curses.A_DIM)
            scr.refresh()
    curses.wrapper(ui)


_csv = {'w': None, 'next': 0.0}


def log_row(mon, a):
    if not a.csv:
        return
    now = mon.clock.now()
    if now < _csv['next']:
        return
    _csv['next'] = now + 0.1
    if _csv['w'] is None:
        f = open(a.csv, 'w', newline='')
        _csv['f'], _csv['w'] = f, csv.writer(f)
        _csv['w'].writerow(['time', 'id', 'iface', 'mode', 'faults', 'q', 'v', 'tau', 'temp'] +
                           [n for _, n, _ in FAST + SLOW] + ['status_hz', 'cmd_hz'])
    stamp = f'{time.time():.3f}'
    for m in sorted(mon.motors.values(), key=lambda m: m.id):
        st = m.st
        _csv['w'].writerow([stamp, m.id, m.iface] +
                           ([st.mode, st.faults, f'{st.q:.4f}', f'{st.v:.3f}', f'{st.tau:.3f}', f'{st.temp:.1f}']
                            if st else [''] * 6) +
                           [m.reg.get(n, '') for _, n, _ in FAST + SLOW] +
                           [m.status_rate.get(now), m.cmd_rate.get(now)])
    _csv['f'].flush()


# ============================================================================= buses
class TaggedBus:
    """A bus whose frames carry the interface name, so one wait over both adapters can tell them apart."""

    def __init__(self, name, bus):
        self.name, self.bus = name, bus
        if hasattr(bus, 'sock'):
            self.sock = bus.sock

    def send(self, cid, data):
        self.bus.send(cid, data)

    def recv(self, timeout):
        fr = self.bus.recv(timeout)
        return fr and (*fr, self.name)

    def close(self):
        self.bus.close()


def open_buses(a, clock):
    names = [a.iface] if a.iface != 'auto' else chirp.can_interfaces()
    buses = {}
    for name in names:
        try:
            buses[name] = TaggedBus(name, chirp.SocketCanBus(name, clock))
        except SetupError as e:
            print(f'skipping {name}: {e}')
    return buses


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--iface', default='auto', help='one CAN interface; auto = every one that is up')
    ap.add_argument('--model', choices=sorted(rs.RANGES),
                    help='model for IDs outside 11-15 / 21-25 (scales position/velocity/torque)')
    ap.add_argument('--passive', action='store_true', help='send nothing; only listen')
    ap.add_argument('--once', action='store_true', help='print one snapshot (after ~1.5 s) and exit')
    ap.add_argument('--seconds', type=float, default=1.5, help='--once: how long to collect')
    ap.add_argument('--csv', help='log every motor to this CSV 10x a second')
    ap.add_argument('--warn-temp', type=float, default=55.0)
    ap.add_argument('--max-temp', type=float, default=70.0)
    ap.add_argument('--sim', action='store_true', help='simulated motors 11-15 and 21-25')
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    if a.sim:
        clock = chirp.SimClock()
        sims = [chirp.SimMotor(mid, chirp.MODEL_BY_SLOT[mid % 10], q0=0.05 * mid, seed=mid)
                for mid in (11, 12, 13, 14, 15, 21, 22, 23, 24, 25)]
        buses = {'sim': TaggedBus('sim', chirp.SimMultiBus(sims, clock))}
    else:
        clock = chirp.RealClock()
        buses = open_buses(a, clock)
    if not buses:
        print('no CAN interface is up. Bring the adapters up (tools/jetson/can_status.sh does).')
        return 2
    mon = Monitor(buses, clock, a)

    def on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        if a.once or a.sim or not sys.stdout.isatty():
            end = clock.now() + a.seconds
            while clock.now() < end:
                mon.step(0.002)
                log_row(mon, a)
            print(snapshot_text(mon, clock.now()))
            if not a.once and not a.sim:   # piped: keep printing a snapshot every second
                while True:
                    end = clock.now() + 1.0
                    while clock.now() < end:
                        mon.step(0.002)
                        log_row(mon, a)
                    print('\n' + snapshot_text(mon, clock.now()), flush=True)
        else:
            run_curses(mon, a)
    except KeyboardInterrupt:
        pass
    finally:
        for b in buses.values():
            b.close()
        if _csv.get('f'):
            _csv['f'].close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
