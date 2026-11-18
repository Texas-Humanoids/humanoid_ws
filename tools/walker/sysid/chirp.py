#!/usr/bin/env python3
"""Actuator system-ID runs for ONE RobStride motor: friction sweep, torque chirp, position chirp,
steps and top speed. fit.py turns the logs into the motor model that training uses.

Talks to SocketCAN directly (no ROS) at a fixed rate, and timestamps every command and every
status reply in one process, so the fit sees the real timing. Standard library only.

Setup: nothing attached to the motor's output, housing CLAMPED to the table (direct drive: the
reaction torque spins an unclamped housing), a tape flag on the output face, a hand on the power
switch. Stop robot.launch.py and every robstride_motor_node first: they share the bus.

  python3 tools/walker/sysid/chirp.py --id 14 friction
  python3 tools/walker/sysid/chirp.py --id 14 torque-chirp
  python3 tools/walker/sysid/chirp.py --id 14 pos-chirp                  # v6 gains, 500 Hz targets
  python3 tools/walker/sysid/chirp.py --id 14 pos-chirp --hold-rate 50   # targets held like the policy
  python3 tools/walker/sysid/chirp.py --id 14 step
  python3 tools/walker/sysid/chirp.py --id 14 all          # the five runs above (friction last)
  python3 tools/walker/sysid/chirp.py --id 14 top-speed    # separately: spins up to ~top speed

Locked-output torque (a different bench setup, so never part of `all`): a rigid rod on the output
pushes down on a scale, L metres from the motor axis. The script steps the torque, and you type
the scale reading at each step. fit.py turns it into true torque / commanded torque, peak torque
and the torque constant.
  python3 tools/walker/sysid/chirp.py --id 21 stall --arm 0.15 --dir + --stall-max 12
  python3 tools/walker/sysid/chirp.py --id 21 stall-hold --arm 0.15 --dir + --stall-max 20 --duration 180

Add --sim to run against a simulated motor (no CAN, any OS) to preview a run. Each run writes
<out>/<date>_m<id>_<test>/{meta.json, cmd.csv, fb.csv}. Exit code 0 = completed, 2 = aborted
(the logs up to the abort are still written).
"""
from __future__ import annotations

import argparse
import collections
import csv
import errno
import json
import math
import os
import platform
import random
import signal
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rsproto as rs  # noqa: E402

# Motor of each joint slot: IDs 11-15 / 21-25 = hip pitch, hip roll, hip yaw, knee, ankle.
MODEL_BY_SLOT = {1: 'RS03', 2: 'RS03', 3: 'RS02', 4: 'RS03', 5: 'RS06'}
# v6 policy gains (policies/walker_v2/v6/policy_spec.json), the ones the robot runs.
DEPLOY_GAINS = {'RS03': (60.0, 3.0), 'RS02': (25.0, 1.0), 'RS06': (30.0, 1.5)}
# Training's armature values (datasheet-based guesses); only used to size the torque chirp.
# Measured 2026-11-15 (commanded-torque units): RS03 0.07 (m21), RS02 0.012 (m13). RS06 is a guess.
J_GUESS = {'RS03': 0.07, 'RS02': 0.012, 'RS06': 0.008}
# Default torque caps for the tests, ~20-25 % of peak. Written to the drive (0x700B).
TORQUE_CAP = {'RS03': 12.0, 'RS02': 4.0, 'RS06': 8.0}
NO_LOAD_48V = {'RS03': 20.4, 'RS02': 42.9, 'RS06': 50.3}  # rad/s, datasheet @ 48 V
FRICTION_KD = {'RS03': 1.0, 'RS02': 0.5, 'RS06': 1.0}
TOP_SPEED_KD = {'RS03': 2.0, 'RS02': 1.0, 'RS06': 2.0}

# Spinning test last, so the position tests start near the motor's zero.
ALL_SEQUENCE = ('torque-chirp', 'pos-chirp', 'pos-chirp@50', 'step', 'friction')
# Locked-output tests: rod on the output, pressing on a scale. Never in `all` (opposite setup).
STALL_TESTS = ('stall', 'stall-hold')


class Abort(Exception):
    pass


class SetupError(Exception):
    pass


# ============================================================================= profiles
def taper(t: float, duration: float, edge: float) -> float:
    """Cosine fade in and out, so no run starts or ends with a jump."""
    if edge <= 0:
        return 1.0
    x = min(t, duration - t) / edge
    return 0.0 if x <= 0 else 1.0 if x >= 1 else 0.5 - 0.5 * math.cos(math.pi * x)


def log_sweep(t: float, f0: float, f1: float, duration: float):
    """Exponential sweep: (instantaneous frequency Hz, phase rad)."""
    k = math.log(f1 / f0)
    f = f0 * math.exp(k * t / duration)
    phase = 2 * math.pi * f0 * duration / k * (math.exp(k * t / duration) - 1.0)
    return f, phase


@dataclass
class Command:
    dq: float = 0.0     # position target relative to the start angle, rad
    v: float = 0.0      # velocity target, rad/s
    kp: float = 0.0
    kd: float = 0.0
    tau: float = 0.0    # feed-forward torque, Nm


@dataclass
class Profile:
    name: str
    duration: float
    fn: object                       # t -> Command
    max_excursion: float | None      # rad from the start angle; None = spins (velocity tests)
    hold_rate: float = 0.0           # Hz; >0 = zero-order hold the command like the policy does
    describe: str = ''
    params: dict = field(default_factory=dict)
    absolute_targets: bool = True    # sends MIT position targets (needs a sane start angle)

    def __call__(self, t: float) -> Command:
        if self.hold_rate > 0:
            t = math.floor(t * self.hold_rate) / self.hold_rate
        return self.fn(t)


def friction_profile(speeds, hold, kd, lead=0.5):
    seq = [0.0]
    for s in speeds:
        seq += [s, -s]
    seq.append(0.0)
    durs = [lead] + [hold] * (2 * len(speeds)) + [lead]
    ends = list(itertools_accumulate(durs))

    def fn(t):
        i = next((k for k, e in enumerate(ends) if t < e), len(seq) - 1)
        return Command(v=seq[i], kd=kd)
    return Profile('friction', ends[-1], fn, None,
                   describe=f'velocity steps to +/-{max(speeds):g} rad/s with kd {kd:g}, kp 0. '
                            'The output SPINS continuously.',
                   params=dict(speeds=list(speeds), hold=hold, kd=kd))


def torque_chirp_profile(f0, f1, duration, amp, j_guess, max_torque, kp, kd, edge=1.0):
    def fn(t):
        f, ph = log_sweep(t, f0, f1, duration)
        a = min(j_guess * (2 * math.pi * f) ** 2 * amp, max_torque)
        return Command(kp=kp, kd=kd, tau=taper(t, duration, edge) * a * math.sin(ph))
    return Profile('torque-chirp', duration, fn, max(4 * amp, 1.0),
                   describe=f'feed-forward torque sweep {f0:g}->{f1:g} Hz over {duration:g} s, '
                            f'~+/-{amp:g} rad, up to {max_torque:g} Nm; kp {kp:g} keeps it centred',
                   params=dict(f0=f0, f1=f1, duration=duration, amp=amp, j_guess=j_guess,
                               max_torque=max_torque, kp=kp, kd=kd, edge=edge))


def pos_chirp_profile(f0, f1, duration, amp, corner, kp, kd, hold_rate, edge=1.0):
    def fn(t):
        f, ph = log_sweep(t, f0, f1, duration)
        a = amp * min(1.0, corner / f)  # smaller above `corner` Hz so the PD stays unsaturated
        return Command(dq=taper(t, duration, edge) * a * math.sin(ph), kp=kp, kd=kd)
    name = 'pos-chirp' if hold_rate <= 0 else f'pos-chirp@{hold_rate:g}'
    return Profile(name, duration, fn, amp + 0.3, hold_rate=hold_rate,
                   describe=f'position sweep {f0:g}->{f1:g} Hz over {duration:g} s, +/-{amp:g} rad '
                            f'(rolled off above {corner:g} Hz), kp {kp:g} kd {kd:g}'
                            + (f', targets held at {hold_rate:g} Hz' if hold_rate > 0 else ''),
                   params=dict(f0=f0, f1=f1, duration=duration, amp=amp, corner=corner, kp=kp,
                               kd=kd, hold_rate=hold_rate, edge=edge))


def step_profile(amps, hold, kp, kd, lead=0.5):
    seq, durs = [0.0], [lead]
    for a in amps:
        seq += [a, 0.0, -a, 0.0]
        durs += [hold] * 4
    ends = list(itertools_accumulate(durs))

    def fn(t):
        i = next((k for k, e in enumerate(ends) if t < e), len(seq) - 1)
        return Command(dq=seq[i], kp=kp, kd=kd)
    return Profile('step', ends[-1], fn, max(amps) + 0.3,
                   describe=f'position steps of +/-{", ".join(f"{a:g}" for a in amps)} rad, '
                            f'kp {kp:g} kd {kd:g}',
                   params=dict(amps=list(amps), hold=hold, kp=kp, kd=kd))


def top_speed_profile(vmax, kd, ramp=3.0, hold=2.0, pause=1.0):
    # (t_end, v_start, v_end) pieces: up, hold, down, pause, then the same negative.
    pieces, t = [], 0.0
    for s in (1.0, -1.0):
        for dur, a, b in ((ramp, 0.0, s * vmax), (hold, s * vmax, s * vmax), (ramp, s * vmax, 0.0),
                          (pause, 0.0, 0.0)):
            t += dur
            pieces.append((t, dur, a, b))

    def fn(t):
        for end, dur, a, b in pieces:
            if t < end:
                return Command(v=a + (b - a) * (1 - (end - t) / dur), kd=kd)
        return Command(kd=kd)
    return Profile('top-speed', t, fn, None,
                   describe=f'velocity ramps to +/-{vmax:.1f} rad/s with kd {kd:g}, kp 0. '
                            'The output SPINS at close to the motor top speed.',
                   params=dict(vmax=vmax, kd=kd, ramp=ramp, hold=hold, pause=pause))


def spin_profile(v, kd, hold, ramp=1.0, pause=0.5):
    """Constant velocity v (signed, rad/s): ramp up, hold, ramp down, short pause. kp 0, kd only."""
    pieces = [(ramp, 0.0, v), (ramp + hold, v, v), (2 * ramp + hold, v, 0.0),
              (2 * ramp + hold + pause, 0.0, 0.0)]

    def fn(t):
        start = 0.0
        for end, a, b in pieces:
            if t < end:
                return Command(v=a + (b - a) * (t - start) / (end - start), kd=kd)
            start = end
        return Command(kd=kd)
    return Profile('spin', pieces[-1][0], fn, None,
                   describe=f'constant {v:+g} rad/s for {hold:g} s ({v * hold / (2 * math.pi):+.1f} turns), '
                            f'{ramp:g} s ramps, kd {kd:g}, kp 0. The output SPINS.',
                   params=dict(v=v, kd=kd, hold=hold, ramp=ramp, pause=pause, rewind=False))


SCALE_UNITS = {'kg': 9.80665, 'g': 0.00980665, 'lb': 4.4482216, 'N': 1.0}   # -> newtons
STALL_WINDOW = 0.5   # s of feedback before each scale reading that it is compared with


class StallProfile:
    """Locked output: a rod on the output presses on a scale. Feed-forward torque only (kp 0),
    stepped through `levels` (Nm, magnitudes); `sign` is the direction that presses the rod onto
    the scale. Each level is held until the operator types the scale reading (or `max_hold` s
    pass), so this profile is driven by input as well as time: run_profile calls tick() every
    cycle and stops when `done`.

    hold_mode (stall-hold): one level held for `duration` s, readings taken whenever typed, for
    heating and torque drop at a constant load."""

    absolute_targets = False
    hold_rate = 0.0

    def __init__(self, name, levels, legs, sign, kd, arm, unit, travel, max_hold, brake_kd, ramp=0.3,
                 hold_mode=False, duration=0.0, prompt_every=15.0, max_speed=1.5):
        self.name, self.levels, self.legs, self.sign = name, list(levels), list(legs), sign
        self.kd, self.arm, self.unit, self.max_excursion = kd, arm, unit, travel
        # A locked output barely moves, so speed catches a slip within a few ms, before the travel
        # limit; then the drive brakes with brake_kd (the deploy kd), not the gentle idle damping.
        self.max_speed, self.brake_kd = max_speed, brake_kd
        self.max_hold, self.ramp, self.hold_mode = max_hold, ramp, hold_mode
        self.hold_seconds, self.prompt_every = duration, prompt_every
        self.reader, self.log, self.link = None, None, None
        # state
        self.phase, self.i, self.t_phase, self.from_tau = 'ramp', 0, 0.0, 0.0
        self.t_last_reading = 0.0
        self.t_last_iq = -1.0
        self.done = False
        per = duration if hold_mode else max_hold
        self.duration = len(self.levels) * (ramp + per) + ramp + 1.0   # upper bound, for the plan
        top = max(self.levels)
        self.describe = (f'feed-forward torque, kp 0 kd {kd:g}, pressing {"+" if sign > 0 else "-"}: '
                         + (f'{top:g} Nm held {duration:g} s, scale read every ~{prompt_every:g} s'
                            if hold_mode else
                            f'{len(self.levels)} steps up to {top:g} Nm and back, each held until '
                            f'you type the scale reading (max {max_hold:g} s)')
                         + f'. Aborts if the output moves {travel:g} rad (rod slipped)')
        self.params = dict(levels=self.levels, legs=self.legs, sign=sign, kd=kd, arm=arm, unit=unit,
                           travel=travel, max_speed=max_speed, brake_kd=brake_kd, max_hold=max_hold,
                           ramp=ramp, hold_mode=hold_mode,
                           hold_seconds=duration, stall=True)

    # ------------------------------------------------------------------ command
    def level(self, t: float) -> float:
        """Torque magnitude at run time t."""
        if self.phase == 'ramp':
            x = min(max((t - self.t_phase) / self.ramp, 0.0), 1.0)
            return self.from_tau + (self.levels[self.i] - self.from_tau) * x
        if self.phase == 'hold':
            return self.levels[self.i]
        if self.phase == 'end':
            x = min(max((t - self.t_phase) / self.ramp, 0.0), 1.0)
            return self.from_tau * (1.0 - x)
        return 0.0

    def __call__(self, t: float) -> Command:
        return Command(kd=self.kd, tau=self.sign * self.level(t))

    # ------------------------------------------------------------------ operator / state
    def attach(self, reader, log, link):
        self.reader, self.log, self.link = reader, log, link
        log.setdefault('readings', [])
        log.setdefault('iq', [])

    def waiting(self) -> bool:
        return self.phase == 'hold'

    def wants_reading(self, t: float) -> bool:
        """For the simulated operator: when a person would read the scale."""
        if not self.waiting():
            return False
        if self.hold_mode:
            return t - max(self.t_phase, self.t_last_reading) >= self.prompt_every
        return t - self.t_phase >= 1.0

    def window(self, t: float):
        """Mean (drive torque estimate, q, temp, iq) over the STALL_WINDOW before t, within this hold."""
        lo = max(t - STALL_WINDOW, self.t_phase)
        fb = [r for r in self.log['fb'] if lo <= r[0] <= t]
        iq = [v for ti, v in self.log['iq'] if lo <= ti <= t]
        mean = (lambda xs: sum(xs) / len(xs) if xs else float('nan'))
        return (mean([r[4] for r in fb]), mean([r[2] for r in fb]), mean([r[5] for r in fb]), mean(iq))

    def prompt(self):
        lvl = self.levels[self.i]
        if self.hold_mode:
            print(f'  holding {lvl:g} Nm for {self.hold_seconds:g} s. Type the scale reading ({self.unit}) '
                  f'+ Enter whenever you like (about every {self.prompt_every:g} s); q + Enter stops.',
                  flush=True)
        else:
            print(f'  step {self.i + 1}/{len(self.levels)} ({self.legs[self.i]}): {lvl:g} Nm. Scale '
                  f'reading in {self.unit} + Enter (Enter alone skips, q stops):', flush=True)

    def record(self, t: float, value: float):
        lvl = self.levels[self.i]
        tau_est, q, temp, iq = self.window(t)
        true = value * SCALE_UNITS[self.unit] * self.arm
        self.log['readings'].append(dict(t=t, t_from=max(t - STALL_WINDOW, self.t_phase), step=self.i,leg=self.legs[self.i], tau_cmd=lvl,
                                         reading=value, unit=self.unit, tau_true=true,
                                         tau_drive=abs(tau_est), iq=abs(iq), temp=temp, q=q))
        self.t_last_reading = t
        print(f'    {lvl:g} Nm commanded -> scale {value:g} {self.unit} = {true:.2f} Nm at the output '
              f'({true / lvl:.3f}x); drive estimate {abs(tau_est):.2f} Nm, iq {abs(iq):.2f} A, '
              f'{temp:.1f} C', flush=True)

    def next_level(self, t: float):
        self.from_tau = self.level(t)
        self.i += 1
        if self.i >= len(self.levels):
            self.phase, self.t_phase = 'end', t
        else:
            self.phase, self.t_phase = 'ramp', t

    def stop(self, t: float):
        self.from_tau = self.level(t)
        self.phase, self.t_phase = 'end', t

    def tick(self, t: float):
        # q-axis current at 20 Hz, answered between status replies (on_other_frame)
        if self.link is not None and t - self.t_last_iq >= 0.05:
            self.link.send(rs.read_param(self.link.id, rs.P_IQ))
            self.t_last_iq = t
        if self.phase == 'ramp' and t - self.t_phase >= self.ramp:
            if self.reader:
                self.reader.poll(self, t)   # drop anything typed during the ramp: it was the old level
            self.phase, self.t_phase, self.t_last_reading = 'hold', t, t
            self.prompt()
        elif self.phase == 'hold':
            for line in self.reader.poll(self, t) if self.reader else []:
                line = line.strip()
                if line.lower() in ('q', 'quit', 'stop'):
                    print('  stopping: ramping the torque down', flush=True)
                    self.stop(t)
                    return
                if not line:
                    if not self.hold_mode:
                        print('    skipped', flush=True)
                        self.next_level(t)
                        return
                    continue
                try:
                    value = float(line)
                except ValueError:
                    print(f'    "{line}" is not a number; type the reading in {self.unit}', flush=True)
                    continue
                if value < 0:
                    print('    a reading below 0: was the scale tared with the motor off?', flush=True)
                self.record(t, value)
                if not self.hold_mode:
                    self.next_level(t)
                    return
            held = t - self.t_phase
            if self.hold_mode and held >= self.hold_seconds:
                self.next_level(t)
            elif not self.hold_mode and held >= self.max_hold:
                print(f'    no reading after {self.max_hold:g} s: moving on', flush=True)
                self.next_level(t)
        elif self.phase == 'end' and t - self.t_phase >= self.ramp:
            self.phase, self.done = 'done', True

    def on_other_frame(self, fr, t0: float):
        cid, data, t_rx = fr
        if rs.comm_type(cid) != rs.T_READ or self.link is None:
            return
        mid, idx, val = rs.decode_param_reply(cid, data)
        if mid == self.link.id and idx == rs.P_IQ:
            self.log['iq'].append((t_rx - t0, val))


class StdinReader:
    """Lines the operator typed, without blocking the control loop (POSIX: select on stdin)."""

    def poll(self, prof, t):
        import select
        lines = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline()
            if line == '':      # EOF
                break
            lines.append(line)
        return lines


class SimScale:
    """The simulated operator: reads the simulated scale ~1 s into each step."""

    def __init__(self, motor, arm, unit, noise=0.003, seed=1):
        self.motor, self.arm, self.unit = motor, arm, unit
        self.rng = random.Random(seed)
        self.noise = noise

    def poll(self, prof, t):
        if not prof.wants_reading(t):
            return []
        newton = self.motor.wall_force / self.arm
        return [f'{newton / SCALE_UNITS[self.unit] * (1 + self.rng.gauss(0, self.noise)):.4f}']


def stall_profile(test: str, a, model: str):
    sign = 1.0 if a.dir in ('+', 'pos', 'positive') else -1.0
    kd = a.kd if a.kd is not None else a.damping_kd
    brake = DEPLOY_GAINS[model][1]
    top = a.stall_max
    if test == 'stall-hold':
        return StallProfile('stall-hold', [top], ['hold'], sign, kd, a.arm, a.scale_unit, a.stall_travel,
                            a.max_hold, brake, hold_mode=True, duration=a.duration or 120.0,
                            prompt_every=a.prompt_every)
    n = a.levels
    up = [round(top * k / n, 3) for k in range(1, n + 1)]
    down = up[-2::-1]
    return StallProfile('stall', up + down, ['up'] * n + ['down'] * (n - 1), sign, kd, a.arm,
                        a.scale_unit, a.stall_travel, a.max_hold, brake)


def itertools_accumulate(xs):
    total = 0.0
    for x in xs:
        total += x
        yield total


def build_profile(test: str, model: str, a) -> Profile:
    kp_dep, kd_dep = DEPLOY_GAINS[model]
    kp = a.kp if a.kp is not None else None
    kd = a.kd if a.kd is not None else None
    if test == 'friction':
        return friction_profile(a.speeds, a.hold or 2.0, kd if kd is not None else FRICTION_KD[model])
    if test == 'torque-chirp':
        return torque_chirp_profile(a.f0 or 2.0, a.f1 or 40.0, a.duration or 30.0, a.amp or 0.2,
                                    a.j_guess or J_GUESS[model], 0.8 * a.torque_limit,
                                    kp if kp is not None else 0.5, kd if kd is not None else 0.0)
    if test.startswith('pos-chirp'):
        hold_rate = a.hold_rate if a.hold_rate is not None else (50.0 if test == 'pos-chirp@50' else 0.0)
        return pos_chirp_profile(a.f0 or 0.2, a.f1 or 20.0, a.duration or 40.0, a.amp or 0.2,
                                 a.corner, kp if kp is not None else kp_dep,
                                 kd if kd is not None else kd_dep, hold_rate)
    if test == 'step':
        return step_profile(a.amps, a.hold or 1.0, kp if kp is not None else kp_dep,
                            kd if kd is not None else kd_dep)
    if test == 'top-speed':
        r = rs.RANGES[model]
        vmax = a.vmax or min(0.98 * r.velocity_max, 1.25 * NO_LOAD_48V[model] * a.bus_voltage / 48.0)
        return top_speed_profile(vmax, kd if kd is not None else TOP_SPEED_KD[model])
    if test == 'spin':
        sign = 1.0 if a.dir in ('+', 'pos', 'positive') else -1.0
        return spin_profile(sign * a.speed, kd if kd is not None else TOP_SPEED_KD[model],
                            a.duration or 5.0, a.ramp)
    if test in STALL_TESTS:
        return stall_profile(test, a, model)
    raise ValueError(test)


def spin_speed_cap(model: str, bus_voltage: float) -> float:
    """Highest speed `spin` accepts: just above no-load at the bus voltage, inside the wire range."""
    return min(0.98 * rs.RANGES[model].velocity_max, 1.1 * NO_LOAD_48V[model] * bus_voltage / 48.0)


# ============================================================================= clocks and buses
class RealClock:
    def now(self) -> float:
        return time.perf_counter()

    def sleep(self, s: float) -> None:
        if s > 0:
            time.sleep(s)


class SocketCanBus:
    """Raw SocketCAN, non-blocking, receive timestamps taken the moment a frame is read."""

    def __init__(self, iface: str, clock):
        if not hasattr(socket, 'AF_CAN'):
            raise SetupError('SocketCAN needs Linux (use --sim to preview elsewhere)')
        self.clock = clock
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.bind((iface,))
        except OSError as e:
            raise SetupError(f'cannot open CAN interface {iface!r}: {e}. '
                             f'Check `ip -details link show {iface}`.') from e
        self.sock.setblocking(False)

    def send(self, cid: int, data: bytes) -> None:
        frame = struct.pack('=IB3x8s', cid | socket.CAN_EFF_FLAG, 8, bytes(data).ljust(8, b'\0'))
        for _ in range(5):
            try:
                self.sock.send(frame)
                return
            except BlockingIOError:
                pass
            except OSError as e:
                if e.errno != errno.ENOBUFS:
                    raise
            time.sleep(0.001)
        raise Abort('CAN transmit queue full: nothing acknowledges frames '
                    '(motor power, ground wire, CAN_H/CAN_L?)')

    def recv(self, timeout: float):
        import select
        ready, _, _ = select.select([self.sock], [], [], max(timeout, 0.0))
        if not ready:
            return None
        try:
            frame = self.sock.recv(16)
        except BlockingIOError:
            return None
        t = self.clock.now()
        cid, dlc, data = struct.unpack('=IB3x8s', frame)
        if not cid & socket.CAN_EFF_FLAG:
            return None
        return cid & socket.CAN_EFF_MASK, data[:dlc], t

    def close(self):
        self.sock.close()


class SimClock:
    def __init__(self):
        self.t = 1000.0

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += max(s, 0.0)


class SimMotor:
    """One simulated RobStride motor with nothing attached: J dv/dt = tau - b v - Fc tanh(v/vs).
    The drive's PD runs every `dt`; commands take effect `delay` s after they are sent; replies
    leave `reply_latency` s after a command. Used by --sim and by the tests."""

    def __init__(self, motor_id, model, J=0.07, b=0.01, Fc=0.15, delay=0.003, vbus=36.4,
                 reply_latency=0.0006, tau_noise=0.01, temp=32.0, q0=0.3, seed=0,
                 tau_gain=1.0, kt=2.0, wall_k=0.0, wall_c=20.0, wall_dir=0.0):
        self.id, self.r = motor_id, rs.RANGES[model]
        self.J, self.b, self.Fc, self.delay = J, b, Fc, delay
        # Locked-output tests: output torque = tau_gain x what the drive thinks it applies; iq =
        # output torque / kt. A one-sided spring (the rod on the scale) at wall_q resists motion in
        # wall_dir; wall_force is the torque it carries (what the scale sees).
        self.tau_gain, self.kt = tau_gain, kt
        self.wall_k, self.wall_c, self.wall_dir, self.wall_q = wall_k, wall_c, wall_dir, None
        self.wall_force = 0.0
        self.wall_off_at = None              # inject: the rod slips off the scale at this time
        self.vnl = NO_LOAD_48V[model] * vbus / 48.0
        self.vbus, self.reply_latency, self.tau_noise, self.temp = vbus, reply_latency, tau_noise, temp
        self.q, self.v, self.tau, self.t = q0, 0.0, 0.0, None
        self.enabled, self.mode, self.faults = False, rs.MODE_RESET, 0
        self.torque_limit = self.r.torque_max
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0)
        self.pending = collections.deque()   # (t_apply, cmd)
        self.outbox = []                     # (t_ready, kind, payload)
        self.rng = random.Random(seed)
        self.mute = False
        self.mute_at = None                  # inject: stop replying from this time on
        self.received = []                   # comm types the motor got, in order
        self.commands = []                   # decoded MIT commands (q, v, kp, kd, tau)
        self.fault_at = None                 # inject: (time, fault bits)
        self.drop_writes = set()             # inject: parameter writes the drive loses
        self.bus_down_at = None              # inject: the host's CAN interface goes down
        self.unreadable = set()              # inject: parameters that read back as 0 (RS06 0x7028)
        self.last_rx = None                  # time of the last frame for this motor (CAN timeout)
        self.written = {}                    # parameter index -> raw value bytes
        self.dt = 1e-4

    def advance(self, t: float) -> None:
        if self.t is None:
            self.t = t
        while self.t < t:
            while self.pending and self.pending[0][0] <= self.t:
                self.cmd = self.pending.popleft()[1]
            if self.fault_at and self.t >= self.fault_at[0]:
                self.faults, self.enabled, self.mode = self.fault_at[1], False, rs.MODE_RESET
            to = self.can_timeout()
            if self.enabled and to and self.last_rx is not None and self.t - self.last_rx > to:
                self.enabled, self.mode = False, rs.MODE_RESET
            if self.enabled:
                q_ref, v_ref, kp, kd, tff = self.cmd
                tau = kp * (q_ref - self.q) + kd * (v_ref - self.v) + tff
                lim = min(self.torque_limit, self.r.torque_max)
                hi = min(lim, self.r.torque_max * (1 - self.v / self.vnl))   # back-EMF line
                lo = max(-lim, self.r.torque_max * (-1 - self.v / self.vnl))
                tau = min(max(tau, lo), hi)
            else:
                tau = 0.0
            self.tau = tau
            if self.wall_off_at is not None and self.t >= self.wall_off_at:
                self.wall_k = 0.0
            tw = 0.0
            if self.wall_k > 0 and self.wall_q is not None:
                pen = self.wall_dir * (self.q - self.wall_q)
                f = max(0.0, self.wall_k * pen + self.wall_c * self.wall_dir * self.v) if pen > 0 else 0.0
                tw = -self.wall_dir * f
                self.wall_force = f
            else:
                self.wall_force = 0.0
            h = min(self.dt, t - self.t)
            a = (self.tau_gain * tau + tw - self.b * self.v - self.Fc * math.tanh(self.v / 0.01)) / self.J
            self.v += h * a
            self.q += h * self.v
            self.t += h

    def can_timeout(self) -> float:
        raw = self.written.get(rs.P_CAN_TIMEOUT)
        return struct.unpack('<I', raw)[0] / 20000.0 if raw else 0.0

    def on_frame(self, t: float, cid: int, data: bytes) -> None:
        self.advance(t)
        if cid & 0xFF != self.id:  # target ID
            return
        self.last_rx = t
        ct = rs.comm_type(cid)
        self.received.append(ct)
        if ct == rs.T_MOTION:
            r = self.r
            raw = struct.unpack('>4H', data)
            cmd = (rs.u16_decode(raw[0], r.position_max), rs.u16_decode(raw[1], r.velocity_max),
                   raw[2] * r.kp_max / 65535.0, raw[3] * r.kd_max / 65535.0,
                   rs.u16_decode(cid >> 8 & 0xFFFF, r.torque_max))
            self.pending.append((t + self.delay, cmd))
            self.commands.append(cmd)
            self.outbox.append((t + self.reply_latency, 'status', self.mode))  # mode as handled
        elif ct == rs.T_STOP:
            self.enabled, self.mode = False, rs.MODE_RESET
            if data[0] == 1:
                self.faults = 0
            self.outbox.append((t + self.reply_latency, 'status', self.mode))  # mode as handled
        elif ct == rs.T_ENABLE:
            if not self.faults:
                self.enabled, self.mode = True, rs.MODE_RUNNING
                self.cmd = (self.q, 0.0, 0.0, 0.0, 0.0)
            self.outbox.append((t + self.reply_latency, 'status', self.mode))  # mode as handled
        elif ct == rs.T_READ:
            index = struct.unpack('<H', data[:2])[0]
            val = {rs.P_VBUS: self.vbus, rs.P_MECH_POS: self.q, rs.P_MECH_VEL: self.v,
                   rs.P_TORQUE_LIMIT: self.torque_limit,
                   rs.P_IQ: self.tau_gain * self.tau / self.kt}.get(index, self.written.get(index, 0.0))
            if index in self.unreadable:
                val = 0.0
            self.outbox.append((t + self.reply_latency, 'param', (index, val)))
        elif ct == rs.T_WRITE:
            index = struct.unpack('<H', data[:2])[0]
            if index in self.drop_writes:
                return
            self.written[index] = bytes(data[4:8])
            if index == rs.P_TORQUE_LIMIT:
                self.torque_limit = struct.unpack('<f', data[4:8])[0]
        self.outbox.sort(key=lambda e: e[0])

    def next_event(self):
        return self.outbox[0][0] if self.outbox else None

    def pop_event(self):
        t, kind, payload = self.outbox.pop(0)
        self.advance(t)
        if self.mute or (self.mute_at is not None and t >= self.mute_at):
            return None
        if kind == 'status':
            mode = self.mode if payload is None else min(payload, self.mode)  # a later reset wins
            st = rs.Status(mode, self.faults, self.q, self.v,
                           self.tau + self.rng.gauss(0.0, self.tau_noise), self.temp)
            cid, data = rs.status_frame(self.id, st, self.r)
        else:
            cid, data = rs.param_reply(self.id, *payload)
        return cid, data, t


class SimBus:
    def __init__(self, motor: SimMotor, clock: SimClock):
        self.motor, self.clock = motor, clock
        self.sent = []

    def send(self, cid: int, data: bytes) -> None:
        down = self.motor.bus_down_at
        if down is not None and self.clock.now() >= down:
            raise OSError(100, 'Network is down')
        self.sent.append((self.clock.now(), cid, bytes(data)))
        self.motor.on_frame(self.clock.now(), cid, bytes(data))

    def recv(self, timeout: float):
        deadline = self.clock.now() + max(timeout, 0.0)
        while True:
            te = self.motor.next_event()
            if te is None or te > deadline:
                self.clock.t = max(self.clock.t, deadline)
                return None
            self.clock.t = max(self.clock.t, te)
            frame = self.motor.pop_event()
            if frame is not None:
                return frame

    def close(self):
        pass


class SimMultiBus:
    """Several simulated motors on one bus (mit_all.py and monitor.py with --sim)."""

    def __init__(self, motors, clock: SimClock):
        self.motors, self.clock = list(motors), clock
        self.sent = []

    def send(self, cid: int, data: bytes) -> None:
        self.sent.append((self.clock.now(), cid, bytes(data)))
        for m in self.motors:
            m.on_frame(self.clock.now(), cid, bytes(data))

    def recv(self, timeout: float):
        deadline = self.clock.now() + max(timeout, 0.0)
        while True:
            pending = [(m.next_event(), i) for i, m in enumerate(self.motors) if m.next_event() is not None]
            if not pending or min(pending)[0] > deadline:
                self.clock.t = max(self.clock.t, deadline)
                return None
            te, i = min(pending)
            self.clock.t = max(self.clock.t, te)
            frame = self.motors[i].pop_event()
            if frame is not None:
                return frame

    def close(self):
        pass


def wait_frames(buses, timeout: float):
    """Frames from any of `buses` (one per CAN adapter): returns as soon as one has something,
    with everything that bus has queued, or [] after `timeout`."""
    if len(buses) == 1 or not all(hasattr(b, 'sock') for b in buses):
        fr = buses[0].recv(timeout)
        return [fr] if fr else []
    import select
    socks = {b.sock: b for b in buses}
    ready, _, _ = select.select(list(socks), [], [], max(timeout, 0.0))
    out = []
    for s in ready:
        while fr := socks[s].recv(0.0):
            out.append(fr)
    return out


# ============================================================================= motor link
class Link:
    def __init__(self, bus, clock, motor_id: int, model: str):
        self.bus, self.clock, self.id, self.r = bus, clock, motor_id, rs.RANGES[model]
        self.fault_reports = []

    def send(self, frame) -> None:
        self.bus.send(*frame)

    def read_param_raw(self, index: int, timeout: float = 0.2):
        """-> the 4 raw value bytes, or None."""
        self.send(rs.read_param(self.id, index))
        end = self.clock.now() + timeout
        while (left := end - self.clock.now()) > 0:
            fr = self.bus.recv(left)
            if fr and rs.comm_type(fr[0]) == rs.T_READ:
                mid, idx, _ = rs.decode_param_reply(fr[0], fr[1])
                if mid == self.id and idx == index:
                    return bytes(fr[1][4:8])
        return None

    def read_param(self, index: int, timeout: float = 0.2):
        raw = self.read_param_raw(index, timeout)
        return None if raw is None else struct.unpack('<f', raw)[0]

    def write(self, frame) -> None:
        self.send(frame)
        self.clock.sleep(0.02)

    def command(self, q, v, kp, kd, tau) -> None:
        self.send(rs.motion(self.id, q, v, kp, kd, tau, self.r))

    def classify(self, fr):
        """-> ('status', Status, t, ctype) | ('fault', bytes, t) | None for other traffic."""
        cid, data, t = fr
        ct = rs.comm_type(cid)
        if rs.frame_motor(cid) != self.id:
            return None
        if ct in (rs.T_STATUS, rs.T_ACTIVE):
            return 'status', rs.decode_status(cid, data, self.r), t, ct
        if ct == rs.T_FAULT:
            return 'fault', bytes(data), t
        return None

    def fresh_exchange(self, q, v, kp, kd, tau, timeout=0.05):
        """exchange(), after dropping queued replies, so the status answers this command."""
        while self.bus.recv(0.0):
            pass
        return self.exchange(q, v, kp, kd, tau, timeout)

    def exchange(self, q, v, kp, kd, tau, timeout=0.05):
        self.command(q, v, kp, kd, tau)
        end = self.clock.now() + timeout
        while (left := end - self.clock.now()) > 0:
            fr = self.bus.recv(left)
            c = fr and self.classify(fr)
            if c and c[0] == 'status':
                return c[1]
        return None


# ============================================================================= run
@dataclass
class Session:
    q_start: float
    vbus_start: float | None
    mech_pos: float | None
    temp_start: float


def timeout_trips(link: Link, a) -> bool:
    """Enable with damping only, stay silent for 1.5x the CAN timeout, then check the drive has
    left run mode by itself. The caller stops the drive either way."""
    link.write(rs.enable(link.id))
    st = link.fresh_exchange(0.0, 0.0, 0.0, a.damping_kd, 0.0)
    if st is None or st.mode != rs.MODE_RUNNING:
        return False
    link.clock.sleep(1.5 * a.can_timeout)
    st = link.fresh_exchange(0.0, 0.0, 0.0, a.damping_kd, 0.0)
    return st is not None and st.mode != rs.MODE_RUNNING


def position_problem(q: float, mech: float, r: rs.Ranges, margin: float = 1.5) -> str | None:
    """Why absolute MIT position targets are unsafe from this angle, or None. The MIT angle must be
    the true angle and well inside the +/-4 pi wire range."""
    if abs(q - mech) > 0.05:
        return (f'MIT position {q:.3f} != mechPos {mech:.3f}: wrong model/ranges, or the output is '
                f'outside the +/-4 pi MIT range after a spinning test')
    if abs(q) > r.position_max - margin:
        return f'start angle {q:.2f} rad is too close to the +/-4 pi MIT range'
    return None


def start(link: Link, a, needs_position: bool) -> Session:
    vbus, mech = configure(link, a)
    link.write(rs.enable(link.id))
    # Replies to the setup frames (mode 0) may still be queued; this one must answer the enable.
    st = link.fresh_exchange(0.0, 0.0, 0.0, a.damping_kd, 0.0)
    if st is None:
        raise SetupError('no status reply to the first MIT command')
    if st.faults or st.mode != rs.MODE_RUNNING:
        raise SetupError(f'motor not running after enable: mode {st.mode}, faults {st.fault_names()}')
    # Position tests send absolute MIT targets. Velocity tests (kp = 0) don't care.
    if needs_position and (why := position_problem(st.q, mech, link.r)):
        raise SetupError(f'{why}. Run any test with --rewind-only (turns it back to 0), or '
                         f'power-cycle the motor.')
    return Session(st.q, vbus, mech, st.temp)


def configure(link: Link, a):
    """Everything before enabling: check the motor answers, MIT mode, the drive's torque limit and
    CAN timeout (both read back, or the timeout checked by behaviour). Leaves the drive stopped.
    Returns (bus voltage, mechPos)."""
    vbus = link.read_param(rs.P_VBUS)
    mech = link.read_param(rs.P_MECH_POS)
    if vbus is None or mech is None:
        raise SetupError(f'motor {link.id} does not answer parameter reads. Is it powered, at this '
                         f'ID (find_motor_id.sh), and is every motor node stopped?')
    link.write(rs.stop(link.id, clear_faults=True))
    link.write(rs.write_u8(link.id, rs.P_RUN_MODE, rs.RUN_MODE_MIT))
    link.write(rs.write_f32(link.id, rs.P_TORQUE_LIMIT, a.torque_limit))
    link.write(rs.write_u32(link.id, rs.P_CAN_TIMEOUT, rs.can_timeout_counts(a.can_timeout)))
    link.write(rs.active_report(link.id, False))  # every command gets a reply; no extra traffic
    # A drive sent frames faster than it handles them drops some silently (seen on an RS03), so
    # confirm the two safety settings actually landed before enabling.
    tl = link.read_param(rs.P_TORQUE_LIMIT)
    to = link.read_param_raw(rs.P_CAN_TIMEOUT)
    want_to = rs.can_timeout_counts(a.can_timeout)
    if tl is None or abs(tl - a.torque_limit) > 1e-3:
        raise SetupError(f'torque limit reads back {tl} instead of {a.torque_limit:g} Nm; not enabling')
    got = None if to is None else struct.unpack('<I', to)[0]
    if got != want_to:
        # Some firmware reads this register back as 0 although the write takes effect (RS06 m25).
        # Then check the behaviour instead: it must drop out of run mode on its own.
        if not timeout_trips(link, a):
            raise SetupError(f'CAN timeout reads back {got} instead of {want_to} counts, and the drive '
                             f'kept running through {1.5 * a.can_timeout:g} s of silence; not enabling')
        print(f'  (CAN timeout reads back {got}; checked by behaviour instead: the drive stopped '
              f'itself after {1.5 * a.can_timeout:g} s of silence)', flush=True)
        link.write(rs.stop(link.id, clear_faults=True))
    return vbus, mech


def rewind(link: Link, clock, a, target: float, speed: float = 2.0, timeout: float = 30.0) -> bool:
    """After a spinning test: turn the output back to `target` with kd-only velocity control,
    reading the true multi-turn angle (mechPos) every cycle. Returns True when within 0.1 rad."""
    kd = FRICTION_KD[a.model]
    end = clock.now() + timeout
    while clock.now() < end:
        m = link.read_param(rs.P_MECH_POS, timeout=0.05)
        if m is None:
            return False
        err = target - m
        if abs(err) < 0.1:
            return True
        # At least 1 Nm of kd drive: a slower v_ref can sit below breakaway friction and stall
        # short of the target (RS03 m21 needs ~0.57 Nm to break away).
        v = math.copysign(min(speed, max(abs(3.0 * err), 1.0 / kd)), err)
        st = link.exchange(0.0, v, 0.0, kd, 0.0, timeout=0.02)
        if st is None or st.faults or st.mode != rs.MODE_RUNNING:
            return False
        clock.sleep(0.01)
    return False


def run_profile(link: Link, clock, prof: Profile, sess: Session, a, log):
    """Fixed-rate loop. Returns the abort reason, or None if the profile completed."""
    period = 1.0 / a.rate
    q0 = sess.q_start
    while link.bus.recv(0.0):  # replies to the setup frames; not part of the run
        pass
    t0 = clock.now()
    log['t0'] = t0
    next_tick = t0
    last_fb = t0
    reason = None
    try:
        tick = getattr(prof, 'tick', None)              # input-driven profiles (stall)
        other = getattr(prof, 'on_other_frame', None)   # parameter replies they asked for
        while True:
            now = clock.now()
            t = now - t0
            if tick:
                tick(t)
                if prof.done:
                    break
            elif t >= prof.duration:
                break
            c = prof(t)
            q_ref = q0 + c.dq
            link.command(q_ref, c.v, c.kp, c.kd, c.tau)
            t_send = clock.now() - t0
            log['cmd'].append((t_send, q_ref, c.v, c.kp, c.kd, c.tau))
            if len(log['cmd']) > 1:
                log['periods'].append(t_send - log['cmd'][-2][0])
            next_tick += period
            while (left := next_tick - clock.now()) > 0:
                fr = link.bus.recv(left)
                if not fr:
                    continue
                cl = link.classify(fr)
                if not cl:
                    if other:
                        other(fr, t0)
                    continue
                if cl[0] == 'fault':
                    raise Abort(f'fault report from the motor: {cl[1].hex()}')
                _, st, t_rx, ct = cl
                last_fb = t_rx
                log['fb'].append((t_rx - t0, ct, st.q, st.v, st.tau, st.temp, st.mode, st.faults))
                if st.faults:
                    raise Abort(f'motor faults: {st.fault_names()}')
                if st.mode != rs.MODE_RUNNING:
                    raise Abort(f'motor left running mode (mode {st.mode}): CAN timeout or a fault')
                if st.temp > a.max_temp:
                    raise Abort(f'temperature {st.temp:.1f} C > {a.max_temp:g} C')
                if tick and abs(st.v) > prof.max_speed:
                    raise Abort(f'output turning at {st.v:+.2f} rad/s (limit {prof.max_speed:g}): the rod '
                                f'slipped off the scale, or --dir lifts it instead of pressing it down')
                if prof.max_excursion is not None and abs(st.q - q0) > prof.max_excursion:
                    if tick:
                        raise Abort(f'moved {st.q - q0:+.3f} rad from the start (limit '
                                    f'{prof.max_excursion:g}): the rod slipped off the scale, or '
                                    f'--dir lifts it instead of pressing it down')
                    raise Abort(f'moved {st.q - q0:+.2f} rad from the start (limit '
                                f'{prof.max_excursion:g}): gain scaling or a profile problem')
            if clock.now() - last_fb > a.feedback_timeout:
                raise Abort(f'no status reply for {a.feedback_timeout * 1e3:.0f} ms')
            late = clock.now() - next_tick
            if late > period:
                log['overruns'] += 1
                next_tick = clock.now()
    except Abort as e:
        reason = str(e)
    except KeyboardInterrupt:
        reason = 'interrupted (Ctrl-C / SIGTERM)'
    return reason


def finish(link: Link, clock, a, kd: float | None = None) -> float | None:
    """Damping (a.damping_kd, or `kd`) for 0.5 s, then stop. Returns the last temperature seen."""
    temp = None
    kd = a.damping_kd if kd is None else kd
    end = clock.now() + 0.5
    try:
        while clock.now() < end:
            st = link.exchange(0.0, 0.0, 0.0, kd, 0.0, timeout=0.01)
            if st:
                temp = st.temp
            clock.sleep(0.01)
    except (Abort, OSError):
        pass
    finally:
        try:
            link.send(rs.stop(link.id, clear_faults=False))
        except Exception:  # noqa: BLE001  the motor-side CAN timeout still stops it
            pass
    return temp


def git_rev():
    try:
        return subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True,
                              cwd=os.path.dirname(os.path.abspath(__file__)), timeout=2).stdout.strip()
    except Exception:  # noqa: BLE001
        return ''


def write_logs(out_dir, meta, log):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'cmd.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'q_ref', 'v_ref', 'kp', 'kd', 'tau_ff'])
        w.writerows([[f'{x:.6f}' if isinstance(x, float) else x for x in row] for row in log['cmd']])
    with open(os.path.join(out_dir, 'fb.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'type', 'q', 'v', 'tau', 'temp', 'mode', 'faults'])
        w.writerows([[f'{x:.6f}' if isinstance(x, float) else x for x in row] for row in log['fb']])
    if log.get('iq'):
        with open(os.path.join(out_dir, 'iq.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'iq'])
            w.writerows([[f'{t:.6f}', f'{v:.4f}'] for t, v in log['iq']])
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)


def percentile(xs, p):
    if not xs:
        return float('nan')
    s = sorted(xs)
    return s[min(len(s) - 1, int(p / 100.0 * len(s)))]


def report_spin(prof: Profile, log) -> None:
    """Measured speed and torque over the middle of the constant-velocity hold."""
    p = prof.params
    t0 = log.get('t0')
    if t0 is None or not log['fb']:
        return
    lo, hi = p['ramp'] + 0.25 * p['hold'], p['ramp'] + 0.9 * p['hold']   # fb times are run-relative
    rows = [r for r in log['fb'] if lo <= r[0] <= hi]
    if not rows:
        return
    v = sum(r[3] for r in rows) / len(rows)
    tau = sum(r[4] for r in rows) / len(rows)
    # Integrate velocity: the MIT position wraps at +/-4 pi, so end - start is wrong after ~2 turns.
    fb = log['fb']
    turns = sum(b[3] * (b[0] - a[0]) for a, b in zip(fb, fb[1:])) / (2 * math.pi)
    print(f'[spin] commanded {p["v"]:+.2f} rad/s, measured {v:+.2f} rad/s ({100 * v / p["v"]:.0f}%), '
          f'torque {tau:+.3f} Nm (= friction at this speed), {turns:+.2f} turns in total', flush=True)


def one_run(test: str, a, model: str, make_link) -> bool:
    prof = build_profile(test, model, a)
    bus, clock, link, sim_motor = make_link()
    log = {'cmd': [], 'fb': [], 'periods': [], 'overruns': 0}
    stamp = time.strftime('%Y-%m-%d_%H%M%S')
    out_dir = os.path.join(a.out, f'{stamp}_m{a.id}_{prof.name.replace("@", "_zoh")}')
    meta = dict(motor_id=a.id, model=model, iface=a.iface, sim=a.sim, test=prof.name,
                profile=prof.params, describe=prof.describe, duration=prof.duration, rate=a.rate,
                torque_limit=a.torque_limit, damping_kd=a.damping_kd, can_timeout=a.can_timeout,
                max_temp=a.max_temp,
                ranges=rs.RANGES[model].__dict__, date=stamp, host=platform.node(), git=git_rev(),
                note=a.note)
    if sim_motor is not None:
        meta['sim_truth'] = dict(J=sim_motor.J, b=sim_motor.b, Fc=sim_motor.Fc, delay=sim_motor.delay)
    if isinstance(prof, StallProfile):
        if sim_motor is not None:
            # The rod rests on the scale where the output is now; pressing `sign` compresses it.
            # (Unless a test hook placed the scale already, e.g. on the wrong side.)
            if sim_motor.wall_q is None:
                sim_motor.wall_q, sim_motor.wall_dir = sim_motor.q, prof.sign
                sim_motor.wall_k = sim_motor.wall_k or 2000.0
            meta['sim_truth'].update(tau_gain=sim_motor.tau_gain, kt=sim_motor.kt)
            reader = SimScale(sim_motor, prof.arm, prof.unit)
        else:
            reader = StdinReader()
        prof.attach(reader, log, link)
    reason = None
    try:
        sess = start(link, a, needs_position=prof.max_excursion is not None and prof.absolute_targets)
        meta.update(q_start=sess.q_start, vbus_start=sess.vbus_start, mech_pos_start=sess.mech_pos,
                    temp_start=sess.temp_start)
        print(f'[{prof.name}] start angle {sess.q_start:+.3f} rad, bus {sess.vbus_start:.1f} V, '
              f'{sess.temp_start:.1f} C. Running {prof.duration:.0f} s ...', flush=True)
        reason = run_profile(link, clock, prof, sess, a, log)
        if prof.name == 'spin':
            report_spin(prof, log)
        if prof.max_excursion is None and reason is None and prof.params.get('rewind', True):
            ok = rewind(link, clock, a, target=0.0)
            meta['rewound'] = ok
            print(f'[{prof.name}] rewind to 0 rad: '
                  + ('ok' if ok else 'FAILED (power-cycle before a position test)'))
    except SetupError as e:
        reason = f'setup: {e}'
    except Abort as e:
        reason = str(e)
    except OSError as e:  # the adapter dropped off USB, or the interface went down
        reason = f'CAN interface error: {e}'
        print(f'[{prof.name}] {reason}. If "Network is down": the USB-CAN adapter reconnected (check '
              f'dmesg), so bring it up with  sudo ip link set {a.iface} up type can bitrate 1000000. '
              f'The motor stops itself after the {a.can_timeout:g} s CAN timeout.', flush=True)
    finally:
        temp_end = finish(link, clock, a, kd=getattr(prof, 'brake_kd', None))
        try:
            meta['vbus_end'] = link.read_param(rs.P_VBUS)
        except Exception:  # noqa: BLE001
            meta['vbus_end'] = None
        bus.close()
    per = log['periods']
    if 'readings' in log:
        meta['readings'] = log['readings']
    meta.update(aborted=reason, completed=reason is None, temp_end=temp_end,
                n_cmd=len(log['cmd']), n_fb=len(log['fb']), overruns=log['overruns'],
                period_ms=dict(p50=percentile(per, 50) * 1e3, p99=percentile(per, 99) * 1e3,
                               max=(max(per) * 1e3 if per else float('nan'))))
    if log['cmd'] or reason is None:
        write_logs(out_dir, meta, log)
    pm = meta['period_ms']
    nan = float('nan')
    ratio = len(log['fb']) / max(len(log['cmd']), 1)
    print(f'[{prof.name}] {"DONE" if reason is None else "ABORTED: " + reason}\n'
          f'  {len(log["cmd"])} commands, {len(log["fb"])} replies ({ratio:.0%}), period p50 '
          f'{pm["p50"]:.2f} ms p99 {pm["p99"]:.2f} ms max {pm["max"]:.2f} ms, {log["overruns"]} overruns\n'
          f'  temp {meta.get("temp_start", nan):.1f} -> {nan if temp_end is None else temp_end:.1f} C'
          + (f', logs in {out_dir}' if log['cmd'] else ''), flush=True)
    if reason is None and (pm['p99'] > 2.5 / a.rate * 1e3 or ratio < 0.9):
        print('  WARNING: loop timing is poor (p99 period > 2.5x nominal or <90% replies). Try a lower '
              '--rate, `sudo chrt -f 50 python3 ...`, or check bus load with `canbusload`.')
    return reason is None


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('test', choices=['friction', 'torque-chirp', 'pos-chirp', 'step', 'top-speed', 'spin',
                                     'all', *STALL_TESTS])
    ap.add_argument('--id', type=int, required=True, help='motor CAN ID')
    ap.add_argument('--model', choices=sorted(rs.RANGES), help='default: from the ID (11-15, 21-25)')
    ap.add_argument('--iface', default='auto', help='CAN interface; auto = the one the motor answers on')
    ap.add_argument('--out', default='sysid_logs')
    ap.add_argument('--rate', type=float, default=500.0, help='command rate, Hz')
    ap.add_argument('--torque-limit', type=float, help='Nm, written to the drive. Default ~20%% of peak')
    ap.add_argument('--damping-kd', type=float, default=0.3, help='kd while idle and at the end')
    ap.add_argument('--can-timeout', type=float, default=0.2, help='motor-side watchdog, s')
    ap.add_argument('--max-temp', type=float, default=70.0)
    ap.add_argument('--feedback-timeout', type=float, default=0.05, help='s without a reply -> abort')
    ap.add_argument('--bus-voltage', type=float, default=36.0, help='nominal V, sizes top-speed')
    ap.add_argument('--kp', type=float)
    ap.add_argument('--kd', type=float)
    ap.add_argument('--f0', type=float)
    ap.add_argument('--f1', type=float)
    ap.add_argument('--duration', type=float)
    ap.add_argument('--amp', type=float, help='rad (chirps)')
    ap.add_argument('--j-guess', type=float,
                    help='torque-chirp: total inertia it is sized for, kg m^2 (motor + any load). '
                         'Default: the measured motor value for the model')
    ap.add_argument('--corner', type=float, default=5.0, help='pos-chirp amplitude roll-off, Hz')
    ap.add_argument('--hold-rate', type=float, help='pos-chirp: hold targets at this rate (policy: 50)')
    ap.add_argument('--hold', type=float, help='s per level (friction, step)')
    ap.add_argument('--speeds', type=float, nargs='+', default=[0.05, 0.1, 0.2, 0.5, 1, 2, 4, 8])
    ap.add_argument('--amps', type=float, nargs='+', default=[0.1, 0.3])
    ap.add_argument('--vmax', type=float, help='top-speed target, rad/s')
    ap.add_argument('--speed', type=float, help='spin: rad/s (> 0)')
    ap.add_argument('--dir', choices=['+', '-', 'pos', 'neg', 'positive', 'negative'],
                    help='spin: direction (motor frame, + = increasing angle). stall: the direction '
                         'that presses the rod down onto the scale')
    ap.add_argument('--ramp', type=float, default=1.0, help='spin: s to reach the speed and to stop')
    ap.add_argument('--arm', type=float, help='stall: m from the motor axis to where the rod touches the scale')
    ap.add_argument('--stall-max', type=float,
                    help='stall: highest torque step, Nm (default: the test cap, RS03 12 / RS02 4 / RS06 8). '
                         'The drive torque limit is set to 1.1x this unless --torque-limit is given')
    ap.add_argument('--levels', type=int, default=8, help='stall: steps up to --stall-max (then back down)')
    ap.add_argument('--max-hold', type=float, default=15.0,
                    help='stall: s to wait for a reading before moving to the next step')
    ap.add_argument('--stall-travel', type=float, default=0.05,
                    help='stall: abort if the output turns this far, rad (rod slipped)')
    ap.add_argument('--scale-unit', choices=sorted(SCALE_UNITS), default='kg', help='stall: what the scale shows')
    ap.add_argument('--prompt-every', type=float, default=15.0, help='stall-hold: s between scale readings')
    ap.add_argument('--note', default='', help='free text stored in meta.json')
    ap.add_argument('--yes', action='store_true', help='skip the confirmation prompt')
    ap.add_argument('--rewind-only', action='store_true',
                    help='just turn the output back to 0 rad (after a spinning test), then exit')
    ap.add_argument('--sim', action='store_true', help='simulated motor instead of CAN')
    ap.add_argument('--sim-params', default='', help='e.g. J=0.02,b=0.01,Fc=0.15,delay=0.003')
    a = ap.parse_args(argv)
    if a.model is None:
        slot = a.id % 10
        if a.id // 10 not in (1, 2) or slot not in MODEL_BY_SLOT:
            ap.error(f'ID {a.id} is not in the 11-15 / 21-25 scheme; pass --model')
        a.model = MODEL_BY_SLOT[slot]
    if a.test in STALL_TESTS:
        peak = rs.RANGES[a.model].torque_max
        if a.arm is None or a.dir is None:
            ap.error(f'{a.test} needs --arm (m, motor axis to the scale contact) and --dir (+ or -, the '
                     f'direction that presses the rod onto the scale)')
        if not 0.03 <= a.arm <= 2.0:
            ap.error('--arm must be in [0.03, 2] m')
        if a.stall_max is None:
            a.stall_max = TORQUE_CAP[a.model]
        if not 0 < a.stall_max <= peak:
            ap.error(f'--stall-max must be in (0, {peak:g}] Nm for {a.model}')
        if a.torque_limit is None:
            a.torque_limit = min(peak, round(1.1 * a.stall_max, 2))
        elif a.torque_limit < a.stall_max:
            ap.error(f'--torque-limit {a.torque_limit:g} is below --stall-max {a.stall_max:g}: the drive '
                     f'would clip the top steps')
        if not 2 <= a.levels <= 30 or not 1 <= a.max_hold <= 120 or not 0.01 <= a.stall_travel <= 0.5:
            ap.error('stall: --levels in [2, 30], --max-hold in [1, 120] s, --stall-travel in [0.01, 0.5] rad')
        if a.test == 'stall-hold' and not 0 < (a.duration or 120.0) <= 1800:
            ap.error('stall-hold: --duration in (0, 1800] s')
    if a.torque_limit is None:
        a.torque_limit = TORQUE_CAP[a.model]
    if not 0 < a.torque_limit <= rs.RANGES[a.model].torque_max:
        ap.error(f'--torque-limit must be in (0, {rs.RANGES[a.model].torque_max:g}]')
    if not 0 < a.can_timeout <= 10:
        ap.error('--can-timeout must be in (0, 10] s')
    if a.test == 'spin':
        if a.speed is None or a.dir is None:
            ap.error('spin needs --speed (rad/s) and --dir (+ or -)')
        cap = spin_speed_cap(a.model, a.bus_voltage)
        if not 0 < a.speed <= cap:
            ap.error(f'--speed must be in (0, {cap:.1f}] rad/s for {a.model} at {a.bus_voltage:g} V')
        if not 0 < (a.duration or 5.0) <= 120 or not 0.1 <= a.ramp <= 10:
            ap.error('spin: --duration in (0, 120] s, --ramp in [0.1, 10] s')
    return a


def can_interfaces():
    """CAN interfaces that are up (ARPHRD_CAN = 280), USB adapters first."""
    out = []
    for name in sorted(os.listdir('/sys/class/net')):
        try:
            with open(f'/sys/class/net/{name}/type') as f:
                if f.read().strip() != '280':
                    continue
            with open(f'/sys/class/net/{name}/operstate') as f:
                state = f.read().strip()
        except OSError:
            continue
        if state != 'down':
            out.append(name)
    return sorted(out, key=lambda n: (not n.startswith('can_'), n))


def find_interface(motor_id: int, model: str) -> str:
    """The CAN interface on which `motor_id` answers a parameter read (with two adapters, one per
    leg, it is on exactly one of them)."""
    names = can_interfaces()
    for name in names:
        clock = RealClock()
        try:
            bus = SocketCanBus(name, clock)
        except (OSError, SetupError):
            continue
        try:
            if Link(bus, clock, motor_id, model).read_param(rs.P_VBUS, timeout=0.1) is not None:
                return name
        except OSError:   # a bus with nothing on it can refuse the frame (no buffer space)
            pass
        finally:
            bus.close()
    raise SetupError(f'motor {motor_id} does not answer on any CAN interface that is up ({", ".join(names) or "none"}). '
                     f'Run tools/jetson/can_status.sh to see each adapter and its motors.')


def main(argv=None, sim_motor_hook=None) -> int:
    a = parse_args(argv)
    model = a.model
    tests = ALL_SEQUENCE if a.test == 'all' else (a.test,)
    if a.iface == 'auto' and not a.sim:
        try:
            a.iface = find_interface(a.id, model)
        except SetupError as e:
            print(f'cannot start: {e}')
            return 2
    print(f'Motor {a.id} ({model}) on {"SIMULATED bus" if a.sim else a.iface}, drive torque limit '
          f'{a.torque_limit:g} Nm, CAN timeout {a.can_timeout:g} s, {a.rate:g} Hz:')
    for t in tests:
        p = build_profile(t, model, a)
        print(f'  {p.name:14s} {p.duration:5.0f} s  {p.describe}')
    if a.test in STALL_TESTS:
        print(f'Rod on the output, its contact point {a.arm:g} m from the motor axis, resting on the scale; '
              f'turning {a.dir} presses it DOWN. Scale tared with the rod resting and the motor off, in '
              f'{a.scale_unit}. Expect up to {a.stall_max / a.arm / SCALE_UNITS[a.scale_unit]:.2f} '
              f'{a.scale_unit} on the scale: it must read that much. Nobody in the rod\'s swing.')
    if not a.sim and not a.yes:
        print(('Housing clamped' if a.test in STALL_TESTS else
               'Nothing attached to the output, housing clamped') +
              ', hand on the power switch, motor nodes stopped?')
        if input('Run? [y/N] ').strip().lower() != 'y':
            print('Not run.')
            return 1

    def make_link():
        if a.sim:
            clock = SimClock()
            kw = {k: float(v) for k, v in (kv.split('=') for kv in a.sim_params.split(',') if kv)}
            motor = SimMotor(a.id, model, **kw)
            if sim_motor_hook:
                sim_motor_hook(motor)
            bus = SimBus(motor, clock)
            return bus, clock, Link(bus, clock, a.id, model), motor
        clock = RealClock()
        bus = SocketCanBus(a.iface, clock)
        return bus, clock, Link(bus, clock, a.id, model), None

    def on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    if a.rewind_only:
        bus, clock, link, _ = make_link()
        try:
            start(link, a, needs_position=False)
            ok = rewind(link, clock, a, target=0.0)
        finally:
            finish(link, clock, a)
            bus.close()
        print('rewound to 0 rad' if ok else 'rewind FAILED')
        return 0 if ok else 2
    for i, t in enumerate(tests):
        try:
            ok = one_run(t, a, model, make_link)
        except SetupError as e:
            print(f'[{t}] cannot start: {e}')
            return 2
        if not ok:
            return 2
        if i + 1 < len(tests) and not a.sim:
            time.sleep(2.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
