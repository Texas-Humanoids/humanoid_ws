#!/usr/bin/env python3
"""Walker bring-up console: jog each actuator, find its direction, record its hard stops,
calibrate offsets, all from one terminal dashboard (keyboard or gamepad). Raw SocketCAN, no ROS.

  sudo python3 tools/walker/bringup/jog.py --robot src/th_bringup/config/walker_demo_robot.yaml
  sudo python3 tools/walker/bringup/jog.py --robot <yaml> --spec policies/walker_demo/cad_v1/policy_spec.json
  python3 tools/walker/bringup/jog.py --sim             # try it with no hardware (10 simulated motors)

Stop robot.launch.py and every motor node first: this tool must be the only thing on the CAN buses
(it checks, and refuses if it sees someone else's MIT commands).

What it does with the motors:
  * At start: every motor that answers is configured (MIT mode, jog torque limit, 0.2 s CAN timeout,
    all read back) but NOT enabled. Disabled motors are limp; they're pinged at 20 Hz so you see
    their position live. Move a limp joint by hand to each hard stop: the "seen" range records it.
  * Enable one motor (e) and it holds where it is (soft jog gains). Jog it with the arrows or the
    gamepad. Every enabled motor holds its last target, so you can hold a leg up while jogging another.
  * Safety, always on: jog window +-0.6 rad from where it was enabled; joint limits from the yaml
    once offsets are calibrated (u unlocks them, the window stays); stall detection (tracking error
    > 0.15 rad = it hit something: target freezes where the joint is); fault / mode change / > 70 C /
    100 ms without feedback on ANY enabled motor -> all damp then stop. q quits the same way.

Keys
  left/right or tab  select motor           e  enable/disable selected (holds in place)
  up/down            jog speed +/- step      E  enable ALL (each holds in place)
  r                  reverse jog direction   space  stop jogging (hold)
  p                  pulse test: +0.15 rad (motor frame) and back, then answer y / n:
                     y = the joint did what "+joint means" says -> direction +1, n -> direction -1
  f  flip direction by hand    u  unlock yaml joint limits (window and stall detection stay)
  d  damp all (kp 0)           x  disable all (limp)          c  clear the seen range (selected)
  z z  calibrate offsets: robot held in --cal-pose (zero: legs straight down, feet flat, toes
       forward; or default: the policy's ready pose). offset = q_motor - direction * pose
  g  ramp every enabled motor to the policy default pose (needs offsets)   s  slow squat demo x3
     (with the feet on the floor run with --policy-gains: the soft jog gains can't hold the robot up)
  w  write direction + offset to the robot yaml, and the session (ranges) to bringup_session.json
  q  quit (damp, then stop)
Gamepad (Xbox map, see gamepad.py): hold LB + left stick up/down = jog (analog); dpad left/right
  = select; dpad up/down = speed step; A = enable/disable selected; B = damp all; X = reverse;
  Y = pulse test; BACK + START = quit.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sysid"))
sys.path.insert(0, HERE)
import chirp  # noqa: E402
import rsproto as rs  # noqa: E402
from chirp import Abort, SetupError  # noqa: E402
from gamepad import Gamepad, load_map  # noqa: E402

# +joint means (clean convention, same for both legs). Matches wiggle_test.py and the URDF.
EXPECT = {
    "hip_pitch": "foot swings BACKWARD (leg goes behind the body)",
    "hip_roll": "foot moves LEFT (+y), for BOTH legs (left leg: out, right leg: in)",
    "hip_yaw": "toes turn LEFT (counter-clockwise seen from above)",
    "knee_pitch": "knee FLEXES (foot moves backward/up)",
    "ankle_pitch": "toes go DOWN",
}
DEFAULT_MAP = [  # used when no robot yaml is given: (id, joint, model)
    (11, "left_hip_pitch_joint", "RS03"), (12, "left_hip_roll_joint", "RS03"), (13, "left_hip_yaw_joint", "RS02"),
    (14, "left_knee_pitch_joint", "RS03"), (15, "left_ankle_pitch_joint", "RS06"),
    (21, "right_hip_pitch_joint", "RS03"), (22, "right_hip_roll_joint", "RS03"), (23, "right_hip_yaw_joint", "RS02"),
    (24, "right_knee_pitch_joint", "RS03"), (25, "right_ankle_pitch_joint", "RS06"),
]
JOG_GAINS = {"RS03": (40.0, 2.0), "RS02": (15.0, 0.8), "RS06": (30.0, 1.5)}   # soft: a bump just stalls it
JOG_TORQUE = {"RS03": 15.0, "RS02": 5.0, "RS06": 10.0}                       # drive torque limit while jogging
POLICY_TORQUE = {"RS03": 40.0, "RS02": 12.0, "RS06": 25.0}                   # with --policy-gains (standing on the floor)
RATE = 100.0
STALL_ERR = 0.15
FB_TIMEOUT = 0.10
TEMP_MAX = 70.0
PING_EVERY = 5
PULSE_AMP, PULSE_RAMP, PULSE_HOLD = 0.15, 0.8, 0.4


def joint_type(j: str) -> str:
    return j.replace("left_", "").replace("right_", "").replace("_joint", "")


def smooth(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Motor:
    def __init__(self, mid, joint, model, iface, bus, clock, cfg):
        self.id, self.joint, self.model, self.iface = mid, joint, model, iface
        self.r = rs.RANGES[model]
        self.link = chirp.Link(bus, clock, mid, model)
        self.direction = float(cfg.get("direction") or 1.0)
        self.offset = cfg.get("offset")
        self.lower, self.upper = cfg.get("lower"), cfg.get("upper")
        self.dir_checked = False
        self.st = None
        self.last_fb = None
        self.state = "off"          # off | hold | damp
        self.enabled = False
        self.q_ref = 0.0
        self.q_en = 0.0             # where it was enabled (jog window centre)
        self.v_cmd = 0.0
        self.kp, self.kd = JOG_GAINS[model]
        self.seen = None            # [min, max] motor frame
        self.vbus = None
        self.ramp = None            # (t0, duration, q_from, q_to) or a callable(t) -> q (ends at ramp_end)
        self.ramp_end = None
        self.pulse = None           # t0 of a pulse test
        self.flag = ""

    # motor <-> joint frame: motor = direction * joint + offset
    @property
    def calibrated(self):
        return self.offset is not None

    def to_joint(self, q):
        return None if (q is None or self.offset is None) else (q - self.offset) / self.direction

    def to_motor(self, qj):
        return self.direction * qj + self.offset

    def motor_limits(self):
        if not self.calibrated or self.lower is None or self.upper is None:
            return None
        a, b = self.to_motor(self.lower), self.to_motor(self.upper)
        return min(a, b), max(a, b)


class Bringup:
    def __init__(self, motors, buses, clock, a, spec=None):
        self.motors, self.buses, self.clock, self.a = motors, buses, clock, a
        self.by_id = {m.id: m for m in motors}
        self.buslist = list(buses.values())
        self.sel = 0
        self.speed = 0.0                     # rad/s, sticky (keyboard)
        self.step = a.speed_step
        self.unlocked = False
        self.msg = "Ready. Motors are configured but limp. Select one and press e."
        self.msg_t = clock.now()
        self.asking = None                   # motor awaiting a pulse y/n answer
        self.zarm = 0.0                      # time of first z press
        self.tick_n = 0
        self.t_last = clock.now()
        self.estop_reason = None
        self.spec = spec
        self.log = []

    # ------------------------------------------------------------------ helpers
    @property
    def cur(self) -> Motor:
        return self.motors[self.sel]

    def say(self, text):
        self.msg, self.msg_t = text, self.clock.now()

    def window(self, m):
        lo, hi = m.q_en - self.a.window, m.q_en + self.a.window
        lim = None if self.unlocked else m.motor_limits()
        if lim:
            lo, hi = max(lo, lim[0]), min(hi, lim[1])
        return lo, hi

    # ------------------------------------------------------------------ actions
    def enable(self, m: Motor):
        if m.enabled:
            return
        if m.st is None or m.last_fb is None:
            self.say(f"{m.id}: no feedback yet, not enabling")
            return
        if abs(m.st.q) > m.r.position_max - self.a.window - 0.3:
            self.say(f"{m.id}: at {m.st.q:.2f} rad, too close to the +/-4pi MIT range. Run mit_all.py home first.")
            return
        m.link.send(rs.enable(m.id))
        self.clock.sleep(0.005)
        for o in self.motors:
            if o.enabled and o is not m:
                self.command(o, 0.0, self.clock.now())
        st = m.link.fresh_exchange(m.st.q, 0.0, m.kp, m.kd, 0.0)
        if st is None or st.faults or st.mode != rs.MODE_RUNNING:
            m.link.send(rs.stop(m.id, clear_faults=False))
            self.say(f"{m.id}: did not enter run mode ({'no reply' if st is None else f'mode {st.mode}, faults {st.fault_names()}'})")
            return
        m.st, m.last_fb = st, self.clock.now()
        m.q_ref = m.q_en = st.q
        m.v_cmd, m.enabled, m.state, m.flag = 0.0, True, "hold", ""
        self.say(f"{m.id} {m.joint}: enabled, holding at {st.q:+.3f} (kp {m.kp:g} kd {m.kd:g}, {self.a.torque_limit_text(m.model)})")

    def disable(self, m: Motor, why=""):
        if m.enabled:
            m.link.send(rs.stop(m.id, clear_faults=False))
        m.enabled, m.state, m.v_cmd, m.ramp, m.pulse = False, "off", 0.0, None, None
        if m is self.cur:
            self.speed = 0.0
        if why:
            self.say(f"{m.id}: disabled ({why})")

    def damp_all(self):
        for m in self.motors:
            if m.enabled:
                m.state, m.ramp, m.pulse, m.v_cmd = "damp", None, None, 0.0
        self.speed = 0.0
        self.say("All enabled motors damping (kp 0). x = limp, e = hold again.")

    def disable_all(self, why="x pressed"):
        for m in self.motors:
            self.disable(m)
        self.speed = 0.0
        self.say(f"All motors disabled (limp): {why}")

    def emergency(self, reason):
        """Damp everything 0.4 s, then stop. Used for any safety trip."""
        if getattr(self, "_in_estop", False):
            return
        self._in_estop = True
        self.estop_reason = reason
        end = self.clock.now() + 0.4
        while self.clock.now() < end:
            for m in self.motors:
                if m.enabled:
                    m.link.command(0.0, 0.0, 0.0, self.a.damping_kd, 0.0)
            self.drain(0.01)
        self.disable_all(f"SAFETY: {reason}")
        self._in_estop = False

    def set_speed(self, v):
        self.speed = max(-self.a.max_speed, min(self.a.max_speed, v))

    def start_pulse(self):
        m = self.cur
        if not m.enabled:
            self.enable(m)
            if not m.enabled:
                return
        m.q_en = m.q_ref = m.st.q if m.st else m.q_ref
        m.pulse, m.ramp = self.clock.now(), None
        self.speed = 0.0
        self.say(f"{m.id}: pulsing +{PULSE_AMP} rad (motor +). Watch the joint...")

    def answer(self, yes: bool):
        m = self.asking
        if m is None:
            return
        m.direction = 1.0 if yes else -1.0
        m.dir_checked = True
        self.asking = None
        self.say(f"{m.id} {m.joint}: direction = {m.direction:+.0f} (motor + is joint {'+' if yes else '-'}). "
                 + ("Offset is now stale: re-calibrate (z z)." if m.calibrated else ""))

    def calibrate(self, pose: dict):
        missing = [m.id for m in self.motors if m.st is None]
        unchecked = [m.id for m in self.motors if not m.dir_checked]
        if missing:
            self.say(f"calibrate: no feedback from {missing}")
            return
        if unchecked and not self.a.trust_directions:
            self.say(f"calibrate: directions not checked this session for {unchecked} (p test, or --trust-directions)")
            return
        for m in self.motors:
            m.offset = m.st.q - m.direction * pose.get(m.joint, 0.0)
        self.say(f"Offsets set at the '{self.a.cal_pose}' pose for {len(self.motors)} motors. Check the q_joint column, then w to save.")

    def go_pose(self, pose: dict, duration=3.0):
        not_ready = [m.id for m in self.motors if not (m.enabled and m.calibrated)]
        if not_ready:
            self.say(f"g: needs every motor enabled and calibrated; not ready: {not_ready}")
            return
        t0 = self.clock.now()
        for m in self.motors:
            m.ramp = (t0, duration, m.q_ref, m.to_motor(pose[m.joint]))
            m.q_en = m.to_motor(pose[m.joint])
        self.say(f"Ramping to the policy default pose over {duration:g} s")

    def squat(self, pose: dict, depth=0.25, period=4.0, cycles=3):
        not_ready = [m.id for m in self.motors if not (m.enabled and m.calibrated)]
        if not_ready:
            self.say(f"s: needs every motor enabled and calibrated (g first); not ready: {not_ready}")
            return
        t0 = self.clock.now()

        def make(m):
            jt = joint_type(m.joint)
            k = {"knee_pitch": 1.0, "hip_pitch": -0.5, "ankle_pitch": -0.5}.get(jt, 0.0)
            base = pose[m.joint]

            def f(t):
                s = (1 - math.cos(2 * math.pi * min((t - t0) / period, cycles))) / 2  # 0 -> 1 -> 0
                return m.to_motor(base + k * depth * s)
            return f
        for m in self.motors:
            m.ramp = make(m)
            m.ramp_end = t0 + period * cycles
            m.q_en = m.q_ref
        self.squat_end = t0 + period * cycles
        self.say(f"Squat: knee +{depth} rad, hip/ankle -{depth / 2} rad, {cycles} x {period:g} s. space stops.")

    # ------------------------------------------------------------------ the loop
    def command(self, m: Motor, dt: float, now: float):
        if not m.enabled:
            if self.tick_n % PING_EVERY == 0:
                m.link.send(rs.stop(m.id, clear_faults=False))   # replies with status: live position
            return
        if m.state == "damp":
            m.link.command(0.0, 0.0, 0.0, self.a.damping_kd, 0.0)
            return
        v = 0.0
        if m.pulse is not None:
            t = now - m.pulse
            T = 2 * PULSE_RAMP + PULSE_HOLD
            if t >= T + 0.2:
                m.pulse, m.q_ref = None, m.q_en
                self.asking = m
                self.say(f"{m.id} {m.joint}: did it go '{EXPECT[joint_type(m.joint)]}'?  y / n")
            else:
                s = smooth(t / PULSE_RAMP) if t < PULSE_RAMP + PULSE_HOLD else smooth((T - t) / PULSE_RAMP)
                m.q_ref = m.q_en + PULSE_AMP * s
        elif m.ramp is not None:
            if callable(m.ramp):
                m.q_ref = m.ramp(now)
                if now >= (m.ramp_end if m.ramp_end is not None else getattr(self, "squat_end", 0)):
                    m.ramp = m.ramp_end = None
            else:
                t0, T, qa, qb = m.ramp
                m.q_ref = qa + (qb - qa) * smooth((now - t0) / T)
                if now - t0 >= T:
                    m.ramp = None
        elif m is self.cur and m.v_cmd != 0.0:
            v = m.v_cmd
            lo, hi = self.window(m)
            nq = m.q_ref + v * dt
            if nq < lo or nq > hi:
                nq = min(max(nq, lo), hi)
                v = 0.0
                if not m.flag:
                    m.flag = "AT LIMIT" if (self.window(m) != (m.q_en - self.a.window, m.q_en + self.a.window)) else "AT WINDOW"
            else:
                m.flag = ""
            m.q_ref = nq
        m.link.command(m.q_ref, v, m.kp, m.kd, 0.0)

    def on_frame(self, fr):
        m = self.by_id.get(rs.frame_motor(fr[0]))
        if m is None:
            return
        c = m.link.classify(fr)
        if not c:
            return
        if c[0] == "fault":
            if m.enabled:
                self.emergency(f"motor {m.id} fault frame {c[1].hex()}")
            return
        st = c[1]
        m.st, m.last_fb = st, c[2]
        m.seen = [st.q, st.q] if m.seen is None else [min(m.seen[0], st.q), max(m.seen[1], st.q)]

    def drain(self, timeout):
        end = self.clock.now() + timeout
        while (left := end - self.clock.now()) > 0:
            for fr in chirp.wait_frames(self.buslist, left):
                self.on_frame(fr)

    def safety(self, now):
        for m in self.motors:
            if not m.enabled:
                continue
            st = m.st
            if m.last_fb is None or now - m.last_fb > FB_TIMEOUT:
                return self.emergency(f"motor {m.id}: no feedback for {FB_TIMEOUT * 1000:.0f} ms")
            if st.faults:
                return self.emergency(f"motor {m.id}: faults {st.fault_names()}")
            if st.mode != rs.MODE_RUNNING:
                return self.emergency(f"motor {m.id}: left run mode (mode {st.mode}); CAN timeout or power dip?")
            if st.temp > TEMP_MAX:
                return self.emergency(f"motor {m.id}: {st.temp:.0f} C")
            if m.state == "hold" and abs(m.q_ref - st.q) > STALL_ERR and m.pulse is None:
                # Hit a stop or something is pushing it. Don't keep winding up torque: hold here.
                m.q_ref, m.v_cmd, m.ramp = st.q, 0.0, None
                if m is self.cur:
                    self.speed = 0.0
                m.flag = "STALL"
                qj = m.to_joint(st.q)
                self.say(f"{m.id}: STALLED at motor {st.q:+.3f}" + (f" / joint {qj:+.3f}" if qj is not None else "")
                         + " - hard stop or obstruction. Holding here.")

    def tick(self):
        now = self.clock.now()
        dt = min(now - self.t_last, 0.05)
        self.t_last = now
        c = self.cur
        for m in self.motors:
            m.v_cmd = self.speed if (m is c and m.enabled and m.state == "hold") else 0.0
        for m in self.motors:
            self.command(m, dt, now)
        self.tick_n += 1
        if self.a.log:
            self.log.append([round(now, 4)] + [x for m in self.motors for x in
                            (m.q_ref if m.enabled else float("nan"), m.st.q if m.st else float("nan"),
                             m.st.v if m.st else float("nan"), m.st.tau if m.st else float("nan"))])

    def session(self):
        return {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "cal_pose": self.a.cal_pose, "motors": [
            dict(id=m.id, joint=m.joint, model=m.model, iface=m.iface, direction=m.direction,
                 direction_checked=m.dir_checked, offset=m.offset,
                 seen_motor=m.seen, seen_joint=None if (m.seen is None or not m.calibrated) else
                 sorted([m.to_joint(m.seen[0]), m.to_joint(m.seen[1])]),
                 yaml_limits=[m.lower, m.upper]) for m in self.motors]}

    def write(self):
        out = os.path.join(os.path.dirname(os.path.abspath(self.a.robot)) if self.a.robot else ".", "bringup_session.json")
        json.dump(self.session(), open(out, "w"), indent=1)
        if not self.a.robot:
            self.say(f"Session -> {out} (no --robot yaml to update)")
            return
        text = open(self.a.robot).read()
        n_ok = 0
        for m in self.motors:
            for key, val in (("direction", m.direction), ("offset", m.offset)):
                if val is None:
                    continue
                pat = rf"(- joint: {re.escape(m.joint)}\n(?:    .*\n)*?    {key}: )\S+"
                text, n = re.subn(pat, rf"\g<1>{val:.4f}" if key == "offset" else rf"\g<1>{val:.1f}", text)
                n_ok += n
        open(self.a.robot, "w").write(text)
        self.say(f"Wrote {n_ok} direction/offset values to {self.a.robot}; ranges -> {out}")


def loop_once(b: Bringup, t_next: float, period: float, sim: bool) -> float:
    """One control period: command every motor, collect replies until t_next, safety checks."""
    b.tick()
    while (left := t_next - b.clock.now()) > 0:
        for fr in chirp.wait_frames(b.buslist, left):
            b.on_frame(fr)
    if sim:
        time.sleep(period)
    b.safety(b.clock.now())
    if b.clock.now() - t_next > 0.05:   # fell behind (terminal stall): resync, don't burst
        t_next = b.clock.now()
    return t_next + period


# ============================================================================= UI (curses)
def fmt(x, w=7, p=3):
    return f"{x:+{w}.{p}f}" if isinstance(x, (int, float)) and x is not None and not math.isnan(x) else " " * (w - 2) + "--"


def draw(scr, b: Bringup, pad: Gamepad, sim: bool):
    import curses
    scr.erase()
    H, W = scr.getmaxyx()

    def put(y, x, s, attr=0):
        if 0 <= y < H:
            try:
                scr.addnstr(y, x, s, max(0, W - x - 1), attr)
            except curses.error:
                pass
    vb = [m.vbus for m in b.motors if m.vbus]
    put(0, 0, f" WALKER BRING-UP {'[SIM] ' if sim else ''}| {len(b.motors)} motors on {', '.join(sorted(b.buses))} | "
              f"VBUS {min(vb):.1f}-{max(vb):.1f} V | gamepad {'OK' if pad.ok else 'none'} | limits {'UNLOCKED' if b.unlocked else 'on'}"
        if vb else " WALKER BRING-UP", curses.A_REVERSE)
    put(1, 0, "    ID  joint                  mdl  state  q_motor  q_joint      v    tau    T  dir  offset   seen (joint or motor)    flag")
    for i, m in enumerate(b.motors):
        st = m.st
        q = st.q if st else None
        seen = m.seen
        if seen and m.calibrated:
            s = sorted([m.to_joint(seen[0]), m.to_joint(seen[1])])
            seen_s = f"J[{s[0]:+.2f},{s[1]:+.2f}]"
        elif seen:
            seen_s = f"M[{seen[0]:+.2f},{seen[1]:+.2f}]"
        else:
            seen_s = ""
        state = m.state.upper() if m.enabled else "off"
        if m.pulse is not None:
            state = "PULSE"
        elif m.ramp is not None:
            state = "RAMP"
        elif m.enabled and m is b.cur and m.v_cmd:
            state = "JOG"
        line = (f" {'>' if i == b.sel else ' '}  {m.id:<3d} {m.joint.replace('_joint', ''):22s} {m.model:4s} {state:6s}"
                f" {fmt(q)}  {fmt(m.to_joint(q))} {fmt(st.v if st else None, 6, 2)} {fmt(st.tau if st else None, 6, 2)}"
                f" {st.temp if st else 0:4.0f} {m.direction:+.0f}{'*' if m.dir_checked else ' '} {fmt(m.offset)}   {seen_s:22s} {m.flag}")
        attr = curses.A_BOLD if i == b.sel else 0
        if st and st.faults:
            attr |= curses.A_STANDOUT
        put(2 + i, 0, line, attr)
    y = 3 + len(b.motors)
    c = b.cur
    jt = joint_type(c.joint)
    bar = int(round(12 * b.speed / b.a.max_speed)) if b.a.max_speed else 0
    put(y, 0, f" Selected {c.id} {c.joint}  ({c.model}, {c.iface})   jog speed {b.speed:+.2f} rad/s "
              f"[{('<' * -bar).rjust(12)}|{('>' * bar).ljust(12)}]   window +-{b.a.window:g} rad", curses.A_BOLD)
    put(y + 1, 0, f" +joint means: {EXPECT[jt]}.   yaml limits: [{fmt(c.lower)}, {fmt(c.upper)}] (joint)")
    lim = b.window(c) if c.enabled else None
    put(y + 2, 0, f" target {fmt(c.q_ref if c.enabled else None)} motor, allowed [{fmt(lim[0]) if lim else ''}, {fmt(lim[1]) if lim else ''}]")
    put(y + 4, 0, " <-/-> select | e enable | E enable all | up/dn speed | r reverse | space stop | p pulse test (then y/n) | f flip dir")
    put(y + 5, 0, " u unlock limits | d damp all | x limp all | c clear seen | z z calibrate | g default pose | s squat | w write | q quit")
    put(y + 6, 0, " pad: hold LB + left stick = jog | dpad <-/-> select, up/dn speed | A enable | B damp all | X reverse | Y pulse | BACK+START quit")
    put(y + 8, 0, " " + b.msg, curses.A_REVERSE if b.asking or b.estop_reason else curses.A_BOLD)
    scr.refresh()


def handle_key(b: Bringup, k: int, pose: dict, cal_pose: dict) -> bool:
    """Returns False to quit."""
    import curses
    c = b.cur
    if k in (ord("q"), 27):
        return False
    if b.asking is not None and k in (ord("y"), ord("n")):
        b.answer(k == ord("y"))
        return True
    if k in (curses.KEY_RIGHT, 9):
        b.speed, b.sel = 0.0, (b.sel + 1) % len(b.motors)
    elif k == curses.KEY_LEFT:
        b.speed, b.sel = 0.0, (b.sel - 1) % len(b.motors)
    elif k == curses.KEY_UP:
        b.set_speed(round(b.speed + b.step, 3))
    elif k == curses.KEY_DOWN:
        b.set_speed(round(b.speed - b.step, 3))
    elif k == ord("r"):
        b.set_speed(-b.speed)
    elif k == ord(" "):
        b.speed = 0.0
        for m in b.motors:
            if m.enabled and m.ramp is not None:
                m.ramp, m.q_en = None, m.q_ref
        b.say("Stopped (holding).")
    elif k == ord("e"):
        b.disable(c, "e") if c.enabled else b.enable(c)
    elif k == ord("E"):
        for m in b.motors:
            b.enable(m)
    elif k == ord("p"):
        b.start_pulse()
    elif k == ord("f"):
        c.direction *= -1
        c.dir_checked = True
        b.say(f"{c.id}: direction flipped to {c.direction:+.0f}" + (" (offset now stale: z z)" if c.calibrated else ""))
    elif k == ord("u"):
        b.unlocked = not b.unlocked
        b.say("Joint limits UNLOCKED (window + stall detection only): use to find the real hard stops slowly."
              if b.unlocked else "Joint limits back on.")
    elif k == ord("d"):
        b.damp_all()
    elif k == ord("x"):
        b.disable_all()
    elif k == ord("c"):
        c.seen = None
    elif k == ord("z"):
        now = b.clock.now()
        if now - b.zarm < 3.0:
            b.calibrate(cal_pose)
            b.zarm = 0.0
        else:
            b.zarm = now
            b.say(f"Calibrate offsets with the robot held in the '{b.a.cal_pose}' pose? Press z again within 3 s.")
    elif k == ord("g"):
        b.go_pose(pose)
    elif k == ord("s"):
        b.squat(pose)
    elif k == ord("w"):
        b.write()
    return True


def handle_pad(b: Bringup, pad: Gamepad, pose) -> bool:
    for name in pad.poll():
        if name == "a":
            b.disable(b.cur, "pad A") if b.cur.enabled else b.enable(b.cur)
        elif name == "b":
            b.damp_all()
        elif name == "x":
            b.set_speed(-b.speed)
        elif name == "y":
            b.start_pulse()
    if pad.held("back") and pad.held("start"):
        return False
    dx = pad.axis("dpad_x", 0.5)
    dy = pad.axis("dpad_y", 0.5)
    now = time.monotonic()
    if (dx or dy) and now - getattr(b, "_dpad_t", 0) > 0.25:
        b._dpad_t = now
        if dx:
            b.speed, b.sel = 0.0, (b.sel + (1 if dx > 0 else -1)) % len(b.motors)
        if dy:
            b.set_speed(round(b.speed + (-b.step if dy > 0 else b.step), 3))
    if pad.ok:
        if pad.held("lb"):
            b.speed = -pad.axis("ly") * b.a.max_speed    # stick up = +
            b._pad_jog = True
        elif getattr(b, "_pad_jog", False):
            b.speed, b._pad_jog = 0.0, False             # dead-man released
    return True


# ============================================================================= setup
def load_cfg(path):
    if not path:
        return {mid: dict(joint=j, motor_model=mo) for mid, j, mo in DEFAULT_MAP}
    import yaml
    robot = yaml.safe_load(open(path)) or {}
    return {int(m["motor_id"]): m for m in robot.get("motors", []) if m.get("motor_id") is not None}


def other_traffic(buses, clock, seconds=0.3):
    """MIT commands from another program (robot.launch.py still running?)."""
    seen = set()
    end = clock.now() + seconds
    while (left := end - clock.now()) > 0:
        for fr in chirp.wait_frames(list(buses.values()), left):
            if rs.comm_type(fr[0]) == rs.T_MOTION:
                seen.add(fr[0] & 0xFF)
    return seen


def build(a):
    cfg = load_cfg(a.robot)
    ids = sorted(cfg)
    if a.sim:
        clock = chirp.SimClock()
        sims = [chirp.SimMotor(mid, cfg[mid].get("motor_model", "RS03"), q0=0.05 * (i - 4), seed=i, J=0.05, Fc=0.2)
                for i, mid in enumerate(ids)]
        buses = {"sim": chirp.SimMultiBus(sims, clock)}
        found = {mid: ("sim", 36.4) for mid in ids}
    else:
        clock = chirp.RealClock()
        sys.path.insert(0, os.path.join(HERE, "..", "sysid"))
        import mit_all
        found, buses = mit_all.discover(ids, clock, a)
        if not found:
            sys.exit(f"No motor answered on {', '.join(chirp.can_interfaces()) or 'any CAN interface (none is up)'}. "
                     f"Run tools/jetson/can_status.sh.")
        busy = other_traffic(buses, clock)
        if busy:
            sys.exit(f"Someone else is sending MIT commands to motors {sorted(busy)}. Stop robot.launch.py / motor nodes first.")
    missing = [i for i in ids if i not in found]
    motors = [Motor(mid, cfg[mid].get("joint", f"motor_{mid}"), cfg[mid].get("motor_model", "RS03"), found[mid][0],
                    buses[found[mid][0]], clock, cfg[mid]) for mid in ids if mid in found]
    gains = {}
    if a.policy_gains:
        spec = json.load(open(a.spec))
        gains = {j: (kp, kd) for j, kp, kd in zip(spec["joint_order"], spec["kp"], spec["kd"])}
    for m in motors:
        m.vbus = found[m.id][1]
        if m.joint in gains:
            m.kp, m.kd = gains[m.joint]
        if m.model != chirp.MODEL_BY_SLOT.get(m.id % 10, m.model):
            print(f"note: motor {m.id} is {m.model} in the yaml but slot {m.id % 10} is usually {chirp.MODEL_BY_SLOT[m.id % 10]}")
        print(f"configuring {m.id} {m.joint} ({m.model} on {m.iface}, {m.vbus:.1f} V) ...", flush=True)
        s = types.SimpleNamespace(torque_limit=min(a.torque_limit_for(m.model), m.r.torque_max),
                                  can_timeout=a.can_timeout, damping_kd=a.damping_kd)
        chirp.configure(m.link, s)
    return motors, buses, clock, missing


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--robot", help="robot yaml (motor map, directions, offsets, limits); written by w")
    ap.add_argument("--spec", help="policy_spec.json (default pose for g / s / --cal-pose default)")
    ap.add_argument("--cal-pose", choices=("zero", "default"), default="zero")
    ap.add_argument("--trust-directions", action="store_true", help="allow z z without pulse-checking every direction")
    ap.add_argument("--max-speed", type=float, default=1.0, help="rad/s (jog speed cap)")
    ap.add_argument("--speed-step", type=float, default=0.1)
    ap.add_argument("--window", type=float, default=0.6, help="rad either side of where a motor was enabled")
    ap.add_argument("--torque-limit", type=float, default=None, help="drive torque limit for every motor (Nm); "
                    f"default {JOG_TORQUE}")
    ap.add_argument("--can-timeout", type=float, default=0.2)
    ap.add_argument("--damping-kd", type=float, default=1.0)
    ap.add_argument("--policy-gains", action="store_true",
                    help="use the spec's kp/kd (and torque limits %s) instead of the soft jog gains. Needed for g / s "
                         "with the feet on the floor: the jog ankle kp can't hold the robot up" % POLICY_TORQUE)
    ap.add_argument("--map", help="gamepad map json (see gamepad.py)")
    ap.add_argument("--log", help="write a CSV of every motor's target/position/velocity/torque at 100 Hz")
    ap.add_argument("--sim", action="store_true")
    a = ap.parse_args(argv)
    if a.policy_gains and not a.spec:
        ap.error("--policy-gains needs --spec")
    a.torque_limit_for = lambda model: (a.torque_limit if a.torque_limit is not None
                                        else (POLICY_TORQUE if a.policy_gains else JOG_TORQUE)[model])
    a.torque_limit_text = lambda model: f"torque limit {a.torque_limit_for(model):g} Nm"
    return a


def poses(a, motors):
    zero = {m.joint: 0.0 for m in motors}
    default = dict(zero)
    if a.spec:
        spec = json.load(open(a.spec))
        default.update(dict(zip(spec["joint_order"], spec["default_joint_pos"])))
    return default, (default if a.cal_pose == "default" else zero)


def main(argv=None):
    a = parse_args(argv)
    if a.cal_pose == "default" and not a.spec:
        sys.exit("--cal-pose default needs --spec (the policy's default pose)")
    motors, buses, clock, missing = build(a)
    if missing:
        print(f"not found (skipped): {missing}")
    b = Bringup(motors, buses, clock, a)
    default_pose, cal_pose = poses(a, motors)
    pad = Gamepad(mapping=load_map(a.map))
    import curses

    def ui(scr):
        curses.curs_set(0)
        scr.nodelay(True)
        scr.keypad(True)
        period = 1.0 / RATE
        t_next = clock.now() + period
        draw_t = 0.0
        while True:
            t_next = loop_once(b, t_next, period, a.sim)
            keep = True
            while (k := scr.getch()) != -1:
                keep = keep and handle_key(b, k, default_pose, cal_pose)
            keep = keep and handle_pad(b, pad, default_pose)
            if not keep:
                return
            if clock.now() - draw_t > 0.05:
                draw(scr, b, pad, a.sim)
                draw_t = clock.now()

    try:
        curses.wrapper(ui)
    except KeyboardInterrupt:
        pass
    except (Abort, OSError) as e:
        print(f"CAN error: {e}")
    finally:
        try:
            if any(m.enabled for m in motors):
                b.emergency("quit")
        except Exception:  # noqa: BLE001 the CAN timeout still stops them
            pass
        for bus in buses.values():
            bus.close()
        if a.log and b.log:
            import csv
            with open(a.log, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t"] + [f"{m.id}_{k}" for m in motors for k in ("qref", "q", "v", "tau")])
                w.writerows(b.log)
            print(f"log -> {a.log}")
        print(json.dumps(b.session(), indent=1))
        if b.estop_reason:
            print(f"last safety stop: {b.estop_reason}")


if __name__ == "__main__":
    main()
