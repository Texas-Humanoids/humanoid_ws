#!/usr/bin/env python3
"""Measure the delays in the real policy loop on one bench motor (Jetson, robot container).

The real loop at 50 Hz: the drive replies with its position -> motor node publishes it -> the
bridge's tick builds an observation -> policy_node runs the ONNX policy and publishes joint
targets -> the bridge forwards them -> the motor node sends the MIT command on CAN. Sim (v6)
sees fresh state every step and applies the action 0-20 ms later, so the number to compare is
**state age at command**: from the CAN reply carrying the position the observation used, to
the CAN command computed from it. Times come from one clock: kernel timestamps on a CAN sniffer
socket and wall time at each ROS callback.

  # 1. no motor node running: a one-motor robot config, offset so arming doesn't swing it
  python3 tools/walker/sysid/latency.py setup --id 25
  # 2. the real pipeline, as on the robot (second shell)
  ros2 launch th_bringup robot.launch.py motors_config:=sysid_logs/latency_m25.yaml \\
      require_imu:=false policy_spec:=policies/walker_v2/v6/policy_spec.json \\
      model_path:=policies/walker_v2/v6/policy.onnx
  # 3. arm (START), run the policy 30 s, disarm (BACK), save and report
  python3 tools/walker/sysid/latency.py record --config sysid_logs/latency_m25.yaml --seconds 30
  # anywhere, later
  python3 tools/walker/sysid/latency.py analyze sysid_logs/<date>_m25_latency [--plots DIR]
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import os
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rsproto as rs  # noqa: E402

SPEC = 'policies/walker_v2/v6/policy_spec.json'
ROBOT = 'src/th_bringup/config/walker_v2_robot.yaml'
# Python's socket module lacks it; 29 on Linux (asm-generic, incl. aarch64 and x86_64).
SO_TIMESTAMP = getattr(socket, 'SO_TIMESTAMP', 29)
SIM_DELAY_MS = (0.0, 20.0)   # v6: DelayedDCMotor 0..4 steps of 5 ms, observation fresh
DRIVE_MS = 1.4               # command on the bus -> torque (torque-chirp phase, 2026-11-15)


# ============================================================================= setup
def cmd_setup(a):
    import yaml
    with open(a.robot) as f:
        robot = yaml.safe_load(f)
    entry = next((m for m in robot['motors'] if int(m['motor_id']) == a.id), None)
    if entry is None:
        raise SystemExit(f'motor {a.id} is not in {a.robot}')
    with open(a.spec) as f:
        spec = json.load(f)
    joint = entry['joint']
    default = spec['default_joint_pos'][spec['joint_order'].index(joint)]
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import chirp
    if a.iface == 'auto':
        try:
            a.iface = chirp.find_interface(a.id, entry['motor_model'])
        except chirp.SetupError as e:
            raise SystemExit(str(e))
    clock = chirp.RealClock()
    link = chirp.Link(chirp.SocketCanBus(a.iface, clock), clock, a.id, entry['motor_model'])
    mech = link.read_param(rs.P_MECH_POS)
    link.bus.close()
    if mech is None:
        raise SystemExit(f'motor {a.id} does not answer on {a.iface}: powered? any motor node running?')
    direction = float(entry.get('direction', 1.0))
    motor = {**{k: v for k, v in entry.items() if k != 'offset'},
             'can_interface': a.iface,
             'offset': round(mech - direction * default, 4)}
    out = {'control_mode': 'mit', 'damping_kd': 1.0, 'feedback_rate': robot.get('feedback_rate', 100.0),
           'motors': [motor]}
    path = a.out or os.path.join('sysid_logs', f'latency_m{a.id}.yaml')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        f.write(f'# latency.py setup: motor {a.id} ({entry["motor_model"]}) as {joint}. offset puts the\n'
                f'# policy default ({default:+.3f} rad) where the motor sits now ({mech:.4f} rad).\n')
        yaml.safe_dump(out, f, sort_keys=False)
    print(f'wrote {path}: motor {a.id} {entry["motor_model"]} as {joint}, offset {motor["offset"]:.4f} '
          f'(it sits at {mech:.4f} rad = the policy default {default:+.3f})')


# ============================================================================= record
class Sniffer(threading.Thread):
    """Every frame to or from one motor, with the kernel's receive timestamp (wall clock)."""

    def __init__(self, iface, motor_id, model):
        super().__init__(daemon=True)
        self.id, self.r = motor_id, rs.RANGES[model]
        self.s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMP, 1)
        self.s.bind((iface,))
        self.s.settimeout(0.2)
        self.rows = []   # (t, kind, q, v, tau, kp, kd, mode)
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                frame, anc, _, _ = self.s.recvmsg(16, socket.CMSG_SPACE(16))
            except socket.timeout:
                continue
            t = time.time()
            for level, kind, data in anc:
                if level == socket.SOL_SOCKET and kind == SO_TIMESTAMP:
                    sec, usec = struct.unpack('qq', data[:16])
                    t = sec + usec * 1e-6
            cid, _, data = struct.unpack('=IB3x8s', frame)
            cid &= socket.CAN_EFF_MASK
            ct = rs.comm_type(cid)
            if ct == rs.T_MOTION and cid & 0xFF == self.id:
                q, v, kp, kd = struct.unpack('>4H', data)
                self.rows.append((t, 'cmd', rs.u16_decode(q, self.r.position_max),
                                  rs.u16_decode(v, self.r.velocity_max),
                                  rs.u16_decode(cid >> 8 & 0xFFFF, self.r.torque_max),
                                  kp * self.r.kp_max / 65535.0, kd * self.r.kd_max / 65535.0, ''))
            elif ct == rs.T_STATUS and rs.frame_motor(cid) == self.id:
                st = rs.decode_status(cid, data, self.r)
                self.rows.append((t, 'reply', st.q, st.v, st.tau, '', '', st.mode))


def motor_namespace(joint):
    return '/' + joint.removesuffix('_joint')


def cmd_record(a):
    import yaml
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray, Float64, String

    with open(a.config) as f:
        motor = yaml.safe_load(f)['motors'][0]
    joint, ns = motor['joint'], motor_namespace(motor['joint'])
    sn = Sniffer(a.iface or motor['can_interface'], int(motor['motor_id']), motor['motor_model'])
    sn.start()

    rclpy.init()
    node = rclpy.create_node('latency_watch')
    ev = {'obs': [], 'targets': [], 'motor_cmd': [], 'actual': [], 'mode': []}
    node.create_subscription(Float32MultiArray, '/policy/observation',
                             lambda m: ev['obs'].append((time.time(),)), 50)

    def on_targets(m):
        t = time.time()
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        q = m.position[list(m.name).index(joint)] if joint in m.name else float('nan')
        ev['targets'].append((t, stamp, q))
    node.create_subscription(JointState, '/policy/joint_targets', on_targets, 50)
    node.create_subscription(Float64, ns + '/motor/target_position',
                             lambda m: ev['motor_cmd'].append((time.time(), m.data)), 50)
    node.create_subscription(Float64, ns + '/motor/actual_position',
                             lambda m: ev['actual'].append((time.time(), m.data)), 50)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/policy_bridge/mode',
                             lambda m: ev['mode'].append((time.time(), m.data)), latched)
    buttons = node.create_publisher(String, a.button_topic, 10)
    cmd_vel = node.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(seconds, walk=0.0):
        end = time.time() + seconds
        next_cmd = 0.0
        while time.time() < end:
            if time.time() >= next_cmd:
                tw = Twist()
                tw.linear.x = walk
                cmd_vel.publish(tw)
                next_cmd = time.time() + 0.05
            rclpy.spin_once(node, timeout_sec=0.005)

    print(f'waiting for the pipeline ({ns}, policy observations) ...', flush=True)
    end = time.time() + 15
    while time.time() < end and not (ev['obs'] and ev['actual']):
        rclpy.spin_once(node, timeout_sec=0.05)
    if not ev['actual']:
        raise SystemExit(f'no {ns}/motor/actual_position: is robot.launch.py running with {a.config}?')
    if not ev['obs']:
        raise SystemExit('no /policy/observation: is the policy_bridge running?')
    if not a.no_arm:
        print('arming in 3 s (the motor will follow the policy) ...', flush=True)
        spin_for(3.0)
        buttons.publish(String(data=a.arm_event))
    t_start = time.time()
    print(f'recording {a.seconds:g} s (cmd_vel x = {a.walk:g}) ...', flush=True)
    try:
        spin_for(a.seconds, a.walk)
    finally:
        t_end = time.time()
        if not a.no_arm:
            buttons.publish(String(data=a.disarm_event))
            spin_for(0.5)
        sn.stop = True
        sn.join(1.0)
        node.destroy_node()
        rclpy.shutdown()

    out = a.out or os.path.join('sysid_logs', time.strftime('%Y-%m-%d_%H%M%S') + f'_m{motor["motor_id"]}_latency')
    os.makedirs(out, exist_ok=True)
    write_csv(os.path.join(out, 'can.csv'), ['t', 'kind', 'q', 'v', 'tau', 'kp', 'kd', 'mode'], sn.rows)
    write_csv(os.path.join(out, 'obs.csv'), ['t'], ev['obs'])
    write_csv(os.path.join(out, 'targets.csv'), ['t', 'stamp', 'joint_target'], ev['targets'])
    write_csv(os.path.join(out, 'motor_cmd.csv'), ['t', 'motor_target'], ev['motor_cmd'])
    write_csv(os.path.join(out, 'actual.csv'), ['t', 'motor_position'], ev['actual'])
    meta = dict(motor=motor, joint=joint, t_start=t_start, t_end=t_end, walk=a.walk,
                armed=not a.no_arm, modes=ev['mode'], config=a.config, host=socket.gethostname())
    with open(os.path.join(out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f'logs in {out}')
    return analyze(out, None)


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


# ============================================================================= analyze
def read_csv(path):
    with open(path) as f:
        r = csv.reader(f)
        next(r)
        return [row for row in r]


def pct(xs, p):
    if not xs:
        return float('nan')
    s = sorted(xs)
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def stats(xs):
    return dict(n=len(xs), p50=pct(xs, 50), p90=pct(xs, 90), p99=pct(xs, 99),
                min=min(xs) if xs else float('nan'), max=max(xs) if xs else float('nan'))


def last_before(ts, t):
    """Index of the last element of sorted `ts` <= t, or None."""
    i = bisect.bisect_right(ts, t) - 1
    return i if i >= 0 else None


def analyze(path, plots):
    with open(os.path.join(path, 'meta.json')) as f:
        meta = json.load(f)
    t0, t1 = meta['t_start'] + 1.0, meta['t_end']   # skip the arming transient
    can = read_csv(os.path.join(path, 'can.csv'))
    cmds = [(float(r[0]), float(r[2])) for r in can if r[1] == 'cmd']
    replies = sorted(float(r[0]) for r in can if r[1] == 'reply')
    obs = sorted(float(r[0]) for r in read_csv(os.path.join(path, 'obs.csv')))
    targets = [(float(r[0]), float(r[1])) for r in read_csv(os.path.join(path, 'targets.csv'))]
    mcmd = [(float(r[0]), float(r[1])) for r in read_csv(os.path.join(path, 'motor_cmd.csv'))]
    actual = sorted(float(r[0]) for r in read_csv(os.path.join(path, 'actual.csv')))
    cmd_t = [c[0] for c in cmds]

    ms = lambda x: x * 1e3  # noqa: E731
    res = dict(tick=[], inference=[], to_bus=[], obs_reply_age=[], obs_ros_age=[], state_age=[],
               cmd_period=[], reply_period=[])
    used = set()
    timeline = []
    mcmd.sort()
    mcmd_t = [m[0] for m in mcmd]
    for _, stamp in targets:
        if not t0 <= stamp <= t1:
            continue
        # Each tick's observation produces one targets message. Inference (~0.3 ms) can beat this
        # script's own receipt of the observation, so pair by nearest, not "last before": that
        # would credit the previous tick and add a phantom 20 ms. The tick happened no later than
        # either receipt, so take the earlier as its time.
        io = bisect.bisect_left(obs, stamp)
        near = [i for i in (io - 1, io) if 0 <= i < len(obs) and -0.010 <= obs[i] - stamp <= 0.005]
        if not near:
            continue
        t_obs = min(obs[min(near, key=lambda i: abs(obs[i] - stamp))], stamp)
        res['inference'].append(ms(stamp - t_obs))
        # The bridge forwards each target to the motor topic at once (in WALK); the motor node
        # sends it straight on CAN. Match by value: the first command on the bus after the policy
        # published, carrying the forwarded motor target (u16 resolution ~0.4 mrad).
        im = bisect.bisect_left(mcmd_t, stamp)
        if im >= len(mcmd) or mcmd[im][0] - stamp > 0.05:
            continue
        target = mcmd[im][1]
        ic = bisect.bisect_left(cmd_t, stamp)
        hit = None
        for k in range(ic, min(ic + 20, len(cmds))):
            if cmds[k][0] - stamp > 0.05:
                break
            if k not in used and abs(cmds[k][1] - target) < 2e-3:
                hit = k
                break
        if hit is None:
            continue
        used.add(hit)
        t_bus = cmds[hit][0]
        res['to_bus'].append(ms(t_bus - stamp))
        ir = last_before(replies, t_obs)
        if ir is not None:
            res['obs_reply_age'].append(ms(t_obs - replies[ir]))
            res['state_age'].append(ms(t_bus - replies[ir]))
            timeline.append((replies[ir], t_obs, stamp, t_bus))
        ia = last_before(actual, t_obs)
        if ia is not None:
            res['obs_ros_age'].append(ms(t_obs - actual[ia]))
    win = [t for t in obs if t0 <= t <= t1]
    res['tick'] = [ms(b - a) for a, b in zip(win, win[1:])]
    win = [t for t in cmd_t if t0 <= t <= t1]
    res['cmd_period'] = [ms(b - a) for a, b in zip(win, win[1:])]
    win = [t for t in replies if t0 <= t <= t1]
    res['reply_period'] = [ms(b - a) for a, b in zip(win, win[1:])]

    names = [
        ('tick', 'bridge tick period (observation to observation)'),
        ('inference', 'observation -> policy targets published (inference; 0 = faster than we see)'),
        ('to_bus', 'targets published -> MIT command on CAN (bridge + motor node)'),
        ('obs_reply_age', 'position age at the observation (last CAN reply -> tick)'),
        ('obs_ros_age', '  same, from the last actual_position message'),
        ('state_age', 'STATE AGE AT COMMAND: CAN reply -> the command computed from it'),
        ('cmd_period', 'MIT command period on CAN'),
        ('reply_period', 'status reply period on CAN'),
    ]
    summary = {k: stats(v) for k, v in res.items()}
    print(f'\n=== {os.path.basename(os.path.normpath(path))}: {meta["joint"]} (motor {meta["motor"]["motor_id"]}, '
          f'{meta["motor"]["motor_model"]}), {t1 - t0:.0f} s analysed, {len(res["state_age"])} matched cycles')
    print(f'{"":64s}{"p50":>7s}{"p90":>7s}{"p99":>7s}{"max":>7s}  ms')
    for k, label in names:
        s = summary[k]
        print(f'{label:64s}{s["p50"]:7.2f}{s["p90"]:7.2f}{s["p99"]:7.2f}{s["max"]:7.2f}')
    sa = summary['state_age']
    tot = [x + DRIVE_MS for x in res['state_age']]
    print(f'\nstate measured -> torque at the motor = state age + ~{DRIVE_MS} ms in the drive: '
          f'p50 {pct(tot, 50):.1f}, p90 {pct(tot, 90):.1f}, max {max(tot) if tot else float("nan"):.1f} ms')
    print(f'sim (v6): fresh observation, action applied {SIM_DELAY_MS[0]:g}..{SIM_DELAY_MS[1]:g} ms later '
          f'(uniform per episode). Real p90 {"inside" if pct(tot, 90) <= SIM_DELAY_MS[1] else "OUTSIDE"} that range.')
    if not res['state_age']:
        print('no matched cycles: was the bridge armed (WALK) and the policy publishing?')
    summary['state_to_torque_ms'] = stats(tot)
    summary['sim_delay_range_ms'] = SIM_DELAY_MS
    # Missed ticks: the motor node drops to damping after target_timeout (0.2 s) without one.
    stalls = [g for g in res['tick'] if g > 30.0]
    summary['tick_stalls_over_30ms'] = stalls
    print(f'bridge stalls (tick gap > 30 ms): {len(stalls)} in {t1 - t0:.0f} s'
          + (f', worst {max(stalls):.0f} ms' + ('  <- longer than the 200 ms target timeout: the '
                                                 'joint went to damping' if max(stalls) > 200 else '')
             if stalls else ''))
    with open(os.path.join(path, 'latency_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)
    if plots:
        plot(path, res, timeline, plots, meta)
    return 0


def plot(path, res, timeline, out, meta):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(out, exist_ok=True)
    fig, ax = plt.subplots(1, 3, figsize=(18, 4.8))
    for k, label in (('inference', 'inference'), ('to_bus', 'targets -> CAN'),
                     ('obs_reply_age', 'position age at tick'), ('state_age', 'state age at command')):
        if res[k]:
            ax[0].hist(res[k], bins=60, histtype='step', lw=1.5, label=label)
    ax[0].axvspan(*SIM_DELAY_MS, color='gray', alpha=.12, label='sim action delay (v6)')
    ax[0].set(xlabel='ms', ylabel='cycles', title='Delay per stage')
    ax[0].legend(fontsize=8)
    for k, label in (('tick', 'bridge tick'), ('cmd_period', 'CAN command'), ('reply_period', 'CAN reply')):
        if res[k]:
            ax[1].hist(res[k], bins=80, histtype='step', lw=1.5, label=label)
    ax[1].set(xlabel='ms', ylabel='count', title='Periods (jitter)')
    ax[1].legend(fontsize=8)
    t0 = timeline[0][0] if timeline else 0.0
    for i, (tr, to, tp, tb) in enumerate(timeline[:12]):
        y = i
        ax[2].plot([ms_(tr - t0), ms_(tb - t0)], [y, y], '-', color='gray', lw=1)
        for x, c, lab in ((tr, 'C0', 'CAN reply (state)'), (to, 'C1', 'bridge tick'),
                          (tp, 'C2', 'targets published'), (tb, 'C3', 'command on CAN')):
            ax[2].plot(ms_(x - t0), y, 'o', color=c, label=lab if i == 0 else None)
    ax[2].set(xlabel='ms', ylabel='cycle', title='First cycles: state -> command')
    ax[2].legend(fontsize=8, loc='lower right')
    for a_ in ax:
        a_.grid(alpha=.3)
    fig.suptitle(f'{meta["joint"]} (motor {meta["motor"]["motor_id"]}): policy loop latency')
    fig.tight_layout()
    name = os.path.join(out, os.path.basename(os.path.normpath(path)) + '.png')
    fig.savefig(name, dpi=110)
    print(f'plot: {name}')


def ms_(x):
    return x * 1e3


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('setup', help='write a one-motor robot config (no motor node running)')
    s.add_argument('--id', type=int, required=True)
    s.add_argument('--robot', default=ROBOT)
    s.add_argument('--spec', default=SPEC)
    s.add_argument('--iface', default='auto', help='auto = the interface the motor answers on')
    s.add_argument('--out')
    r = sub.add_parser('record', help='arm, run the policy, log CAN + ROS timing, disarm')
    r.add_argument('--config', required=True, help='the yaml from setup (also given to robot.launch.py)')
    r.add_argument('--seconds', type=float, default=30.0)
    r.add_argument('--walk', type=float, default=0.0, help='cmd_vel linear.x while recording')
    r.add_argument('--iface', help="default: the motor's can_interface in --config")
    r.add_argument('--no-arm', action='store_true', help='arm and disarm yourself (gamepad)')
    r.add_argument('--arm-event', default='START pressed')
    r.add_argument('--disarm-event', default='BACK pressed')
    r.add_argument('--button-topic', default='/gamepad/button_event')
    r.add_argument('--out')
    n = sub.add_parser('analyze', help='report (and plot) a recorded run')
    n.add_argument('path')
    n.add_argument('--plots')
    a = ap.parse_args(argv)
    if a.cmd == 'setup':
        return cmd_setup(a)
    if a.cmd == 'record':
        return cmd_record(a)
    return analyze(a.path, a.plots)


if __name__ == '__main__':
    sys.exit(main())
