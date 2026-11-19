#!/usr/bin/env python3
"""Fit the actuator model to chirp.py logs (desktop: numpy + scipy, matplotlib for --plots).

Model, the same one training simulates for each joint (no load on the output):
    tau_cmd = kp_s * kp (q_ref - q) + kd_s * kd (v_ref - v) + tau_ff     (commands delayed by d)
    tau     = clip(tau_cmd, +/- drive torque limit)
    J dv/dt = tau - b v - Fc tanh(v / 0.01)
J is in units of *commanded* torque: if the drive's torque constant is off by 10 %, J absorbs it,
which is what sim wants, since sim also applies the commanded torque.

What each run gives:
    friction      Fc, b from steady velocity holds (and kd / torque-estimate check)
    torque-chirp  J and the delay d (Fc, b from the friction run when given), plus Bode
    pos-chirp     validation at the deploy gains: sim vs real Bode and fit errors; also fits
                  kp/kd scale factors (1.0 = the drive applies the gains we send)
    step          validation in the time domain
    top-speed     no-load speed at the bus voltage
    stall         locked output, rod on a scale: true / commanded torque (the torque gain k),
                  where the torque saturates, the friction band (up vs down steps), the torque
                  constant Kt (torque / iq), and how far the drive's own torque estimate is off.
                  Sim applies commanded torque, so k is the centre of motor-strength randomisation,
                  and the output-side inertia is J * k
    stall-hold    constant locked torque: heating rate and whether the torque drops as it warms

  python3 tools/walker/sysid/fit.py sysid_logs/*_m14_* --plots plots/ --out m14.yaml
  python3 tools/walker/sysid/fit.py sysid_logs/* --out actuators.yaml    # all motors + per-model summary
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

VS = 0.01          # rad/s, Coulomb smoothing
DT = 5e-4          # s, fit integration step
NO_LOAD_48V = {'RS03': 20.4, 'RS02': 42.9, 'RS06': 50.3}
# Starting point and bounds (0.1x..10x) for the inertia fit. Measured: RS03 0.066..0.074 (m21), RS02 0.011..0.013 (m13).
J_GUESS = {'RS03': 0.07, 'RS02': 0.012, 'RS06': 0.008}


# ============================================================================= loading
@dataclass
class Run:
    path: str
    meta: dict
    cmd: np.ndarray   # [N, 6] t, q_ref, v_ref, kp, kd, tau_ff
    fb: np.ndarray    # [M, 5] t, q, v, tau, temp   (type 2 replies only)

    @property
    def test(self):
        return self.meta['test']

    @property
    def name(self):
        return os.path.basename(os.path.normpath(self.path))


def load_run(path) -> Run | None:
    try:
        with open(os.path.join(path, 'meta.json')) as f:
            meta = json.load(f)
        cmd = np.loadtxt(os.path.join(path, 'cmd.csv'), delimiter=',', skiprows=1, ndmin=2)
        fb = np.loadtxt(os.path.join(path, 'fb.csv'), delimiter=',', skiprows=1, ndmin=2)
    except (OSError, ValueError) as e:
        print(f'skip {path}: {e}')
        return None
    if len(cmd) < 100 or len(fb) < 100:
        print(f'skip {path}: only {len(cmd)} commands / {len(fb)} replies')
        return None
    fb = fb[fb[:, 1] == 2][:, [0, 2, 3, 4, 5]]
    if meta.get('aborted'):
        print(f'note {os.path.basename(path)}: aborted ({meta["aborted"]}); using the part before it')
    return Run(path, meta, cmd, fb)


# ============================================================================= simulation
class WindowSim:
    """Simulates many short windows in parallel, each started from the measured state, so slow
    drift (free rotor, tiny centering spring) can't accumulate. Residuals at the reply times."""

    def __init__(self, run: Run, t_len: float, t_skip: float = 0.05):
        self.run = run
        c, fb = run.cmd, run.fb
        t_first, t_last = max(c[0, 0], fb[0, 0]) + t_skip, min(c[-1, 0], fb[-1, 0])
        n_w = max(1, int((t_last - t_first) // t_len))
        self.ts = t_first + t_len * np.arange(n_w)
        self.n_s = int(round(t_len / DT))
        self.tg = self.ts[:, None] + DT * np.arange(self.n_s + 1)[None, :]
        self.q0 = np.interp(self.ts, fb[:, 0], fb[:, 1])
        self.v0 = np.interp(self.ts, fb[:, 0], fb[:, 2])
        # reply samples inside windows -> (window, fractional step)
        rel = fb[:, 0] - t_first
        w = np.floor(rel / t_len).astype(int)
        ok = (rel >= 0) & (w < n_w)
        s = (rel[ok] - w[ok] * t_len) / DT
        self.w, self.s0 = w[ok], np.minimum(np.floor(s).astype(int), self.n_s - 1)
        self.frac = s - self.s0
        self.q_meas, self.v_meas = fb[ok, 1], fb[ok, 2]
        self.t_meas = fb[ok, 0]
        self.lim = float(run.meta['torque_limit'])

    def commands(self, delay):
        c = self.run.cmd
        idx = np.clip(np.searchsorted(c[:, 0] + delay, self.tg, side='right') - 1, 0, len(c) - 1)
        return [c[idx, k] for k in range(1, 6)]

    def simulate(self, p):
        qr, vr, kp, kd, tff = self.commands(p['delay'])
        J, b, Fc = p['J'], p['b'], p['Fc']
        kps, kds = p.get('kp_scale', 1.0), p.get('kd_scale', 1.0)
        q, v = self.q0.copy(), self.v0.copy()
        Q = np.empty_like(self.tg)
        V = np.empty_like(self.tg)
        Q[:, 0], V[:, 0] = q, v
        for s in range(self.n_s):
            tau = kps * kp[:, s] * (qr[:, s] - q) + kds * kd[:, s] * (vr[:, s] - v) + tff[:, s]
            np.clip(tau, -self.lim, self.lim, out=tau)
            v = v + DT * (tau - b * v - Fc * np.tanh(v / VS)) / J
            q = q + DT * v
            Q[:, s + 1], V[:, s + 1] = q, v
        return Q, V

    def at_meas(self, X):
        return X[self.w, self.s0] * (1 - self.frac) + X[self.w, self.s0 + 1] * self.frac

    def residuals(self, p, wq, wv):
        Q, V = self.simulate(p)
        return np.concatenate([(self.at_meas(Q) - self.q_meas) / wq, (self.at_meas(V) - self.v_meas) / wv])


def fit(run, base, free, bounds, t_len, delays=None, label=''):
    """least_squares over `free`, with the delay found by grid search (it is piecewise constant
    for the integrator, so it has no useful gradient). Returns (params, rms dict)."""
    ws = WindowSim(run, t_len)
    wq = max(np.std(ws.q_meas - np.mean(ws.q_meas)), 1e-3)
    wv = max(np.std(ws.v_meas), 1e-2)
    x0 = np.array([base[k] for k in free])
    lo = np.array([bounds[k][0] for k in free])
    hi = np.array([bounds[k][1] for k in free])
    x0 = np.clip(x0, lo + 1e-9, hi - 1e-9)

    def solve(d, x_init):
        if not free:
            p = {**base, 'delay': d}
            r = ws.residuals(p, wq, wv)
            return float(r @ r), x_init
        res = least_squares(lambda x: ws.residuals({**base, **dict(zip(free, x)), 'delay': d}, wq, wv),
                            x_init, bounds=(lo, hi), x_scale='jac', max_nfev=40)
        return 2 * res.cost, res.x

    if delays is None:
        delays = [base['delay']]
    results = []
    for d in delays:
        cost, x = solve(d, x0)
        results.append((cost, d, x))
    cost, d_best, x_best = min(results, key=lambda r: r[0])
    if len(delays) > 1:  # refine around the coarse optimum
        step = delays[1] - delays[0]
        for d in np.arange(d_best - step, d_best + step + 1e-9, step / 4):
            if d >= 0:
                c2, x2 = solve(d, x_best)
                if c2 < cost:
                    cost, d_best, x_best = c2, d, x2
    p = {**base, **dict(zip(free, x_best)), 'delay': float(d_best)}
    Q, V = ws.simulate(p)
    eq, ev = ws.at_meas(Q) - ws.q_meas, ws.at_meas(V) - ws.v_meas
    rms = dict(q_mrad=float(np.sqrt(np.mean(eq ** 2)) * 1e3), v=float(np.sqrt(np.mean(ev ** 2))),
               q_signal_mrad=float(wq * 1e3), v_signal=float(wv))
    if label:
        print(f'  {label}: ' + ', '.join(f'{k} {fmt(p[k], k)}' for k in [*free, 'delay'])
              + f'  | rms q {rms["q_mrad"]:.2f} mrad (signal {rms["q_signal_mrad"]:.0f}), '
                f'v {rms["v"]:.3f} rad/s (signal {wv:.2f})')
    return p, rms, ws, (Q, V)


def fmt(x, k):
    if k == 'delay':
        return f'{x * 1e3:.2f} ms'
    if k == 'J':
        return f'{x:.5f}'
    return f'{x:.4g}'


# ============================================================================= frequency response
def zoh(t_cmd, x_cmd, t):
    i = np.clip(np.searchsorted(t_cmd, t, side='right') - 1, 0, len(t_cmd) - 1)
    return x_cmd[i]


def bode(t, x, y, f0, f1, n_bins=24, fs=500.0):
    """Empirical transfer function y/x averaged in log-spaced bins over [f0, f1]."""
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    xu = np.interp(tu, t, x) - np.mean(x)
    yu = np.interp(tu, t, y) - np.mean(y)
    X, Y = np.fft.rfft(xu), np.fft.rfft(yu)
    f = np.fft.rfftfreq(len(tu), 1.0 / fs)
    edges = np.geomspace(f0, f1, n_bins + 1)
    fc, H = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (f >= lo) & (f < hi)
        den = np.sum(np.abs(X[m]) ** 2)
        if m.sum() < 2 or den <= 0:
            continue
        fc.append(math.sqrt(lo * hi))
        H.append(np.sum(Y[m] * np.conj(X[m])) / den)
    H = np.array(H)
    return np.array(fc), 20 * np.log10(np.abs(H) + 1e-12), np.degrees(np.unwrap(np.angle(H)))


def run_bode(run, ws, Q, input_col, f_lo, f_hi):
    """Measured and simulated Bode for a chirp run (input = q_ref or tau_ff)."""
    t = ws.t_meas
    x = zoh(run.cmd[:, 0], run.cmd[:, input_col], t)
    y_meas = ws.q_meas
    y_sim = ws.at_meas(Q)
    f, g_m, p_m = bode(t, x, y_meas, f_lo, f_hi)
    _, g_s, p_s = bode(t, x, y_sim, f_lo, f_hi)
    return f, g_m, p_m, g_s, p_s


# ============================================================================= per-test analysis
def analyze_friction(run, out, plots):
    c, fb = run.cmd, run.fb
    # constant-velocity holds
    change = np.flatnonzero(np.diff(c[:, 2]) != 0) + 1
    bounds = np.concatenate([[0], change, [len(c)]])
    rows = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        t0, t1, vr, kd = c[a, 0], c[b - 1, 0], c[a, 2], c[a, 4]
        if t1 - t0 < 0.8 or vr == 0:
            continue
        m = (fb[:, 0] > t0 + 0.5 * (t1 - t0)) & (fb[:, 0] <= t1)
        if m.sum() < 20:
            continue
        v = float(np.mean(fb[m, 2]))
        rows.append(dict(v_ref=float(vr), v=v, tau_meas=float(np.mean(fb[m, 3])),
                         tau_cmd=float(kd * (vr - v)), moving=abs(v) > 0.3 * abs(vr)))
    mov = [r for r in rows if r['moving']]
    if len(mov) < 4:
        print('  friction: too few moving holds to fit')
        return {}
    v = np.array([r['v'] for r in mov])
    res = {}
    for src in ('tau_cmd', 'tau_meas'):
        tau = np.array([r[src] for r in mov])
        A = np.stack([np.sign(v), v], 1)
        (Fc, b), *_ = np.linalg.lstsq(A, tau, rcond=None)
        pos, neg = v > 0, v < 0
        Fp = np.linalg.lstsq(A[pos], tau[pos], rcond=None)[0][0] if pos.sum() >= 2 else float('nan')
        Fn = np.linalg.lstsq(A[neg], tau[neg], rcond=None)[0][0] if neg.sum() >= 2 else float('nan')
        res[src] = dict(Fc=float(Fc), b=float(b), Fc_pos=float(Fp), Fc_neg=float(Fn))
    tc = np.array([r['tau_cmd'] for r in mov])
    tm = np.array([r['tau_meas'] for r in mov])
    ratio = float(tm @ tc / (tc @ tc))
    stuck = [r['v_ref'] for r in rows if not r['moving']]
    f = res['tau_cmd']
    print(f'  friction (from kd (v_ref - v), the torque sim sees): Fc {f["Fc"]:.3f} Nm '
          f'(+{f["Fc_pos"]:.3f} / -{f["Fc_neg"]:.3f}), b {f["b"]:.4f} Nm s/rad')
    print(f'  friction (from the drive torque estimate): Fc {res["tau_meas"]["Fc"]:.3f} Nm, '
          f'b {res["tau_meas"]["b"]:.4f}.  estimate / commanded torque = {ratio:.2f} '
          f'(1.0: kd scaling and torque estimate agree)')
    if stuck:
        print(f'  did not break away at |v_ref| <= {max(abs(s) for s in stuck):g} rad/s: static friction '
              f'> kd * v_ref there')
    out.update(Fc=f['Fc'], b=f['b'], Fc_pos=f['Fc_pos'], Fc_neg=f['Fc_neg'],
               Fc_from_estimate=res['tau_meas']['Fc'], b_from_estimate=res['tau_meas']['b'],
               torque_estimate_ratio=ratio,
               friction_points=[dict(v=r['v'], tau=r['tau_cmd'], tau_drive=r['tau_meas'],
                                     moving=r['moving']) for r in rows])
    if plots:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot([r['v'] for r in rows], [r['tau_cmd'] for r in rows], 'o', label='kd (v_ref - v)')
        ax.plot([r['v'] for r in rows], [r['tau_meas'] for r in rows], 'x', label='drive estimate')
        vv = np.linspace(min(r['v'] for r in rows), max(r['v'] for r in rows), 200)
        ax.plot(vv, f['Fc'] * np.tanh(vv / VS) + f['b'] * vv, 'k-', lw=1, label='fit')
        ax.set(xlabel='velocity (rad/s)', ylabel='torque (Nm)', title=run.name)
        ax.grid(alpha=.3)
        ax.legend()
        save(fig, plots, run.name)
    return res


def bode_inertia_delay(f, g_db, phase_deg):
    """Free rotor: q/tau = e^(-s d) / (J s^2) once friction is small next to inertia. J from the
    magnitude (f >= 3 Hz), d from the slope of the phase beyond -180 deg (f >= 8 Hz). Independent
    of the time-domain fit, which stick-slip friction can pull around. -> (J, d) or (None, None)."""
    m = f >= 3.0
    if m.sum() < 3:
        return None, None
    w = 2 * np.pi * f[m]
    J = float(np.exp(np.mean(np.log(1.0 / (10 ** (g_db[m] / 20) * w ** 2)))))
    ph = phase_deg - 360.0 * np.round((np.median(phase_deg[m]) + 180.0) / 360.0)
    h = f >= 8.0
    if h.sum() < 3:
        return J, None
    slope = np.polyfit(f[h], ph[h], 1)[0]   # deg per Hz
    return J, float(max(0.0, -slope / 360.0))


def analyze_torque_chirp(run, prior, out, plots):
    model = run.meta['model']
    jg = run.meta['profile'].get('j_guess') or J_GUESS[model]  # what the run was sized for
    base = dict(J=jg, b=prior.get('b', 0.01), Fc=prior.get('Fc', 0.1), delay=0.0)
    bounds = dict(J=(0.1 * jg, 10 * jg), b=(0.0, 0.5), Fc=(0.0, 3.0))
    free = ['J'] if 'Fc' in prior else ['J', 'b', 'Fc']
    delays = list(np.arange(0.0, 0.0161, 0.001))
    label = 'torque chirp' + (' (Fc, b fixed from friction run)' if 'Fc' in prior else '')
    p, rms, ws, (Q, V) = fit(run, base, free, bounds, t_len=0.5, delays=delays, label=label)
    prof = run.meta['profile']
    f, g_m, p_m, g_s, p_s = run_bode(run, ws, Q, 5, prof['f0'] * 1.2, prof['f1'] / 1.1)
    out.update(J=p['J'], delay=p['delay'], torque_chirp_rms=rms)
    jb, db = bode_inertia_delay(f, g_m, p_m)
    if jb is not None:
        print(f'  torque chirp, from the Bode plot instead: J {jb:.5f} (|q/tau| = 1/(J w^2), f >= 3 Hz), '
              f'delay {db * 1e3:.2f} ms (phase slope, f >= 8 Hz)')
        out.update(J_bode=jb, delay_bode=db)
    if 'Fc' not in prior:
        out.update(Fc_torque_chirp=p['Fc'], b_torque_chirp=p['b'])
    if plots:
        plot_run(run, ws, Q, f, (g_m, p_m), (g_s, p_s), 'q / tau_ff', plots)
    return p


def analyze_closed_loop(run, params, out, plots):
    """pos-chirp / step: how well the identified model predicts the deploy-gain behaviour."""
    base = {**params, 'kp_scale': 1.0, 'kd_scale': 1.0}
    t_len = 2.0 if run.test.startswith('pos-chirp') else 1.0
    _, rms0, ws, (Q0, _) = fit(run, base, [], {}, t_len=t_len, label=f'{run.test} predicted')
    p, rms, ws, (Q, _) = fit(run, base, ['kp_scale', 'kd_scale'],
                             dict(kp_scale=(0.5, 1.5), kd_scale=(0.2, 3.0)), t_len=t_len,
                             delays=list(np.arange(0.0, 0.0161, 0.002)), label=f'{run.test} refit')
    key = run.test.replace('-', '_').replace('@', '_zoh')
    entry = dict(predicted_rms=rms0, refit=dict(kp_scale=p['kp_scale'], kd_scale=p['kd_scale'],
                                                delay=p['delay'], rms=rms))
    if run.test.startswith('pos-chirp'):
        prof = run.meta['profile']
        hi = min(prof['f1'], 0.4 * prof['hold_rate']) if prof.get('hold_rate') else prof['f1']
        f, g_m, p_m, g_s, p_s = run_bode(run, ws, Q0, 1, prof['f0'] * 1.5, hi / 1.1)
        m = f <= 10.0
        if m.any():
            entry['bode_err_to_10Hz'] = dict(gain_db=float(np.max(np.abs(g_m[m] - g_s[m]))),
                                             phase_deg=float(np.max(np.abs(p_m[m] - p_s[m]))))
            e = entry['bode_err_to_10Hz']
            verdict = 'PASS' if e['gain_db'] <= 1.0 and e['phase_deg'] <= 10.0 else 'CHECK'
            print(f'  {run.test} sim vs real up to 10 Hz: max gain error {e["gain_db"]:.2f} dB, '
                  f'phase {e["phase_deg"]:.1f} deg -> {verdict} (target 1 dB / 10 deg)')
        bw = bandwidth(f, g_m)
        if bw:
            entry['bandwidth_hz_measured'] = bw
            print(f'  {run.test} measured -3 dB bandwidth: {bw:.1f} Hz')
        if plots:
            plot_run(run, ws, Q0, f, (g_m, p_m), (g_s, p_s), 'q / q_ref', plots)
    elif plots:
        plot_run(run, ws, Q0, None, None, None, None, plots)
    out.setdefault('validation', {})[key] = entry


def bandwidth(f, g):
    below = np.flatnonzero(g < -3.0)
    if not len(below) or below[0] == 0:
        return None
    i = below[0]
    return float(np.interp(-3.0, [g[i], g[i - 1]], [f[i], f[i - 1]]))


def analyze_top_speed(run, out):
    c, fb = run.cmd, run.fb
    vmax = run.meta['profile']['vmax']
    res = {}
    for sgn, key in ((1, 'pos'), (-1, 'neg')):
        hold = np.abs(c[:, 2] - sgn * vmax) < 1e-6
        if not hold.any():
            continue
        t0, t1 = c[hold, 0].min(), c[hold, 0].max()
        m = (fb[:, 0] > t0 + 0.4 * (t1 - t0)) & (fb[:, 0] <= t1)
        if m.sum() > 10:
            res[key] = (float(np.median(np.abs(fb[m, 2]))), float(np.median(fb[m, 3])))
    if not res:
        print('  top-speed: no hold phase found')
        return
    vbus = run.meta.get('vbus_start') or float('nan')
    model = run.meta['model']
    expected = NO_LOAD_48V[model] * vbus / 48.0
    speeds = [r[0] for r in res.values()]
    sat = max(speeds) > 0.97 * vmax
    print(f'  top speed at {vbus:.1f} V: ' + ', '.join(f'{k} {s:.2f} rad/s (torque {t:+.2f} Nm)'
                                                       for k, (s, t) in res.items())
          + f'; datasheet scaled to this voltage: {expected:.1f} rad/s'
          + ('  NOTE: reached the commanded speed, so the true top speed is higher (raise --vmax)'
             if sat else ''))
    out.update(top_speed=min(speeds), top_speed_vbus=vbus, top_speed_saturated_command=sat)


SCALE_N = {'kg': 9.80665, 'g': 0.00980665, 'lb': 4.4482216, 'N': 1.0}   # same as chirp.SCALE_UNITS


def stall_points(run):
    """Readings of a stall / stall-hold run -> dict of arrays. True torque is recomputed from the raw
    scale reading and the arm in meta.json, so a corrected arm length there takes effect."""
    rd = run.meta.get('readings') or []
    if not rd:
        return None
    arm = float(run.meta['profile']['arm'])
    col = (lambda k: np.array([float(r[k]) if r.get(k) is not None else np.nan for r in rd]))
    unit = rd[0]['unit']
    return dict(t=col('t'), cmd=col('tau_cmd'), true=col('reading') * SCALE_N[unit] * arm,
                drive=col('tau_drive'), iq=col('iq'), temp=col('temp'),
                up=np.array([r['leg'] != 'down' for r in rd]), arm=arm)


def through_origin(x, y):
    m = np.isfinite(x) & np.isfinite(y) & (np.abs(x) > 1e-9)
    return float(x[m] @ y[m] / (x[m] @ x[m])) if m.sum() >= 2 else float('nan')


def analyze_stall(run, out, plots):
    p = stall_points(run)
    if p is None or len(p['cmd']) < 3:
        print('  stall: fewer than 3 scale readings, nothing to fit')
        return
    cmd, true = p['cmd'], p['true']
    (k, c), *_ = np.linalg.lstsq(np.stack([cmd, np.ones_like(cmd)], 1), true, rcond=None)
    # The linear part (lower 70 % of the steps), then how far the top steps fall below it.
    lin = cmd <= 0.7 * cmd.max() + 1e-9
    k_lin, c_lin = (np.linalg.lstsq(np.stack([cmd[lin], np.ones(lin.sum())], 1), true[lin], rcond=None)[0]
                    if lin.sum() >= 2 else (k, c))
    top = cmd >= cmd.max() - 1e-9
    top_ratio = float(np.mean(true[top] / (k_lin * cmd[top] + c_lin)))
    # Friction band: at the same command, the down step reads higher than the up step, because the
    # gearbox's static friction first resists compressing the scale and then resists releasing it.
    band = []
    for lvl in np.unique(cmd):
        u, d = true[(cmd == lvl) & p['up']], true[(cmd == lvl) & ~p['up']]
        if len(u) and len(d):
            band.append((float(np.mean(d)) - float(np.mean(u))) / 2.0)
    kt = through_origin(p['iq'], true)
    est = through_origin(true, p['drive'])
    res = dict(torque_gain=float(k), torque_offset=float(c), torque_gain_linear=float(k_lin),
               top_step=float(cmd.max()), top_step_vs_linear=top_ratio,
               friction_band=float(np.mean(band)) if band else None, Kt=kt,
               drive_estimate_ratio=est, arm=p['arm'], n=int(len(cmd)),
               points=[dict(cmd=float(a), true=float(b), drive=float(d), iq=float(i), up=bool(u))
                       for a, b, d, i, u in zip(cmd, true, p['drive'], p['iq'], p['up'])])
    out['stall'] = res
    out['torque_gain'], out['Kt'] = res['torque_gain'], kt
    print(f'  stall: true torque = {k:.3f} x commanded {c:+.3f} Nm  ({len(cmd)} readings up to '
          f'{cmd.max():g} Nm, arm {p["arm"]:g} m)')
    print(f'  stall: top step {cmd.max():g} Nm gives {100 * top_ratio:.0f}% of the linear prediction'
          + ('  -> SATURATING: the drive no longer adds torque here' if top_ratio < 0.95 else ''))
    if band:
        print(f'  stall: friction band {np.mean(band):+.3f} Nm (half the down-minus-up gap at the same '
              f'command; about the static friction at the output, ~0 means none)')
    if np.isfinite(kt):
        print(f'  stall: torque constant {kt:.3f} Nm/A at the output (true torque / iq)')
    if np.isfinite(est):
        print(f'  stall: drive torque estimate / true torque = {est:.3f}'
              + ('' if abs(est - 1) < 0.05 else '  (status-frame torque is off: log iq, not tau, for torque)'))
    if plots:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4.5))
        ax.plot(cmd[p['up']], true[p['up']], 'o', label='scale, stepping up')
        ax.plot(cmd[~p['up']], true[~p['up']], 's', mfc='none', label='scale, stepping down')
        ax.plot(cmd, p['drive'], 'x', label='drive estimate')
        xx = np.linspace(0, cmd.max(), 50)
        ax.plot(xx, xx, 'k:', lw=1, label='1:1')
        ax.plot(xx, k * xx + c, 'k-', lw=1, label=f'fit {k:.3f}x {c:+.2f}')
        ax.set(xlabel='commanded torque (Nm)', ylabel='torque (Nm)', title=run.name)
        ax.grid(alpha=.3)
        ax.legend()
        save(fig, plots, run.name)


def analyze_stall_hold(run, out, plots):
    fb = run.fb
    hold = run.meta['profile']['hold_seconds']
    ramp = run.meta['profile']['ramp']
    m = (fb[:, 0] >= ramp) & (fb[:, 0] <= ramp + hold)
    if m.sum() < 100:
        print('  stall-hold: too little data in the hold')
        return
    t, temp, drive = fb[m, 0] - ramp, fb[m, 4], np.abs(fb[m, 3])
    first = t <= min(60.0, t[-1])
    rate = float(np.polyfit(t[first], temp[first], 1)[0] * 60.0) if first.sum() > 10 else float('nan')
    res = dict(torque=float(run.meta['profile']['levels'][0]), seconds=float(t[-1]),
               temp_start=float(temp[0]), temp_end=float(temp[-1]), heating_c_per_min=rate,
               drive_estimate_start=float(np.mean(drive[:50])), drive_estimate_end=float(np.mean(drive[-50:])))
    limit = run.meta.get('max_temp', 70.0)
    if rate > 0.05:   # below this the 0.1 C temperature steps are all there is
        res['minutes_to_limit_at_start_rate'] = float((limit - temp[-1]) / rate)
    p = stall_points(run)
    if p is not None and len(p['true']) >= 2:
        res['true_torque_start'], res['true_torque_end'] = float(p['true'][0]), float(p['true'][-1])
        res['true_torque_drop'] = float(1 - p['true'][-1] / p['true'][0])
    out.setdefault('stall_hold', []).append(res)
    print(f'  stall-hold {res["torque"]:g} Nm for {res["seconds"]:.0f} s: {res["temp_start"]:.1f} -> '
          f'{res["temp_end"]:.1f} C, {rate:.2f} C/min over the first minute'
          + (f', ~{res["minutes_to_limit_at_start_rate"]:.1f} more min to {limit:g} C at that rate'
             if 'minutes_to_limit_at_start_rate' in res else ''))
    if 'true_torque_drop' in res:
        print(f'  stall-hold: true torque {res["true_torque_start"]:.2f} -> {res["true_torque_end"]:.2f} Nm '
              f'({100 * res["true_torque_drop"]:+.1f}% drop as it warmed)')
    if plots:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        ax.plot(t, temp, label='temperature (C)')
        ax.set(xlabel='t (s)', ylabel='C', title=run.name)
        if p is not None and len(p['true']):
            ax2 = ax.twinx()
            ax2.plot(p['t'] - ramp, p['true'], 'o', color='C1', label='true torque (Nm)')
            ax2.set(ylabel='Nm')
        ax.grid(alpha=.3)
        save(fig, plots, run.name)


# ============================================================================= plots
def save(fig, plots, name):
    os.makedirs(plots, exist_ok=True)
    path = os.path.join(plots, name + '.png')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    import matplotlib.pyplot as plt
    plt.close(fig)
    print(f'  plot: {path}')


def plot_run(run, ws, Q, f, meas, sim, label, plots):
    import matplotlib.pyplot as plt
    n = 3 if f is not None else 1
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.6 * n))
    axes = np.atleast_1d(axes)
    ax = axes[0]
    ax.plot(ws.t_meas, ws.q_meas, lw=.8, label='measured')
    ax.plot(ws.t_meas, ws.at_meas(Q), lw=.8, label='model')
    ax.set(xlabel='t (s)', ylabel='q (rad)', title=run.name)
    ax.legend(loc='upper right')
    ax.grid(alpha=.3)
    if f is not None:
        axes[1].semilogx(f, meas[0], 'o-', ms=3, label='measured')
        axes[1].semilogx(f, sim[0], 's--', ms=3, label='model')
        axes[1].set(ylabel=f'|{label}| dB')
        axes[2].semilogx(f, meas[1], 'o-', ms=3)
        axes[2].semilogx(f, sim[1], 's--', ms=3)
        axes[2].set(ylabel='phase (deg)', xlabel='Hz')
        axes[1].legend()
        for a in axes[1:]:
            a.grid(alpha=.3, which='both')
    save(fig, plots, run.name)


# ============================================================================= driver
ORDER = {'friction': 0, 'top-speed': 1, 'torque-chirp': 2, 'pos-chirp': 3, 'step': 4, 'stall': 5,
         'stall-hold': 6}


def analyze_motor(mid, runs, plots):
    runs = sorted(runs, key=lambda r: (ORDER.get(r.test.split('@')[0], 9), r.name))
    model = runs[0].meta['model']
    print(f'\n=== motor {mid} ({model}): ' + ', '.join(r.test for r in runs))
    out = dict(motor_id=mid, model=model, runs=[r.name for r in runs],
               vbus=float(np.nanmean([r.meta.get('vbus_start') or np.nan for r in runs])))
    truth = next((r.meta['sim_truth'] for r in runs if r.meta.get('sim_truth')), None)
    if truth:
        out['sim_truth'] = truth
    prior, params = {}, None
    for r in runs:
        if r.test == 'friction':
            analyze_friction(r, out, plots)
            if 'Fc' in out:
                prior = dict(Fc=out['Fc'], b=out['b'])
        elif r.test == 'top-speed':
            analyze_top_speed(r, out)
        elif r.test == 'torque-chirp':
            params = analyze_torque_chirp(r, prior, out, plots)
        elif r.test == 'stall':
            analyze_stall(r, out, plots)
        elif r.test == 'stall-hold':
            analyze_stall_hold(r, out, plots)
    if 'J' in out and 'torque_gain' in out:
        # J came out in commanded-torque units; in true torque it is J * k.
        out['J_true_torque'] = out['J'] * out['torque_gain']
        print(f'  inertia in true-torque units: J {out["J"]:.5f} x k {out["torque_gain"]:.3f} = '
              f'{out["J_true_torque"]:.5f} (use this with motor strength = k in sim)')
    if params is None and runs and not all(r.test.startswith('stall') for r in runs):
        if any(r.test.startswith(('pos-chirp', 'step')) for r in runs):
            print('  no torque-chirp run: validating with training\'s datasheet values instead')
        params = dict(J=J_GUESS[model], b=prior.get('b', 0.0), Fc=prior.get('Fc', 0.0), delay=0.0)
    for r in runs:
        if r.test.startswith(('pos-chirp', 'step')):
            analyze_closed_loop(r, {k: params[k] for k in ('J', 'b', 'Fc', 'delay')}, out, plots)
    if truth:
        print('  sim truth: ' + ', '.join(f'{k} {fmt(v, k)}' for k, v in truth.items()))
    return out


def summarize(results):
    by_model = defaultdict(list)
    for r in results:
        by_model[r['model']].append(r)
    summary = {}
    print('\n=== per-model summary (feed into training; ranges = unit-to-unit spread) ===')
    for model, rs_ in sorted(by_model.items()):
        s = {}
        for k in ('J', 'Fc', 'b', 'delay', 'top_speed', 'torque_gain', 'Kt', 'J_true_torque'):
            xs = [r[k] for r in rs_ if k in r and r[k] is not None and np.isfinite(r[k])]
            if xs:
                s[k] = dict(mean=float(np.mean(xs)), min=float(np.min(xs)), max=float(np.max(xs)), n=len(xs))
        summary[model] = s
        line = ', '.join(f'{k} {fmt(v["mean"], k)} [{fmt(v["min"], k)}..{fmt(v["max"], k)}]'
                         for k, v in s.items())
        print(f'{model} (n={len(rs_)}): {line}')
        if 'J' in s:
            print(f'    mjlab MOTORS["{model}"]: armature={s["J"]["mean"]:.4f}, friction='
                  f'{s.get("Fc", {}).get("mean", float("nan")):.3f}'
                  + (f', no_load_speed (at {rs_[0]["vbus"]:.0f} V) = {s["top_speed"]["mean"]:.1f}'
                     if 'top_speed' in s else ''))
        if 'torque_gain' in s:
            g = s['torque_gain']
            print(f'    motor strength (true / commanded torque): {g["mean"]:.3f} [{g["min"]:.3f}..{g["max"]:.3f}]'
                  f' -> DelayedDCMotorCfg strength_range centred here, e.g. ({0.9 * g["min"]:.2f}, '
                  f'{1.1 * g["max"]:.2f})')
        if 'delay' in s:
            print(f'    host-observed delay {s["delay"]["mean"] * 1e3:.1f} ms = '
                  f'{s["delay"]["mean"] / 0.005:.1f} physics steps of 5 ms (CAN round trip only; '
                  f'the ROS pipeline adds its own)')
    print('Isaac Lab note: check how its joint friction is defined before plugging in Fc '
          '(PhysX joint friction is a coefficient, not a torque in Nm).')
    return summary


def to_builtin(x):
    if isinstance(x, dict):
        return {k: to_builtin(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [to_builtin(v) for v in x]
    if isinstance(x, np.generic):  # numpy floats, ints and bools
        return x.item()
    return x


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('runs', nargs='+', help='run folders written by chirp.py')
    ap.add_argument('--out', help='results file (.yaml or .json)')
    ap.add_argument('--plots', help='folder for PNG plots')
    a = ap.parse_args(argv)
    runs = [r for r in (load_run(p) for p in a.runs if os.path.isdir(p)) if r is not None]
    if not runs:
        print('no usable runs')
        return 1
    by_motor = defaultdict(list)
    for r in runs:
        by_motor[r.meta['motor_id']].append(r)
    results = [analyze_motor(mid, rr, a.plots) for mid, rr in sorted(by_motor.items())]
    summary = summarize(results)
    doc = to_builtin(dict(motors=results, per_model=summary))
    if a.out:
        with open(a.out, 'w') as f:
            if a.out.endswith(('.yaml', '.yml')):
                try:
                    import yaml
                    yaml.safe_dump(doc, f, sort_keys=False)
                except ImportError:
                    json.dump(doc, f, indent=1)
            else:
                json.dump(doc, f, indent=1)
        print(f'\nwrote {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
