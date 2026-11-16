#!/usr/bin/env python3
"""One-page summary figure for one motor's chirp.py runs (desktop: numpy, scipy, matplotlib).

Runs the same analysis as fit.py on each labelled set of runs (e.g. two bus voltages) and draws
what matters for training on one page: friction curve, inertia and delay from the torque chirp,
top speed, sim vs real at the deploy gains (Bode and a step), and a parameter table.

  python3 tools/walker/sysid/report.py --set 36V logs36/* --set 48V logs48/* --out m21_summary.png
"""
from __future__ import annotations

import argparse
import contextlib
import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fit  # noqa: E402

COLORS = ['#1f77b4', '#d62728', '#2ca02c', '#9467bd']


def analyze(label, dirs):
    runs = [r for r in (fit.load_run(d) for d in dirs if os.path.isdir(d)) if r is not None]
    if not runs:
        raise SystemExit(f'set {label}: no usable runs')
    mids = {r.meta['motor_id'] for r in runs}
    if len(mids) != 1:
        raise SystemExit(f'set {label}: runs from several motors {sorted(mids)}; one motor per report')
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fit.analyze_motor(mids.pop(), runs, None)
    print(f'--- {label}' + buf.getvalue())
    by_test = {}
    for r in runs:
        by_test.setdefault(r.test, r)
    return dict(label=label, out=out, runs=by_test)


def params(s):
    o = s['out']
    return dict(J=o['J'], b=o.get('b', 0.0), Fc=o.get('Fc', 0.0), delay=o['delay'])


def measured_bode(run, col, f_lo, f_hi):
    t = run.fb[:, 0]
    x = fit.zoh(run.cmd[:, 0], run.cmd[:, col], t)
    return fit.bode(t, x, run.fb[:, 1], f_lo, f_hi)


def panel_friction(ax, sets):
    for c, s in zip(COLORS, sets):
        o = s['out']
        pts = o.get('friction_points')
        if not pts:
            continue
        mv = [p for p in pts if p['moving']]
        st = [p for p in pts if not p['moving']]
        ax.plot([p['v'] for p in mv], [p['tau'] for p in mv], 'o', color=c, ms=5,
                label=f"{s['label']}: Fc {o['Fc']:.2f}, b {o['b']:.4f}")
        ax.plot([p['v'] for p in st], [p['tau'] for p in st], 'o', mfc='none', color=c, ms=6)
        vv = np.linspace(min(p['v'] for p in pts), max(p['v'] for p in pts), 300)
        ax.plot(vv, o['Fc'] * np.tanh(vv / fit.VS) + o['b'] * vv, '-', color=c, lw=1)
    ax.plot([], [], 'o', mfc='none', color='gray', label='did not break away (static)')
    ax.set(title='Friction (velocity holds, kd only)', xlabel='velocity (rad/s)', ylabel='torque (Nm)')
    ax.legend(fontsize=7, loc='upper left')


def panel_inertia(ax_mag, ax_ph, sets):
    for c, s in zip(COLORS, sets):
        r = s['runs'].get('torque-chirp')
        if r is None:
            continue
        p = r.meta['profile']
        f, g, ph = measured_bode(r, 5, p['f0'] * 1.2, p['f1'] / 1.1)
        o = s['out']
        ax_mag.semilogx(f, g, 'o', color=c, ms=4, label=f"{s['label']} measured")
        J = o.get('J_bode', o['J'])
        ff = np.geomspace(f[0], f[-1], 100)
        ax_mag.semilogx(ff, -20 * np.log10(J * (2 * np.pi * ff) ** 2), '-', color=c, lw=1,
                        label=f'1/(J w^2), J = {J:.4f}')
        ph = ph - 360.0 * np.round((np.median(ph[f >= 3]) + 180.0) / 360.0)
        ax_ph.semilogx(f, ph + 180.0, 'o', color=c, ms=4, label=f"{s['label']} measured")
        if o.get('delay_bode') is not None:
            ax_ph.semilogx(ff, -360 * ff * o['delay_bode'], '-', color=c, lw=1,
                           label=f"pure delay {o['delay_bode'] * 1e3:.1f} ms")
    ax_mag.set(title='Torque chirp: |q / tau_ff| (inertia)', xlabel='Hz', ylabel='dB')
    ax_mag.legend(fontsize=7)
    ax_ph.set(title='Torque chirp: phase beyond -180 deg (delay)', xlabel='Hz', ylabel='deg')
    ax_ph.legend(fontsize=7)


def panel_top_speed(ax, sets):
    for c, s in zip(COLORS, sets):
        r = s['runs'].get('top-speed')
        if r is None:
            continue
        t0 = r.cmd[0, 0]
        ax.plot(r.fb[:, 0] - t0, r.fb[:, 2], '-', color=c, lw=.8,
                label=f"{s['label']} measured ({r.meta.get('vbus_start', 0):.1f} V): "
                      f"{s['out'].get('top_speed', float('nan')):.1f} rad/s")
        ax.plot(r.cmd[:, 0] - t0, r.cmd[:, 2], ':', color=c, lw=.8)
        vb = r.meta.get('vbus_start')
        if vb:
            nl = fit.NO_LOAD_48V[r.meta['model']] * vb / 48.0
            ax.axhline(nl, color=c, ls='--', lw=.6)
            ax.axhline(-nl, color=c, ls='--', lw=.6)
    ax.plot([], [], ':', color='gray', label='command')
    ax.plot([], [], '--', color='gray', label='datasheet no-load at that voltage')
    ax.set(title='Top speed (kd only, no load)', xlabel='t (s)', ylabel='velocity (rad/s)')
    ax.legend(fontsize=7, loc='lower left')


def panel_pos_chirp(ax_mag, ax_ph, sets):
    for c, s in zip(COLORS, sets):
        r = s['runs'].get('pos-chirp')
        if r is None:
            continue
        p = r.meta['profile']
        ws = fit.WindowSim(r, 2.0)
        Q, _ = ws.simulate(params(s))
        f, g_m, p_m, g_s, p_s = fit.run_bode(r, ws, Q, 1, p['f0'] * 1.5, p['f1'] / 1.1)
        ax_mag.semilogx(f, g_m, 'o', color=c, ms=4, label=f"{s['label']} real")
        ax_mag.semilogx(f, g_s, '-', color=c, lw=1, label=f"{s['label']} model")
        ax_ph.semilogx(f, p_m, 'o', color=c, ms=4, label=f"{s['label']} real")
        ax_ph.semilogx(f, p_s, '-', color=c, lw=1, label=f"{s['label']} model")
        kp, kd = p['kp'], p['kd']
    ax_mag.axhline(-3, color='gray', lw=.6, ls=':')
    ax_mag.set(title=f'Position tracking at deploy gains (kp {kp:g}, kd {kd:g}): |q / q_ref|',
               xlabel='Hz', ylabel='dB')
    ax_mag.legend(fontsize=7)
    ax_ph.set(title='Position tracking: phase', xlabel='Hz', ylabel='deg')
    ax_ph.legend(fontsize=7)


def panel_step(ax, sets):
    for i, (c, s) in enumerate(zip(COLORS, sets)):
        r = s['runs'].get('step')
        if r is None:
            continue
        p = r.meta['profile']
        amp = max(p['amps'])
        # Sequence: lead, then +a, 0, -a, 0 for each amp: the largest + step starts here.
        k = list(p['amps']).index(amp)
        t_step = r.cmd[0, 0] + 0.5 + 4 * p['hold'] * k
        ws = fit.WindowSim(r, 1.0)
        Q, _ = ws.simulate(params(s))
        qs = ws.at_meas(Q)
        m = (ws.t_meas > t_step - 0.05) & (ws.t_meas < t_step + 0.5)
        q0 = np.interp(t_step, ws.t_meas, ws.q_meas)
        tt = (ws.t_meas[m] - t_step) * 1e3
        ax.plot(tt, ws.q_meas[m] - q0, '-', color=c, lw=1.5, label=f"{s['label']} real")
        ax.plot(tt, qs[m] - q0, '--', color=c, lw=1, label=f"{s['label']} model")
        if i == 0:
            ax.axhline(amp, color='gray', lw=.6, ls=':')
    ax.set(title=f'Step {amp:g} rad at deploy gains', xlabel='ms after the step', ylabel='q - q0 (rad)')
    ax.legend(fontsize=7, loc='lower right')


def panel_table(ax, sets):
    ax.axis('off')
    rows = [('bus voltage (V)', lambda o, s: f"{o.get('vbus', float('nan')):.1f}"),
            ('J, time fit / Bode', lambda o, s: f"{o['J']:.4f} / {o.get('J_bode', float('nan')):.4f}"),
            ('Fc Nm (+ / -)', lambda o, s: f"{o['Fc']:.3f} ({o['Fc_pos']:.2f} / {o['Fc_neg']:.2f})"),
            ('b Nm s/rad', lambda o, s: f"{o['b']:.4f}"),
            ('torque est / cmd', lambda o, s: f"{o.get('torque_estimate_ratio', float('nan')):.2f}"),
            ('delay ms, fit / Bode', lambda o, s: f"{o['delay'] * 1e3:.1f} / "
                                                  f"{(o.get('delay_bode') or float('nan')) * 1e3:.1f}"),
            ('top speed rad/s', lambda o, s: f"{o.get('top_speed', float('nan')):.2f}"),
            ('  same, scaled to 36 V', lambda o, s: f"{o.get('top_speed', float('nan')) * 36.0 / o.get('top_speed_vbus', float('nan')):.2f}"
                if o.get('top_speed') and not o.get('top_speed_saturated_command') else 'n/a (hit cmd)'),
            ('pos-chirp err to 10 Hz', lambda o, s: _bode_err(o)),
            ('bandwidth Hz', lambda o, s: f"{o.get('validation', {}).get('pos_chirp', {}).get('bandwidth_hz_measured', float('nan')):.1f}"),
            ('kp / kd scale (refit)', lambda o, s: _scales(o))]
    text = [['', *[s['label'] for s in sets]]]
    for name, fn in rows:
        line = [name]
        for s in sets:
            try:
                line.append(fn(s['out'], s))
            except (KeyError, TypeError, ZeroDivisionError):
                line.append('-')
        text.append(line)
    tb = ax.table(cellText=text, loc='center', cellLoc='left', colLoc='left')
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.35)
    o = sets[0]['out']
    ax.set_title(f"motor {o['motor_id']} ({o['model']}): identified parameters")


def _bode_err(o):
    e = o.get('validation', {}).get('pos_chirp', {}).get('bode_err_to_10Hz')
    if not e:
        return '-'
    ok = e['gain_db'] <= 1.0 and e['phase_deg'] <= 10.0
    return f"{e['gain_db']:.2f} dB / {e['phase_deg']:.1f} deg {'PASS' if ok else 'CHECK'}"


def _scales(o):
    r = o.get('validation', {}).get('pos_chirp', {}).get('refit')
    return f"{r['kp_scale']:.2f} / {r['kd_scale']:.2f}" if r else '-'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--set', nargs='+', action='append', required=True, metavar='LABEL_THEN_RUNS',
                    help='a label, then the run folders for it (repeat for more sets)')
    ap.add_argument('--out', required=True, help='PNG file')
    a = ap.parse_args(argv)
    sets = [analyze(s[0], s[1:]) for s in a.set]

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 4, figsize=(22, 10))
    panel_friction(ax[0, 0], sets)
    panel_inertia(ax[0, 1], ax[0, 2], sets)
    panel_top_speed(ax[0, 3], sets)
    panel_pos_chirp(ax[1, 0], ax[1, 1], sets)
    panel_step(ax[1, 2], sets)
    panel_table(ax[1, 3], sets)
    for a_ in ax.flat:
        a_.grid(alpha=.3, which='both')
    o = sets[0]['out']
    fig.suptitle(f"{o['model']} motor {o['motor_id']}: actuator system ID", fontsize=14)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(f'wrote {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
