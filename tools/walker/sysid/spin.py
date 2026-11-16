#!/usr/bin/env python3
"""Spin one motor at a constant velocity: give an ID, a speed and a direction.

One spin:
  python3 tools/walker/sysid/spin.py 11 2 +          # motor 11, 2 rad/s, positive, 5 s
  python3 tools/walker/sysid/spin.py 14 1.5 - 10     # motor 14, 1.5 rad/s, negative, 10 s

Interactive (asks once, then one spin per line):
  python3 tools/walker/sysid/spin.py
  spin> 11 2 +
  spin> 15 4 - 8
  spin> q

Each spin: enable (torque limit, CAN timeout and the safety checks of chirp.py), ramp up over 1 s,
hold the speed, ramp down, damp, stop. Velocity control only (kp 0, kd = chirp.py's top-speed kd),
so the output turns freely: nothing may be attached. Prints the measured speed and the torque it
took (the friction at that speed). Logs go to sysid_logs/<date>_m<id>_spin/ like the other tests.
Speeds are capped just above the no-load speed at the bus voltage (--bus-voltage, default 36 V).
Direction is the motor's own frame (+ = increasing angle); a joint's sign also depends on its
`direction` in the robot yaml. Ctrl-C at any time damps and stops the motor.

Extra options go to chirp.py after the arguments, e.g. `spin.py 11 2 + 5 --ramp 2 --kd 1`
or `--model RS02` for an ID outside 11-15 / 21-25. Stop every motor node (robot.launch.py) first.
"""
import os
import shlex
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import chirp  # noqa: E402

DIRS = {'+': '+', 'pos': '+', 'positive': '+', '-': '-', 'neg': '-', 'negative': '-'}


def to_chirp_args(words, extra, yes=True):
    """['11', '2', '+', '8'] -> chirp.py arguments, or raise ValueError with a usage hint."""
    if len(words) not in (3, 4):
        raise ValueError('give: ID SPEED DIRECTION [SECONDS], e.g. 11 2 +')
    mid, speed, d = int(words[0]), float(words[1]), DIRS.get(words[2].lower())
    if d is None:
        raise ValueError(f'direction {words[2]!r}: use + or -')
    args = ['spin', '--id', str(mid), '--speed', str(abs(speed)), '--dir', d]
    if len(words) == 4:
        args += ['--duration', str(float(words[3]))]
    return args + extra + (['--yes'] if yes else [])


def run(words, extra, yes=True):
    try:
        args = to_chirp_args(words, extra, yes)
    except ValueError as e:
        print(e)
        return 2
    try:
        return chirp.main(args)
    except SystemExit as e:   # argparse errors (bad speed, unknown ID) end one spin, not the session
        return e.code if isinstance(e.code, int) else 2
    except KeyboardInterrupt:  # chirp.py has already damped and stopped the motor (its finally)
        print('\n(stopped: the motor was damped and stopped)')
        return 130


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    first_opt = next((i for i, w in enumerate(argv) if w.startswith('--')), len(argv))
    words, extra = argv[:first_opt], argv[first_opt:]
    if words:   # one spin; chirp.py asks for confirmation unless --yes is in extra
        return run(words, extra, yes='--yes' in extra or '--sim' in extra)
    print(__doc__.split('\n\n')[2])
    if '--sim' not in extra:
        print('Nothing attached to the outputs, housings clamped, hand on the power switch, '
              'motor nodes stopped?')
        if input('Start? [y/N] ').strip().lower() != 'y':
            return 1
    while True:
        try:
            line = input('spin> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if line.lower() in ('q', 'quit', 'exit'):
            return 0
        if not line:
            continue
        parts = shlex.split(line)
        opt = next((i for i, w in enumerate(parts) if w.startswith('--')), len(parts))
        run(parts[:opt], extra + parts[opt:])


if __name__ == '__main__':
    sys.exit(main())
