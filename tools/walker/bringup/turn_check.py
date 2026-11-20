#!/usr/bin/env python3
"""Is any joint's calibration a full output turn off? RobStride drives can boot into a different
multi-turn count after a power cycle (the right hip pitch did it 3x on 2026-11-19), which shifts the
reading by 2 pi; the bridge would then ramp that joint through a whole turn at START (the speed trip
stops it). Reads the latest motor positions from a robot.launch.py log (no CAN traffic, safe while
the stack runs):
  python3 tools/walker/bringup/turn_check.py [bringup_logs/<launch>.log] [--robot <yaml>] [--fix]
Exit 0 = all in range, 1 = out of range (DO NOT press START), 2 = --fix rewrote offsets (restart).
--fix only touches joints that are exactly one turn off and in range after the fix; anything else
stays out of range and blocks.
"""
import argparse
import math
import os
import re
import shutil
import time

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("log", nargs="?", default=None)
ap.add_argument("--robot", default="src/th_bringup/config/walker_demo_robot.yaml")
ap.add_argument("--fix", action="store_true",
                help="rewrite the offset of any joint that is exactly one turn off (backup first); exit 2 if it did")
a = ap.parse_args()
log_path = a.log or os.path.join("bringup_logs", os.readlink("bringup_logs/latest.log"))
log = open(log_path, "rb").read().decode(errors="ignore")
bad, fixes = 0, {}
for m in yaml.safe_load(open(a.robot))["motors"]:
    ns = m["joint"].removesuffix("_joint")
    hits = re.findall(r"\[" + ns + r"\.robstride_motor_node\]: Actual position: (-?[\d.]+) rad", log)
    if not hits:
        print(f"{m['joint']:26s} no position in the log yet")
        continue
    raw = float(hits[-1])
    d = float(m["direction"])
    q = (raw - float(m["offset"])) / d
    lo, hi = float(m["lower"]), float(m["upper"])
    if lo - 0.3 <= q <= hi + 0.3:
        verdict = "ok"
    else:
        bad += 1
        k = min((-2, -1, 1, 2), key=lambda k: abs(q + k * 2 * math.pi * d - (lo + hi) / 2))
        fixed = float(m["offset"]) - k * 2 * math.pi
        q_fixed = (raw - fixed) / d
        verdict = f"OUT OF RANGE: likely {abs(k)} turn(s) off; offset {fixed:+.4f} would read {q_fixed:+.3f}"
        if abs(k) == 1 and lo - 0.3 <= q_fixed <= hi + 0.3:
            fixes[m["joint"]] = (float(m["offset"]), fixed)
    print(f"{m['joint']:26s} raw {raw:+8.3f}  offset {float(m['offset']):+8.4f}  joint {q:+7.3f}  "
          f"limits [{lo:+.2f}, {hi:+.2f}]  {verdict}")

if a.fix and fixes and len(fixes) == bad:
    shutil.copy(a.robot, a.robot + time.strftime(".bak_turnfix_%H%M%S"))
    lines = open(a.robot).read().split("\n")
    for joint, (old, new) in fixes.items():
        start = next(i for i, l in enumerate(lines) if re.match(r"\s*-?\s*joint:\s*" + re.escape(joint) + r"\s*(#.*)?$", l))
        idx = next(i for i in range(start + 1, len(lines)) if re.match(r"\s*offset:", lines[i]))
        indent = lines[idx][: len(lines[idx]) - len(lines[idx].lstrip())]
        lines[idx] = (f"{indent}offset: {new:.4f}  # auto turn-fix {time.strftime('%Y-%m-%d %H:%M')}: "
                      f"drive booted one output turn off (was {old:.4f})")
        print(f"FIXED {joint}: offset {old:+.4f} -> {new:+.4f} (backup next to {a.robot}); restart the stack")
    text = "\n".join(lines)
    check = yaml.safe_load(text)
    assert all(abs(float(m["offset"]) - fixes[m["joint"]][1]) < 1e-3 for m in check["motors"] if m["joint"] in fixes)
    open(a.robot, "w").write(text)
    raise SystemExit(2)
print("all joints in range" if not bad else f"{bad} joint(s) out of range: DO NOT press START")
raise SystemExit(1 if bad else 0)
