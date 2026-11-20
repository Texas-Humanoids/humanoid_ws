"""Scorecard: every packaged demo-walker policy side by side, from its validation files.

  python -m th_training.compare_demo [--policies <dir with cad_v1/, wide_v1/, ...>] [--md out.md]

Reads, per policy folder: sim2sim_sweep.json (MuJoCo), isaac_sweep.json + isaac_mu04.json (PhysX),
used_ranges.json. Prints one table. How to read it:
  falls          the number that matters most. Any fall in any condition is a reason not to deploy.
  fwd/back/turn  achieved / commanded, nominal. ~0.8+ is good; below 0.5 the policy isn't tracking.
  worst fwd/turn the same ratio, minimum over every condition: how badly it degrades.
  ankle peak     peak ankle torque / RS06 peak (36 Nm). Near 1.0 = saturating in pushes.
  hip-roll rms   RMS hip-roll torque / RS03 rated (20 Nm). Over ~0.8 = heat in long demos.
"""
from __future__ import annotations

import argparse
import json
import os

DEFAULT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "humanoid_ws_sysid", "policies", "walker_demo")

# segment index in eval_script.SCRIPT -> (label, axis index into (vx, vy, wz))
SEGS = {"fwd0.2": (1, 0), "fwd0.4": (2, 0), "back0.2": (4, 0), "turn+": (7, 2), "turn-": (8, 2)}


def _load(p):
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return None


def _ratio(run, seg):
    i, ax = SEGS[seg]
    segs = run.get("segments", [])
    if i >= len(segs):
        return None              # fell before reaching it
    s = segs[i]
    got = [s["vx"], s["vy"], s["wz"]][ax]
    cmd = s["cmd"][ax]
    return got / cmd if cmd else None


def summarize(d):
    mj = _load(os.path.join(d, "sim2sim_sweep.json")) or []
    isw = _load(os.path.join(d, "isaac_sweep.json"))
    imu = _load(os.path.join(d, "isaac_mu04.json"))
    isaac = (isw or {}).get("results", []) + (imu or {}).get("results", [])
    out = dict(name=os.path.basename(d))
    for tag, runs in (("mj", mj), ("is", isaac)):
        if not runs:
            continue
        falls = [f"{r['name']}@{r['fell_at']}s" for r in runs if not r["survived"]]
        out[f"{tag}_falls"] = f"{len(falls)}/{len(runs)}"
        out[f"{tag}_fall_list"] = falls
        nom = runs[0]
        for seg in SEGS:
            out[f"{tag}_{seg}"] = _ratio(nom, seg)
        for seg in ("fwd0.2", "turn+"):
            vals = [_ratio(r, seg) for r in runs]
            out[f"{tag}_worst_{seg}"] = min((v for v in vals if v is not None), default=None) if any(v is not None for v in vals) else None
        out[f"{tag}_ankle_peak"] = nom.get("peak_tau_frac", {}).get("ankle_pitch")
        out[f"{tag}_hiproll_rms"] = nom.get("rms_tau_over_rated", {}).get("hip_roll")
    return out


def fmt(v):
    return "--" if v is None else (f"{v:.2f}" if isinstance(v, float) else str(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policies", default=DEFAULT)
    ap.add_argument("--md", default=None)
    a = ap.parse_args()
    dirs = sorted(os.path.join(a.policies, x) for x in os.listdir(a.policies)
                  if os.path.isdir(os.path.join(a.policies, x)) and not x.startswith("."))
    rows = [summarize(d) for d in dirs]
    cols = [("policy", "name")]
    for tag, label in (("mj", "MuJoCo"), ("is", "Isaac")):
        cols += [(f"{label} falls", f"{tag}_falls"), (f"{label} fwd .2/.4", None), (f"{label} back", f"{tag}_back0.2"),
                 (f"{label} turn +/-", None), (f"{label} worst fwd/turn", None),
                 (f"{label} ankle pk", f"{tag}_ankle_peak"), (f"{label} roll rms", f"{tag}_hiproll_rms")]
    lines = ["| " + " | ".join(c for c, _ in cols) + " |", "|" + "---|" * len(cols)]
    for r in rows:
        cells = []
        for label, key in cols:
            tag = "mj" if label.startswith("MuJoCo") else "is"
            if key:
                cells.append(fmt(r.get(key)))
            elif "fwd .2/.4" in label:
                cells.append(f"{fmt(r.get(tag + '_fwd0.2'))}/{fmt(r.get(tag + '_fwd0.4'))}")
            elif "turn +/-" in label:
                cells.append(f"{fmt(r.get(tag + '_turn+'))}/{fmt(r.get(tag + '_turn-'))}")
            else:
                cells.append(f"{fmt(r.get(tag + '_worst_fwd0.2'))}/{fmt(r.get(tag + '_worst_turn+'))}")
        lines.append("| " + " | ".join(cells) + " |")
    text = "\n".join(lines)
    falls = [f"- {r['name']}: " + "; ".join(r.get("mj_fall_list", []) + [f"isaac {x}" for x in r.get("is_fall_list", [])])
             for r in rows if r.get("mj_fall_list") or r.get("is_fall_list")]
    text += "\n\nRatios are achieved / commanded (nominal). Falls:\n" + ("\n".join(falls) if falls else "- none")
    print(text)
    if a.md:
        open(a.md, "w").write(text + "\n")


if __name__ == "__main__":
    main()
