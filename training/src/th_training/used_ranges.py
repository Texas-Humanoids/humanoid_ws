"""Joint range a packaged policy actually uses, over the whole sim2sim sweep (writes used_ranges.json).
bringup.py `ranges` compares the robot's measured hard stops against this.

  python -m th_training.used_ranges <policy dir>
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import onnxruntime as ort

from th_training import sim2sim_demo as S
from th_training.eval_script import PUSHES, SCRIPT
from th_training.robots import walker_demo as R


def used_ranges(d: str) -> dict:
    spec = json.load(open(os.path.join(d, "policy_spec.json")))
    sess = ort.InferenceSession(os.path.join(d, "policy.onnx"), providers=["CPUExecutionProvider"])
    name = sess.get_inputs()[0].name
    lo, hi = np.full(10, 9.0), np.full(10, -9.0)
    for c in S.SWEEP:
        kw = {k: c[k] for k in ("kp_scale", "kd_scale", "friction", "strength", "voltage") if k in c}
        sim = S.DemoSim(spec, variant=spec.get("limits_variant", "cad"), delay_steps=c.get("delay", 0), **kw)
        k = 0
        for dur, vx, vy, wz in SCRIPT:
            for _ in range(int(dur / S.POLICY_DT)):
                a = sess.run(None, {name: sim.obs(np.array([vx, vy, wz]))[None]})[0][0]
                sim.step(a, push=PUSHES.get(k))
                k += 1
                q = sim.d.qpos[sim.qadr]
                lo, hi = np.minimum(lo, q), np.maximum(hi, q)
                if sim.fallen():
                    break
            if sim.fallen():
                break
    out = {j: [round(float(lo[i]), 3), round(float(hi[i]), 3)] for i, j in enumerate(R.JOINT_ORDER)}
    json.dump(out, open(os.path.join(d, "used_ranges.json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    for j, r in used_ranges(sys.argv[1]).items():
        print(f"{j:26s} [{r[0]:+.3f}, {r[1]:+.3f}]")
