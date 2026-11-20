"""ONNX metadata fix-up (+ deploy spec) for policies trained with explicit-PD actuators.

mjlab's exporter reads joint_stiffness / joint_damping from MuJoCo's actuator gain/bias params.
With DcMotorActuator the PD runs in torch and MuJoCo only sees a torque motor, so the exported
metadata says kp=1, kd=0. A deploy node that trusted it would run the robot with kp=1.
This rewrites those fields from the robot module's GAINS and adds the gait period and actuator details.
For walker_demo it also writes policy_spec.json next to the ONNX (the file th_bringup's
robot.launch.py reads), so every checkpoint is deployable as-is.

usage:  python -m th_training.export <policy.onnx> [--robot walker_v2|walker_demo] [--variant cad|wide]
"""
from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path

import onnx


def _robot(name: str):
    return importlib.import_module(f"th_training.robots.{name}")


def fix_metadata(onnx_path: str, robot: str = "walker_v2", gait_period: float = 0.72) -> None:
    R = _robot(robot)
    m = onnx.load(onnx_path, load_external_data=False)
    md = {p.key: p.value for p in m.metadata_props}
    names = md.get("joint_names", ",".join(R.JOINT_ORDER)).split(",")
    assert names == R.JOINT_ORDER, f"ONNX joint order {names} != JOINT_ORDER"
    md["joint_stiffness"] = ",".join(f"{R.gains_of(j)[0]:.3f}" for j in names)
    md["joint_damping"] = ",".join(f"{R.gains_of(j)[1]:.3f}" for j in names)
    md["effort_limit"] = ",".join(f"{R.MOTORS[R.MOTOR_OF_JOINT[j]].peak_torque:.1f}" for j in names)
    md["motor_model"] = ",".join(R.MOTOR_OF_JOINT[j] for j in names)
    md["bus_voltage"] = str(R.BUS_VOLTAGE)
    md["actuator"] = "explicit PD (MIT mode kp/kd above) + linear torque-speed clip"
    md["gait_period"] = f"{gait_period}"
    md["gait_clock"] = "phase = ((t - t_start) mod gait_period) / gait_period; obs += [sin(2 pi phase), cos(2 pi phase)]"
    md["policy_dt"] = "0.02"
    del m.metadata_props[:]
    for k, v in md.items():
        p = m.metadata_props.add()
        p.key, p.value = k, v
    onnx.save(m, onnx_path)


def write_demo_spec(onnx_path: str, variant: str, source: str = "", gait_clock: str = "free") -> Path:
    """policy_spec.json for th_bringup (same format as policies/walker_v2/v6/policy_spec.json).
    gait_clock "free": phase = ((t - t_walk) mod T) / T (v1). "held" (v2): per policy step,
    phase = 0 on the first WALK step or when |v_xy| + |w_z| <= gait_stand_threshold, else
    phase = (phase + policy_dt / T) mod 1 (tasks/walker_demo_velocity/mdp.py)."""
    from th_training.robots import walker_demo as R
    from th_training.tasks.walker_demo_velocity.env_cfg import ACTOR_TERMS_TO_BRIDGE, GAIT_PERIOD

    dj = R.default_joint_pos(variant)
    terms = [{"name": b, "scale": 1.0} for b in ACTOR_TERMS_TO_BRIDGE.values()]
    n = len(R.JOINT_ORDER)
    size = sum({"base_ang_vel": 3, "projected_gravity": 3, "velocity_command": 3, "gait_phase": 2}.get(t["name"], n) for t in terms)
    spec = {
        "_source": source or f"mjlab TH-Velocity-Flat-WalkerDemo-{variant.capitalize()} ({Path(onnx_path).name})",
        "robot": "walker_demo",
        "limits_variant": variant,
        "joint_order": R.JOINT_ORDER,
        "default_joint_pos": [dj[j] for j in R.JOINT_ORDER],
        "kp": [R.gains_of(j)[0] for j in R.JOINT_ORDER],
        "kd": [R.gains_of(j)[1] for j in R.JOINT_ORDER],
        "joint_limits": {j: list(R.limits_of(j, variant)) for j in R.JOINT_ORDER},
        "action_scale": R.ACTION_SCALE,
        "action_clip": 4.0,
        "policy_dt": 0.02,
        "gait_period": GAIT_PERIOD,
        "gait_clock": gait_clock,
        **({"gait_stand_threshold": 0.1, "targets_clipped_to_joint_limits_in_training": True} if gait_clock == "held" else {}),
        "observations": terms,
        "observation_size": size,
    }
    out = Path(onnx_path).with_name("policy_spec.json")
    out.write_text(json.dumps(spec, indent=1))
    return out


def make_runner_cls(robot: str = "walker_v2", gait_clock: str = "free"):
    from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

    class Runner(VelocityOnPolicyRunner):
        def save(self, path: str, infos=None):
            super().save(path, infos)
            try:
                _, _, onnx_path = self._get_export_paths(path)
                fix_metadata(str(onnx_path), robot)
                if robot == "walker_demo":
                    variant = "wide" if "wide" in str(onnx_path).lower() else "cad"
                    write_demo_spec(str(onnx_path), variant, source=f"{onnx_path} after {Path(path).name}",
                                    gait_clock=gait_clock)
            except Exception as e:  # never kill training over metadata
                print(f"[WARN] ONNX metadata fix failed: {e}")

    return Runner


# Back-compat for the walker_v2 task.
fix_walker_v2_metadata = fix_metadata

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx", nargs="+")
    ap.add_argument("--robot", default="walker_v2")
    ap.add_argument("--variant", default="cad")
    ap.add_argument("--gait-clock", default="free", choices=["free", "held"])
    a = ap.parse_args()
    for p in a.onnx:
        fix_metadata(p, a.robot)
        if a.robot == "walker_demo":
            print("spec ->", write_demo_spec(p, a.variant, gait_clock=a.gait_clock))
        print("fixed", p)
