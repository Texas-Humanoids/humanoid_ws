"""Cross-simulator check: run a demo-walker ONNX policy in Isaac Sim (PhysX) instead of MuJoCo.

Training (mjlab) and sim2sim_demo both use MuJoCo's contact model, so they can't catch a gait that
only works because of how MuJoCo does contacts. This runs the same policy, the same spec-driven
observation, the same explicit PD + torque-speed clip + Coulomb friction, and the same scripted
commands and pushes (eval_script.py), on PhysX.

  OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=training/src \
    /c/Users/karti/.venvs/bhl/Scripts/python.exe -m th_training.isaac_check_demo \
      --onnx policies/walker_demo/cad_v1/policy.onnx --spec policies/walker_demo/cad_v1/policy_spec.json \
      [--friction 0.8] [--out result.json]

Runs PhysX on the CPU (one robot; leaves the GPU to training). The robot is the cleaned URDF
(walker_demo.urdf) converted to USD with visuals stripped: box colliders only, as in training.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET

# onnxruntime MUST load before Kit: Kit brings an older MSVCP140.dll, and onnxruntime imported after it
# crashes in Thrd_yield (seen 2026-11-17, Isaac Sim 5.1 / onnxruntime on Windows).
import onnxruntime as ort  # noqa: E402,I001

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--onnx", default=None)
ap.add_argument("--stand", action="store_true",
                help="no policy: stand-only demo controller (stiff PD + CenteringTrim, th_training.stand_balance), push bisection")
ap.add_argument("--spec", required=True)
ap.add_argument("--friction", type=float, default=0.8)
ap.add_argument("--out", default=None)
ap.add_argument("--conditions", default="all", help="comma list of condition names, or all")
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
args.device = "cpu"
app = AppLauncher(args).app

# ---- everything below needs the running app
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

from th_training.eval_script import PUSHES, SCRIPT, GaitClock, target_limits  # noqa: E402
from th_training.paths import WALKER_DEMO_DIR  # noqa: E402
from th_training.robots import walker_demo as R  # noqa: E402

PHYS_DT, DECIMATION = 0.005, 4
POLICY_DT = PHYS_DT * DECIMATION
J = R.JOINT_ORDER

CONDITIONS = [  # same names/values as sim2sim_demo.SWEEP (friction is per launch: --friction)
    dict(name="nominal"),
    dict(name="kp0.8 kd0.7", kp_scale=0.8, kd_scale=0.7),
    dict(name="kp1.2 kd1.3", kp_scale=1.2, kd_scale=1.3),
    dict(name="delay 15ms", delay=3),
    dict(name="delay 25ms (OOD)", delay=5),
    dict(name="strength 0.85", strength=0.85),
    dict(name="strength 1.15", strength=1.15),
    dict(name="30V pack", voltage=30.0),
    dict(name="combined", kp_scale=0.85, kd_scale=0.8, delay=3, strength=0.9, voltage=32.0),
]


def make_usd(variant: str) -> str:
    """walker_demo.urdf without visuals (no package:// resolution needed), limits of the variant."""
    tree = ET.parse(WALKER_DEMO_DIR / "urdf" / "walker_demo.urdf")
    root = tree.getroot()
    for link in root.findall("link"):
        for v in link.findall("visual"):
            link.remove(v)
    for j in root.findall("joint"):
        if j.get("type") == "revolute":
            lo, hi = R.limits_of(j.get("name"), variant)
            lim = j.find("limit")
            lim.set("lower", f"{lo:.4f}")
            lim.set("upper", f"{hi:.4f}")
            dyn = j.find("dynamics")
            if dyn is not None:     # friction/damping are applied explicitly below, not by PhysX
                j.remove(dyn)
    d = os.path.join(tempfile.gettempdir(), f"walker_demo_isaac_{variant}")
    os.makedirs(d, exist_ok=True)
    urdf = os.path.join(d, "walker_demo_novis.urdf")
    tree.write(urdf)
    cfg = UrdfConverterCfg(
        asset_path=urdf, usd_dir=d, usd_file_name="walker_demo.usd", force_usd_conversion=True,
        make_instanceable=False, fix_base=False, merge_fixed_joints=True, self_collision=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="none", gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
    )
    return UrdfConverter(cfg).usd_path


def main():
    spec = json.load(open(args.spec))
    assert spec["joint_order"] == J
    variant = spec.get("limits_variant", "cad")
    q0 = torch.tensor(spec["default_joint_pos"], dtype=torch.float32)
    kp_n = torch.tensor(spec["kp"], dtype=torch.float32)
    kd_n = torch.tensor(spec["kd"], dtype=torch.float32)
    scale, clip = float(spec["action_scale"]), float(spec.get("action_clip", 1e9))
    terms = [(t["name"], float(t.get("scale", 1.0))) for t in spec["observations"]]
    period = float(spec.get("gait_period", 0.0))
    clock = GaitClock(spec)
    t_lo, t_hi = (torch.tensor(x, dtype=torch.float32) for x in target_limits(spec, J))  # the bridge clamp
    motors = [R.MOTORS[R.MOTOR_OF_JOINT[j]] for j in J]
    peak = torch.tensor([m.peak_torque for m in motors])
    fric = torch.tensor([m.friction for m in motors])
    rated = torch.tensor([m.rated_torque for m in motors])

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=PHYS_DT, device="cpu"))
    mat = sim_utils.RigidBodyMaterialCfg(static_friction=args.friction, dynamic_friction=args.friction,
                                         restitution=0.0, friction_combine_mode="min")
    sim_utils.GroundPlaneCfg(physics_material=mat).func("/World/ground", sim_utils.GroundPlaneCfg(physics_material=mat))
    sim_utils.DomeLightCfg(intensity=1000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=1000.0))
    usd = make_usd(variant)
    robot = Articulation(ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd, activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=1),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, R.STAND_BASE_HEIGHT + 0.003), joint_pos=dict(zip(J, spec["default_joint_pos"]))),
        actuators={"legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_joint"], stiffness=0.0, damping=0.0, friction=0.0,
            armature={j: R.MOTORS[R.MOTOR_OF_JOINT[j]].armature for j in J},
            effort_limit_sim=200.0, velocity_limit_sim=100.0)},
    ))
    # Robot feet: same friction, "min" so the ground's value is the effective one.
    sim_utils.spawn_rigid_body_material("/World/Materials/robot", mat)
    sim_utils.bind_physics_material("/World/Robot", "/World/Materials/robot")
    sim.reset()
    jids, names = robot.find_joints(J, preserve_order=True)
    assert list(names) == J, names
    jids = torch.tensor(jids)
    total_mass = float(robot.root_physx_view.get_masses().sum())

    sess = None if args.stand else ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    iname = None if args.stand else sess.get_inputs()[0].name

    def reset():
        st = robot.data.default_root_state.clone()
        robot.write_root_pose_to_sim(st[:, :7])
        robot.write_root_velocity_to_sim(torch.zeros(1, 6))
        jp = robot.data.default_joint_pos.clone()
        robot.write_joint_state_to_sim(jp, torch.zeros_like(jp))
        robot.reset()
        sim.step(render=False)
        robot.update(PHYS_DT)

    if args.stand:
        return stand_main(robot, sim, jids, q0, fric, peak, total_mass, reset)

    def run(c):
        kp, kd = kp_n * c.get("kp_scale", 1.0), kd_n * c.get("kd_scale", 1.0)
        delay, strength = c.get("delay", 0), c.get("strength", 1.0)
        vmax = torch.tensor([R.no_load_speed_at(R.MOTOR_OF_JOINT[j], c.get("voltage", R.BUS_VOLTAGE)) for j in J])
        reset()
        targets = [q0.clone() for _ in range(delay + 1)]
        last_action = torch.zeros(10)
        k, fell_at, rows, tau_log, qd_log = 0, None, [], [], []
        for dur, vx, vy, wz in SCRIPT:
            cmd = torch.tensor([vx, vy, wz])
            seg = []
            for _ in range(int(dur / POLICY_DT)):
                d = robot.data
                q = d.joint_pos[0, jids]
                qd = d.joint_vel[0, jids]
                parts = []
                for name, s in terms:
                    v = {"base_ang_vel": lambda: d.root_ang_vel_b[0], "projected_gravity": lambda: d.projected_gravity_b[0],
                         "velocity_command": lambda: cmd, "joint_pos_rel": lambda: q - q0, "joint_vel": lambda: qd,
                         "last_action": lambda: last_action,
                         "gait_phase": lambda: torch.from_numpy(clock(k, cmd.numpy())),
                         }[name]()
                    parts.append(s * v.float())
                obs = torch.cat(parts).numpy()[None].astype(np.float32)
                act = torch.from_numpy(sess.run(None, {iname: obs})[0][0]).clamp(-clip, clip)
                target = torch.minimum(torch.maximum(q0 + scale * act, t_lo), t_hi)
                for _ in range(DECIMATION):
                    targets.append(target)
                    targets.pop(0)
                    q = robot.data.joint_pos[0, jids]
                    qd = robot.data.joint_vel[0, jids]
                    tau = kp * (targets[0] - q) - kd * qd
                    v = qd.clamp(-vmax, vmax)
                    tau = torch.minimum(torch.maximum(tau, peak * (-1 - v / vmax)), peak * (1 - v / vmax))
                    tau_log.append(tau.abs())
                    qd_log.append(qd.abs())
                    applied = tau * strength - fric * torch.tanh(qd / 0.02)   # drive output minus Coulomb friction
                    robot.set_joint_effort_target(applied[None], joint_ids=jids)
                    robot.write_data_to_sim()
                    sim.step(render=False)
                    robot.update(PHYS_DT)
                if k in PUSHES:
                    rv = robot.data.root_vel_w.clone()
                    rv[0, :3] += torch.tensor(PUSHES[k], dtype=torch.float32)
                    robot.write_root_velocity_to_sim(rv)
                last_action = act
                k += 1
                d = robot.data
                seg.append((float(d.root_lin_vel_b[0, 0]), float(d.root_lin_vel_b[0, 1]), float(d.root_ang_vel_b[0, 2])))
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(d.projected_gravity_b[0, 2])))))
                if float(d.root_pos_w[0, 2]) < 0.35 or tilt > 60:
                    fell_at = round(k * POLICY_DT, 2)
                    break
            s = np.array(seg)
            st = s[int(len(s) * 0.4):] if len(s) > 10 else s
            rows.append(dict(cmd=[vx, vy, wz], vx=round(float(st[:, 0].mean()), 3), vy=round(float(st[:, 1].mean()), 3),
                             wz=round(float(st[:, 2].mean()), 3)))
            if fell_at:
                break
        tl, ql = torch.stack(tau_log), torch.stack(qd_log)
        mx = lambda x: torch.maximum(x[:5], x[5:])
        J5 = [R.joint_type(j) for j in J[:5]]
        return dict(name=c["name"], survived=fell_at is None, fell_at=fell_at, segments=rows,
                    peak_tau_frac=dict(zip(J5, (mx(tl.max(0).values) / peak[:5]).round(decimals=2).tolist())),
                    rms_tau_over_rated=dict(zip(J5, (mx(tl.pow(2).mean(0).sqrt()) / rated[:5]).round(decimals=2).tolist())),
                    peak_speed_frac=dict(zip(J5, (mx(ql.max(0).values) / vmax[:5]).round(decimals=2).tolist())))

    wanted = None if args.conditions == "all" else set(args.conditions.split(","))
    results = []
    print(f"\nIsaac Sim / PhysX check: {args.onnx}\n  variant {variant}, foot-ground friction {args.friction}, "
          f"robot mass in PhysX {total_mass:.3f} kg")
    print(f"{'condition':18s} {'fell':6s} " + " ".join(f"{str(c[1:]):>16s}" for c in SCRIPT[1:10]))
    for c in CONDITIONS:
        if wanted and c["name"] not in wanted:
            continue
        o = run(c)
        results.append(o)
        cells = []
        for s in o["segments"][1:10]:
            cmd = s["cmd"]
            i = int(np.argmax(np.abs(cmd))) if any(cmd) else 0
            cells.append(f"{[s['vx'], s['vy'], s['wz']][i]:+.2f}/{cmd[i]:+.2f}".rjust(16))
        print(f"{c['name']:18s} {('no' if o['survived'] else str(o['fell_at'])):6s} " + " ".join(cells), flush=True)
    if results:
        print("peak tau / motor peak (nominal):", results[0]["peak_tau_frac"])
        print("rms tau / rated (nominal):      ", results[0]["rms_tau_over_rated"])
    if args.out:
        json.dump(dict(friction=args.friction, mass=total_mass, results=results), open(args.out, "w"), indent=1)


def stand_main(robot, sim, jids, q0, fric, peak, total_mass, reset):
    """Stand-only demo controller on PhysX: same CenteringTrim and stand gains as the MuJoCo study
    (th_training.stand_balance), same explicit PD + Coulomb friction as the policy check."""
    from th_training import stand_balance as SB

    kp_s, kd_s = (torch.tensor(x, dtype=torch.float32) for x in SB.gains_array())
    base_id = robot.find_bodies("base_link")[0][0]
    view = robot.root_physx_view
    com0 = view.get_coms().clone()

    def one(trim, push=None, push_t=20.0, seconds=26.0, bias=None, com_dx=0.0, delay=0, kp_scale=1.0):
        coms = com0.clone()
        coms[0, base_id, 0] += com_dx
        view.set_coms(coms, torch.tensor([0]))
        reset()
        if trim:
            trim.reset()
        b = torch.zeros(10) if bias is None else torch.tensor(bias, dtype=torch.float32)
        kp, kd = kp_s * kp_scale, kd_s
        target = q0.clone()
        targets = [q0.clone() for _ in range(delay + 1)]
        fell, pitches = None, []
        for k in range(int(seconds / POLICY_DT)):
            d = robot.data
            qm, vm = d.joint_pos[0, jids] + b, d.joint_vel[0, jids]
            tau_est = kp * (target - qm) - kd * vm
            lean = trim.update(float(tau_est[SB.ANKLES[0]]), float(tau_est[SB.ANKLES[1]]), POLICY_DT) if trim else 0.0
            target = q0.clone()
            target[SB.ANKLES] += lean
            for _ in range(DECIMATION):
                targets.append(target - b)   # the drive servos the measured angle
                targets.pop(0)
                q, qd = robot.data.joint_pos[0, jids], robot.data.joint_vel[0, jids]
                tau = (kp * (targets[0] - q) - kd * qd).clamp(-peak, peak)
                robot.set_joint_effort_target((tau - fric * torch.tanh(qd / 0.02))[None], joint_ids=jids)
                robot.write_data_to_sim()
                sim.step(render=False)
                robot.update(PHYS_DT)
            if push is not None and k == int(push_t / POLICY_DT):
                rv = robot.data.root_vel_w.clone()
                rv[0, :3] += torch.tensor(push, dtype=torch.float32)
                robot.write_root_velocity_to_sim(rv)
            g = robot.data.projected_gravity_b[0]
            pitches.append(math.degrees(math.asin(max(-1.0, min(1.0, float(g[0]))))))
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(g[2])))))
            if float(robot.data.root_pos_w[0, 2]) < 0.35 or tilt > 60:
                fell = round(k * POLICY_DT, 2)
                break
        return dict(fell=fell, lean=round(float(lean), 4), pitch=round(float(np.mean(pitches[-50:])), 2))

    def max_push(make_trim, direction, cond, lo=0.0, hi=0.8, iters=6):
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            r = one(make_trim(), push=[x * mid for x in direction], **cond)
            lo, hi = (mid, hi) if r["fell"] is None else (lo, mid)
        return round(lo, 3)

    print(f"Isaac Sim / PhysX stand-only check (stand gains, no policy). Robot mass {total_mass:.2f} kg, "
          f"foot friction {args.friction}. Max push survived (m/s), at 20 s:")
    conds = {"nominal": {}, "COM 3 cm back": dict(com_dx=-0.03),
             "ankle encoders 2 deg off": dict(bias=[0, 0, 0, 0, 0.035, 0, 0, 0, 0, 0.035]),
             "real-ish (COM -3cm, 15 ms, kp x0.9, offsets)": dict(
                 com_dx=-0.03, delay=3, kp_scale=0.9, bias=[0.02, -0.02, 0.01, -0.02, 0.03, -0.02, 0.02, -0.01, 0.02, 0.02])}
    out = {}
    for cname, cond in conds.items():
        out[cname] = {}
        for label, make in (("PD", lambda: None), ("PD+trim", lambda: SB.CenteringTrim())):
            settle = one(make(), **cond)
            pushes = {dn: max_push(make, dv, cond) for dn, dv in (("fwd", (1, 0, 0)), ("back", (-1, 0, 0)), ("side", (0, 1, 0)))}
            out[cname][label] = dict(settle=settle, **pushes)
            print(f"  {cname:44s} {label:8s} fwd {pushes['fwd']:.2f} back {pushes['back']:.2f} side {pushes['side']:.2f} | "
                  f"standing pitch {settle['pitch']:+.1f} deg, lean {settle['lean']:+.3f}{'  FELL standing' if settle['fell'] else ''}",
                  flush=True)
    view.set_coms(com0, torch.tensor([0]))
    if args.out:
        json.dump(out, open(args.out, "w"), indent=1)


if __name__ == "__main__":
    code = 0
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    # Kit's app.close() can hang forever on Windows after a headless run (seen 2026-11-17: 2.5 h).
    # Results are already written, so flush and hard-exit instead.
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
