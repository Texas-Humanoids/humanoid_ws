"""Native-MuJoCo sim2sim for the demo walker. No mjlab, no GPU (bhl venv: mujoco 3.13 + onnxruntime).

The observation is built from policy_spec.json exactly the way th_policy_bridge builds it
(observation.hpp): terms by name, one scale per term, gait phase from the moment WALK starts,
target = default + action_scale * clip(action). If a policy walks here, the spec is right.

  python -m th_training.sim2sim_demo stand [--variant cad|wide] [--kp-scale 0.8] [--delay 3] [--view]
  python -m th_training.sim2sim_demo eval --onnx <policy.onnx> --spec <policy_spec.json> [--view] [--video out.mp4]
        [--kp-scale 0.8] [--kd-scale 0.7] [--delay 3] [--friction 0.5] [--effort-scale 0.8] [--strength 1.1]
  python -m th_training.sim2sim_demo sweep --onnx ... --spec ...     # the robustness table
"""
from __future__ import annotations

import argparse
import json
import math
import time

import mujoco
import numpy as np

from th_training.paths import WALKER_DEMO_SCENE_XML
from th_training.robots import walker_demo as R

PHYS_DT = 0.005
DECIMATION = 4
POLICY_DT = PHYS_DT * DECIMATION


def _quat2mat(q):
    M = np.zeros(9)
    mujoco.mju_quat2Mat(M, q)
    return M.reshape(3, 3)


from th_training.eval_script import GaitClock, target_limits  # noqa: E402

class DemoSim:
    def __init__(self, spec: dict | None = None, variant: str = "cad", kp_scale=1.0, kd_scale=1.0, delay_steps=0,
                 friction=0.8, effort_scale=1.0, strength=1.0, voltage=R.BUS_VOLTAGE, visual=False):
        ms = mujoco.MjSpec.from_file(str(WALKER_DEMO_SCENE_XML))
        if not visual:
            for g in list(ms.geoms):
                if g.type == mujoco.mjtGeom.mjGEOM_MESH:
                    ms.delete(g)
            for m in list(ms.meshes):
                ms.delete(m)
        for g in ms.geoms:
            if g.name.endswith("_foot_collision"):
                g.contype, g.conaffinity, g.condim, g.priority = 1, 1, 3, 1
                g.friction = [friction, 0.005, 0.0001]
            elif g.name.endswith("_collision"):
                g.contype, g.conaffinity, g.condim, g.priority = 0, 1, 1, 0
                g.friction = [0.6, 0.005, 0.0001]
        J = R.JOINT_ORDER
        for j in J:
            mot = R.MOTORS[R.MOTOR_OF_JOINT[j]]
            js = ms.joint(j)
            js.range = list(R.limits_of(j, variant))
            js.armature, js.frictionloss = mot.armature, mot.friction
            ms.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                            ctrlrange=[-mot.peak_torque * 2, mot.peak_torque * 2], ctrllimited=True)
        ms.option.timestep = PHYS_DT
        ms.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        ms.option.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
        self.m = ms.compile()
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(j).qposadr[0] for j in J])
        self.vadr = np.array([self.m.joint(j).dofadr[0] for j in J])

        if spec is not None:
            assert spec["joint_order"] == J, "spec joint order != robot joint order"
            self.q0 = np.array(spec["default_joint_pos"], float)
            kp, kd = np.array(spec["kp"], float), np.array(spec["kd"], float)
            self.scale = float(spec["action_scale"])
            self.clip = float(spec.get("action_clip", 1e9))
            self.terms = [(t["name"], float(t.get("scale", 1.0))) for t in spec["observations"]]
            self.gait_period = float(spec.get("gait_period", 0.0))
            self.clock = GaitClock(spec)
            self.lo, self.hi = target_limits(spec, J)  # the bridge clamps targets to these
            assert abs(float(spec["policy_dt"]) - POLICY_DT) < 1e-9
        else:
            dj = R.default_joint_pos(variant)
            self.q0 = np.array([dj[j] for j in J])
            kp = np.array([R.gains_of(j)[0] for j in J])
            kd = np.array([R.gains_of(j)[1] for j in J])
            self.scale, self.clip, self.terms, self.gait_period = R.ACTION_SCALE, 1e9, [], 0.0
            self.clock, self.lo, self.hi = None, np.full(10, -1e9), np.full(10, 1e9)
        self.kp, self.kd = kp * kp_scale, kd * kd_scale
        self.peak = np.array([R.MOTORS[R.MOTOR_OF_JOINT[j]].peak_torque for j in J])
        self.limit = self.peak * effort_scale
        self.strength = strength  # applied torque / commanded torque (RS06 measured ~1.1)
        self.vmax = np.array([R.no_load_speed_at(R.MOTOR_OF_JOINT[j], voltage) for j in J])
        self.delay = delay_steps
        self.gyro = self.m.sensor("imu_ang_vel").adr[0]
        self.feet = [self.m.geom(f"{s}_foot_collision").id for s in ("left", "right")]
        self.floor = self.m.geom("floor").id
        self.reset()

    def reset(self, z: float | None = None):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[2] = (R.STAND_BASE_HEIGHT + 0.003) if z is None else z
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[self.qadr] = self.q0
        mujoco.mj_forward(self.m, self.d)
        self.last_action = np.zeros(10)
        self.targets = [self.q0.copy()] * (self.delay + 1)
        self.walk_steps = 0
        self.tau_log, self.qd_log = [], []

    def _torque(self, target):
        q, qd = self.d.qpos[self.qadr], self.d.qvel[self.vadr]
        tau = self.kp * (target - q) - self.kd * qd
        v = np.clip(qd, -self.vmax, self.vmax)
        hi = np.minimum(self.peak * (1 - v / self.vmax), self.limit)
        lo = np.maximum(self.peak * (-1 - v / self.vmax), -self.limit)
        return np.clip(tau, lo, hi)

    def step(self, action, push=None):
        action = np.clip(action, -self.clip, self.clip)
        target = np.clip(self.q0 + self.scale * action, self.lo, self.hi)
        for _ in range(DECIMATION):
            self.targets.append(target)
            self.targets.pop(0)
            tau = self._torque(self.targets[0])
            self.d.ctrl[:] = tau * self.strength
            self.tau_log.append(np.abs(tau))
            self.qd_log.append(np.abs(self.d.qvel[self.vadr]))
            mujoco.mj_step(self.m, self.d)
        if push is not None:
            self.d.qvel[0:3] += push
        self.last_action = action.copy()
        self.walk_steps += 1

    def obs(self, cmd):
        Rb = _quat2mat(self.d.qpos[3:7])
        parts = []
        for name, s in self.terms:
            if name == "base_ang_vel":
                v = self.d.sensordata[self.gyro:self.gyro + 3]
            elif name == "projected_gravity":
                v = Rb.T @ np.array([0.0, 0.0, -1.0])
            elif name == "velocity_command":
                v = cmd
            elif name == "joint_pos_rel":
                v = self.d.qpos[self.qadr] - self.q0
            elif name == "joint_vel":
                v = self.d.qvel[self.vadr]
            elif name == "last_action":
                v = self.last_action
            elif name == "gait_phase":
                v = self.clock(self.walk_steps, cmd)
            else:
                raise KeyError(name)
            parts.append(s * np.asarray(v, float))
        return np.concatenate(parts).astype(np.float32)

    def base_vel_b(self):
        Rb = _quat2mat(self.d.qpos[3:7])
        return Rb.T @ self.d.qvel[0:3], self.d.qvel[3:6]

    def tilt(self):
        return math.degrees(math.acos(np.clip(_quat2mat(self.d.qpos[3:7])[2, 2], -1, 1)))

    def foot_contacts(self):
        c = [False, False]
        for i in range(self.d.ncon):
            con = self.d.contact[i]
            for k, f in enumerate(self.feet):
                if {con.geom1, con.geom2} == {f, self.floor}:
                    c[k] = True
        return c

    def fallen(self):
        return self.d.qpos[2] < 0.35 or self.tilt() > 60


def _viewer(sim, on):
    if not on:
        return None
    import mujoco.viewer
    v = mujoco.viewer.launch_passive(sim.m, sim.d)
    v.cam.distance, v.cam.elevation = 2.0, -10
    return v


def run_stand(a):
    sim = DemoSim(variant=a.variant, kp_scale=a.kp_scale, delay_steps=a.delay, visual=a.view)
    v = _viewer(sim, a.view)
    zs, tilts, k = [], [], 0
    for k in range(int(a.seconds / POLICY_DT)):
        sim.step(np.zeros(10), push=np.array([a.push, 0, 0]) if (a.push and k == 50) else None)
        zs.append(sim.d.qpos[2]); tilts.append(sim.tilt())
        if v:
            v.cam.lookat[:] = sim.d.qpos[:3]; v.sync(); time.sleep(POLICY_DT)
        if sim.fallen():
            break
    tau = np.max(np.array(sim.tau_log), axis=0)
    res = dict(stood=not sim.fallen(), seconds=round((k + 1) * POLICY_DT, 2), variant=a.variant,
               kp_scale=a.kp_scale, delay_steps=a.delay, base_z_final=round(zs[-1], 4), tilt_max_deg=round(max(tilts), 2),
               contacts_final=sim.foot_contacts(),
               max_tau=dict(zip([R.joint_type(j) for j in R.JOINT_ORDER[:5]], np.maximum(tau[:5], tau[5:]).round(1).tolist())),
               joint_sag=dict(zip([R.joint_type(j) for j in R.JOINT_ORDER[:5]], (sim.d.qpos[sim.qadr] - sim.q0)[:5].round(3).tolist())))
    print(json.dumps(res))
    return res


from th_training.eval_script import PUSHES, SCRIPT  # noqa: E402  (shared with isaac_check_demo)


def run_eval(a, quiet=False):
    import onnxruntime as ort

    spec = json.load(open(a.spec))
    sess = ort.InferenceSession(a.onnx, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    sim = DemoSim(spec, variant=spec.get("limits_variant", "cad"), kp_scale=a.kp_scale, kd_scale=a.kd_scale,
                  delay_steps=a.delay, friction=a.friction, effort_scale=a.effort_scale, strength=a.strength,
                  voltage=a.voltage, visual=bool(a.view or a.video))
    assert sess.get_inputs()[0].shape[-1] == len(sim.obs(np.zeros(3))), "ONNX input size != spec observation size"
    v = _viewer(sim, a.view)
    rec = None
    if a.video:
        import cv2
        renderer = mujoco.Renderer(sim.m, 480, 640)
        cam = mujoco.MjvCamera(); cam.distance, cam.elevation, cam.azimuth = 2.4, -12, 135
        rec = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 1 / POLICY_DT, (640, 480))
    rows, k, fell_at = [], 0, None
    for dur, vx, vy, wz in SCRIPT:
        cmd = np.array([vx, vy, wz])
        seg = []
        for _ in range(int(dur / POLICY_DT)):
            act = sess.run(None, {iname: sim.obs(cmd)[None]})[0][0]
            sim.step(act, push=None if a.no_push else PUSHES.get(k))
            lv, av = sim.base_vel_b()
            seg.append((lv[0], lv[1], av[2], *sim.foot_contacts(), sim.d.qpos[2]))
            k += 1
            if v:
                v.cam.lookat[:] = sim.d.qpos[:3]; v.sync(); time.sleep(POLICY_DT)
            if rec is not None:
                cam.lookat[:] = sim.d.qpos[:3] - np.array([0, 0, 0.2])
                renderer.update_scene(sim.d, cam)
                img = renderer.render()[:, :, ::-1].copy()
                cv2.putText(img, f"t={k * POLICY_DT:5.1f}s  vx {vx:+.2f} vy {vy:+.2f} wz {wz:+.2f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                rec.write(img)
            if sim.fallen():
                fell_at = round(k * POLICY_DT, 2)
                break
        s = np.array(seg)
        st = s[int(len(s) * 0.4):] if len(s) > 10 else s
        rows.append(dict(cmd=[vx, vy, wz], vx=round(float(st[:, 0].mean()), 3), vy=round(float(st[:, 1].mean()), 3),
                         wz=round(float(st[:, 2].mean()), 3), both_down=round(float((st[:, 3] * st[:, 4]).mean()), 2)))
        if fell_at:
            break
    if rec is not None:
        rec.release()
    tau, qd = np.array(sim.tau_log), np.array(sim.qd_log)
    J5 = [R.joint_type(j) for j in R.JOINT_ORDER[:5]]
    mx = lambda x: np.maximum(x[:5], x[5:])
    rated = np.array([R.MOTORS[R.MOTOR_OF_JOINT[j]].rated_torque for j in R.JOINT_ORDER[:5]])
    out = dict(survived=fell_at is None, fell_at=fell_at, segments=rows,
               peak_tau_frac=dict(zip(J5, (mx(tau.max(0)) / sim.peak[:5]).round(2).tolist())),
               rms_tau_over_rated=dict(zip(J5, (mx(np.sqrt((tau ** 2).mean(0))) / rated).round(2).tolist())),
               peak_speed_frac=dict(zip(J5, (mx(qd.max(0)) / sim.vmax[:5]).round(2).tolist())),
               conditions=dict(kp=a.kp_scale, kd=a.kd_scale, delay=a.delay, mu=a.friction, effort=a.effort_scale,
                               strength=a.strength, voltage=a.voltage, pushes=not a.no_push))
    if not quiet:
        print(json.dumps(out, indent=1))
    return out


SWEEP = [
    dict(name="nominal"),
    dict(name="kp0.8 kd0.7", kp_scale=0.8, kd_scale=0.7),
    dict(name="kp1.2 kd1.3", kp_scale=1.2, kd_scale=1.3),
    dict(name="delay 15ms", delay=3),
    dict(name="delay 25ms (OOD)", delay=5),
    dict(name="mu 0.4", friction=0.4),
    dict(name="strength 0.85", strength=0.85),
    dict(name="strength 1.15", strength=1.15),
    dict(name="30V pack", voltage=30.0),
    dict(name="combined", kp_scale=0.85, kd_scale=0.8, delay=3, friction=0.5, strength=0.9, voltage=32.0),
]


def run_sweep(a):
    base = dict(kp_scale=1.0, kd_scale=1.0, delay=0, friction=0.8, effort_scale=1.0, strength=1.0, voltage=R.BUS_VOLTAGE,
                no_push=False, view=False, video=None, onnx=a.onnx, spec=a.spec)
    print(f"{'condition':18s} {'fell':6s} " + " ".join(f"{str(c[1:]):>16s}" for c in SCRIPT[1:10]))
    res = []
    for c in SWEEP:
        args = argparse.Namespace(**{**base, **{k: v for k, v in c.items() if k != "name"}})
        o = run_eval(args, quiet=True)
        res.append(dict(name=c["name"], **o))
        segs = o["segments"]
        cells = []
        for i, s in enumerate(segs[1:10]):
            cmd = s["cmd"]
            idx = int(np.argmax(np.abs(cmd))) if any(cmd) else 0
            got = [s["vx"], s["vy"], s["wz"]][idx]
            cells.append(f"{got:+.2f}/{cmd[idx]:+.2f}".rjust(16))
        print(f"{c['name']:18s} {('no' if o['survived'] else str(o['fell_at'])):6s} " + " ".join(cells))
    print("peak tau / motor peak (nominal):", res[0]["peak_tau_frac"])
    print("rms tau / rated (nominal):      ", res[0]["rms_tau_over_rated"])
    print("peak speed / 36V top (nominal): ", res[0]["peak_speed_frac"])
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)
    return res


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("stand")
    s.add_argument("--variant", default="cad", choices=list(R.LIMITS))
    s.add_argument("--seconds", type=float, default=10.0)
    s.add_argument("--kp-scale", type=float, default=1.0)
    s.add_argument("--delay", type=int, default=0)
    s.add_argument("--push", type=float, default=0.0, help="forward base velocity kick at t=1 s (m/s)")
    s.add_argument("--view", action="store_true")
    for name in ("eval", "sweep"):
        e = sub.add_parser(name)
        e.add_argument("--onnx", required=True)
        e.add_argument("--spec", required=True)
        e.add_argument("--out", default=None)
        e.add_argument("--kp-scale", type=float, default=1.0)
        e.add_argument("--kd-scale", type=float, default=1.0)
        e.add_argument("--delay", type=int, default=0, help="command delay in 5 ms physics steps")
        e.add_argument("--friction", type=float, default=0.8)
        e.add_argument("--effort-scale", type=float, default=1.0)
        e.add_argument("--strength", type=float, default=1.0)
        e.add_argument("--voltage", type=float, default=R.BUS_VOLTAGE)
        e.add_argument("--no-push", action="store_true")
        e.add_argument("--view", action="store_true")
        e.add_argument("--video", default=None)
    a = ap.parse_args()
    {"stand": run_stand, "eval": run_eval, "sweep": run_sweep}[a.mode](a)


if __name__ == "__main__":
    main()
