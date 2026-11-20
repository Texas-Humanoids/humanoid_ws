"""Native-MuJoCo harness for WalkerV2: no mjlab, no GPU. Used for
  phase 2  `stand`: PD only, hold the ready pose. Does the model + gains stand on their own?
  phase 5  `eval` : run an exported ONNX policy through a scripted command/push sequence.

It deliberately re-implements everything the robot-side controller will have to do (obs
layout, gait clock, action scaling, PD + torque-speed clip) from robots/walker_v2.py, without
reusing mjlab code. If the policy works here, the obs/action contract is right.

Runs in any venv with mujoco + numpy (+ onnxruntime for eval), e.g. the bhl venv:
  python -m th_training.sim2sim stand [--view]
  python -m th_training.sim2sim eval --onnx <policy.onnx> [--view] [--kp-scale 0.8] [--delay 2]
"""
from __future__ import annotations

import argparse
import json
import math
import time

import mujoco
import numpy as np

from th_training.paths import WALKER_V2_SCENE_XML
from th_training.robots.walker_v2 import (
    DEFAULT_JOINT_POS,
    JOINT_ORDER,
    MOTOR_OF_JOINT,
    MOTORS,
    STAND_BASE_HEIGHT,
    action_scale,
    BUS_VOLTAGE,
    DATASHEET_VOLTAGE,
    gains_of,
)

PHYS_DT = 0.005
DECIMATION = 4
POLICY_DT = PHYS_DT * DECIMATION
GAIT_PERIOD = 0.72  # must match tasks/walker_v2_velocity/env_cfg.py


class WalkerSim:
    def __init__(self, kp_scale: float = 1.0, kd_scale: float = 1.0, delay_steps: int = 0,
                 friction: float = 0.8, effort_scale: float = 1.0, bus_voltage: float = BUS_VOLTAGE):
        spec = mujoco.MjSpec.from_file(str(WALKER_V2_SCENE_XML))
        # Same collision policy as robots/walker_v2.get_walker_v2_robot_cfg().
        for g in spec.geoms:
            if g.name.endswith("_foot_collision"):
                g.contype, g.conaffinity, g.condim, g.priority = 1, 1, 3, 1
                g.friction = [friction, 0.005, 0.0001]
            elif g.name.endswith("_collision"):
                g.contype, g.conaffinity, g.condim, g.priority = 0, 1, 1, 0
                g.friction = [0.6, 0.005, 0.0001]
        for j in JOINT_ORDER:
            mot = MOTORS[MOTOR_OF_JOINT[j]]
            js = spec.joint(j)
            js.armature = mot.armature
            js.frictionloss = mot.friction
            spec.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                              ctrlrange=[-mot.peak_torque, mot.peak_torque], ctrllimited=True)
        spec.option.timestep = PHYS_DT
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.iterations = 10
        spec.option.ls_iterations = 20
        spec.option.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)

        self.qadr = np.array([self.m.joint(j).qposadr[0] for j in JOINT_ORDER])
        self.vadr = np.array([self.m.joint(j).dofadr[0] for j in JOINT_ORDER])
        self.q0 = np.array([DEFAULT_JOINT_POS[j] for j in JOINT_ORDER])
        self.kp = np.array([gains_of(j)[0] for j in JOINT_ORDER]) * kp_scale
        self.kd = np.array([gains_of(j)[1] for j in JOINT_ORDER]) * kd_scale
        self.peak = np.array([MOTORS[MOTOR_OF_JOINT[j]].peak_torque for j in JOINT_ORDER])
        self.limit = self.peak * effort_scale
        # Back-EMF limit: no-load speed scales with bus voltage. Peak torque is current-limited.
        self.vmax = np.array([MOTORS[MOTOR_OF_JOINT[j]].no_load_speed for j in JOINT_ORDER]) * (bus_voltage / DATASHEET_VOLTAGE)
        self.scale = np.array([action_scale()[j] for j in JOINT_ORDER])
        self.delay = delay_steps
        self.gyro = self.m.sensor("imu_ang_vel").adr[0]
        self.base = self.m.body("base_link").id
        self.feet = [self.m.geom(f"{s}_foot_collision").id for s in ("left", "right")]
        self.floor = self.m.geom("floor").id
        self.reset()

    def reset(self, z: float = STAND_BASE_HEIGHT + 0.005):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[2] = z
        self.d.qpos[3:7] = [1, 0, 0, 0]
        self.d.qpos[self.qadr] = self.q0
        mujoco.mj_forward(self.m, self.d)
        self.last_action = np.zeros(10)
        self.targets = [self.q0.copy()] * (self.delay + 1)
        self.step_count = 0
        self.tau_log = []
        self.qd_log = []

    # ------------------------------------------------------------------ actuator (matches mjlab DcMotorActuator)
    def _torque(self, target):
        q, qd = self.d.qpos[self.qadr], self.d.qvel[self.vadr]
        tau = self.kp * (target - q) - self.kd * qd
        v_at = self.vmax * (1 + self.limit / self.peak)
        v = np.clip(qd, -v_at, v_at)
        hi = np.minimum(self.peak * (1 - v / self.vmax), self.limit)
        lo = np.maximum(self.peak * (-1 - v / self.vmax), -self.limit)
        return np.clip(tau, lo, hi)

    def step(self, action: np.ndarray, push: np.ndarray | None = None):
        target = self.q0 + self.scale * action
        for _ in range(DECIMATION):
            self.targets.append(target)
            self.targets.pop(0)
            tau = self._torque(self.targets[0])
            self.d.ctrl[:] = tau
            self.tau_log.append(np.abs(tau))
            self.qd_log.append(np.abs(self.d.qvel[self.vadr]))
            mujoco.mj_step(self.m, self.d)
        if push is not None:
            self.d.qvel[0:3] += push
        self.last_action = action.copy()
        self.step_count += 1

    # ------------------------------------------------------------------ observation (actor group, mjlab order)
    def obs(self, cmd: np.ndarray) -> np.ndarray:
        quat = self.d.qpos[3:7]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        grav_b = R.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
        ang_vel = self.d.sensordata[self.gyro:self.gyro + 3]
        ph = 2 * math.pi * ((self.step_count * POLICY_DT / GAIT_PERIOD) % 1.0)
        return np.concatenate([
            ang_vel, grav_b,
            self.d.qpos[self.qadr] - self.q0,
            self.d.qvel[self.vadr],
            self.last_action, cmd,
            [math.sin(ph), math.cos(ph)],
        ]).astype(np.float32)

    # ------------------------------------------------------------------ state helpers
    def base_vel_b(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[3:7])
        R = R.reshape(3, 3)
        return R.T @ self.d.qvel[0:3], self.d.qvel[3:6]  # freejoint angular vel is already local

    def tilt(self):
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.d.qpos[3:7])
        return math.degrees(math.acos(np.clip(R[8], -1, 1)))

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


def _viewer(sim, enabled):
    if not enabled:
        return None
    import mujoco.viewer
    v = mujoco.viewer.launch_passive(sim.m, sim.d)
    v.cam.distance, v.cam.elevation = 2.0, -10
    return v


def run_stand(args):
    sim = WalkerSim(kp_scale=args.kp_scale, delay_steps=args.delay)
    v = _viewer(sim, args.view)
    zs, tilts = [], []
    for k in range(int(args.seconds / POLICY_DT)):
        sim.step(np.zeros(10))
        zs.append(sim.d.qpos[2])
        tilts.append(sim.tilt())
        if v:
            v.cam.lookat[:] = sim.d.qpos[:3]
            v.sync()
            time.sleep(POLICY_DT)
        if sim.fallen():
            break
    tau = np.max(np.array(sim.tau_log), axis=0)
    res = dict(stood=not sim.fallen(), seconds=round((k + 1) * POLICY_DT, 2),
               base_z_final=round(zs[-1], 4), base_z_min=round(min(zs), 4), tilt_max_deg=round(max(tilts), 2),
               contacts_final=sim.foot_contacts(),
               max_tau=dict(zip(JOINT_ORDER[:5], np.maximum(tau[:5], tau[5:]).round(1).tolist())),
               joint_sag=dict(zip(JOINT_ORDER[:5], (sim.d.qpos[sim.qadr] - sim.q0)[:5].round(3).tolist())))
    print(json.dumps(res, indent=1))
    return res


# Scripted eval: (duration s, vx, vy, wz)
SCRIPT = [
    (2.0, 0.0, 0.0, 0.0),
    (4.0, 0.3, 0.0, 0.0),
    (4.0, 0.6, 0.0, 0.0),
    (4.0, 0.8, 0.0, 0.0),
    (3.0, 0.0, 0.0, 0.0),
    (4.0, -0.3, 0.0, 0.0),
    (4.0, 0.0, 0.25, 0.0),
    (4.0, 0.0, -0.25, 0.0),
    (4.0, 0.0, 0.0, 0.8),
    (4.0, 0.4, 0.0, -0.6),
    (3.0, 0.0, 0.0, 0.0),
]
PUSHES = {  # policy step index -> base velocity kick (m/s), during the stand/walk segments
    int(1.5 / POLICY_DT): np.array([0.3, 0.0, 0.0]),
    int(9.0 / POLICY_DT): np.array([0.0, 0.25, 0.0]),
    int(15.0 / POLICY_DT): np.array([-0.3, 0.0, 0.0]),
    int(28.0 / POLICY_DT): np.array([0.0, -0.25, 0.0]),
}


def run_eval(args):
    import onnxruntime as ort

    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    iname = sess.get_inputs()[0].name
    sim = WalkerSim(kp_scale=args.kp_scale, kd_scale=args.kd_scale, delay_steps=args.delay,
                    friction=args.friction, effort_scale=args.effort_scale, bus_voltage=args.voltage)
    v = _viewer(sim, args.view)
    rec = None
    if args.video:
        import cv2  # bhl venv has opencv
        renderer = mujoco.Renderer(sim.m, 480, 640)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 2.6, -15, 120
        rec = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"), 1 / POLICY_DT, (640, 480))
    rows, k, fell_at = [], 0, None
    for dur, vx, vy, wz in SCRIPT:
        cmd = np.array([vx, vy, wz], dtype=np.float32)
        seg = []
        for _ in range(int(dur / POLICY_DT)):
            a = sess.run(None, {iname: sim.obs(cmd)[None]})[0][0]
            sim.step(a, push=PUSHES.get(k) if not args.no_push else None)
            lv, av = sim.base_vel_b()
            seg.append((lv[0], lv[1], av[2], *sim.foot_contacts()))
            k += 1
            if v:
                v.cam.lookat[:] = sim.d.qpos[:3]
                v.sync()
                time.sleep(POLICY_DT)
            if rec is not None:
                cam.lookat[:] = sim.d.qpos[:3] - np.array([0, 0, 0.2])
                renderer.update_scene(sim.d, cam)
                img = renderer.render()[:, :, ::-1].copy()
                cv2.putText(img, f"t={k*POLICY_DT:5.1f}s cmd vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                rec.write(img)
            if sim.fallen():
                fell_at = round(k * POLICY_DT, 2)
                break
        s = np.array(seg)
        settle = s[int(len(s) * 0.4):] if len(s) > 10 else s  # skip the transient
        rows.append(dict(cmd=[vx, vy, wz], vx=round(float(settle[:, 0].mean()), 3),
                         vy=round(float(settle[:, 1].mean()), 3), wz=round(float(settle[:, 2].mean()), 3),
                         both_feet_down=round(float((settle[:, 3] * settle[:, 4]).mean()), 2)))
        if fell_at:
            break
    if rec is not None:
        rec.release()
    tau = np.array(sim.tau_log)
    peak = np.max(tau, axis=0)
    p99 = np.percentile(tau, 99, axis=0)
    out = dict(fell_at=fell_at, survived=fell_at is None, segments=rows,
               peak_tau_frac=dict(zip(JOINT_ORDER[:5], (np.maximum(peak[:5], peak[5:]) / sim.peak[:5]).round(2).tolist())),
               p99_tau_frac=dict(zip(JOINT_ORDER[:5], (np.maximum(p99[:5], p99[5:]) / sim.peak[:5]).round(2).tolist())),
               peak_speed_frac=dict(zip(JOINT_ORDER[:5], (np.maximum(np.max(qd := np.array(sim.qd_log), axis=0)[:5], qd.max(axis=0)[5:]) / sim.vmax[:5]).round(2).tolist())),
               conditions=dict(voltage=args.voltage, kp_scale=args.kp_scale, kd_scale=args.kd_scale, delay_steps=args.delay,
                               friction=args.friction, effort_scale=args.effort_scale, pushes=not args.no_push))
    print(json.dumps(out, indent=1))
    return out


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    s = sub.add_parser("stand")
    s.add_argument("--seconds", type=float, default=10.0)
    s.add_argument("--kp-scale", type=float, default=1.0)
    s.add_argument("--delay", type=int, default=0)
    s.add_argument("--view", action="store_true")
    e = sub.add_parser("eval")
    e.add_argument("--onnx", required=True)
    e.add_argument("--kp-scale", type=float, default=1.0)
    e.add_argument("--kd-scale", type=float, default=1.0)
    e.add_argument("--delay", type=int, default=0, help="command delay in 5 ms physics steps")
    e.add_argument("--friction", type=float, default=0.8)
    e.add_argument("--effort-scale", type=float, default=1.0)
    e.add_argument("--voltage", type=float, default=BUS_VOLTAGE, help="motor bus voltage (V); scales no-load speed")
    e.add_argument("--no-push", action="store_true")
    e.add_argument("--view", action="store_true")
    e.add_argument("--video", default=None, help="write an mp4 of the run")
    a = ap.parse_args()
    run_stand(a) if a.mode == "stand" else run_eval(a)


if __name__ == "__main__":
    main()
