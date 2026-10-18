"""Exercise the installed ROS policy node in a Berkeley MuJoCo feedback loop.

This runs a simulated robot only. It never publishes motor commands or opens CAN.
Run from the workspace root after sourcing ROS 2 and install/setup.bash.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import mujoco
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import yaml


def sensor(data, model, name):
    item = model.sensor(name)
    start = model.sensor_adr[item.id]
    return np.asarray(data.sensordata[start : start + model.sensor_dim[item.id]])


def projected_gravity(quaternion):
    rotation = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(rotation, quaternion)
    return rotation.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("policies/policy_humanoid.onnx"))
    parser.add_argument("--config", type=Path, default=Path(
        "src/th_policy_node/config/policy_humanoid.yaml"))
    parser.add_argument("--berkeley-config", type=Path, default=Path(
        "../Berkeley-Humanoid-Lite/configs/policy_humanoid.yaml"))
    parser.add_argument("--scene", type=Path, default=Path(
        "src/th_description/robots/berkeley_bot/mjcf/bhl_scene.xml"))
    parser.add_argument("--steps", type=int, default=25)
    parser.add_argument("--command", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        metavar=("FORWARD", "LATERAL", "YAW"))
    parser.add_argument("--domain-id", type=int, default=118)
    parser.add_argument("--output", type=Path, default=Path("log/policy_mujoco.json"))
    args = parser.parse_args()
    if args.steps < 2 or not np.isfinite(args.command).all():
        parser.error("Provide at least two steps and finite command values.")

    policy_path = args.model.resolve(strict=True)
    config_path = args.config.resolve(strict=True)
    berkeley_config_path = args.berkeley_config.resolve(strict=True)
    scene_path = args.scene.resolve(strict=True)
    profile = yaml.safe_load(config_path.read_text())["policy_node"]["ros__parameters"]
    simulation_profile = yaml.safe_load(berkeley_config_path.read_text())
    joint_names = profile["joint_names"]
    if len(joint_names) != 22 or profile["expected_observation_size"] != 75:
        parser.error("This check requires the Berkeley 75-input, 22-joint profile.")
    defaults = np.asarray(profile["default_joint_positions"], dtype=np.float32)
    action_scale = float(profile["action_scale"])
    if action_scale == 0:
        parser.error("action_scale must be nonzero to recover previous actions.")
    if (simulation_profile["joints"] != joint_names or
            not np.allclose(simulation_profile["default_joint_positions"], defaults) or
            simulation_profile["action_scale"] != action_scale or
            simulation_profile["num_observations"] != 75):
        raise ValueError("Berkeley simulation configuration differs from the ROS policy profile.")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.opt.timestep = float(simulation_profile["physics_dt"])
    data = mujoco.MjData(model)
    if model.nu != len(joint_names) or [model.actuator(i).name for i in range(model.nu)] != joint_names:
        raise ValueError("MuJoCo actuator names/order differ from the policy profile.")
    qpos_indices = [model.joint(name).qposadr[0] for name in joint_names]
    qvel_indices = [model.joint(name).dofadr[0] for name in joint_names]
    joint_limits = np.asarray([model.joint(name).range for name in joint_names])
    data.qpos[:3] = [0.0, 0.0, 0.0]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[qpos_indices] = defaults
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    initial_positions = np.asarray(data.qpos[qpos_indices]).copy()
    previous_actions = np.zeros(len(joint_names), dtype=np.float32)

    # Separate topics and domain prevent this diagnostic from reaching motor nodes.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    rclpy.init()
    probe = rclpy.create_node("policy_mujoco_probe")
    publisher = probe.create_publisher(Float32MultiArray, "/policy_mujoco/observation", 10)
    received = []
    subscription = probe.create_subscription(
        JointState, "/policy_mujoco/joint_targets", received.append, 10)
    process = None
    report = {"kind": "closed-loop MuJoCo / real ONNX / ROS transport",
              "scene": str(scene_path), "model": str(policy_path),
              "berkeley_config": str(berkeley_config_path),
              "command": args.command, "policy_dt_s": simulation_profile["policy_dt"],
              "physics_dt_s": model.opt.timestep, "steps": []}
    args.output.parent.mkdir(parents=True, exist_ok=True)

    def wait_until(predicate, description, seconds=15):
        deadline = time.monotonic() + seconds
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError("Policy process exited; inspect log/policy_mujoco_node.log")
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(probe, timeout_sec=0.05)

    try:
        with args.output.with_name("policy_mujoco_node.log").open("w") as log:
            process = subprocess.Popen([
                "ros2", "run", "th_policy_node", "policy_node", "--ros-args",
                "--params-file", str(config_path), "-p", f"model_path:={policy_path}",
                "-r", "/policy/observation:=/policy_mujoco/observation",
                "-r", "/policy/joint_targets:=/policy_mujoco/joint_targets",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait_until(lambda: publisher.get_subscription_count() > 0 and
                           probe.count_publishers("/policy_mujoco/joint_targets") > 0,
                           "ROS discovery did not connect both topics.")
                for step in range(args.steps):
                    joint_positions = np.asarray(data.qpos[qpos_indices], dtype=np.float32)
                    joint_velocities = np.asarray(data.qvel[qvel_indices], dtype=np.float32)
                    gravity = projected_gravity(sensor(data, model, "imu_quat"))
                    observation = np.concatenate([
                        np.asarray(args.command, dtype=np.float32),
                        sensor(data, model, "imu_gyro").astype(np.float32),
                        gravity.astype(np.float32), joint_positions - defaults,
                        joint_velocities, previous_actions,
                    ])
                    if observation.shape != (75,) or not np.isfinite(observation).all():
                        raise AssertionError(f"Invalid simulated observation at step {step}.")
                    received.clear()
                    publisher.publish(Float32MultiArray(data=observation.tolist()))
                    wait_until(lambda: bool(received), f"No policy targets at step {step}.")
                    message = received[0]
                    targets = np.asarray(message.position, dtype=np.float32)
                    if list(message.name) != joint_names or targets.shape != (22,) or not np.isfinite(targets).all():
                        raise AssertionError(f"Invalid policy targets at step {step}.")
                    outside_limits = np.logical_or(targets < joint_limits[:, 0],
                                                   targets > joint_limits[:, 1])
                    previous_actions = (targets - defaults) / action_scale
                    for _ in range(round(simulation_profile["policy_dt"] / model.opt.timestep)):
                        positions = np.asarray(data.qpos[qpos_indices])
                        velocities = np.asarray(data.qvel[qvel_indices])
                        torques = np.asarray(simulation_profile["joint_kp"]) * (targets - positions) - (
                            np.asarray(simulation_profile["joint_kd"]) * velocities)
                        data.ctrl[:] = np.clip(torques, -np.asarray(simulation_profile["effort_limits"]),
                                               np.asarray(simulation_profile["effort_limits"]))
                        mujoco.mj_step(model, data)
                    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                        raise AssertionError(f"Non-finite simulated state at step {step}.")
                    report["steps"].append({
                        "step": step, "time_s": float(data.time),
                        "base_height_m": float(data.qpos[2]),
                        "max_joint_change_rad": float(np.max(np.abs(data.qpos[qpos_indices] - initial_positions))),
                        "max_target_abs_rad": float(np.max(np.abs(targets))),
                        "targets_outside_joint_limits": int(np.count_nonzero(outside_limits)),
                    })
                if report["steps"][-1]["max_joint_change_rad"] <= 1e-4:
                    raise AssertionError("The simulated joints did not respond to policy targets.")
                report["ros_feedback_loop_completed"] = True
                report["steps_with_out_of_range_targets"] = sum(
                    step["targets_outside_joint_limits"] > 0 for step in report["steps"])
                args.output.write_text(json.dumps(report, indent=2) + "\n")
                print(f"Completed: {args.steps} ROS inference steps drove evolving MuJoCo state")
                print(f"Simulated joint change: {report['steps'][-1]['max_joint_change_rad']:.4f} rad")
                print(f"Steps with targets outside joint limits: {report['steps_with_out_of_range_targets']}")
                print(f"Saved {args.output}")
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
    finally:
        probe.destroy_subscription(subscription)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
