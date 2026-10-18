"""Check cmd_vel + synthetic robot feedback -> observation -> real ONNX targets.

This uses a private ROS domain and topic remaps. It does not contact motors.
Run from the workspace root after sourcing ROS and install/setup.bash.
"""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=119)
    args = parser.parse_args()
    model_path = args.model.resolve(strict=True)
    config = Path("src/th_policy_node/config/policy_humanoid.yaml").resolve(strict=True)
    profile = yaml.safe_load(config.read_text())["policy_node"]["ros__parameters"]
    names = profile["joint_names"]
    defaults = np.asarray(profile["default_joint_positions"], dtype=np.float32)
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    rclpy.init()
    node = rclpy.create_node("issue18_probe")
    command_pub = node.create_publisher(Twist, "/issue18/cmd_vel", 10)
    imu_pub = node.create_publisher(Imu, "/issue18/imu", 10)
    joints_pub = node.create_publisher(JointState, "/issue18/joint_states", 10)
    observations = []
    targets = []
    node.create_subscription(Float32MultiArray, "/issue18/observation",
                             observations.append, 10)
    node.create_subscription(JointState, "/issue18/joint_targets", targets.append, 10)
    processes = []
    Path("log").mkdir(exist_ok=True)

    def start(command, log_name):
        log = Path("log", log_name).open("w")
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((process, log))

    def spin_until(predicate, label, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            if any(process.poll() is not None for process, _ in processes):
                raise RuntimeError(f"A node exited while waiting for {label}; see log/issue18_*.log")
            if time.monotonic() > deadline:
                raise TimeoutError(label)
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        start([
            "ros2", "run", "th_policy_integration", "observation_builder", "--ros-args",
            "-r", "/cmd_vel:=/issue18/cmd_vel",
            "-r", "/imu/data:=/issue18/imu",
            "-r", "/joint_states:=/issue18/joint_states",
            "-r", "/policy/observation:=/issue18/observation",
            "-r", "/policy/joint_targets:=/issue18/joint_targets",
        ], "issue18_builder.log")
        start([
            "ros2", "run", "th_policy_node", "policy_node", "--ros-args",
            "--params-file", str(config), "-p", f"model_path:={model_path}",
            "-r", "/policy/observation:=/issue18/observation",
            "-r", "/policy/joint_targets:=/issue18/joint_targets",
        ], "issue18_policy.log")
        spin_until(lambda: command_pub.get_subscription_count() and
                   imu_pub.get_subscription_count() and
                   joints_pub.get_subscription_count() and
                   node.count_publishers("/issue18/observation") and
                   node.count_publishers("/issue18/joint_targets"), "ROS discovery")

        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.1
        imu = Imu()
        imu.orientation.w = 1.0
        joints = JointState()
        joints.name = list(reversed(names))
        joints.position = [float(defaults[names.index(name)]) for name in joints.name]
        joints.velocity = [0.0] * 22

        # Keep state fresh until the first observation is built.
        deadline = time.monotonic() + 10
        while not observations and time.monotonic() < deadline:
            command_pub.publish(command)
            imu_pub.publish(imu)
            joints_pub.publish(joints)
            rclpy.spin_once(node, timeout_sec=0.05)
        if not observations:
            raise TimeoutError("No assembled policy observation")
        first = np.asarray(observations[0].data, dtype=np.float32)
        expected_observation = np.zeros(75, dtype=np.float32)
        expected_observation[:3] = [0.2, 0.0, 0.1]
        expected_observation[8] = -1.0
        np.testing.assert_allclose(first, expected_observation, atol=1e-6)
        spin_until(lambda: bool(targets), "real ONNX joint targets")
        if list(targets[0].name) != names:
            raise AssertionError("Policy joint names differ from configured order")
        raw_actions = session.run(None, {input_name: first.reshape(1, 75)})[0].reshape(-1)
        actions = np.clip(raw_actions, profile["action_limit_lower"],
                          profile["action_limit_upper"])
        expected_targets = defaults + profile["action_scale"] * actions
        np.testing.assert_allclose(targets[0].position, expected_targets,
                                   rtol=1e-5, atol=1e-6)
        print("PASS: gamepad-format Twist and shuffled joint feedback produced the expected 75-value observation")
        print("PASS: real ONNX policy produced 22 named targets matching direct inference")

        observations.clear()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            command_pub.publish(command)
            imu_pub.publish(imu)
            joints_pub.publish(joints)
            rclpy.spin_once(node, timeout_sec=0.05)
            if observations and np.allclose(observations[-1].data[53:], actions, atol=1e-5):
                break
        else:
            raise TimeoutError("Previous policy actions did not enter the next observation")
        print("PASS: next observation carries the previous clipped actions")

        # Stop all feedback. After the freshness window, no new observation
        # should be published, even though the policy timer keeps firing.
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        observations.clear()
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if observations:
            raise AssertionError("Builder published with stale robot feedback")
        print("PASS: stale IMU and joint feedback stop observation publication")
    finally:
        for process, log in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
