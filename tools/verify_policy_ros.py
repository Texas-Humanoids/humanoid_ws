"""Run the installed policy node against real ONNX and synthetic ROS messages.

Run from the workspace root after sourcing ROS and install/setup.bash.
This is an inference/transport check, not a physics simulation or walking test.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import onnxruntime as ort
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("policies/policy_humanoid.onnx"))
    parser.add_argument("--config", type=Path, default=Path(
        "src/th_policy_node/config/policy_humanoid.yaml"))
    parser.add_argument("--output", type=Path, default=Path("log/policy_demo.json"))
    parser.add_argument("--domain-id", type=int, default=117)
    args = parser.parse_args()
    model = args.model.resolve(strict=True)
    config = args.config.resolve(strict=True)
    profile = yaml.safe_load(config.read_text())["policy_node"]["ros__parameters"]
    if profile["expected_observation_size"] != 75 or len(profile["joint_names"]) != 22:
        parser.error("This synthetic fixture is for the Berkeley 75-input/22-joint profile.")

    # Use separate diagnostic topics and a local ROS domain, including in the child.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    session = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
    model_input = session.get_inputs()[0]
    received = []
    report = {
        "kind": "synthetic observation / real ONNX / ROS transport",
        "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "onnxruntime": ort.__version__,
        "input": {"name": model_input.name, "shape": model_input.shape},
        "output_shape": session.get_outputs()[0].shape,
        "joint_names": profile["joint_names"],
        "cases": [],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("policy_demo_probe")
    publisher = node.create_publisher(Float32MultiArray, "/policy_demo/observation", 10)
    subscription = node.create_subscription(
        JointState, "/policy_demo/joint_targets", received.append, 10)
    process = None

    def wait_until(predicate, description, seconds=15):
        deadline = time.monotonic() + seconds
        while not predicate():
            if process.poll() is not None:
                raise RuntimeError("Policy process exited; see policy_demo_node.log")
            if time.monotonic() >= deadline:
                raise TimeoutError(description)
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        with args.output.with_name("policy_demo_node.log").open("w") as log:
            process = subprocess.Popen([
                "ros2", "run", "th_policy_node", "policy_node", "--ros-args",
                "--params-file", str(config), "-p", f"model_path:={model}",
                "-r", "/policy/observation:=/policy_demo/observation",
                "-r", "/policy/joint_targets:=/policy_demo/joint_targets",
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait_until(lambda: publisher.get_subscription_count() > 0 and
                           node.count_publishers("/policy_demo/joint_targets") > 0,
                           "ROS discovery did not connect both topics.")
                previous_actions = np.zeros(22, dtype=np.float32)
                for name, command in (
                    ("stationary", [0.0, 0.0, 0.0]),
                    ("forward", [0.2, 0.0, 0.0]),
                    ("turn", [0.0, 0.0, 0.2]),
                ):
                    # Fixed upright/default-pose state; previous actions are carried
                    # across steps just as Berkeley's runner does. No physics is modeled.
                    observation = np.zeros(75, dtype=np.float32)
                    observation[:3] = command
                    observation[8] = -1.0
                    observation[53:] = previous_actions
                    actions = session.run(None, {
                        model_input.name: observation.reshape(1, 75)})[0].reshape(-1)
                    previous_actions = np.clip(actions, profile["action_limit_lower"],
                                               profile["action_limit_upper"])
                    expected = previous_actions * profile["action_scale"] + np.asarray(
                        profile["default_joint_positions"], dtype=np.float32)
                    received.clear()
                    publisher.publish(Float32MultiArray(data=observation.tolist()))
                    wait_until(lambda: bool(received), f"No targets for {name}.")
                    message = received[0]
                    if list(message.name) != profile["joint_names"]:
                        raise AssertionError("Published joint order differs from the profile.")
                    actual = np.asarray(message.position)
                    if actual.shape != (22,) or not np.isfinite(actual).all():
                        raise AssertionError("Expected 22 finite joint targets.")
                    np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-6)
                    report["cases"].append({
                        "name": name, "observation": observation.tolist(),
                        "targets_rad": actual.tolist(),
                        "max_error_rad": float(np.max(np.abs(actual - expected))),
                    })
                    print(f"PASS {name}: 22 finite, named targets match real ONNX inference")
                for bad in ([0.0] * 74, [float("nan")] + [0.0] * 74):
                    received.clear()
                    publisher.publish(Float32MultiArray(data=bad))
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        if process.poll() is not None:
                            raise RuntimeError("Policy node exited after invalid observation")
                        rclpy.spin_once(node, timeout_sec=0.05)
                    if received:
                        raise AssertionError("Invalid observation produced joint targets")
                if process.poll() is not None:
                    raise RuntimeError("Policy node exited after invalid observations")
                received.clear()
                publisher.publish(Float32MultiArray(data=observation.tolist()))
                wait_until(lambda: bool(received), "No targets after invalid observations.")
                message = received[0]
                if list(message.name) != profile["joint_names"]:
                    raise AssertionError("Joint order changed after invalid observations")
                actual = np.asarray(message.position)
                if actual.shape != (22,) or not np.isfinite(actual).all():
                    raise AssertionError("Invalid targets after invalid observations")
                np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-6)
                print("PASS invalid observations: no targets published; node still responds")
                report["passed"] = True
                args.output.write_text(json.dumps(report, indent=2) + "\n")
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
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
