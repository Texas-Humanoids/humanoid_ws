#!/usr/bin/env python3
"""Run the Berkeley ONNX policy on complete ROS observation vectors."""

from pathlib import Path

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


JOINT_NAMES = [
    "arm_left_shoulder_pitch_joint", "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint", "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint", "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint", "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint", "arm_right_elbow_roll_joint",
    "leg_left_hip_roll_joint", "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint", "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint", "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint", "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint", "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint",
]
DEFAULT_POSITIONS = [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.2, 0.4, -0.3, 0.0, 0.0, 0.0, -0.2, 0.4, -0.3, 0.0,
]


class PolicyNode(Node):
    """Turn one policy-ready observation into 22 named joint targets."""

    def __init__(self):
        super().__init__("policy_node")
        defaults = {
            "model_path": "",
            "observation_topic": "/policy/observation",
            "joint_target_topic": "/policy/joint_targets",
            "expected_observation_size": 75,
            "joint_names": JOINT_NAMES,
            "default_joint_positions": DEFAULT_POSITIONS,
            "action_scale": 0.25,
            "action_limit_lower": -10000.0,
            "action_limit_upper": 10000.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        def parameter(name):
            return self.get_parameter(name).value

        if not parameter("model_path"):
            raise ValueError("Set model_path to a deployed .onnx policy file.")
        model_path = Path(parameter("model_path")).expanduser().resolve(strict=True)
        if not model_path.is_file() or model_path.suffix.lower() != ".onnx":
            raise ValueError(f"Expected an .onnx model, got: {model_path}")

        self._joint_names = list(parameter("joint_names"))
        self._default_positions = np.asarray(
            parameter("default_joint_positions"), dtype=np.float32
        )
        self._observation_size = int(parameter("expected_observation_size"))
        self._action_scale = float(parameter("action_scale"))
        self._action_min = float(parameter("action_limit_lower"))
        self._action_max = float(parameter("action_limit_upper"))
        if (not self._joint_names or
                len(set(self._joint_names)) != len(self._joint_names) or
                len(self._default_positions) != len(self._joint_names) or
                not np.isfinite(self._default_positions).all() or
                self._observation_size <= 0 or
                not np.isfinite([self._action_scale, self._action_min,
                                 self._action_max]).all() or
                self._action_min >= self._action_max):
            raise ValueError("Invalid policy joint profile or action parameters")

        self._session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self._session.get_inputs(), self._session.get_outputs()
        if len(inputs) != 1 or inputs[0].type != "tensor(float)":
            raise ValueError("Expected one float32 ONNX observation input")
        if len(outputs) != 1 or outputs[0].type != "tensor(float)":
            raise ValueError("Expected one float32 ONNX action output")
        self._model_input = inputs[0]
        self._input_shape = self._vector_shape(
            inputs[0].shape, self._observation_size, "input"
        )
        self._vector_shape(outputs[0].shape, len(self._joint_names), "output")

        self._publisher = self.create_publisher(
            JointState, str(parameter("joint_target_topic")), 10
        )
        self._subscription = self.create_subscription(
            Float32MultiArray, str(parameter("observation_topic")),
            self._on_observation, 10
        )
        self.get_logger().info(
            f"Loaded {model_path}; {self._observation_size} inputs, "
            f"{len(self._joint_names)} joint targets"
        )
        self.get_logger().warning("Policy targets are diagnostic; no motor is connected")

    @staticmethod
    def _vector_shape(shape, width, label):
        if len(shape) not in (1, 2):
            raise ValueError(f"ONNX {label} must have one vector")
        if len(shape) == 2 and isinstance(shape[0], int) and shape[0] != 1:
            raise ValueError(f"ONNX {label} batch must be one or dynamic")
        if not isinstance(shape[-1], int) or shape[-1] != width:
            raise ValueError(
                f"The ONNX {label} shape requires {shape[-1]} values; "
                f"configured size is {width}"
            )
        return (width,) if len(shape) == 1 else (1, width)

    def _on_observation(self, message):
        observation = np.asarray(message.data, dtype=np.float32)
        if observation.size != self._observation_size or not np.isfinite(observation).all():
            self.get_logger().warning("Ignoring invalid policy observation")
            return
        try:
            result = self._session.run(None, {
                self._model_input.name: observation.reshape(self._input_shape)
            })[0]
        except Exception as error:
            self.get_logger().error(f"Policy inference failed: {error}")
            return
        actions = np.asarray(result, dtype=np.float32).reshape(-1)
        if actions.size != len(self._joint_names) or not np.isfinite(actions).all():
            self.get_logger().error("Policy returned invalid actions")
            return
        with np.errstate(over="ignore", invalid="ignore"):
            targets = (np.clip(actions, self._action_min, self._action_max) *
                       self._action_scale + self._default_positions)
        if not np.isfinite(targets).all():
            self.get_logger().error("Computed non-finite joint targets")
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._joint_names
        message.position = targets.tolist()
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
