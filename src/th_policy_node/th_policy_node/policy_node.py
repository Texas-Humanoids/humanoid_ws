#!/usr/bin/env python3
"""Run a trained ONNX policy on a policy-ready observation vector."""

from pathlib import Path

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


BERKELEY_HUMANOID_JOINTS = [
    "arm_left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint",
    "arm_left_elbow_pitch_joint",
    "arm_left_elbow_roll_joint",
    "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_pitch_joint",
    "arm_right_elbow_roll_joint",
    "leg_left_hip_roll_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_ankle_roll_joint",
]

BERKELEY_HUMANOID_DEFAULT_POSITIONS = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.2,
    0.4,
    -0.3,
    0.0,
    0.0,
    0.0,
    -0.2,
    0.4,
    -0.3,
    0.0,
]


class PolicyNode(Node):
    """Apply an ONNX policy and publish named joint position targets.

    The input is expected to be the complete, already ordered observation
    vector used to train the selected model. This node intentionally does not
    assemble observations from hardware topics or connect to an actuator API.
    """

    def __init__(self) -> None:
        super().__init__("policy_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("observation_topic", "/policy/observation")
        self.declare_parameter("joint_target_topic", "/policy/joint_targets")
        self.declare_parameter("expected_observation_size", 75)
        self.declare_parameter("joint_names", BERKELEY_HUMANOID_JOINTS)
        self.declare_parameter(
            "default_joint_positions", BERKELEY_HUMANOID_DEFAULT_POSITIONS
        )
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("action_limit_lower", -10000.0)
        self.declare_parameter("action_limit_upper", 10000.0)

        model_path_value = self.get_parameter("model_path").value
        if not model_path_value:
            raise ValueError(
                "Set the model_path parameter to a deployed .onnx policy file."
            )
        model_path = Path(model_path_value).expanduser().resolve(strict=True)
        if not model_path.is_file() or model_path.suffix.lower() != ".onnx":
            raise ValueError(f"Expected an .onnx model, got: {model_path}")

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._default_positions = np.asarray(
            self.get_parameter("default_joint_positions").value, dtype=np.float32
        )
        self._observation_size = int(
            self.get_parameter("expected_observation_size").value
        )
        self._action_scale = float(self.get_parameter("action_scale").value)
        self._action_min = float(self.get_parameter("action_limit_lower").value)
        self._action_max = float(self.get_parameter("action_limit_upper").value)

        if not self._joint_names or len(set(self._joint_names)) != len(
            self._joint_names
        ):
            raise ValueError("joint_names must contain unique joint names.")
        if len(self._default_positions) != len(self._joint_names):
            raise ValueError(
                "default_joint_positions must have one value for each joint_name."
            )
        if not np.isfinite(self._default_positions).all():
            raise ValueError("default_joint_positions must contain finite values.")
        if self._observation_size <= 0:
            raise ValueError("expected_observation_size must be positive.")
        if self._action_min >= self._action_max:
            raise ValueError("action_limit_lower must be less than action_limit_upper.")
        if not np.isfinite(
            [self._action_scale, self._action_min, self._action_max]
        ).all():
            raise ValueError("Action scale and limits must be finite.")

        self._session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        model_inputs = self._session.get_inputs()
        if len(model_inputs) != 1:
            raise ValueError(
                f"Expected one ONNX model input, found {len(model_inputs)}."
            )
        self._model_input = model_inputs[0]
        if self._model_input.type != "tensor(float)":
            raise ValueError("The ONNX observation input must use float32 values.")
        model_outputs = self._session.get_outputs()
        if len(model_outputs) != 1 or model_outputs[0].type != "tensor(float)":
            raise ValueError("Expected one float32 ONNX action output.")
        self._input_shape = self._make_input_shape(self._model_input.shape)

        self._publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_target_topic").value),
            10,
        )
        self._subscription = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("observation_topic").value),
            self._on_observation,
            10,
        )

        self.get_logger().info(
            f"Loaded policy {model_path}; expecting {self._observation_size} "
            f"observation values and publishing targets for {len(self._joint_names)} joints."
        )
        self.get_logger().warning(
            "Policy output is diagnostic only. No motor or ros2_control command "
            "interface is connected by this node."
        )

    def _make_input_shape(self, model_shape: list[object]) -> tuple[int, ...]:
        """Resolve dynamic ONNX dimensions while preserving fixed dimensions."""
        if not model_shape:
            raise ValueError("The ONNX model input must have a vector shape.")

        shape = tuple(
            dimension if isinstance(dimension, int) and dimension > 0 else 1
            for dimension in model_shape
        )
        if int(np.prod(shape)) != self._observation_size:
            raise ValueError(
                "The ONNX input shape requires "
                f"{int(np.prod(shape))} values, but expected_observation_size is "
                f"{self._observation_size}."
            )
        return shape

    def _on_observation(self, message: Float32MultiArray) -> None:
        observation = np.asarray(message.data, dtype=np.float32)
        if observation.size != self._observation_size:
            self.get_logger().warning(
                f"Ignoring observation with {observation.size} values; expected "
                f"{self._observation_size}."
            )
            return
        if not np.isfinite(observation).all():
            self.get_logger().warning("Ignoring observation containing non-finite values.")
            return

        try:
            model_input = observation.reshape(self._input_shape)
            model_output = self._session.run(
                None, {self._model_input.name: model_input}
            )[0]
        except Exception as error:  # ONNX Runtime reports model-specific failures.
            self.get_logger().error(f"Policy inference failed: {error}")
            return

        actions = np.asarray(model_output, dtype=np.float32).reshape(-1)
        if actions.size != len(self._joint_names):
            self.get_logger().error(
                f"Policy returned {actions.size} actions for "
                f"{len(self._joint_names)} configured joints; not publishing."
            )
            return
        if not np.isfinite(actions).all():
            self.get_logger().error("Policy returned non-finite actions; not publishing.")
            return

        clipped_actions = np.clip(actions, self._action_min, self._action_max)
        with np.errstate(over="ignore", invalid="ignore"):
            target_positions = (
                clipped_actions * self._action_scale + self._default_positions
            )
        if not np.isfinite(target_positions).all():
            self.get_logger().error("Computed non-finite joint targets; not publishing.")
            return

        targets = JointState()
        targets.header.stamp = self.get_clock().now().to_msg()
        targets.name = self._joint_names
        targets.position = target_positions.tolist()
        self._publisher.publish(targets)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PolicyNode | None = None
    try:
        node = PolicyNode()
        rclpy.spin(node)
    except (OSError, ValueError) as error:
        print(f"Could not start the policy node: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
