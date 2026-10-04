"""Boundary tests with real NumPy and stand-ins for ROS and ONNX Runtime.

Run with: python3 -m unittest discover -s src/th_policy_node/test -v
These tests do not validate ROS transport or execution of a real ONNX model.
"""

import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.logger = Mock()

    def declare_parameter(self, name, default):
        self.parameters[name] = self.overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Mock()

    def create_subscription(self, *args):
        return Mock()

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return Mock()


runtime = SimpleNamespace(InferenceSession=Mock())
stubs = {
    "onnxruntime": runtime,
    "rclpy": SimpleNamespace(),
    "rclpy.node": SimpleNamespace(Node=FakeNode),
    "sensor_msgs": SimpleNamespace(),
    "sensor_msgs.msg": SimpleNamespace(
        JointState=lambda: SimpleNamespace(header=SimpleNamespace())
    ),
    "std_msgs": SimpleNamespace(),
    "std_msgs.msg": SimpleNamespace(Float32MultiArray=object),
}
source = Path(__file__).resolve().parents[1] / "th_policy_node/policy_node.py"
spec = importlib.util.spec_from_file_location("policy_under_test", source)
policy = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, stubs):
    spec.loader.exec_module(policy)


class PolicyBoundaryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        model = Path(temporary.name) / "fixture.onnx"
        model.touch()  # Loading is stubbed; this is not an executable model.
        overrides = patch.dict(FakeNode.overrides, {
            "model_path": str(model),
            "expected_observation_size": 3,
            "joint_names": ["left", "right"],
            "default_joint_positions": [0.1, -0.1],
            "action_limit_lower": -1.0,
            "action_limit_upper": 1.0,
        }, clear=True)
        overrides.start()
        self.addCleanup(overrides.stop)
        self.session = Mock()
        self.session.get_inputs.return_value = [
            SimpleNamespace(name="obs", shape=["batch", 3], type="tensor(float)")
        ]
        self.session.get_outputs.return_value = [
            SimpleNamespace(name="actions", type="tensor(float)")
        ]
        self.session.run.return_value = [np.array([[2.0, -2.0]], dtype=np.float32)]
        runtime.InferenceSession.return_value = self.session
        self.node = policy.PolicyNode()

    def feed(self, values):
        self.node._on_observation(SimpleNamespace(data=values))

    def test_valid_input_publishes_named_scaled_clipped_targets(self):
        self.feed([1.0, 2.0, 3.0])
        sent = self.node._publisher.publish.call_args.args[0]
        self.assertEqual(sent.name, ["left", "right"])
        np.testing.assert_allclose(sent.position, [0.35, -0.35])
        observation = self.session.run.call_args.args[1]["obs"]
        self.assertEqual(observation.shape, (1, 3))
        self.assertEqual(observation.dtype, np.float32)

    def test_invalid_observations_never_reach_inference(self):
        for values in ([1.0], [0.0, np.nan, 0.0], [np.inf, 0.0, 0.0]):
            with self.subTest(values=values):
                self.feed(values)
        self.session.run.assert_not_called()
        self.node._publisher.publish.assert_not_called()

    def test_invalid_actions_are_not_published(self):
        for values in ([[1.0]], [[np.nan, 0.0]], [[np.inf, 0.0]]):
            with self.subTest(values=values):
                self.session.run.return_value = [np.array(values)]
                self.feed([0.0, 0.0, 0.0])
        self.node._publisher.publish.assert_not_called()

    def test_inference_failure_is_not_published(self):
        self.session.run.side_effect = RuntimeError("inference failed")
        self.feed([0.0, 0.0, 0.0])
        self.node._publisher.publish.assert_not_called()
        self.node.logger.error.assert_called()

    def test_target_overflow_is_not_published(self):
        self.node._action_scale = 1e38
        self.node._action_max = 1e38
        self.session.run.return_value = [np.array([[1e38, 1e38]], dtype=np.float32)]
        self.feed([0.0, 0.0, 0.0])
        self.node._publisher.publish.assert_not_called()

    def test_incompatible_input_type_fails_at_startup(self):
        self.session.get_inputs.return_value[0].type = "tensor(double)"
        with self.assertRaisesRegex(ValueError, "float32"):
            policy.PolicyNode()

    def test_ambiguous_output_fails_at_startup(self):
        self.session.get_outputs.return_value *= 2
        with self.assertRaisesRegex(ValueError, "one float32"):
            policy.PolicyNode()

    def test_incompatible_shape_fails_at_startup(self):
        self.session.get_inputs.return_value[0].shape = [1, 4]
        with self.assertRaisesRegex(ValueError, "requires 4 values"):
            policy.PolicyNode()


if __name__ == "__main__":
    unittest.main()
