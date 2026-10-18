"""Build Berkeley's 75-value observation from live ROS topics."""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from th_policy_node.profile import (
    BERKELEY_HUMANOID_DEFAULT_POSITIONS,
    BERKELEY_HUMANOID_JOINTS,
)


def projected_gravity(orientation):
    """Return world gravity expressed in the body frame (ROS x/y/z/w)."""
    quaternion = np.asarray(
        [orientation.w, orientation.x, orientation.y, orientation.z], dtype=np.float64
    )
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError("IMU orientation is missing or invalid")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        dtype=np.float32,
    ) * -1.0


def ordered_joint_state(message, joint_names):
    """Extract positions and velocities in policy order, rejecting gaps."""
    if len(message.name) != len(set(message.name)):
        raise ValueError("joint feedback has duplicate names")
    if len(message.position) != len(message.name) or len(message.velocity) != len(message.name):
        raise ValueError("joint feedback needs positions and velocities for every name")
    lookup = {name: index for index, name in enumerate(message.name)}
    if any(name not in lookup for name in joint_names):
        raise ValueError("joint feedback is missing a policy joint")
    indices = [lookup[name] for name in joint_names]
    positions = np.asarray(message.position, dtype=np.float32)[indices]
    velocities = np.asarray(message.velocity, dtype=np.float32)[indices]
    if not np.isfinite(positions).all() or not np.isfinite(velocities).all():
        raise ValueError("joint feedback contains non-finite values")
    return positions, velocities


class ObservationBuilder(Node):
    """Publish an observation only while complete robot feedback is fresh."""

    def __init__(self):
        super().__init__("observation_builder")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("policy_target_topic", "/policy/joint_targets")
        self.declare_parameter("observation_topic", "/policy/observation")
        self.declare_parameter("max_state_age_sec", 0.2)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("policy_period_sec", 0.04)
        self.declare_parameter("joint_names", BERKELEY_HUMANOID_JOINTS)
        self.declare_parameter("default_joint_positions", BERKELEY_HUMANOID_DEFAULT_POSITIONS)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("action_limit_lower", -10000.0)
        self.declare_parameter("action_limit_upper", 10000.0)

        self._names = list(self.get_parameter("joint_names").value)
        self._defaults = np.asarray(
            self.get_parameter("default_joint_positions").value, dtype=np.float32
        )
        self._scale = float(self.get_parameter("action_scale").value)
        self._lower = float(self.get_parameter("action_limit_lower").value)
        self._upper = float(self.get_parameter("action_limit_upper").value)
        self._max_age = float(self.get_parameter("max_state_age_sec").value)
        self._cmd_timeout = float(self.get_parameter("command_timeout_sec").value)
        period = float(self.get_parameter("policy_period_sec").value)
        if (len(self._names) != 22 or len(set(self._names)) != 22 or
                self._defaults.shape != (22,) or not np.isfinite(self._defaults).all() or
                not np.isfinite([self._scale, self._lower, self._upper,
                                 self._max_age, self._cmd_timeout, period]).all() or
                self._scale == 0 or self._lower >= self._upper or
                min(self._max_age, self._cmd_timeout, period) <= 0):
            raise ValueError("Invalid Berkeley observation configuration")

        self._command = np.zeros(3, dtype=np.float32)
        self._command_at = None
        self._gyro = None
        self._gravity = None
        self._imu_at = None
        self._positions = None
        self._velocities = None
        self._joints_at = None
        self._previous_actions = np.zeros(22, dtype=np.float32)

        self._publisher = self.create_publisher(
            Float32MultiArray, str(self.get_parameter("observation_topic").value), 10
        )
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value),
                                 self._on_command, 10)
        self.create_subscription(Imu, str(self.get_parameter("imu_topic").value),
                                 self._on_imu, 10)
        self.create_subscription(JointState, str(self.get_parameter("joint_state_topic").value),
                                 self._on_joints, 10)
        self.create_subscription(JointState, str(self.get_parameter("policy_target_topic").value),
                                 self._on_targets, 10)
        self.create_timer(period, self._publish_observation)
        self.get_logger().info("Waiting for fresh IMU and 22-joint feedback")

    def _on_command(self, message):
        command = np.asarray(
            [message.linear.x, message.linear.y, message.angular.z], dtype=np.float32
        )
        if np.isfinite(command).all():
            self._command = command
            self._command_at = time.monotonic()

    def _on_imu(self, message):
        try:
            gyro = np.asarray(
                [message.angular_velocity.x, message.angular_velocity.y,
                 message.angular_velocity.z], dtype=np.float32
            )
            gravity = projected_gravity(message.orientation)
            if not np.isfinite(gyro).all():
                raise ValueError("IMU angular velocity is non-finite")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._gyro, self._gravity = gyro, gravity
        self._imu_at = time.monotonic()

    def _on_joints(self, message):
        try:
            positions, velocities = ordered_joint_state(message, self._names)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._positions, self._velocities = positions, velocities
        self._joints_at = time.monotonic()

    def _on_targets(self, message):
        if list(message.name) != self._names or len(message.position) != 22:
            return
        targets = np.asarray(message.position, dtype=np.float32)
        if np.isfinite(targets).all():
            self._previous_actions = np.clip(
                (targets - self._defaults) / self._scale, self._lower, self._upper
            )

    def _publish_observation(self):
        now = time.monotonic()
        if (self._imu_at is None or self._joints_at is None or
                now - self._imu_at > self._max_age or
                now - self._joints_at > self._max_age):
            return
        command = (self._command if self._command_at is not None and
                   now - self._command_at <= self._cmd_timeout else
                   np.zeros(3, dtype=np.float32))
        observation = np.concatenate((
            command, self._gyro, self._gravity,
            self._positions - self._defaults, self._velocities,
            self._previous_actions,
        ))
        if observation.shape == (75,) and np.isfinite(observation).all():
            self._publisher.publish(Float32MultiArray(data=observation.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObservationBuilder()
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
