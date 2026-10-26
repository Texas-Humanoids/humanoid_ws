"""Run the real policy_bridge executable over DDS with fake motors, IMU and policy."""

import os
import signal
import subprocess
import tempfile
import time
import uuid

from geometry_msgs.msg import Quaternion, Twist, Vector3
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Float64, String

# The ankle has no motor, so it always reads as its default pose.
JOINTS = ['hip_joint', 'knee_joint', 'ankle_joint']
DEFAULTS = [0.0, 0.4, -0.3]
SCALE = 0.25
MOTORS = ['hip', 'knee']


class Harness:
    """Starts policy_bridge in an isolated namespace and plays the rest of the robot."""

    def __init__(self, **parameters):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.ns = '/test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('fake_robot', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.observations = []
        self.commands = {motor: [] for motor in MOTORS}
        self.motor_state = {motor: (0.0, 0.0) for motor in MOTORS}  # (position, velocity)

        topic = self.topic
        self.node.create_subscription(
            Float32MultiArray, topic('observation'), self.observations.append, 10)
        self.position = {}
        self.velocity = {}
        for motor in MOTORS:
            self.node.create_subscription(
                Float64, topic(motor + '/motor/target_position'), self.commands[motor].append, 10)
            self.position[motor] = self.node.create_publisher(
                Float64, topic(motor + '/motor/actual_position'), 10)
            self.velocity[motor] = self.node.create_publisher(
                Float64, topic(motor + '/motor/actual_velocity'), 10)
        self.cmd_vel = self.node.create_publisher(Twist, topic('cmd_vel'), 10)
        self.imu = self.node.create_publisher(Imu, topic('imu'), qos_profile_sensor_data)
        self.targets = self.node.create_publisher(JointState, topic('targets'), 10)
        self.buttons = self.node.create_publisher(String, topic('buttons'), 10)

        parameters = {
            'joint_names': JOINTS,
            'default_joint_positions': DEFAULTS,
            'action_scale': SCALE,
            'motor_joints': ['hip_joint', 'knee_joint'],
            'motor_namespaces': [topic('hip'), topic('knee')],
            # The knee motor is mounted reversed with a 1 rad offset.
            'directions': [1.0, -1.0],
            'offsets': [0.0, 1.0],
            'lower_limits': [-0.5, 0.0],
            'upper_limits': [0.5, 1.5],
            'cmd_vel_topic': topic('cmd_vel'),
            'imu_topic': topic('imu'),
            'joint_target_topic': topic('targets'),
            'button_topic': topic('buttons'),
            'observation_topic': topic('observation'),
            **parameters,
        }
        args = [os.environ['POLICY_BRIDGE_EXECUTABLE'], '--ros-args']
        for name, value in parameters.items():
            if isinstance(value, list):
                value = '[' + ', '.join(
                    f"'{v}'" if isinstance(v, str) else repr(float(v)) for v in value) + ']'
            elif isinstance(value, bool):
                value = str(value).lower()
            args.extend(['-p', f'{name}:={value}'])
        self.log = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)

    def topic(self, name):
        return self.ns + '/' + name

    def diagnostics(self):
        self.log.seek(0)
        return self.log.read()

    def spin_for(self, seconds, step=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if step:
                step()
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, predicate, timeout=5.0, step=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if step:
                step()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail('Timed out waiting for policy_bridge:\n' + self.diagnostics())

    def publish_motors(self, motors=MOTORS):
        for motor in motors:
            position, velocity = self.motor_state[motor]
            self.position[motor].publish(Float64(data=position))
            self.velocity[motor].publish(Float64(data=velocity))

    def publish_imu(self):
        msg = Imu(orientation=Quaternion(w=1.0))
        msg.angular_velocity.z = 0.5
        self.imu.publish(msg)

    def publish_sensors(self):
        self.publish_motors()
        self.publish_imu()

    def publish_targets(self, positions):
        self.targets.publish(JointState(name=JOINTS, position=positions))

    def arm(self, step):
        self.spin_for(0.3, step=step)
        for commands in self.commands.values():
            commands.clear()
        self.buttons.publish(String(data='START pressed'))
        self.wait(lambda: all(self.commands.values()), step=step)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.log.close()


@pytest.fixture
def harness():
    started = []

    def start(**parameters):
        h = Harness(**parameters)
        started.append(h)
        h.wait(lambda: h.targets.get_subscription_count() == 1 and
               h.buttons.get_subscription_count() == 1 and
               h.node.count_publishers(h.topic('observation')) == 1)
        return h

    yield start
    for h in started:
        h.close()


def test_observation_combines_command_imu_motors_and_previous_actions(harness):
    h = harness()
    # Knee motor 0.7 rad, -2 rad/s is joint -(0.7 - 1.0) = 0.3 rad, +2 rad/s.
    h.motor_state = {'hip': (0.1, 1.5), 'knee': (0.7, -2.0)}

    def step():
        h.publish_sensors()
        h.publish_targets([0.25, 0.5, -0.3])  # actions 1.0, 0.4, 0.0
        h.cmd_vel.publish(Twist(linear=Vector3(x=0.3)))

    h.wait(lambda: any(abs(o.data[0] - 0.3) < 1e-6 and abs(o.data[15] - 1.0) < 1e-6
                       for o in h.observations), step=step)
    obs = [o for o in h.observations if abs(o.data[15] - 1.0) < 1e-6][-1].data
    assert len(obs) == 18
    assert obs[5] == pytest.approx(0.5)  # yaw rate
    assert list(obs[6:9]) == pytest.approx([0.0, 0.0, -1.0])
    assert list(obs[9:12]) == pytest.approx([0.1, -0.1, 0.0], abs=1e-6)  # relative to default
    assert list(obs[12:15]) == pytest.approx([1.5, 2.0, 0.0])
    assert list(obs[15:18]) == pytest.approx([1.0, 0.4, 0.0], abs=1e-6)


def test_stale_command_reads_as_stop(harness):
    h = harness()
    h.cmd_vel.publish(Twist(linear=Vector3(x=0.3)))
    h.spin_for(0.7, step=h.publish_sensors)
    h.observations.clear()
    h.wait(lambda: bool(h.observations), step=h.publish_sensors)
    assert h.observations[-1].data[0] == 0.0


def test_observation_waits_for_imu_and_every_motor(harness):
    h = harness()

    def hip_only():
        h.publish_imu()
        h.publish_motors(['hip'])

    h.spin_for(0.5, step=h.publish_motors)
    assert not h.observations
    h.spin_for(0.3, step=hip_only)
    h.observations.clear()
    h.spin_for(0.5, step=hip_only)
    assert not h.observations
    assert 'motor feedback is stale' in h.diagnostics()


def test_without_imu_assumes_upright(harness):
    h = harness(require_imu=False)
    h.wait(lambda: bool(h.observations), step=h.publish_motors)
    assert list(h.observations[-1].data[3:9]) == pytest.approx([0, 0, 0, 0, 0, -1.0])


def test_targets_reach_motors_only_while_armed(harness):
    h = harness()

    def step():
        h.publish_sensors()
        h.publish_targets([0.2, 0.6, 5.0])

    h.spin_for(0.5, step=step)
    assert not any(h.commands.values())

    h.arm(step)
    assert h.commands['hip'][-1].data == pytest.approx(0.2)
    assert h.commands['knee'][-1].data == pytest.approx(0.4)  # -0.6 + 1.0

    h.publish_targets([9.0, -9.0, 0.0])  # clamped to joint limits 0.5 and 0.0
    h.wait(lambda: h.commands['hip'][-1].data == pytest.approx(0.5) and
           h.commands['knee'][-1].data == pytest.approx(1.0))

    h.buttons.publish(String(data='BACK pressed'))
    h.spin_for(0.3, step=step)
    for commands in h.commands.values():
        commands.clear()
    h.spin_for(0.3, step=step)
    assert not any(h.commands.values())


def test_refuses_to_arm_when_not_ready(harness):
    h = harness()
    h.spin_for(0.3, step=lambda: h.publish_targets([0.0, 0.4, -0.3]))
    h.buttons.publish(String(data='START pressed'))
    h.spin_for(0.3, step=lambda: h.publish_targets([0.0, 0.4, -0.3]))
    assert not any(h.commands.values())
    assert 'Not arming: motor feedback is stale' in h.diagnostics()

    h.spin_for(0.3, step=h.publish_sensors)
    h.buttons.publish(String(data='START pressed'))
    h.spin_for(0.3, step=h.publish_sensors)
    assert 'Not arming: the policy is not publishing targets' in h.diagnostics()


def test_disarms_when_policy_or_feedback_stops(harness):
    h = harness()

    def step():
        h.publish_sensors()
        h.publish_targets([0.1, 0.4, -0.3])

    h.arm(step)
    h.spin_for(0.5, step=h.publish_sensors)
    h.commands['hip'].clear()
    h.publish_targets([0.3, 0.4, -0.3])
    h.spin_for(0.3, step=h.publish_sensors)
    assert not h.commands['hip']
    assert 'Disarmed (policy targets timed out)' in h.diagnostics()

    h.arm(step)
    h.spin_for(0.4, step=lambda: h.publish_targets([0.1, 0.4, -0.3]))
    h.commands['hip'].clear()
    h.spin_for(0.3, step=lambda: h.publish_targets([0.1, 0.4, -0.3]))
    assert not h.commands['hip']
    assert 'Disarmed (motor feedback is stale)' in h.diagnostics()


def test_rejects_motor_for_unknown_joint():
    h = Harness(motor_joints=['wrist_joint', 'knee_joint'])
    try:
        assert h.process.wait(timeout=10) == 1
        assert 'wrist_joint is not a policy joint' in h.diagnostics()
    finally:
        h.close()
