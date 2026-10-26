"""Run both real executables over DDS with fake motors, IMU and policy."""

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

JOINTS = ['hip_joint', 'knee_joint']
DEFAULTS = [0.0, 0.4]
SCALE = 0.25


class Harness:
    """Starts observation_builder and joint_motor_bridge in an isolated namespace."""

    def __init__(self, builder_params=None):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.ns = '/test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('fake_robot', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.observations = []
        self.joint_states = []
        self.motor_commands = {'hip': [], 'knee': []}
        self.motor_positions = {'hip': 0.0, 'knee': 0.0}

        topic = self.topic
        self.node.create_subscription(
            Float32MultiArray, topic('observation'), self.observations.append, 10)
        self.node.create_subscription(
            JointState, topic('joint_states'), self.joint_states.append, 10)
        self.feedback = {}
        for motor in self.motor_commands:
            self.node.create_subscription(
                Float64, topic(motor + '/motor/target_position'),
                self.motor_commands[motor].append, 10)
            self.feedback[motor] = self.node.create_publisher(
                Float64, topic(motor + '/motor/actual_position'), 10)
        self.cmd_vel = self.node.create_publisher(Twist, topic('cmd_vel'), 10)
        self.imu = self.node.create_publisher(Imu, topic('imu'), qos_profile_sensor_data)
        self.targets = self.node.create_publisher(JointState, topic('targets'), 10)
        self.buttons = self.node.create_publisher(String, topic('buttons'), 10)

        common = {
            'joint_state_topic': topic('joint_states'),
            'joint_target_topic': topic('targets'),
        }
        builder = {
            **common,
            'joint_names': JOINTS,
            'default_joint_positions': DEFAULTS,
            'action_scale': SCALE,
            'rate': 25.0,
            'cmd_vel_topic': topic('cmd_vel'),
            'imu_topic': topic('imu'),
            'observation_topic': topic('observation'),
            **(builder_params or {}),
        }
        bridge = {
            **common,
            'joint_names': JOINTS,
            'motor_namespaces': [topic('hip'), topic('knee')],
            # The knee motor is mounted reversed with a 1 rad offset.
            'directions': [1.0, -1.0],
            'offsets': [0.0, 1.0],
            'lower_limits': [-0.5, 0.0],
            'upper_limits': [0.5, 1.5],
            'button_topic': topic('buttons'),
        }
        self.log = tempfile.TemporaryFile(mode='w+')
        self.processes = [
            self.start('OBSERVATION_BUILDER_EXECUTABLE', builder),
            self.start('JOINT_MOTOR_BRIDGE_EXECUTABLE', bridge),
        ]

    def topic(self, name):
        return self.ns + '/' + name

    def start(self, variable, parameters):
        args = [os.environ[variable], '--ros-args']
        for name, value in parameters.items():
            if isinstance(value, list):
                value = '[' + ', '.join(
                    f"'{v}'" if isinstance(v, str) else repr(float(v)) for v in value) + ']'
            elif isinstance(value, bool):
                value = str(value).lower()
            args.extend(['-p', f'{name}:={value}'])
        return subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)

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
        pytest.fail('Timed out waiting for pipeline behavior:\n' + self.diagnostics())

    def publish_sensors(self, imu=True):
        for motor, position in self.motor_positions.items():
            self.feedback[motor].publish(Float64(data=position))
        if imu:
            msg = Imu()
            msg.orientation.w = 1.0
            msg.angular_velocity.z = 0.5
            self.imu.publish(msg)

    def publish_targets(self, positions):
        self.targets.publish(JointState(name=JOINTS, position=positions))

    def ready(self):
        self.wait(lambda: self.targets.get_subscription_count() == 2 and
                  self.buttons.get_subscription_count() == 1 and
                  self.node.count_publishers(self.topic('observation')) == 1)

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)
        self.log.close()


@pytest.fixture
def harness():
    started = []

    def start(**kwargs):
        started.append(Harness(**kwargs))
        started[-1].ready()
        return started[-1]

    yield start
    for h in started:
        h.close()


def test_observation_combines_command_imu_feedback_and_previous_actions(harness):
    h = harness()
    h.motor_positions = {'hip': 0.1, 'knee': 0.7}  # knee joint = -(0.7 - 1.0) = 0.3

    def step():
        h.publish_sensors()
        h.cmd_vel.publish(Twist())

    h.wait(lambda: bool(h.observations), step=step)
    h.cmd_vel.publish(Twist(linear=Vector3(x=0.3)))
    h.publish_targets([0.25, 0.5])  # actions (0.25 - 0) / 0.25 = 1, (0.5 - 0.4) / 0.25 = 0.4
    h.observations.clear()
    h.wait(lambda: any(abs(o.data[0] - 0.3) < 1e-6 and abs(o.data[14] - 0.4) < 1e-5
                       for o in h.observations), step=h.publish_sensors)
    obs = next(o for o in h.observations if abs(o.data[0] - 0.3) < 1e-6)
    assert len(obs.data) == 15
    assert obs.data[5] == pytest.approx(0.5)  # yaw rate
    assert list(obs.data[6:9]) == pytest.approx([0.0, 0.0, -1.0])
    assert list(obs.data[9:11]) == pytest.approx([0.1, -0.1], abs=1e-5)  # 0.3 - 0.4 default
    assert list(obs.data[13:15]) == pytest.approx([1.0, 0.4], abs=1e-5)


def test_observation_waits_for_imu_and_all_joints(harness):
    h = harness()
    h.spin_for(0.5, step=lambda: h.publish_sensors(imu=False))
    assert not h.observations

    def hip_only():
        h.feedback['hip'].publish(Float64(data=0.0))
        h.imu.publish(Imu(orientation=Quaternion(w=1.0)))

    # Let the knee feedback from the first phase go stale before checking.
    h.spin_for(0.5, step=hip_only)
    h.observations.clear()
    h.spin_for(0.5, step=hip_only)
    assert not h.observations


def test_bench_mode_runs_without_imu_or_motors(harness):
    h = harness(builder_params={'require_imu': False, 'require_all_joints': False})
    h.wait(lambda: bool(h.observations))
    obs = h.observations[-1].data
    assert list(obs[6:9]) == pytest.approx([0.0, 0.0, -1.0])
    assert list(obs[9:11]) == pytest.approx([0.0, 0.0])


def test_targets_reach_motors_only_while_armed(harness):
    h = harness()

    def step():
        h.publish_sensors()
        h.publish_targets([0.2, 0.6])

    h.wait(lambda: len(h.joint_states) > 2, step=step)
    h.spin_for(0.3, step=step)
    assert not h.motor_commands['hip'] and not h.motor_commands['knee']

    h.buttons.publish(String(data='START pressed'))
    h.wait(lambda: h.motor_commands['hip'] and h.motor_commands['knee'], step=step)
    assert h.motor_commands['hip'][-1].data == pytest.approx(0.2)
    assert h.motor_commands['knee'][-1].data == pytest.approx(0.4)  # -0.6 + 1.0

    h.publish_targets([9.0, -9.0])  # clamped to joint limits 0.5 and 0.0
    h.wait(lambda: h.motor_commands['hip'][-1].data == pytest.approx(0.5) and
           h.motor_commands['knee'][-1].data == pytest.approx(1.0))

    h.buttons.publish(String(data='BACK pressed'))
    h.spin_for(0.3, step=step)
    for commands in h.motor_commands.values():
        commands.clear()
    h.spin_for(0.3, step=step)
    assert not h.motor_commands['hip'] and not h.motor_commands['knee']


def test_bridge_refuses_to_arm_without_feedback_and_disarms_on_timeouts(harness):
    h = harness()
    h.spin_for(0.3, step=lambda: h.publish_targets([0.0, 0.4]))
    h.buttons.publish(String(data='START pressed'))
    h.spin_for(0.3, step=lambda: h.publish_targets([0.0, 0.4]))
    assert not h.motor_commands['hip']

    def step():
        h.publish_sensors()
        h.publish_targets([0.1, 0.4])

    h.spin_for(0.3, step=step)
    h.buttons.publish(String(data='START pressed'))
    h.wait(lambda: bool(h.motor_commands['hip']), step=step)

    # Policy stops: the bridge disarms, so a late target is not forwarded.
    h.spin_for(0.5, step=h.publish_sensors)
    h.motor_commands['hip'].clear()
    h.publish_targets([0.3, 0.4])
    h.spin_for(0.3, step=h.publish_sensors)
    assert not h.motor_commands['hip']
    assert 'policy targets timed out' in h.diagnostics()

    # Motor feedback stops: the bridge disarms too.
    h.spin_for(0.3, step=step)
    h.buttons.publish(String(data='START pressed'))
    h.wait(lambda: bool(h.motor_commands['hip']), step=step)
    h.spin_for(0.6, step=lambda: h.publish_targets([0.1, 0.4]))
    h.motor_commands['hip'].clear()
    h.spin_for(0.3, step=lambda: h.publish_targets([0.1, 0.4]))
    assert not h.motor_commands['hip']
    assert 'motor feedback timed out' in h.diagnostics()
