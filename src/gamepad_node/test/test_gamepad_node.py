"""Exercise the real executable and DDS transport without physical input devices."""

import os
import signal
import subprocess
import tempfile
import time
import uuid

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class RunningNode:
    """Isolated process and observer, with bounded waits and captured diagnostics."""

    def __init__(self, parameters):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        namespace = '/test_' + uuid.uuid4().hex
        self.observer = rclpy.create_node('observer', namespace=namespace, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.observer)
        self.velocities = []
        self.buttons = []
        self.observer.create_subscription(Twist, 'cmd_vel', self.velocities.append, 100)
        self.observer.create_subscription(String, 'gamepad/button_event', self.buttons.append, 10)
        self.joy = self.observer.create_publisher(Joy, 'joy', 10)
        self.log = tempfile.TemporaryFile(mode='w+')
        args = [os.environ['GAMEPAD_NODE_EXECUTABLE'], '--ros-args', '-r', '__ns:=' + namespace]
        for name, value in parameters.items():
            args.extend(['-p', f'{name}:={value}'])
        self.process = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT)

    def diagnostics(self):
        self.log.seek(0)
        return self.log.read()

    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail('Timed out waiting for node behavior:\n' + self.diagnostics())

    def ready(self):
        self.wait(lambda: self.joy.get_subscription_count() == 1)
        self.wait(lambda: self.observer.count_publishers('cmd_vel') == 1)

    def send(self, axes=None, enabled=True):
        msg = Joy()
        msg.axes = axes if axes is not None else [0.0, 1.0, 0.0, 0.0]
        msg.buttons = [0, 0, 0, 0, int(enabled)]
        self.joy.publish(msg)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.executor.shutdown()
        self.observer.destroy_node()
        rclpy.shutdown(context=self.context)
        self.log.close()


@pytest.fixture
def running_node():
    nodes = []

    def start(**parameters):
        node = RunningNode({'print_velocity': 'false', **parameters})
        nodes.append(node)
        node.ready()
        return node

    yield start
    for node in nodes:
        node.close()


def is_stop(msg):
    return all(value == 0.0 for value in (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z))


@pytest.mark.parametrize('shutdown_signal', [signal.SIGINT, signal.SIGTERM])
@pytest.mark.parametrize('busy_input', [False, True])
def test_shutdown_sends_final_stop(running_node, shutdown_signal, busy_input):
    # Disable the watchdog: a zero must come from shutdown, not an input timeout.
    node = running_node(joy_timeout='0.0')
    node.send()
    node.wait(lambda: any(msg.linear.x == 0.5 for msg in node.velocities))
    timer = node.observer.create_timer(0.005, node.send) if busy_input else None
    node.velocities.clear()
    node.process.send_signal(shutdown_signal)
    node.wait(lambda: any(is_stop(msg) for msg in node.velocities))
    node.wait(lambda: node.process.poll() is not None)
    if timer:
        timer.cancel()
    assert node.process.returncode == 0, node.diagnostics()
    # Drain commands already in DDS to catch any motion published after the stop.
    deadline = time.monotonic() + 0.1
    while time.monotonic() < deadline:
        node.executor.spin_once(timeout_sec=0.02)
    assert is_stop(node.velocities[-1])


def test_watchdog_stops_and_input_can_resume(running_node):
    node = running_node(joy_timeout='0.15')
    node.send()
    node.wait(lambda: any(msg.linear.x == 0.5 for msg in node.velocities))
    node.velocities.clear()
    node.wait(lambda: any(is_stop(msg) for msg in node.velocities))
    node.velocities.clear()
    node.send()
    node.wait(lambda: any(msg.linear.x == 0.5 for msg in node.velocities))


def test_enable_release_mapping_and_button_edges(running_node):
    node = running_node(joy_timeout='0.0')
    node.send(axes=[0.5, 1.0, 0.0, -0.25])
    node.wait(lambda: bool(node.velocities) and bool(node.buttons))
    msg = node.velocities[-1]
    assert (msg.linear.x, msg.linear.y, msg.angular.z) == (0.5, 0.25, -0.25)
    assert (msg.linear.z, msg.angular.x, msg.angular.y) == (0.0, 0.0, 0.0)
    assert [msg.data for msg in node.buttons] == ['LB pressed']
    node.send()
    node.wait(lambda: len(node.velocities) >= 2)
    assert len(node.buttons) == 1
    node.send(enabled=False)
    node.wait(lambda: is_stop(node.velocities[-1]))
    count = len(node.velocities)
    node.send(enabled=False)
    # Allow the released message to be handled; it must not generate another command.
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        node.executor.spin_once(timeout_sec=0.02)
    assert len(node.velocities) == count


def test_disabled_deadman_and_malformed_axes(running_node):
    node = running_node(enable_button='-1', joy_timeout='0.0')
    node.send(axes=[2.0, float('nan'), 0.0, float('inf')], enabled=False)
    node.wait(lambda: bool(node.velocities))
    msg = node.velocities[-1]
    assert (msg.linear.x, msg.linear.y, msg.angular.z) == (0.0, 0.5, 0.0)
    node.velocities.clear()
    node.send(axes=[], enabled=False)
    node.wait(lambda: bool(node.velocities))
    assert is_stop(node.velocities[-1])


@pytest.mark.parametrize('parameter', [
    'input_mode:=invalid', 'keyboard_scaling:=1.1', 'joy_timeout:=-0.1',
    'input_mode:=keyboard -p publish_rate:=0.0',
    'input_mode:=keyboard -p keyboard_device:=/nonexistent/keyboard',
])
def test_invalid_configuration_exits_with_error(parameter):
    result = subprocess.run(
        [os.environ['GAMEPAD_NODE_EXECUTABLE'], '--ros-args', '-p', *parameter.split()],
        capture_output=True, text=True, timeout=10)
    assert result.returncode == 1, result.stdout + result.stderr
    assert '[FATAL]' in result.stderr
