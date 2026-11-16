"""The node over DDS, in process: arming rules, buttons, override and following a rendered tag."""

import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String

from th_autonomy.tag_detector import render_tag
from th_autonomy.tag_follower import TagFollower

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def tag_image(left, side):
    image = np.full((720, 1280), 255, np.uint8)
    image[300:300 + side, left:left + side] = render_tag('tag36h11', 0, side)
    ok, jpeg = cv2.imencode('.jpg', image)
    assert ok
    return jpeg.tobytes()


class Harness:

    def __init__(self, ns, node_class=TagFollower, extra=()):
        def topic(name):
            return f'/{ns}/{name}'
        self.follower = node_class(parameter_overrides=list(extra) + [
            Parameter('image_topic', value=topic('image')),
            Parameter('cmd_vel_topic', value=topic('cmd_vel')),
            Parameter('mode_topic', value=topic('mode')),
            Parameter('button_topic', value=topic('buttons')),
            Parameter('joy_topic', value=topic('joy')),
            Parameter('status_namespace', value=topic('follower')),
        ])
        self.node = Node(f'harness_{ns}')
        self.mode = self.node.create_publisher(String, topic('mode'), LATCHED)
        self.buttons = self.node.create_publisher(String, topic('buttons'), 10)
        self.joy = self.node.create_publisher(Joy, topic('joy'), 10)
        self.image = self.node.create_publisher(CompressedImage, topic('image'), qos_profile_sensor_data)
        self.cmds = []
        self.states = []
        self.node.create_subscription(Twist, topic('cmd_vel'), self.cmds.append, 100)
        self.node.create_subscription(
            String, topic('follower/state'), lambda m: self.states.append(m.data), LATCHED)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.follower)
        self.executor.add_node(self.node)
        self.frame = None

    def spin(self, seconds, until=None):
        end = time.monotonic() + seconds
        next_frame = 0.0
        while time.monotonic() < end:
            if self.frame is not None and time.monotonic() >= next_frame:
                self.image.publish(CompressedImage(format='jpeg', data=self.frame))
                next_frame = time.monotonic() + 0.1
            self.executor.spin_once(timeout_sec=0.01)
            if until is not None and until():
                return True
        return False

    def press(self, name):
        self.buttons.publish(String(data=f'{name} pressed'))

    def hold_lb(self, held):
        buttons = [0] * 15
        buttons[9] = int(held)
        self.joy.publish(Joy(buttons=buttons))

    def state(self):
        return self.states[-1] if self.states else None

    def close(self):
        self.executor.shutdown()
        self.follower.destroy_node()
        self.node.destroy_node()


@pytest.fixture
def h(request):
    rclpy.init()
    harness = Harness(request.node.name.replace('[', '_').replace(']', '').replace('-', '_'))
    harness.spin(0.5, lambda: harness.state() == 'DISABLED')
    yield harness
    harness.close()
    rclpy.shutdown()


def test_does_not_enable_unless_walking(h):
    h.mode.publish(String(data='STAND'))
    h.spin(0.3)
    h.press('Y')
    h.spin(0.5)
    assert h.cmds == []
    assert h.state() == 'DISABLED'


def test_follows_tag_and_stops_on_b(h):
    h.mode.publish(String(data='WALK'))
    h.spin(0.3)
    h.press('Y')
    assert h.spin(1.0, lambda: h.state() == 'LOST')  # no tag yet: commands zero
    assert h.cmds and all(c.linear.x == 0.0 and c.angular.z == 0.0 for c in h.cmds)
    h.frame = tag_image(left=500, side=40)  # far tag (2.6 m), a little left
    assert h.spin(3.0, lambda: h.cmds[-1].linear.x > 0.1)
    assert h.state() == 'TRACKING'
    assert h.cmds[-1].angular.z > 0.0  # turning left toward it
    assert max(c.linear.x for c in h.cmds) <= 0.4 + 1e-9
    h.press('B')
    assert h.spin(1.0, lambda: h.state() == 'DISABLED')
    count = len(h.cmds)
    h.spin(0.3)
    assert h.cmds[count - 1].linear.x == 0.0 and h.cmds[count - 1].angular.z == 0.0
    assert len(h.cmds) == count  # quiet after the stop


def test_lb_override_pauses_and_resumes_from_standstill(h):
    h.mode.publish(String(data='WALK'))
    h.spin(0.3)
    h.press('Y')
    h.frame = tag_image(left=600, side=80)
    assert h.spin(3.0, lambda: h.cmds and h.cmds[-1].linear.x > 0.1)
    h.hold_lb(True)
    assert h.spin(0.5, lambda: h.state() == 'OVERRIDE')
    count = len(h.cmds)
    for _ in range(5):
        h.hold_lb(True)
        h.spin(0.05)
    assert len(h.cmds) == count  # silent while the gamepad drives, no stop sent
    h.hold_lb(False)
    assert h.spin(1.0, lambda: len(h.cmds) > count)
    assert h.cmds[count].linear.x <= 0.05  # ramps again from standstill


def test_leaving_walk_disables(h):
    h.mode.publish(String(data='WALK'))
    h.spin(0.3)
    h.press('Y')
    assert h.spin(1.0, lambda: h.state() == 'LOST')
    h.mode.publish(String(data='DISARMED'))
    assert h.spin(1.0, lambda: h.state() == 'DISABLED')
    h.mode.publish(String(data='WALK'))
    h.spin(0.5)
    assert h.state() == 'DISABLED'  # needs Y again
