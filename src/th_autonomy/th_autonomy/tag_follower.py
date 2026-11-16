"""Walks the robot toward an AprilTag and stops `target_distance` in front of it.

  /camera/image/compressed ──► tag_follower ──/cmd_vel──► policy_bridge
  /policy_bridge/mode ───────►      ▲
  /gamepad/button_event ────────────┤
  /joy (override button) ───────────┘

It publishes /cmd_vel only while all of these hold:
  - following is enabled: `toggle_event` (Y) toggles it, but only turns it on
    while the bridge is in WALK (`require_walk`); any `stop_events` (B, BACK)
    and the bridge leaving WALK turn it off, so STAND is never sent commands;
  - the operator is not holding the gamepad's enable button (LB). While it is
    held the gamepad drives, and following resumes from standstill on release.
When it stops publishing it sends one zero command first (like the gamepad),
and the bridge's cmd_vel timeout (0.5 s) stops the robot if this node dies.

Detection runs on every frame even while disabled, so ~/target, ~/state and
~/image/compressed show in Foxglove whether the tag is seen before you enable it.
"""

import math
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String

from th_autonomy.follow_controller import FollowController, FollowParams
from th_autonomy.tag_detector import Intrinsics, TagDetector

DISABLED = 'DISABLED'
OVERRIDE = 'OVERRIDE'  # enabled, operator holding the enable button

DECODE_FLAGS = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
                4: cv2.IMREAD_REDUCED_GRAYSCALE_4, 8: cv2.IMREAD_REDUCED_GRAYSCALE_8}


class TagFollower(Node):

    def __init__(self, **kwargs):
        super().__init__('tag_follower', **kwargs)
        p = self.declare_parameter
        image_topic = p('image_topic', '/camera/image/compressed').value
        cmd_vel_topic = p('cmd_vel_topic', '/cmd_vel').value
        mode_topic = p('mode_topic', '/policy_bridge/mode').value
        button_topic = p('button_topic', '/gamepad/button_event').value
        joy_topic = p('joy_topic', '/joy').value
        self.toggle_event = p('toggle_event', 'Y pressed').value
        self.stop_events = set(p('stop_events', ['B pressed', 'BACK pressed']).value)
        self.override_button = int(p('override_button', 9).value)
        self.joy_timeout = float(p('joy_timeout', 0.5).value)
        self.require_walk = bool(p('require_walk', True).value)
        self.enabled = bool(p('start_enabled', False).value)
        rate = float(p('rate', 20.0).value)
        self.target_id = int(p('target_id', -1).value)
        self.decode_scale = int(p('decode_scale', 1).value)
        if self.decode_scale not in DECODE_FLAGS:
            raise ValueError(f'decode_scale must be one of {sorted(DECODE_FLAGS)}')
        self.publish_debug_image = bool(p('publish_debug_image', True).value)

        self.detector = TagDetector(
            p('tag_family', 'tag36h11').value, float(p('tag_size', 0.16).value),
            bool(p('refine_corners', True).value))
        # Calibrated intrinsics at full resolution (fx > 0), else a guess from hfov_deg.
        self.camera_fx = float(p('camera.fx', 0.0).value)
        self.camera_fy = float(p('camera.fy', 0.0).value)
        self.camera_cx = float(p('camera.cx', 0.0).value)
        self.camera_cy = float(p('camera.cy', 0.0).value)
        self.camera_distortion = tuple(float(v) for v in p('camera.distortion', [0.0]).value)
        self.hfov_deg = float(p('camera.hfov_deg', 90.0).value)

        defaults = FollowParams()
        self.controller = FollowController(FollowParams(**{
            name: float(p(f'follow.{name}', getattr(defaults, name)).value)
            for name in defaults.__dataclass_fields__}))

        self.mode = None
        self.override = False
        self.last_joy = None
        self.publishing = False
        self.state = None
        self.intrinsics = None
        self.intrinsics_size = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '~/state', latched)
        self.target_pub = self.create_publisher(PointStamped, '~/target', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/image/compressed', 1)
        image_qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.create_subscription(CompressedImage, image_topic, self.on_image, image_qos)
        self.create_subscription(String, mode_topic, self.on_mode, latched)
        self.create_subscription(String, button_topic, self.on_button, 10)
        self.create_subscription(Joy, joy_topic, self.on_joy, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.set_state(DISABLED)
        self.get_logger().info(
            f"Tag follower ready: '{self.toggle_event}' toggles following; "
            f'reading {image_topic}, commanding {cmd_vel_topic}')

    def now(self):
        return time.monotonic()

    # Inputs

    def walking(self):
        return self.mode == 'WALK' or not self.require_walk

    def on_mode(self, msg):
        self.mode = msg.data
        if self.enabled and not self.walking():
            self.get_logger().info(f'Bridge in {self.mode}: following off')
            self.enabled = False

    def on_button(self, msg):
        if msg.data == self.toggle_event:
            if not self.enabled and not self.walking():
                self.get_logger().warn(f'Bridge in {self.mode}, not WALK: not following')
                return
            self.enabled = not self.enabled
            self.get_logger().info(f"Following {'on' if self.enabled else 'off'}")
        elif msg.data in self.stop_events and self.enabled:
            self.enabled = False
            self.get_logger().info(f'{msg.data}: following off')

    def on_joy(self, msg):
        held = 0 <= self.override_button < len(msg.buttons) and msg.buttons[self.override_button] == 1
        self.last_joy = self.now()
        self.override = held

    def camera(self, width, height):
        """Intrinsics for an image of this size, as decoded."""
        if self.intrinsics_size != (width, height):
            if self.camera_fx > 0.0:
                full = Intrinsics(self.camera_fx, self.camera_fy or self.camera_fx,
                                  self.camera_cx, self.camera_cy, self.camera_distortion)
                self.intrinsics = full.scaled(1.0 / self.decode_scale)
            else:
                self.intrinsics = Intrinsics.from_hfov(width, height, self.hfov_deg)
            self.intrinsics_size = (width, height)
        return self.intrinsics

    def on_image(self, msg):
        gray = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), DECODE_FLAGS[self.decode_scale])
        if gray is None:
            self.get_logger().warn(f"Can't decode '{msg.format}' image", throttle_duration_sec=10.0)
            return
        detections = self.detector.detect(gray, self.camera(gray.shape[1], gray.shape[0]))
        if self.target_id >= 0:
            detections = [d for d in detections if d.tag_id == self.target_id]
        target = detections[0] if detections else None
        if target is not None:
            self.controller.observe(target.distance, target.bearing, self.now())
            point = PointStamped()
            point.header = msg.header
            point.point.x, point.point.y, point.point.z = (float(v) for v in target.position)
            self.target_pub.publish(point)
        if self.publish_debug_image and self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(msg, gray, detections, target)

    def publish_debug(self, msg, gray, detections, target):
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for d in detections:
            color = (0, 220, 0) if d is target else (0, 160, 255)
            cv2.polylines(image, [d.corners.astype(np.int32)], True, color, 2)
            x, y = d.corners[0].astype(int)
            cv2.putText(image, f'{d.tag_id}: {d.distance:.2f} m {math.degrees(d.bearing):+.0f} deg',
                        (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        cv2.putText(image, self.state or '', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = jpeg.tobytes()
            self.debug_pub.publish(out)

    # Output

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.state_pub.publish(String(data=state))

    def on_timer(self):
        now = self.now()
        override = self.override and self.last_joy is not None and now - self.last_joy <= self.joy_timeout
        if not self.enabled or not self.walking():
            self.enabled = False
            state = DISABLED
        elif override:
            state = OVERRIDE
        else:
            state = None
        if state is not None:
            if self.publishing and state != OVERRIDE:
                self.cmd_pub.publish(Twist())  # one stop, then quiet
            self.publishing = False
            self.set_state(state)
            return
        if not self.publishing:
            self.controller.reset_output()
            self.publishing = True
        forward, yaw, state = self.controller.update(now)
        cmd = Twist()
        cmd.linear.x = forward
        cmd.angular.z = yaw
        self.cmd_pub.publish(cmd)
        self.set_state(state)

    def destroy_node(self):
        if self.publishing:
            self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
