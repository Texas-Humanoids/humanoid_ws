"""What every follower shares: camera in, /cmd_vel out, and the rules for when it may drive.

  /camera/image/compressed ──► <follower> ──/cmd_vel──► policy_bridge
  /policy_bridge/mode ───────►      ▲
  /gamepad/button_event ────────────┤
  /joy (override button) ───────────┘

A subclass only finds the target in each frame (`find_target`) and draws it
(`draw`). This class publishes /cmd_vel only while all of these hold:
  - following is enabled: `toggle_event` (Y) toggles it, but only turns it on
    while the bridge is in WALK (`require_walk`); any `stop_events` (B, BACK)
    and the bridge leaving WALK turn it off, so STAND is never sent commands;
  - the operator is not holding the gamepad's enable button (LB). While it is
    held the gamepad drives, and following resumes from standstill on release.
When it stops publishing it sends one zero command first (like the gamepad),
and the bridge's cmd_vel timeout (0.5 s) stops the robot if this node dies.

Status goes to `status_namespace` (/follower), the same for every follower, so
the Foxglove dashboard's Follow tab works with whichever one runs:
  /follower/state             DISABLED, OVERRIDE, TRACKING, LOST or SEARCH (latched)
  /follower/target            the followed target's center in the camera frame
  /follower/image/compressed  the camera with detections drawn (only while watched)
Detection runs on every frame even while disabled, so you can check in Foxglove
that the target is seen before enabling it.

Images are processed in their own callback group: with a MultiThreadedExecutor
(see `run`) a slow detector delays only the next frame, never the 20 Hz command.
"""

from dataclasses import dataclass
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String

from th_autonomy.follow_controller import FollowController, FollowParams
from th_autonomy.tag_detector import Intrinsics

DISABLED = 'DISABLED'
OVERRIDE = 'OVERRIDE'  # enabled, operator holding the enable button

GRAY_FLAGS = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
              4: cv2.IMREAD_REDUCED_GRAYSCALE_4, 8: cv2.IMREAD_REDUCED_GRAYSCALE_8}
ROTATIONS = {0: None, 90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}
COLOR_FLAGS = {1: cv2.IMREAD_COLOR, 2: cv2.IMREAD_REDUCED_COLOR_2,
               4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}


@dataclass
class Target:
    distance: float    # m, along the floor
    bearing: float     # rad, left positive
    position: tuple    # (x, y, z) in the camera frame (x right, y down, z forward), for /follower/target


class FollowerNode(Node):

    color = False  # subclasses that need BGR frames set True

    def __init__(self, name, follow_defaults=None, **kwargs):
        super().__init__(name, **kwargs)
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
        self.decode_scale = int(p('decode_scale', 1).value)
        flags = COLOR_FLAGS if self.color else GRAY_FLAGS
        if self.decode_scale not in flags:
            raise ValueError(f'decode_scale must be one of {sorted(flags)}')
        self.decode_flag = flags[self.decode_scale]
        self.publish_debug_image = bool(p('publish_debug_image', True).value)
        status = p('status_namespace', '/follower').value.rstrip('/')

        # Calibrated intrinsics at full resolution (fx > 0), else a guess from hfov_deg.
        self.camera_fx = float(p('camera.fx', 0.0).value)
        self.camera_fy = float(p('camera.fy', 0.0).value)
        self.camera_cx = float(p('camera.cx', 0.0).value)
        self.camera_cy = float(p('camera.cy', 0.0).value)
        self.camera_distortion = tuple(float(v) for v in p('camera.distortion', [0.0]).value)
        self.hfov_deg = float(p('camera.hfov_deg', 90.0).value)
        # Degrees clockwise to turn each frame upright (a camera mounted upside down: 180).
        rotation = int(p('camera.rotation', 0).value)
        if rotation not in ROTATIONS:
            raise ValueError(f'camera.rotation must be one of {sorted(ROTATIONS)}')
        self.rotation = ROTATIONS[rotation]

        defaults = follow_defaults or FollowParams()
        self.controller = FollowController(FollowParams(**{
            name: float(p(f'follow.{name}', getattr(defaults, name)).value)
            for name in defaults.__dataclass_fields__}))
        self.lock = threading.Lock()  # the controller is shared by the image and timer threads

        self.mode = None
        self.override = False
        self.last_joy = None
        self.publishing = False
        self.state = None
        self.intrinsics = None
        self.intrinsics_size = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, f'{status}/state', latched)
        self.target_pub = self.create_publisher(PointStamped, f'{status}/target', 10)
        self.debug_pub = self.create_publisher(CompressedImage, f'{status}/image/compressed', 1)
        image_qos = QoSProfile(depth=1, reliability=qos_profile_sensor_data.reliability)
        self.create_subscription(CompressedImage, image_topic, self.on_image, image_qos,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        self.create_subscription(String, mode_topic, self.on_mode, latched)
        self.create_subscription(String, button_topic, self.on_button, 10)
        self.create_subscription(Joy, joy_topic, self.on_joy, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.set_state(DISABLED)
        self.get_logger().info(
            f"Ready: '{self.toggle_event}' toggles following; "
            f'reading {image_topic}, commanding {cmd_vel_topic}')

    # Subclass hooks

    def find_target(self, image, intrinsics):
        """The target in this frame as a Target, or None. `image` is gray or BGR (`color`)."""
        raise NotImplementedError

    def draw(self, image):
        """Draw the last frame's detections on a BGR copy of it, for /follower/image/compressed."""

    def on_enable(self):
        """Following was just turned on."""

    # Inputs

    def now(self):
        return time.monotonic()

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
            if self.enabled:
                self.on_enable()
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
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), self.decode_flag)
        if image is None:
            self.get_logger().warn(f"Can't decode '{msg.format}' image", throttle_duration_sec=10.0)
            return
        if self.rotation is not None:
            image = cv2.rotate(image, self.rotation)
        target = self.find_target(image, self.camera(image.shape[1], image.shape[0]))
        if target is not None:
            with self.lock:
                self.controller.observe(target.distance, target.bearing, self.now())
            point = PointStamped()
            point.header = msg.header
            point.point.x, point.point.y, point.point.z = (float(v) for v in target.position)
            self.target_pub.publish(point)
        if self.publish_debug_image and self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(msg, image)

    def publish_debug(self, msg, image):
        image = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        self.draw(image)
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
        with self.lock:
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


def run(node_class, args=None):
    rclpy.init(args=args)
    node = node_class()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
