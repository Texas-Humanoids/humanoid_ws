"""Reads /joy (from joy_node) and publishes:
  /cmd_vel                geometry_msgs/Twist  (left stick, while LB is held)
  /gamepad/button_event   std_msgs/String      ("A pressed", ...)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START',
                'GUIDE', 'L_STICK', 'R_STICK']


class GamepadNode(Node):
    def __init__(self):
        super().__init__('gamepad_node')
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('enable_button', 4)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.btn_pub = self.create_publisher(String, 'gamepad/button_event', 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.prev_buttons = []
        self.was_enabled = False

    def joy_cb(self, msg):
        p = self.get_parameter
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)
        for i, state in enumerate(msg.buttons):
            if state and not self.prev_buttons[i]:
                name = BUTTON_NAMES[i] if i < len(BUTTON_NAMES) else f'BUTTON_{i}'
                self.btn_pub.publish(String(data=f'{name} pressed'))
        self.prev_buttons = list(msg.buttons)

        enable = p('enable_button').value
        enabled = enable < len(msg.buttons) and msg.buttons[enable] == 1
        twist = Twist()
        if enabled:
            twist.linear.x = msg.axes[p('axis_linear').value] * p('scale_linear').value
            twist.angular.z = msg.axes[p('axis_angular').value] * p('scale_angular').value
            self.cmd_pub.publish(twist)
        elif self.was_enabled:
            self.cmd_pub.publish(twist)  # one stop message on release
        self.was_enabled = enabled


def main():
    rclpy.init()
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
