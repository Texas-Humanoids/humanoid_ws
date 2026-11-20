"""robot_monitor's sampled inputs: a subscription on a node that is never spun keeps only the
newest message (depth 1), and take_latest() returns it once. Needs ROS 2 (rclpy)."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from std_msgs.msg import Float64  # noqa: E402

from th_foxglove.robot_monitor import LATEST, take_latest  # noqa: E402


def test_take_latest_returns_the_newest_message_once():
    rclpy.init()
    try:
        talker = rclpy.create_node('test_sampling_talker')
        sampler = rclpy.create_node('test_sampling_sampler')  # never spun
        sub = sampler.create_subscription(Float64, '/test_sampling', lambda _m: None, LATEST)
        pub = talker.create_publisher(Float64, '/test_sampling', 10)
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        for value in range(5):
            pub.publish(Float64(data=float(value)))
            time.sleep(0.02)
        got = None
        while got is None and time.monotonic() < deadline:
            got = take_latest(sub)
            time.sleep(0.02)
        assert got is not None and got.data == 4.0   # depth 1: only the newest is kept
        assert take_latest(sub) is None              # taken once
        talker.destroy_node()
        sampler.destroy_node()
    finally:
        rclpy.shutdown()
