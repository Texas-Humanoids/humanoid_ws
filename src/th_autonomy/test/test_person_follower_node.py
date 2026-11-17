"""The person follower over DDS with the real model (skipped without it)."""

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from test_person import MODEL
from test_tag_follower_node import Harness

try:
    import tensorrt  # noqa: F401
    HAVE_GPU = True
except ImportError:
    HAVE_GPU = False

pytestmark = [
    pytest.mark.skipif(not MODEL.is_file(), reason='run tools/fetch_person_model.py'),
    # On a CPU a frame takes seconds and starves this single-threaded test's timers.
    pytest.mark.skipif(not HAVE_GPU, reason='needs TensorRT (the GPU image)'),
]


@pytest.fixture
def h():
    from th_autonomy.person_follower import PersonFollower
    rclpy.init()
    harness = Harness('person_node', PersonFollower, [Parameter('model_path', value=str(MODEL))])
    harness.spin(0.5, lambda: harness.state() == 'DISABLED')
    yield harness
    harness.close()
    rclpy.shutdown()


def test_empty_room_stays_put(h):
    ok, jpeg = cv2.imencode('.jpg', np.full((720, 1280, 3), 128, np.uint8))
    h.frame = jpeg.tobytes()
    h.mode.publish(String(data='WALK'))
    h.spin(0.3)
    h.press('Y')
    assert h.spin(2.0, lambda: h.state() == 'LOST')
    h.spin(1.0)
    assert h.cmds and all(c.linear.x == 0.0 and c.angular.z == 0.0 for c in h.cmds)
    h.press('B')
    assert h.spin(1.0, lambda: h.state() == 'DISABLED')
