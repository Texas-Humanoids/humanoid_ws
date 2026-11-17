"""Dashboard overlay: shapes land on the camera's own pixels, and the 3D target where it was seen."""

import math

import cv2
import numpy as np
import pytest
from std_msgs.msg import Header

from th_autonomy import overlay


@pytest.mark.parametrize('rotation', [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180,
                                      cv2.ROTATE_90_COUNTERCLOCKWISE])
@pytest.mark.parametrize('scale', [1, 2])
def test_to_raw_undoes_rotation_and_scale(rotation, scale):
    # Mark one pixel in a camera image, decode it at 1/scale, turn it upright, find it again.
    raw = np.zeros((48, 64), np.uint8)
    raw[10:12, 40:42] = 255  # a 2x2 dot at x 40-41, y 10-11
    small = cv2.resize(raw, (64 // scale, 48 // scale), interpolation=cv2.INTER_AREA)
    upright = small if rotation is None else cv2.rotate(small, rotation)
    ys, xs = np.nonzero(upright == upright.max())
    point = (xs.mean(), ys.mean())
    [(x, y)] = overlay.to_raw([point], rotation, small.shape[1], small.shape[0], scale)
    assert x == pytest.approx(40.5, abs=0.6)
    assert y == pytest.approx(10.5, abs=0.6)


def test_annotations_have_outlines_labels_and_status():
    if overlay.ImageAnnotations is None:
        pytest.skip('foxglove_msgs not installed')
    shapes = [overlay.Shape(overlay.box(10, 20, 50, 120), overlay.FOLLOWED, 'FOLLOW 1.2 m +5°', True),
              overlay.Shape(overlay.box(200, 20, 240, 120))]
    msg = overlay.annotations(Header().stamp, shapes, 'TRACKING', cv2.ROTATE_180, 320, 240, 2)
    assert len(msg.points) == 2
    first = [(p.x, p.y) for p in msg.points[0].points]
    assert first[0] == pytest.approx(((319 - 10 + 0.5) * 2 - 0.5, (239 - 20 + 0.5) * 2 - 0.5))
    assert msg.points[0].thickness > msg.points[1].thickness
    assert [t.text for t in msg.texts] == ['FOLLOW 1.2 m +5°', 'TRACKING']


def test_draw_on_image():
    image = np.zeros((240, 320, 3), np.uint8)
    overlay.draw(image, [overlay.Shape(overlay.box(10, 20, 50, 120), overlay.FOLLOWED, 'x 3°')],
                 'TRACKING')
    b, g, r = image[80, 10].tolist()  # on the left edge (anti-aliased)
    assert g > 100 and b < 50 and r < 50


def test_markers_place_target_ahead_and_left():
    header = Header()
    # 2 m ahead, 0.5 m left (x right is negative), 0.3 m below the camera (y down).
    out = overlay.markers(header, 'follow_camera', 'base_footprint', (-0.5, 0.3, 2.0),
                          '2.1 m +14°', True, 1.5, overlay.TargetMarker())
    by_id = {m.id: m for m in out.markers}
    target = by_id[0]
    assert target.header.frame_id == 'follow_camera'
    p = target.pose.position
    assert (p.x, p.y, p.z) == pytest.approx((2.0, 0.5, -0.3))
    assert target.color.g > target.color.r  # green while following
    assert by_id[2].text == '2.1 m +14°'
    arc = by_id[3]
    assert arc.header.frame_id == 'base_footprint'
    assert all(math.hypot(q.x, q.y) == pytest.approx(1.5) for q in arc.points)


def test_markers_without_target_keep_only_the_stop_arc():
    out = overlay.markers(Header(), 'follow_camera', 'base_footprint', None, '', False, 1.0,
                          overlay.TargetMarker())
    assert sorted(m.id for m in out.markers) == [3, 4]
