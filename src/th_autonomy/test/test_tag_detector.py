import math

import cv2
import numpy as np
import pytest

from th_autonomy.tag_detector import Intrinsics, TagDetector, render_tag

WIDTH, HEIGHT = 1280, 720
CAMERA = Intrinsics.from_hfov(WIDTH, HEIGHT, 90.0)
TAG_SIZE = 0.16


def scene(tags):
    """A white image with (tag_id, left, top, side) tags pasted in, facing the camera."""
    image = np.full((HEIGHT, WIDTH), 255, np.uint8)
    for tag_id, left, top, side in tags:
        image[top:top + side, left:left + side] = render_tag('tag36h11', tag_id, side)
    return image


def expected(left, top, side):
    """Where a fronto-parallel tag of this pixel size and place is, by the pinhole model."""
    z = CAMERA.fx * TAG_SIZE / side
    u = left + (side - 1) / 2.0
    x = (u - CAMERA.cx) * z / CAMERA.fx
    return math.hypot(x, z), math.atan2(-x, z)


def test_centered_tag_distance():
    side = 120
    left, top = (WIDTH - side) // 2, (HEIGHT - side) // 2
    [d] = TagDetector(tag_size=TAG_SIZE).detect(scene([(7, left, top, side)]), CAMERA)
    distance, bearing = expected(left, top, side)
    assert d.tag_id == 7
    assert d.distance == pytest.approx(distance, rel=0.02)
    assert abs(d.bearing) < math.radians(1.0)


@pytest.mark.parametrize('left,sign', [(150, 1), (1000, -1)])
def test_bearing_is_left_positive(left, sign):
    side = 100
    [d] = TagDetector(tag_size=TAG_SIZE).detect(scene([(0, left, 300, side)]), CAMERA)
    distance, bearing = expected(left, 300, side)
    assert math.copysign(1, d.bearing) == sign
    assert d.bearing == pytest.approx(bearing, abs=math.radians(1.0))
    assert d.distance == pytest.approx(distance, rel=0.03)


def test_nearest_first():
    image = scene([(3, 100, 300, 60), (5, 800, 250, 200)])
    detections = TagDetector(tag_size=TAG_SIZE).detect(image, CAMERA)
    assert [d.tag_id for d in detections] == [5, 3]


def test_no_tag():
    assert TagDetector().detect(scene([]), CAMERA) == []


def test_reduced_decode_matches_full_resolution():
    image = scene([(1, 500, 250, 160)])
    ok, jpeg = cv2.imencode('.jpg', image)
    assert ok
    half = cv2.imdecode(jpeg, cv2.IMREAD_REDUCED_GRAYSCALE_2)
    detector = TagDetector(tag_size=TAG_SIZE)
    [full] = detector.detect(image, CAMERA)
    [reduced] = detector.detect(half, CAMERA.scaled(0.5))
    assert reduced.distance == pytest.approx(full.distance, rel=0.03)
    assert reduced.bearing == pytest.approx(full.bearing, abs=math.radians(1.0))


def test_tilted_tag():
    """A tag turned 40 degrees about vertical still gives its distance."""
    detector = TagDetector(tag_size=TAG_SIZE)
    z, yaw = 1.5, math.radians(40)
    rotation, _ = cv2.Rodrigues(np.array([0.0, yaw, 0.0]))
    half = TAG_SIZE / 2  # image-order corners (TL, TR, BR, BL) in the camera frame, y down
    square = np.array([[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]])
    corners = (rotation @ square.T).T + np.array([0.0, 0.0, z])
    projected, _ = cv2.projectPoints(corners, np.zeros(3), np.zeros(3), CAMERA.matrix(), None)
    side = 400
    margin = side // 8  # the rendered tag's white border keeps the quad detectable
    source = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
    tag = cv2.copyMakeBorder(render_tag('tag36h11', 2, side), margin, margin, margin, margin,
                             cv2.BORDER_CONSTANT, value=255)
    warp = cv2.getPerspectiveTransform(source + margin, projected.reshape(4, 2).astype(np.float32))
    image = cv2.warpPerspective(tag, warp, (WIDTH, HEIGHT), borderValue=255)
    [d] = detector.detect(image, CAMERA)
    assert d.distance == pytest.approx(z, rel=0.03)


def test_rejects_unknown_family():
    with pytest.raises(ValueError):
        TagDetector('tag99h1')
