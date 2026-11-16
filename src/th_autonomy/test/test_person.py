"""Person detection decoding, distance, and the lock that keeps following one person."""

import math
from pathlib import Path

import numpy as np
import pytest

from th_autonomy.person_detector import INPUT_SIZE, PersonBox, decode, letterbox
from th_autonomy.person_tracker import PersonLock, TrackParams, locate
from th_autonomy.tag_detector import Intrinsics

MODEL = Path(__file__).resolve().parents[3] / 'models' / 'yolox_nano.onnx'
CAMERA = Intrinsics.from_hfov(1280, 720, 90.0)


def raw_output():
    out = np.zeros((3549, 85), np.float32)
    out[:, 2:4] = -10.0  # tiny boxes, zero scores
    return out


def cell(stride, gx, gy):
    """Row index of a grid cell in the model output."""
    offset = 0
    for s in (8, 16, 32):
        size = INPUT_SIZE // s
        if s == stride:
            return offset + gy * size + gx
        offset += size * size
    raise ValueError(stride)


def put(out, row, dx, dy, w, h, obj, cls_scores):
    out[row, 0:4] = (dx, dy, math.log(w), math.log(h))
    out[row, 4] = obj
    for k, v in cls_scores.items():
        out[row, 5 + k] = v


def test_decode_person_box_in_image_pixels():
    out = raw_output()
    put(out, cell(8, 10, 20), 0.5, 0.5, 5.0, 10.0, 0.9, {0: 0.9})  # 40x80 px at (84, 164)
    [b] = decode(out, 0.5, 0.45, 0.45)  # image was shrunk by half
    assert (b.x1, b.y1, b.x2, b.y2) == pytest.approx((128.0, 248.0, 208.0, 408.0))
    assert b.score == pytest.approx(0.81)


def test_decode_ignores_other_classes_and_merges_duplicates():
    out = raw_output()
    put(out, cell(8, 10, 20), 0.5, 0.5, 5.0, 10.0, 0.9, {0: 0.9})
    put(out, cell(8, 10, 21), 0.5, -0.4, 5.0, 10.0, 0.9, {0: 0.8})  # same person, next cell
    put(out, cell(16, 20, 5), 0.0, 0.0, 4.0, 4.0, 0.95, {2: 0.95})  # a car
    people = decode(out, 1.0, 0.45, 0.45)
    assert len(people) == 1


def test_letterbox_keeps_aspect_top_left():
    image = np.zeros((720, 1280, 3), np.uint8)
    padded, scale = letterbox(image)
    assert padded.shape == (INPUT_SIZE, INPUT_SIZE, 3)
    assert scale == pytest.approx(INPUT_SIZE / 1280)
    assert (padded[300:, :] == 114).all()  # bottom padding


def box_at(distance, u, params=TrackParams()):
    h = CAMERA.fy * params.person_height / distance
    w = h * 0.25  # narrower than shoulders, so height decides
    return PersonBox(u - w / 2, 360 - h / 2, u + w / 2, 360 + h / 2, 0.9)


def test_distance_from_height():
    distance, bearing, (x, _, z) = locate(box_at(3.0, CAMERA.cx), CAMERA, TrackParams())
    assert distance == pytest.approx(3.0)
    assert bearing == pytest.approx(0.0, abs=1e-6)


def test_person_on_the_left_has_positive_bearing():
    _, bearing, _ = locate(box_at(3.0, 200.0), CAMERA, TrackParams())
    assert bearing > 0.3


def test_close_person_cut_off_by_frame_uses_width():
    # Only the top 400 px of a person 1 m away fits; their shoulders are 0.45 m wide.
    w = CAMERA.fx * 0.45 / 1.0
    box = PersonBox(640 - w / 2, 320, 640 + w / 2, 720, 0.9)
    distance, _, _ = locate(box, CAMERA, TrackParams())
    assert distance == pytest.approx(1.0, rel=0.01)


def scene(people):
    """A gray image with (box, shirt BGR color) people painted in."""
    image = np.full((720, 1280, 3), 128, np.uint8)
    for box, color in people:
        x1, y1, x2, y2 = box.as_int()
        h = y2 - y1
        image[y1:y1 + h // 2, x1:x2] = color
        image[y1 + h // 2:y2, x1:x2] = (40, 40, 40)
    return image


RED, BLUE, GREEN = (0, 0, 220), (220, 0, 0), (0, 200, 0)


def person(x, size=300):
    return PersonBox(x, 200, x + size * 0.4, 200 + size, 0.9)


def frame(lock, people, t):
    boxes = [b for b, _ in people]
    return lock.update(boxes, scene(people), t)


def test_locks_on_largest_then_stays_on_them():
    lock = PersonLock()
    near, far = person(300, 400), person(800, 250)
    assert frame(lock, [(near, RED), (far, BLUE)], 0.0) is near
    # A bigger stranger walks up beside them: the lock stays on the red shirt.
    stranger, moved = person(850, 500), person(320, 400)
    assert frame(lock, [(stranger, BLUE), (moved, RED)], 0.15) is moved


def test_does_not_switch_to_a_stranger_after_losing_target():
    lock = PersonLock()
    frame(lock, [(person(300), RED)], 0.0)
    assert frame(lock, [], 0.2) is None
    # Target gone for over hold_time; a stranger in a different shirt appears where they were.
    assert frame(lock, [(person(300), BLUE)], 1.0) is None
    # The target comes back elsewhere: recognized by their shirt.
    back = person(900)
    assert frame(lock, [(person(300), BLUE), (back, RED)], 1.5) is back


def test_forgets_after_forget_time():
    lock = PersonLock(TrackParams(forget_time=2.0))
    frame(lock, [(person(300), RED)], 0.0)
    other = person(700, 350)
    assert frame(lock, [(other, GREEN)], 1.0) is None
    assert frame(lock, [(other, GREEN)], 2.5) is other


def test_reset_relocks_on_largest():
    lock = PersonLock()
    small, big = person(100, 250), person(700, 400)
    frame(lock, [(small, RED)], 0.0)
    lock.reset()
    assert frame(lock, [(small, RED), (big, BLUE)], 0.1) is big


@pytest.mark.skipif(not MODEL.is_file(), reason='run tools/fetch_person_model.py')
def test_real_model_finds_no_one_in_an_empty_room():
    from th_autonomy.person_detector import PersonDetector
    detector = PersonDetector(MODEL)
    assert detector.detect(np.full((720, 1280, 3), 128, np.uint8)) == []
