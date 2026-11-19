"""Pose output decoding, distance, tracking, and who the robot follows (raised-hand claims)."""

from pathlib import Path

import numpy as np
import pytest

from th_autonomy.pose_detector import INPUT_SIZE, Person, postprocess, preprocess
from th_autonomy.person_tracker import (FollowLock, Tracker, TrackParams, hand_raised, locate)
from th_autonomy.tag_detector import Intrinsics

MODEL = Path(__file__).resolve().parents[3] / 'models' / 'rtmo-l.onnx'
CAMERA = Intrinsics.from_hfov(1280, 720, 90.0)
RED, BLUE, GREEN = (0, 0, 220), (220, 0, 0), (0, 200, 0)


def person(x, top=150, height=400, hand_up=False, score=0.9, phone=False):
    """A standing person: box plus COCO keypoints (shoulders a quarter down, hips at half)."""
    w = height * 0.4
    cx = x + w / 2
    k = np.zeros((17, 3), np.float32)
    k[:, 2] = 0.9

    def put(i, px, py):
        k[i, :2] = (px, py)
    sy, hy = top + 0.25 * height, top + 0.5 * height
    put(0, cx, top + 0.1 * height)
    for i, side in ((5, -1), (11, -1), (6, 1), (12, 1)):
        put(i, cx + side * 0.18 * w, sy if i in (5, 6) else hy)
    put(7, cx - 0.3 * w, top + 0.37 * height)
    put(8, cx + 0.3 * w, top + 0.37 * height)
    put(9, cx - 0.3 * w, top + 0.48 * height)  # wrists down by the hips
    put(10, cx + 0.3 * w, top + 0.48 * height)
    if hand_up:
        put(8, cx + 0.35 * w, top + 0.15 * height)  # right elbow above the shoulder
        put(10, cx + 0.3 * w, top)                  # right wrist above the head
    if phone:
        put(10, cx + 0.1 * w, top + 0.1 * height)   # right wrist by the ear
        put(8, cx + 0.3 * w, top + 0.3 * height)    # elbow below the shoulder
    for i, f in ((13, 0.75), (14, 0.75), (15, 0.98), (16, 0.98)):
        put(i, cx + (-1 if i % 2 else 1) * 0.15 * w, top + f * height)
    return Person(x, top, x + w, top + height, score, k)


def scene(people):
    """A gray image with (person, shirt BGR color) painted in: shirt over the torso."""
    image = np.full((720, 1280, 3), 128, np.uint8)
    for p, color in people:
        x1, y1, x2, y2 = p.as_int()
        h = y2 - y1
        image[y1:y1 + h // 2, x1:x2] = color
        image[y1 + h // 2:y2, x1:x2] = (40, 40, 40)
    return image


class World:
    """Tracker + lock fed frame by frame."""

    def __init__(self, params=TrackParams()):
        self.tracker = Tracker(params)
        self.lock = FollowLock(params)

    def frame(self, people, t, following=False):
        seen = self.tracker.update([p for p, _ in people], t)
        self.seen = seen
        followed = self.lock.update(seen, scene(people), t, following)
        return None if followed is None else followed.person


# -- model input and output ------------------------------------------------------------------

def test_preprocess_keeps_aspect_top_left():
    tensor, scale = preprocess(np.zeros((720, 1280, 3), np.uint8))
    assert tensor.shape == (1, 3, INPUT_SIZE, INPUT_SIZE)
    assert scale == pytest.approx(0.5)
    assert (tensor[0, :, 360:, :] == 114).all()  # bottom padding


def test_postprocess_scales_back_and_filters():
    dets = np.array([[[100, 50, 200, 250, 0.9], [0, 0, 10, 10, 0.1]]], np.float32)
    kps = np.zeros((1, 2, 17, 3), np.float32)
    kps[0, 0, :, :2] = (150, 100)
    [p] = postprocess(dets, kps, 0.5, 0.3, 1280, 720)
    assert (p.x1, p.y1, p.x2, p.y2) == pytest.approx((200, 100, 400, 500))
    assert p.keypoints[5, :2] == pytest.approx((300, 200))


# -- distance and hand raise ---------------------------------------------------------------

def test_distance_from_torso():
    p = person(560, height=400)
    distance, bearing, _, how = locate(p, CAMERA, TrackParams())
    torso_px = 0.25 * 400
    assert how == 'torso'
    assert distance == pytest.approx(CAMERA.fy * 0.5 / torso_px, rel=0.02)
    assert abs(bearing) < 0.05


def test_person_on_the_left_has_positive_bearing():
    _, bearing, _, _ = locate(person(100), CAMERA, TrackParams())
    assert bearing > 0.3


def test_box_fallback_without_hips():
    p = person(560)
    p.keypoints[11:, 2] = 0.0  # legs and hips cut off
    _, _, _, how = locate(p, CAMERA, TrackParams())
    assert how == 'box'


def test_hand_raised():
    params = TrackParams()
    assert not hand_raised(person(300), params)
    assert hand_raised(person(300, hand_up=True), params)


def test_phone_to_the_ear_is_not_a_raised_hand():
    assert not hand_raised(person(300, phone=True), TrackParams())


# -- tracking ------------------------------------------------------------------------------

def test_track_ids_follow_people_as_they_move():
    tracker = Tracker()
    a, b = tracker.update([person(100), person(700)], 0.0)
    for t in (0.1, 0.2, 0.3):
        seen = tracker.update([person(700 - 40 * t * 10), person(100 + 40 * t * 10)], t)
        assert {k.id for k in seen} == {a.id, b.id}
    assert next(k for k in seen if k.person.x1 < 400).id == a.id


def test_doubtful_detection_keeps_a_track_but_starts_none():
    tracker = Tracker()
    [a] = tracker.update([person(300)], 0.0)
    [kept] = tracker.update([person(305, score=0.35)], 0.1)
    assert kept.id == a.id
    assert tracker.update([person(900, score=0.35)], 0.2) == []


# -- who is followed -----------------------------------------------------------------------

def test_y_takes_the_largest_when_nobody_claimed():
    w = World()
    w.frame([(person(100, height=300), RED), (person(700, height=450), BLUE)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    assert w.frame([(person(100, height=300), RED), (person(700, height=450), BLUE)], 0.1).x1 == 700


def test_raised_hand_claims_after_claim_time():
    w = World()
    big, small = person(700, height=450), person(100, height=300)
    w.frame([(big, RED), (small, BLUE)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    waving = person(100, height=300, hand_up=True)
    assert w.frame([(big, RED), (waving, BLUE)], 0.1).x1 == 700   # not yet
    assert w.frame([(big, RED), (waving, BLUE)], 0.7).x1 == 100   # held 0.6 s
    assert w.lock.claimed


def test_nobody_can_claim_while_following_someone_in_view():
    w = World()
    w.frame([(person(700), RED), (person(100, height=300), BLUE)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    waving = person(100, height=300, hand_up=True)
    for t in (0.1, 0.5, 1.0):
        assert w.frame([(person(700), RED), (waving, BLUE)], t, following=True).x1 == 700


def test_lost_target_is_won_back_by_clothes_not_by_a_stranger():
    w = World()
    w.frame([(person(300), RED)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    assert w.frame([], 0.5, following=True) is None
    assert w.frame([], 1.6, following=True) is None  # the track itself expired
    assert w.frame([(person(300), BLUE)], 2.0, following=True) is None
    back = person(900)
    assert w.frame([(person(300), BLUE), (back, RED)], 2.3, following=True).x1 == 900


def test_lost_target_can_be_claimed_by_a_new_hand():
    w = World()
    w.frame([(person(300), RED)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    waving = person(800, hand_up=True)
    w.frame([(waving, GREEN)], 0.5, following=True)
    assert w.frame([(waving, GREEN)], 1.1, following=True).x1 == 800


def test_lock_dropped_after_forget_time():
    w = World(TrackParams(forget_time=2.0))
    w.frame([(person(300), RED)], 0.0)
    w.lock.lock_largest(w.seen, 0.0)
    w.frame([(person(700), GREEN)], 1.0, following=True)
    w.frame([(person(700), GREEN)], 2.5, following=True)
    assert w.lock.track_id is None


@pytest.mark.skipif(not MODEL.is_file(), reason='run tools/fetch_person_model.py')
def test_real_model_finds_no_one_in_an_empty_room():
    from th_autonomy.pose_detector import PoseDetector
    detector = PoseDetector(MODEL)
    assert detector.detect(np.full((720, 1280, 3), 128, np.uint8)) == []


def test_skeleton_indices_are_coco():
    from th_autonomy.pose_detector import SKELETON
    assert all(0 <= i < 17 and 0 <= j < 17 for i, j in SKELETON)
