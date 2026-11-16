"""Keeps following the same person when several are in view, and estimates where they are.

Locking: with no lock, the largest (usually closest) person becomes the target,
and the node clears the lock when following is turned on, so whoever stands
in front when Y is pressed is the one followed. Each frame the locked person is
matched by box overlap (IoU) with their last box, helped by a color histogram
of their torso (clothes). If they are lost for longer than `hold_time` (walked
behind someone, left the frame), only the histogram can win them back, so the
robot doesn't switch to a bystander. After `forget_time` the lock is dropped
and the largest person is taken again.

Distance from one camera, assuming the person is upright:
  from height: fy * person_height / box height
  from width:  fx * shoulder_width / box width
and the smaller of the two is used. A box cut off by the frame edge, or a
person close enough that their feet are out of view, looks short and so too
far away; the width estimate catches that, and erring close stops the robot early.
"""

from dataclasses import dataclass
import math

import cv2


@dataclass
class TrackParams:
    person_height: float = 1.7    # m
    shoulder_width: float = 0.45  # m
    min_iou: float = 0.1          # to count as the same box while tracked
    min_similarity: float = 0.5   # histogram correlation to win back a lost person
    hold_time: float = 0.6        # s matched by overlap after the last sighting
    forget_time: float = 4.0      # s before the lock is dropped
    histogram_update: float = 0.2  # how fast the clothing histogram follows lighting


def iou(a, b):
    w = min(a.x2, b.x2) - max(a.x1, b.x1)
    h = min(a.y2, b.y2) - max(a.y1, b.y1)
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / (a.area + b.area - inter)


def torso_histogram(bgr, box):
    """Hue-saturation histogram of the middle of the box (the shirt), normalized."""
    x1, y1, x2, y2 = box.as_int()
    w, h = x2 - x1, y2 - y1
    crop = bgr[y1 + h // 5: y1 + h // 2, x1 + w // 4: x2 - w // 4]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


def similarity(a, b):
    if a is None or b is None:
        return 0.0
    return float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL))


def locate(box, intrinsics, params):
    """(distance m, bearing rad left positive, (x, y, z) of the box center in the camera frame)."""
    u = (box.x1 + box.x2) / 2.0
    v = (box.y1 + box.y2) / 2.0
    candidates = []
    if box.height > 1.0:
        candidates.append(intrinsics.fy * params.person_height / box.height)
    if box.width > 1.0:
        candidates.append(intrinsics.fx * params.shoulder_width / box.width)
    z = min(candidates)
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    return math.hypot(x, z), math.atan2(-x, z), (x, y, z)


class PersonLock:

    def __init__(self, params=None):
        self.params = params or TrackParams()
        self.reset()

    def reset(self):
        self.box = None
        self.histogram = None
        self.last_seen = None

    def update(self, people, bgr, t):
        """The locked person's box in this frame, or None."""
        p = self.params
        if self.last_seen is not None and t - self.last_seen > p.forget_time:
            self.reset()
        if not people:
            return None
        histograms = [torso_histogram(bgr, b) for b in people]
        if self.box is None:
            best = max(range(len(people)), key=lambda i: people[i].area)
        else:
            tracked = t - self.last_seen <= p.hold_time
            best, best_score = None, 0.0
            for i, b in enumerate(people):
                sim = similarity(self.histogram, histograms[i])
                overlap = iou(self.box, b) if tracked else 0.0
                if overlap < p.min_iou and sim < p.min_similarity:
                    continue
                score = overlap + 0.5 * sim
                if score > best_score:
                    best, best_score = i, score
            if best is None:
                return None
        self.box = people[best]
        self.last_seen = t
        hist = histograms[best]
        if self.histogram is None or hist is None:
            self.histogram = hist
        else:
            self.histogram = cv2.addWeighted(
                self.histogram, 1.0 - p.histogram_update, hist, p.histogram_update, 0.0)
        return self.box
