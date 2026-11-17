"""Who is who between frames, who the robot follows, and where they are.

Tracker (ByteTrack-style): every person gets a track id that stays with them.
Boxes are matched to tracks by overlap with where each track is predicted to
be (constant velocity), confident detections first, then the doubtful ones
(people half hidden or blurred), which only keep existing tracks alive.

FollowLock: which track the robot follows.
  - Claim: raise a hand (wrist above the head, elbow above the shoulder, so a
    phone held to the ear doesn't count) for `claim_time` and the lock
    moves to you. While following someone still in view, nobody can take it.
  - Y with nobody claimed locks on to the largest (nearest) person.
  - Lost: a track that disappears (behind someone, out of frame) is won back
    only by a new track whose clothing matches; after `forget_time` the lock
    is dropped and the robot waits for a raised hand (or Y again).

Distance from one camera, assuming the person stands upright: from the torso
(shoulder midpoint to hip midpoint, `torso_length`), which barely changes as
people turn or move their arms and legs; without a visible torso, from the box
(`person_height`, `shoulder_width`, the closer of the two).
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

from th_autonomy.pose_detector import (L_ELBOW, L_HIP, L_SHOULDER, L_WRIST, NOSE, R_ELBOW, R_HIP,
                                       R_SHOULDER, R_WRIST)


@dataclass
class TrackParams:
    high_score: float = 0.5        # detections that can start a track
    match_iou: float = 0.2         # overlap to continue a track with a confident detection
    low_match_iou: float = 0.4     # ... with a doubtful one
    track_buffer: float = 1.0      # s a track survives unseen
    keypoint_score: float = 0.3    # joints below this are ignored
    claim_time: float = 0.5        # s a hand must stay raised to claim the robot
    raise_margin: float = 0.15     # wrist above the nose by this fraction of the torso
    min_similarity: float = 0.5    # clothing match to win back a lost person
    forget_time: float = 4.0       # s lost before the lock is dropped
    histogram_update: float = 0.2  # how fast the clothing histogram follows lighting
    torso_length: float = 0.5      # m, shoulder midpoint to hip midpoint
    person_height: float = 1.7     # m, for the box fallback
    shoulder_width: float = 0.45   # m, for the box fallback


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area if area > 0 else 0.0


def box_of(person):
    return np.array([person.x1, person.y1, person.x2, person.y2], float)


def torso(person, min_score):
    """(shoulder midpoint, hip midpoint) if both shoulders and hips are seen, else None."""
    joints = [person.joint(i, min_score) for i in (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP)]
    if any(j is None for j in joints):
        return None
    ls, rs, lh, rh = joints
    return ((ls[0] + rs[0]) / 2, (ls[1] + rs[1]) / 2), ((lh[0] + rh[0]) / 2, (lh[1] + rh[1]) / 2)


def hand_raised(person, params):
    """A hand up: wrist above the head and elbow above the shoulder (image y grows downward).

    Above the head, not just the shoulder, so a phone held to the ear doesn't count.
    """
    t = torso(person, params.keypoint_score)
    scale = math.dist(*t) if t is not None else person.height / 3.0
    nose = person.joint(NOSE, params.keypoint_score)
    for wrist, elbow, shoulder in ((L_WRIST, L_ELBOW, L_SHOULDER), (R_WRIST, R_ELBOW, R_SHOULDER)):
        w = person.joint(wrist, params.keypoint_score)
        e = person.joint(elbow, params.keypoint_score)
        s = person.joint(shoulder, params.keypoint_score)
        if w is None or e is None or s is None or e[1] > s[1]:
            continue
        head = nose[1] if nose is not None else s[1] - 0.6 * scale
        if w[1] < head - params.raise_margin * scale:
            return True
    return False


def torso_histogram(bgr, person, min_score=0.3):
    """Hue-saturation histogram of the shirt, normalized (None if not visible)."""
    t = torso(person, min_score)
    if t is not None:
        (sx, sy), (hx, hy) = t
        half = max(4.0, 0.25 * math.dist((sx, sy), (hx, hy)))
        x1, x2 = min(sx, hx) - half, max(sx, hx) + half
        y1, y2 = sy + 0.15 * (hy - sy), hy - 0.15 * (hy - sy)
    else:
        w, h = person.width, person.height
        x1, x2, y1, y2 = person.x1 + w / 4, person.x2 - w / 4, person.y1 + h / 5, person.y1 + h / 2
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(bgr.shape[1], int(x2)), min(bgr.shape[0], int(y2))
    crop = bgr[y1:y2, x1:x2]
    if crop.size == 0 or crop.ndim != 3:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist


def similarity(a, b):
    if a is None or b is None:
        return 0.0
    return float(cv2.compareHist(a, b, cv2.HISTCMP_CORREL))


class Track:

    def __init__(self, track_id, person, t):
        self.id = track_id
        self.person = person
        self.box = box_of(person)
        self.velocity = np.zeros(4)
        self.first_seen = t
        self.last_seen = t
        self.raised_since = None
        self.histogram = None

    def predicted(self, t):
        return self.box + self.velocity * (t - self.last_seen)

    def update(self, person, t):
        box = box_of(person)
        dt = t - self.last_seen
        if dt > 1e-3:
            self.velocity = 0.6 * self.velocity + 0.4 * (box - self.box) / dt
        self.box = box
        self.person = person
        self.last_seen = t


class Tracker:

    def __init__(self, params=None):
        self.params = params or TrackParams()
        self.tracks = []
        self.next_id = 1

    def reset(self):
        self.tracks = []

    def update(self, people, t):
        """Tracks seen in this frame (people matched to ids), after updating all tracks."""
        p = self.params
        self.tracks = [k for k in self.tracks if t - k.last_seen <= p.track_buffer]
        high = [x for x in people if x.score >= p.high_score]
        low = [x for x in people if x.score < p.high_score]
        free = list(self.tracks)
        seen = []
        for group, threshold in ((high, p.match_iou), (low, p.low_match_iou)):
            pairs = sorted(((iou(k.predicted(t), box_of(x)), i, j)
                            for i, k in enumerate(free) for j, x in enumerate(group)),
                           reverse=True)
            used_tracks, used_people = set(), set()
            for overlap, i, j in pairs:
                if overlap < threshold:
                    break
                if i in used_tracks or j in used_people:
                    continue
                used_tracks.add(i)
                used_people.add(j)
                free[i].update(group[j], t)
                seen.append(free[i])
            free = [k for i, k in enumerate(free) if i not in used_tracks]
            if group is high:
                for j, x in enumerate(group):
                    if j not in used_people:
                        track = Track(self.next_id, x, t)
                        self.next_id += 1
                        self.tracks.append(track)
                        seen.append(track)
        return seen


class FollowLock:
    """Which track to follow: claimed by a raised hand, or the largest on Y."""

    def __init__(self, params=None):
        self.params = params or TrackParams()
        self.reset()

    def reset(self):
        self.track_id = None
        self.histogram = None
        self.last_seen = None
        self.claimed = False  # by a raised hand (else picked on Y)

    def lock(self, track, t, claimed):
        self.track_id = track.id
        self.histogram = track.histogram
        self.last_seen = t
        self.claimed = claimed

    def lock_largest(self, seen, t):
        if seen:
            self.lock(max(seen, key=lambda k: k.person.area), t, False)

    def update(self, seen, bgr, t, following):
        """The followed track in this frame, or None. `seen`: this frame's tracks."""
        p = self.params
        for track in seen:
            hist = torso_histogram(bgr, track.person, p.keypoint_score)
            if track.histogram is None or hist is None:
                track.histogram = hist if hist is not None else track.histogram
            else:
                track.histogram = cv2.addWeighted(
                    track.histogram, 1.0 - p.histogram_update, hist, p.histogram_update, 0.0)
            if hand_raised(track.person, p):
                if track.raised_since is None:
                    track.raised_since = t
            else:
                track.raised_since = None
        if self.track_id is not None and t - self.last_seen > p.forget_time:
            self.reset()
        current = next((k for k in seen if k.id == self.track_id), None)
        # A raised hand claims the robot, unless it is following someone in view.
        if current is None or not following:
            claimers = [k for k in seen if k.raised_since is not None
                        and t - k.raised_since >= p.claim_time and k.id != self.track_id]
            if claimers:
                current = max(claimers, key=lambda k: k.person.area)
                self.lock(current, t, True)
                return current
        if current is None and self.track_id is not None:
            # Lost: win them back only by clothing, on a track that started after.
            candidates = [(similarity(self.histogram, k.histogram), k) for k in seen
                          if k.first_seen >= self.last_seen - 0.05]
            candidates = [c for c in candidates if c[0] >= p.min_similarity]
            if candidates:
                current = max(candidates, key=lambda c: c[0])[1]
                self.track_id = current.id
        if current is not None:
            self.last_seen = t
            if current.histogram is not None:
                self.histogram = current.histogram
        return current


def locate(person, intrinsics, params):
    """(distance m, bearing rad left positive, (x, y, z) in the camera frame, how)."""
    t = torso(person, params.keypoint_score)
    if t is not None and math.dist(*t) > 2.0:
        (sx, sy), (hx, hy) = t
        z = intrinsics.fy * params.torso_length / math.dist((sx, sy), (hx, hy))
        u, v = (sx + hx) / 2.0, (sy + hy) / 2.0
        how = 'torso'
    else:
        candidates = []
        if person.height > 1.0:
            candidates.append(intrinsics.fy * params.person_height / person.height)
        if person.width > 1.0:
            candidates.append(intrinsics.fx * params.shoulder_width / person.width)
        z = min(candidates)
        u, v = (person.x1 + person.x2) / 2.0, (person.y1 + person.y2) / 2.0
        how = 'box'
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    return math.hypot(x, z), math.atan2(-x, z), (x, y, z), how
