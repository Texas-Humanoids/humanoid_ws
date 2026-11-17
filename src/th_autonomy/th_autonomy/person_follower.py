"""Walks the robot toward a person and stops `follow.target_distance` in front of them.

The camera, gamepad and safety rules are shared with the tag follower (see
follower_node.py). Who is followed (person_tracker.py):
  - raise a hand for half a second to claim the robot (it shows on the
    dashboard), then press Y to start; or press Y and it takes the nearest person;
  - while it follows someone in view, nobody else can claim it;
  - if they disappear it waits for them (matched by clothing) or a new claim.
The pose model (pose_detector.py, RTMO) runs on the GPU with TensorRT.
"""

import math

from th_autonomy import overlay
from th_autonomy.follow_controller import FollowParams
from th_autonomy.follower_node import FollowerNode, Target, run
from th_autonomy.pose_detector import SKELETON, PoseDetector
from th_autonomy.person_tracker import FollowLock, Tracker, TrackParams, locate

# People are bigger than tags and harder to range: stop further away, move gently.
PERSON_FOLLOW = FollowParams(target_distance=1.5, distance_deadband=0.2, max_forward=0.3,
                             max_backward=0.0, hold_time=0.6)
CLAIMING = (255, 0, 255)  # BGR magenta: a hand is up


class PersonFollower(FollowerNode):

    color = True

    def __init__(self, **kwargs):
        super().__init__('person_follower', follow_defaults=PERSON_FOLLOW, **kwargs)
        p = self.declare_parameter
        self.detector = PoseDetector(
            p('model_path', 'models/rtmo-l.onnx').value,
            float(p('score_threshold', 0.3).value), p('backend', 'auto').value,
            int(p('threads', 2).value), log=self.get_logger().info)
        self.get_logger().info(f'Pose model running on {self.detector.backend}')
        defaults = TrackParams()
        params = TrackParams(**{
            name: float(p(f'track.{name}', getattr(defaults, name)).value)
            for name in defaults.__dataclass_fields__})
        self.tracker = Tracker(params)
        self.claim = FollowLock(params)  # (self.lock is the base class's thread lock)
        self.seen = []
        self.followed = None
        self.located = None

    def on_enable(self):
        if self.claim.track_id is None:  # nobody claimed the robot: take the nearest
            self.claim.lock_largest(self.seen, self.now())

    def find_target(self, bgr, intrinsics):
        t = self.now()
        self.seen = self.tracker.update(self.detector.detect(bgr), t)
        self.followed = self.claim.update(self.seen, bgr, t, following=self.enabled)
        if self.followed is None:
            self.located = None
            return None
        self.located = locate(self.followed.person, intrinsics, self.tracker.params)
        distance, bearing, position, _ = self.located
        return Target(distance, bearing, position)

    def status_hint(self):
        if self.claim.track_id is None:
            return 'raise a hand to claim, Y to follow the nearest'
        return f'#{self.claim.track_id} {"claimed" if self.claim.claimed else "locked"}, Y to follow'

    def shapes(self):
        out = []
        t = self.now()
        min_score = self.tracker.params.keypoint_score
        for k in self.seen:
            followed = k is self.followed
            raising = k.raised_since is not None and not followed
            color = overlay.FOLLOWED if followed else CLAIMING if raising else overlay.OTHER
            label = f'#{k.id}'
            if followed and self.located:
                distance, bearing, _, _ = self.located
                label = f'FOLLOW #{k.id} {distance:.1f} m {math.degrees(bearing):+.0f}°'
            elif raising:
                held = min(1.0, (t - k.raised_since) / self.tracker.params.claim_time)
                label = f'#{k.id} hand up {held:.0%}'
            b = k.person
            out.append(overlay.Shape(overlay.box(b.x1, b.y1, b.x2, b.y2), color, label, followed))
            bones = []
            for i, j in SKELETON:
                a, c = b.joint(i, min_score), b.joint(j, min_score)
                if a is not None and c is not None:
                    bones += [a, c]
            if bones:
                out.append(overlay.Shape(bones, color, closed=False))
        return out


def main(args=None):
    run(PersonFollower, args)
