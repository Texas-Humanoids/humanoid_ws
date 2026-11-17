"""Walks the robot toward a person and stops `follow.target_distance` in front of them.

The camera, gamepad and safety rules are shared with the tag follower (see
follower_node.py). Stand in front of the robot and press Y: the largest person
in view is locked on and followed, even with others in frame (person_tracker.py).
"""

import math

from th_autonomy import overlay
from th_autonomy.follow_controller import FollowParams
from th_autonomy.follower_node import FollowerNode, Target, run
from th_autonomy.person_detector import PersonDetector
from th_autonomy.person_tracker import PersonLock, TrackParams, locate

# People are bigger than tags and harder to range: stop further away, move gently.
PERSON_FOLLOW = FollowParams(target_distance=1.5, distance_deadband=0.2, max_forward=0.3,
                             max_backward=0.0, hold_time=0.6)


class PersonFollower(FollowerNode):

    color = True

    def __init__(self, **kwargs):
        super().__init__('person_follower', follow_defaults=PERSON_FOLLOW, **kwargs)
        p = self.declare_parameter
        self.detector = PersonDetector(
            p('model_path', 'models/yolox_nano.onnx').value,
            float(p('score_threshold', 0.45).value), float(p('nms_threshold', 0.45).value),
            int(p('threads', 4).value))
        defaults = TrackParams()
        self.tracker = PersonLock(TrackParams(**{
            name: float(p(f'track.{name}', getattr(defaults, name)).value)
            for name in defaults.__dataclass_fields__}))
        self.people = []
        self.locked = None
        self.located = None

    def on_enable(self):
        self.tracker.reset()  # follow whoever is in front now

    def find_target(self, bgr, intrinsics):
        self.people = self.detector.detect(bgr)
        self.locked = self.tracker.update(self.people, bgr, self.now())
        if self.locked is None:
            self.located = None
            return None
        self.located = locate(self.locked, intrinsics, self.tracker.params)
        distance, bearing, position = self.located
        return Target(distance, bearing, position)

    def shapes(self):
        out = []
        for b in self.people:
            locked = b is self.locked
            label = f'person {b.score:.2f}'
            if locked and self.located:
                distance, bearing, _ = self.located
                label = f'FOLLOW {distance:.1f} m {math.degrees(bearing):+.0f}°'
            out.append(overlay.Shape(overlay.box(b.x1, b.y1, b.x2, b.y2),
                                     overlay.FOLLOWED if locked else overlay.OTHER, label, locked))
        return out


def main(args=None):
    run(PersonFollower, args)
