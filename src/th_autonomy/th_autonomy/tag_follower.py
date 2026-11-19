"""Walks the robot toward an AprilTag and stops `follow.target_distance` in front of it.

The camera, gamepad and safety rules are shared with the person follower (see
follower_node.py). This node only finds the tag: the nearest one, or `target_id`.
"""

import math

from visualization_msgs.msg import Marker

from th_autonomy import overlay
from th_autonomy.follower_node import FollowerNode, Target, run
from th_autonomy.tag_detector import TagDetector


class TagFollower(FollowerNode):

    def __init__(self, **kwargs):
        super().__init__('tag_follower', **kwargs)
        p = self.declare_parameter
        self.target_id = int(p('target_id', -1).value)
        self.detector = TagDetector(
            p('tag_family', 'tag36h11').value, float(p('tag_size', 0.16).value),
            bool(p('refine_corners', True).value))
        self.detections = []
        self.target = None
        size = self.detector.tag_size
        self.look = overlay.TargetMarker(Marker.CUBE, (0.02, size, size))  # a flat tag

    def find_target(self, gray, intrinsics):
        detections = self.detector.detect(gray, intrinsics)
        if self.target_id >= 0:
            detections = [d for d in detections if d.tag_id == self.target_id]
        self.detections = detections
        self.target = detections[0] if detections else None
        if self.target is None:
            return None
        return Target(self.target.distance, self.target.bearing, tuple(self.target.position))

    def shapes(self):
        out = []
        for d in self.detections:
            followed = d is self.target
            label = f'tag {d.tag_id}: {d.distance:.1f} m {math.degrees(d.bearing):+.0f}°'
            out.append(overlay.Shape([tuple(c) for c in d.corners],
                                     overlay.FOLLOWED if followed else overlay.OTHER,
                                     ('FOLLOW ' if followed else '') + label, followed))
        return out


def main(args=None):
    run(TagFollower, args)
