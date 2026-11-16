"""Walks the robot toward an AprilTag and stops `follow.target_distance` in front of it.

The camera, gamepad and safety rules are shared with the person follower (see
follower_node.py). This node only finds the tag: the nearest one, or `target_id`.
"""

import math

import cv2
import numpy as np

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

    def find_target(self, gray, intrinsics):
        detections = self.detector.detect(gray, intrinsics)
        if self.target_id >= 0:
            detections = [d for d in detections if d.tag_id == self.target_id]
        self.detections = detections
        self.target = detections[0] if detections else None
        if self.target is None:
            return None
        return Target(self.target.distance, self.target.bearing, tuple(self.target.position))

    def draw(self, image):
        for d in self.detections:
            color = (0, 220, 0) if d is self.target else (0, 160, 255)
            cv2.polylines(image, [d.corners.astype(np.int32)], True, color, 2)
            x, y = d.corners[0].astype(int)
            cv2.putText(image, f'{d.tag_id}: {d.distance:.2f} m {math.degrees(d.bearing):+.0f} deg',
                        (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main(args=None):
    run(TagFollower, args)
