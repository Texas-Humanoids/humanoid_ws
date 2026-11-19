"""AprilTag detection and pose from one camera, with OpenCV's aruco module.

OpenCV ships the AprilTag dictionaries, so this needs only python3-opencv
(Ubuntu 24.04 has 4.6, which uses the older aruco API; 4.7+ is handled too).

Camera frame (OpenCV): x right, y down, z forward. The tag's bearing is
atan2(-x, z), so positive means left, matching ROS yaw. Distance is measured
along the floor plane, sqrt(x^2 + z^2), assuming the camera faces forward and
level.

`tag_size` is the side of the tag's outer black square, in meters.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np

FAMILIES = {
    'tag16h5': 'DICT_APRILTAG_16h5',
    'tag25h9': 'DICT_APRILTAG_25h9',
    'tag36h10': 'DICT_APRILTAG_36h10',
    'tag36h11': 'DICT_APRILTAG_36h11',
}


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    @classmethod
    def from_hfov(cls, width, height, hfov_deg):
        """A pinhole guess from the horizontal field of view, for an uncalibrated camera."""
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(fx, fx, (width - 1) / 2.0, (height - 1) / 2.0)

    def scaled(self, factor):
        """The same camera at `factor` times the resolution."""
        return Intrinsics(self.fx * factor, self.fy * factor,
                          (self.cx + 0.5) * factor - 0.5, (self.cy + 0.5) * factor - 0.5,
                          self.distortion)

    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass
class TagDetection:
    tag_id: int
    corners: np.ndarray    # 4x2 pixels: top-left, top-right, bottom-right, bottom-left
    position: np.ndarray   # tag center in the camera frame, m
    distance: float        # m, along the floor plane
    bearing: float         # rad, left positive


def _dictionary(family):
    if family not in FAMILIES:
        raise ValueError(f"Unknown tag family '{family}'; use one of {sorted(FAMILIES)}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, FAMILIES[family]))


def render_tag(family, tag_id, side_pixels):
    """The tag's image (black square and code, no white margin), for printing and tests."""
    dictionary = _dictionary(family)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, side_pixels)
    return cv2.aruco.drawMarker(dictionary, tag_id, side_pixels)


class TagDetector:

    def __init__(self, family='tag36h11', tag_size=0.16, refine_corners=True):
        if tag_size <= 0.0:
            raise ValueError('tag_size must be > 0')
        self.tag_size = tag_size
        dictionary = _dictionary(family)
        if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV 4.7+
            params = cv2.aruco.DetectorParameters()
            self._set_refinement(params, refine_corners)
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect = detector.detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._set_refinement(params, refine_corners)
            self._detect = lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=params)
        half = tag_size / 2.0
        # Same order as the detected corners, in the tag's own frame (z out of the tag).
        self.object_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]])

    @staticmethod
    def _set_refinement(params, refine):
        params.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX if refine else cv2.aruco.CORNER_REFINE_NONE)

    def detect(self, gray, intrinsics):
        """All tags in an 8-bit grayscale image, nearest first."""
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return []
        camera = intrinsics.matrix()
        distortion = (np.array(intrinsics.distortion, dtype=float)
                      if any(intrinsics.distortion) else None)
        detections = []
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            image_points = tag_corners.reshape(4, 2).astype(np.float64)
            ok, _, tvec = cv2.solvePnP(self.object_points, image_points, camera, distortion,
                                       flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            x, y, z = (float(v) for v in tvec.flatten())
            if z <= 0.0:
                continue
            detections.append(TagDetection(
                int(tag_id), image_points, np.array([x, y, z]),
                math.hypot(x, z), math.atan2(-x, z)))
        detections.sort(key=lambda d: d.distance)
        return detections
