"""What the follower shows on the dashboard: shapes over the camera and a target in 3D.

Detections are found in the frame as the follower sees it (decoded at 1/decode_scale
and turned upright by camera.rotation). The dashboard shows the camera's own stream,
so `to_raw` maps shapes back to its pixels before they become annotations.

The 3D view gets the target in `follow_camera`, a frame on the robot's chest
(x forward, y left, z up), so it moves and tilts with the robot.
"""

from dataclasses import dataclass
import math

import cv2
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:  # ros-jazzy-foxglove-msgs; without it the camera overlay is skipped
    from foxglove_msgs.msg import Color, ImageAnnotations, Point2, PointsAnnotation, TextAnnotation
except ImportError:
    ImageAnnotations = None

FOLLOWED = (0, 220, 0)    # BGR: the target being followed
OTHER = (0, 160, 255)     # every other detection
FOLLOWING_STATES = ('TRACKING', 'LOST', 'SEARCH')


@dataclass
class Shape:
    points: list            # [(x, y)] closed outline, pixels in the follower's (upright) frame
    color: tuple = OTHER    # BGR 0-255
    label: str = ''
    thick: bool = False


@dataclass
class TargetMarker:
    """How the target looks in 3D: a marker type and its size (x, y, z) in m."""
    kind: int = Marker.CYLINDER
    scale: tuple = (0.35, 0.35, 1.7)


def box(x1, y1, x2, y2):
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def to_raw(points, rotation, width, height, scale):
    """Points in the upright, decoded frame -> pixels in the camera's own image.

    `width`, `height`: the decoded frame before rotation; `rotation`: the cv2.ROTATE_* code
    that was applied (or None); `scale`: decode_scale.
    """
    out = []
    for x, y in points:
        x, y = float(x), float(y)
        if rotation == cv2.ROTATE_180:
            x, y = width - 1 - x, height - 1 - y
        elif rotation == cv2.ROTATE_90_CLOCKWISE:
            x, y = y, height - 1 - x
        elif rotation == cv2.ROTATE_90_COUNTERCLOCKWISE:
            x, y = width - 1 - y, x
        out.append(((x + 0.5) * scale - 0.5, (y + 0.5) * scale - 0.5))
    return out


def _color(bgr, alpha=1.0):
    return Color(r=bgr[2] / 255.0, g=bgr[1] / 255.0, b=bgr[0] / 255.0, a=alpha)


def annotations(stamp, shapes, status, rotation, width, height, scale):
    """foxglove_msgs/ImageAnnotations for the camera's own image, or None without foxglove_msgs."""
    if ImageAnnotations is None:
        return None
    msg = ImageAnnotations()
    font = max(14.0, width * scale / 60.0)
    for s in shapes:
        raw = to_raw(s.points, rotation, width, height, scale)
        msg.points.append(PointsAnnotation(
            timestamp=stamp, type=PointsAnnotation.LINE_LOOP,
            points=[Point2(x=x, y=y) for x, y in raw],
            outline_color=_color(s.color), thickness=4.0 if s.thick else 2.0))
        if s.label:
            # Above the outline's top edge as it appears in the stream.
            x = min(p[0] for p in raw)
            y = min(p[1] for p in raw)
            msg.texts.append(TextAnnotation(
                timestamp=stamp, position=Point2(x=x, y=max(0.0, y - font * 1.4)), text=s.label,
                font_size=font, text_color=_color(s.color), background_color=Color(a=0.6)))
    msg.texts.append(TextAnnotation(
        timestamp=stamp, position=Point2(x=10.0, y=10.0), text=status, font_size=font,
        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0), background_color=Color(a=0.6)))
    return msg


def draw(image, shapes, status):
    """The same shapes drawn into a BGR image, for /follower/image/compressed."""
    for s in shapes:
        pts = [(int(round(x)), int(round(y))) for x, y in s.points]
        for a, b in zip(pts, pts[1:] + pts[:1]):
            cv2.line(image, a, b, s.color, 3 if s.thick else 1, cv2.LINE_AA)
        if s.label:
            x = min(p[0] for p in pts)
            y = min(p[1] for p in pts)
            cv2.putText(image, s.label.replace('°', ' deg'), (x + 4, max(15, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, s.color, 2, cv2.LINE_AA)
    cv2.putText(image, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                cv2.LINE_AA)


def _rgba(bgr, alpha=1.0):
    return ColorRGBA(r=bgr[2] / 255.0, g=bgr[1] / 255.0, b=bgr[0] / 255.0, a=alpha)


def markers(header, camera_frame, floor_frame, position, label, following, stop_distance, look):
    """MarkerArray: the target (when seen), a line to it, its label, and the stop arc.

    `position` is the target in the camera's optical frame (x right, y down, z forward), or
    None. Target markers expire after 0.5 s, so they vanish when it is no longer seen.
    """
    color = (0, 220, 0) if following else (255, 203, 97)  # green / light blue (not following)
    out = MarkerArray()

    def marker(mid, kind, frame, lifetime):
        m = Marker()
        m.header.stamp = header.stamp
        m.header.frame_id = frame
        m.ns = 'follower'
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.frame_locked = True
        m.lifetime = Duration(sec=int(lifetime), nanosec=int((lifetime % 1.0) * 1e9))
        m.color = _rgba(color)
        out.markers.append(m)
        return m

    if position is not None:
        x, y, z = position
        ahead, left, up = z, -x, -y  # optical -> follow_camera
        m = marker(0, look.kind, camera_frame, 0.5)
        m.pose.position = Point(x=ahead, y=left, z=up)
        # Face the robot: yaw toward the camera (only matters for a flat tag).
        yaw = math.atan2(left, ahead)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = look.scale
        m.color = _rgba(color, 0.85)
        line = marker(1, Marker.LINE_LIST, camera_frame, 0.5)
        line.scale.x = 0.02
        line.points = [Point(), Point(x=ahead, y=left, z=up)]
        text = marker(2, Marker.TEXT_VIEW_FACING, camera_frame, 0.5)
        text.pose.position = Point(x=ahead, y=left, z=up + look.scale[2] / 2.0 + 0.15)
        text.scale.z = 0.15
        text.text = label
        text.color = _rgba((255, 255, 255))

    # Where the robot means to stop: an arc on the floor in front of it.
    arc = marker(3, Marker.LINE_STRIP, floor_frame, 1.0)
    arc.scale.x = 0.015
    arc.color = _rgba((255, 255, 255), 0.5)
    arc.points = [Point(x=stop_distance * math.cos(a), y=stop_distance * math.sin(a), z=0.01)
                  for a in (math.radians(d) for d in range(-60, 61, 5))]
    label_m = marker(4, Marker.TEXT_VIEW_FACING, floor_frame, 1.0)
    # At the arc's left end, where the robot never hides it in the view from behind.
    end = math.radians(60.0)
    label_m.pose.position = Point(x=stop_distance * math.cos(end),
                                  y=stop_distance * math.sin(end) + 0.15, z=0.05)
    label_m.scale.z = 0.1
    label_m.text = f'stop {stop_distance:.1f} m'
    label_m.color = _rgba((255, 255, 255), 0.7)
    return out
