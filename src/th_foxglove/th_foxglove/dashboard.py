"""Markers for the dashboard's top-down "command" and "lean" views.

Both are drawn in a fixed frame ("dashboard") that never moves, in the robot's own axes
(x = the robot's forward, y = its left), so the views stay still while the robot walks and
the robot's forward is always up.

  /dashboard/command  speed rings, the commanded velocity (blue arrow), the measured
                      velocity (green, sim only), and the turn command (orange arc)
  /dashboard/lean     a bubble level: rings at 10/20/30 deg, the fall cutoff in red, a dot
                      where the body leans (forward = up, left = left) and a short trail
"""

import collections
import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

GREY = (0.55, 0.58, 0.63, 0.55)
FAINT = (0.55, 0.58, 0.63, 0.25)
BLUE = (0.31, 0.60, 0.89, 1.0)
GREEN = (0.30, 0.80, 0.45, 1.0)
ORANGE = (0.96, 0.47, 0.30, 1.0)
YELLOW = (0.95, 0.80, 0.25, 1.0)
RED = (0.90, 0.25, 0.22, 1.0)
WHITE = (0.92, 0.93, 0.95, 1.0)
# Text needs opaque contrast; translucent grey remains useful for the reference rings.
LABEL = (0.78, 0.81, 0.86, 1.0)

SPEED_RINGS = (0.25, 0.5)     # m/s, drawn 1 m/s = 1 m
TURN_RADIUS = 0.62            # where the turn arc is drawn
LEAN_SCALE = 0.01             # 1 deg of lean = 1 cm
LEAN_RINGS = (10.0, 20.0, 30.0)


def circle(radius, segments=64, start=0.0, sweep=2 * math.pi):
    """Points (x, y) on an arc of `radius`, angles measured from +x toward +y."""
    n = max(2, int(segments * abs(sweep) / (2 * math.pi)) + 1)
    return [(radius * math.cos(start + sweep * i / (n - 1)),
             radius * math.sin(start + sweep * i / (n - 1))) for i in range(n)]


def yaw_of(q):
    """Heading (rad) of an orientation [w, x, y, z] about the world z axis."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def lean_color(degrees):
    return GREEN if degrees < 10.0 else YELLOW if degrees < 25.0 else RED


class SmoothedPose:
    """Low-pass filtered floor position and heading, for a camera that glides instead of
    bouncing with every step. tau is the filter time constant in seconds."""

    def __init__(self, tau=0.8):
        self.tau = tau
        self.state = None
        self.last = None

    def update(self, t, x, y, yaw):
        if self.state is None or self.last is None or t - self.last > 1.0:
            self.state = [x, y, yaw]
        else:
            a = min(1.0, (t - self.last) / self.tau)
            sx, sy, syaw = self.state
            dyaw = math.atan2(math.sin(yaw - syaw), math.cos(yaw - syaw))  # shortest way round
            self.state = [sx + a * (x - sx), sy + a * (y - sy), syaw + a * dyaw]
        self.last = t
        return tuple(self.state)


class VelocityEstimate:
    """Base velocity in the heading frame from successive positions (the sim's ground truth).

    Averaged over `window` seconds: two gait cycles (v6 steps every 0.7 s) cancel the
    side-to-side sway of each step, leaving the velocity the robot is actually walking at.
    """

    def __init__(self, window=1.4):
        self.window = window
        self.samples = collections.deque()

    def update(self, t, x, y, yaw):
        self.samples.append((t, x, y))
        while len(self.samples) > 2 and t - self.samples[0][0] > self.window:
            self.samples.popleft()
        t0, x0, y0 = self.samples[0]
        if t - t0 < 0.05:
            return None
        vx, vy = (x - x0) / (t - t0), (y - y0) / (t - t0)
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * vx + s * vy, -s * vx + c * vy)  # world -> heading frame

    def reset(self):
        self.samples.clear()


class Markers:
    """Builds MarkerArrays in one frame.

    Each array starts with DELETEALL: the number of markers changes from update to update
    (an arrow or arc comes and goes), and without it the viewer keeps the leftovers.
    """

    def __init__(self, frame, ns, stamp):
        self.frame, self.ns, self.stamp = frame, ns, stamp
        self.array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = stamp
        clear.ns = ns
        clear.action = Marker.DELETEALL
        self.array.markers.append(clear)
        self.next_id = 0

    def _marker(self, kind, color):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.stamp
        m.ns = self.ns
        m.id = self.next_id
        self.next_id += 1
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        self.array.markers.append(m)
        return m

    def line(self, points, color, width=0.008, z=0.0):
        m = self._marker(Marker.LINE_STRIP, color)
        m.scale.x = width
        m.points = [Point(x=float(x), y=float(y), z=z) for x, y in points]
        return m

    def arrow(self, tip, color, shaft=0.035, z=0.01):
        m = self._marker(Marker.ARROW, color)
        m.scale.x, m.scale.y, m.scale.z = shaft, shaft * 2.4, shaft * 2.6
        m.points = [Point(x=0.0, y=0.0, z=z), Point(x=float(tip[0]), y=float(tip[1]), z=z)]
        return m

    def dot(self, xy, color, size):
        m = self._marker(Marker.SPHERE, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), 0.02
        m.scale.x = m.scale.y = m.scale.z = size
        return m

    def text(self, xy, words, color=WHITE, size=0.06):
        m = self._marker(Marker.TEXT_VIEW_FACING, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), 0.03
        m.scale.z = size
        m.text = words
        return m


def command_markers(frame, stamp, command, measured):
    """command = (vx, vy, wz) from /cmd_vel; measured = (vx, vy) or None."""
    k = Markers(frame, 'command', stamp)
    vx, vy, wz = command
    for r in SPEED_RINGS:  # labels on the front-right diagonal, clear of the arrows
        k.line(circle(r), GREY)
        k.text((r * 0.72, -r * 0.72), f'{r:g} m/s', LABEL, 0.068)
    k.line([(-0.55, 0.0), (0.55, 0.0)], FAINT, 0.005)
    k.line([(0.0, -0.55), (0.0, 0.55)], FAINT, 0.005)
    k.text((0.6, 0.0), 'forward', LABEL, 0.068)
    # The robot: a small chevron pointing forward.
    k.line([(-0.05, 0.05), (0.07, 0.0), (-0.05, -0.05), (-0.02, 0.0), (-0.05, 0.05)], WHITE, 0.01)
    if math.hypot(vx, vy) > 0.005:
        k.arrow((vx, vy), BLUE)
    if measured is not None and math.hypot(*measured) > 0.02:
        k.arrow(measured, GREEN, shaft=0.025, z=0.02)
    if abs(wz) > 0.01:  # arc from straight ahead, sweeping the commanded turn rate (1 rad/s)
        arc = circle(TURN_RADIUS, start=0.0, sweep=wz)
        k.line(arc, ORANGE, 0.02)
        k.dot(arc[-1], ORANGE, 0.05)
    k.text((-0.56, 0.0), f'cmd ({vx:+.2f}, {vy:+.2f}, {wz:+.2f})', BLUE, 0.068)
    if measured is not None:
        k.text((-0.65, 0.0), f'actual ({measured[0]:+.2f}, {measured[1]:+.2f})', GREEN, 0.068)
    return k.array


def lean_markers(frame, stamp, lean, trail, cutoff_deg):
    """lean = (forward_deg, left_deg) or None; trail = recent (forward, left) samples."""
    k = Markers(frame, 'lean', stamp)
    for ring in LEAN_RINGS:  # labels on the front-right diagonal
        r = ring * LEAN_SCALE
        k.line(circle(r), GREY)
        k.text((r * 0.72, -r * 0.72), f'{ring:g}°', LABEL, 0.05)
    cut = cutoff_deg * LEAN_SCALE
    k.line(circle(cut), RED, 0.012)
    k.text((cut * 0.72, cut * 0.72), f'{cutoff_deg:.0f}° fall', RED, 0.052)
    k.line([(-0.32, 0.0), (0.32, 0.0)], FAINT, 0.004)
    k.line([(0.0, -0.32), (0.0, 0.32)], FAINT, 0.004)
    k.text((0.36, 0.0), 'forward', LABEL, 0.05)
    k.text((0.0, 0.36), 'left', LABEL, 0.05)
    if lean is None:
        k.text((0.0, 0.0), 'no IMU', LABEL, 0.06)
        return k.array
    forward, left = lean
    tilt = math.hypot(forward, left)
    if len(trail) > 1:
        k.line([(f * LEAN_SCALE, l * LEAN_SCALE) for f, l in trail], (*lean_color(tilt)[:3], 0.5),
               0.006)
    shown = min(tilt, cutoff_deg * 1.1) / tilt if tilt > 0 else 1.0  # keep a fall on screen
    k.dot((forward * LEAN_SCALE * shown, left * LEAN_SCALE * shown), lean_color(tilt), 0.045)
    k.text((-0.4, 0.0), f'{tilt:.1f}°  ({forward:+.1f}, {left:+.1f})°',
           lean_color(tilt), 0.059)
    return k.array


def telemetry_markers(frame, stamp, volts, currents, nominal=False):
    """Four dials with numeric readouts; x points up and y points left in the layout.

    Use standard markers so importing the layout needs no desktop extension. The number
    stays truthful outside the dial's range; unavailable inputs remove the needle entirely.
    """
    k = Markers(frame, 'telemetry', stamp)
    # Fixed slots let Foxglove update in place instead of deleting every colored arc.
    k.array.markers.clear()
    specs = [
        (0.60, 1.0, 'BATTERY', volts, 30, 42, 2, 'V', False),
        (0.60, -1.0, 'TOTAL CURRENT', currents['total'], 0, 30, 5, 'A', True),
        (-0.60, 1.0, 'LEFT LEG', currents['left'], 0, 20, 5, 'A', True),
        (-0.60, -1.0, 'RIGHT LEG', currents['right'], 0, 20, 5, 'A', True),
    ]
    for up, left, title, value, lo, hi, tick, unit, estimated in specs:
        def xy(angle, radius):
            return (up + radius * math.sin(angle), left + radius * math.cos(angle))

        def angle(fraction):
            return math.radians(-20 + 220 * fraction)

        valid = value is not None and math.isfinite(value)
        k.text((up + 0.62, left), title, WHITE, 0.078)
        bands = [(0.0, 0.6, GREEN), (0.6, 0.85, YELLOW), (0.85, 1.0, RED)]
        if not estimated:
            bands = [(0.0, 0.15, RED), (0.15, 0.4, YELLOW), (0.4, 1.0, GREEN)]
        for start, end, color in bands:
            points = [xy(angle(start + (end - start) * i / 32), 0.43)
                      for i in range(33)]
            k.line(points, color, 0.045)
        for v in range(lo, hi + 1, tick):
            a = angle((v - lo) / (hi - lo))
            k.line([xy(a, 0.40), xy(a, 0.46)], WHITE, 0.008)
            k.text(xy(a, 0.53), str(v), LABEL, 0.059)
        fraction = max(0.0, min(1.0, (value - lo) / (hi - lo))) if valid else 0.0
        needle = k.line([(up, left), xy(angle(fraction), 0.36)], WHITE, 0.015, z=0.02)
        hub = k.dot((up, left), WHITE, 0.04)
        if not valid:
            needle.action = hub.action = Marker.DELETE
        k.text((up - 0.24, left), f'{value:.1f} {unit}' if valid else 'NO DATA',
               WHITE if valid else LABEL, 0.14 if valid else 0.09)
        note = ('EST. / 36 V NOMINAL' if nominal else 'ESTIMATED') if estimated else '10S / 30-42 V'
        if title == 'TOTAL CURRENT':
            note += '\n30 A BMS'
        if valid and not lo <= value <= hi:
            note = 'OUTSIDE DIAL RANGE'
        k.text((up - 0.40, left), note, YELLOW if estimated else LABEL, 0.055)
    # Expire the complete view if robot_monitor stops, instead of leaving plausible values.
    for marker in k.array.markers:
        marker.lifetime.sec = 1
    return k.array
