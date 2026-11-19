"""Turns a target's distance and bearing into a walking command (no ROS, no OpenCV).

The policy is trained on commands of forward -0.3..0.5 m/s, sideways +-0.1 m/s and
yaw +-0.5 rad/s, so every limit here stays inside those ranges, and the output is
rate-limited so the robot never sees a step from standing to full speed.

    forward = k_forward * (distance - target_distance)   (minus a deadband)
    yaw     = k_yaw * bearing                             (minus a deadband)

Forward speed fades to zero as the bearing grows to `walk_bearing`, so a target
far to the side makes the robot turn in place before walking toward it.

Detections are smoothed (the camera shakes with the gait) and a short dropout
(`hold_time`, e.g. a blurred frame) keeps the last target. After that the
target is LOST and the command ramps to zero; with `search_yaw_rate` > 0 the
robot then turns toward where the target was last seen (SEARCH).
"""

from dataclasses import dataclass
import math


@dataclass
class FollowParams:
    target_distance: float = 1.0   # m, where the robot stops
    distance_deadband: float = 0.1  # m, no forward motion within this of target_distance
    k_forward: float = 0.6         # (m/s) per m of distance error
    max_forward: float = 0.4       # m/s (training max 0.5)
    max_backward: float = 0.15     # m/s backing away when too close; 0 = never (training max 0.3)
    k_yaw: float = 1.2             # (rad/s) per rad of bearing
    yaw_deadband: float = 0.05     # rad
    max_yaw_rate: float = 0.5      # rad/s (training max 0.5)
    walk_bearing: float = 0.6      # rad; at this bearing or more, turn in place
    max_linear_accel: float = 0.5  # m/s^2
    max_yaw_accel: float = 1.5     # rad/s^2
    smoothing: float = 0.4         # weight of each new detection, (0, 1]; 1 = no smoothing
    hold_time: float = 0.3         # s a lost target is still followed
    search_yaw_rate: float = 0.0   # rad/s turned while searching; 0 = just stop
    search_after: float = 1.0      # s after the target was last seen

    def validate(self):
        if not 0.0 < self.smoothing <= 1.0:
            raise ValueError('smoothing must be in (0, 1]')
        for name in ('target_distance', 'distance_deadband', 'k_forward', 'max_forward',
                     'max_backward', 'k_yaw', 'yaw_deadband', 'max_yaw_rate', 'hold_time',
                     'search_yaw_rate', 'search_after'):
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must be >= 0')
        for name in ('walk_bearing', 'max_linear_accel', 'max_yaw_accel'):
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be > 0')
        if self.search_yaw_rate > self.max_yaw_rate:
            raise ValueError('search_yaw_rate must be <= max_yaw_rate')


TRACKING = 'TRACKING'  # target seen within hold_time
LOST = 'LOST'          # stopping
SEARCH = 'SEARCH'      # turning toward where the target was last seen


def _deadband(value, band):
    if abs(value) <= band:
        return 0.0
    return value - math.copysign(band, value)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _step_toward(current, target, max_step):
    return current + _clamp(target - current, -max_step, max_step)


class FollowController:

    def __init__(self, params=None):
        self.params = params or FollowParams()
        self.params.validate()
        self.reset()

    def reset(self):
        """Forget the target and start the output from standstill."""
        self.distance = None
        self.bearing = None
        self.last_seen = None
        self.reset_output()

    def reset_output(self):
        """Start the output from standstill, keeping the target."""
        self.last_update = None
        self.forward = 0.0
        self.yaw = 0.0

    def observe(self, distance, bearing, t):
        """A detection at time t (s): horizontal distance (m) and bearing (rad, left positive)."""
        p = self.params
        fresh = self.last_seen is None or t - self.last_seen > p.hold_time
        if fresh:
            self.distance, self.bearing = distance, bearing
        else:
            self.distance += p.smoothing * (distance - self.distance)
            self.bearing += p.smoothing * (bearing - self.bearing)
        self.last_seen = t

    def state(self, t):
        p = self.params
        if self.last_seen is not None and t - self.last_seen <= p.hold_time:
            return TRACKING
        if (p.search_yaw_rate > 0.0 and self.bearing is not None
                and t - self.last_seen >= p.search_after):
            return SEARCH
        return LOST

    def desired(self, t):
        """(forward m/s, yaw rad/s) before rate limiting, and the state."""
        p = self.params
        state = self.state(t)
        if state == LOST:
            return 0.0, 0.0, state
        if state == SEARCH:
            return 0.0, math.copysign(p.search_yaw_rate, self.bearing), state
        error = _deadband(self.distance - p.target_distance, p.distance_deadband)
        forward = _clamp(p.k_forward * error, -p.max_backward, p.max_forward)
        forward *= max(0.0, 1.0 - abs(self.bearing) / p.walk_bearing)
        yaw = _clamp(p.k_yaw * _deadband(self.bearing, p.yaw_deadband),
                     -p.max_yaw_rate, p.max_yaw_rate)
        return forward, yaw, state

    def update(self, t):
        """The rate-limited command at time t: (forward m/s, yaw rad/s, state)."""
        p = self.params
        forward, yaw, state = self.desired(t)
        dt = 0.0 if self.last_update is None else _clamp(t - self.last_update, 0.0, 0.2)
        self.last_update = t
        self.forward = _step_toward(self.forward, forward, p.max_linear_accel * dt)
        self.yaw = _step_toward(self.yaw, yaw, p.max_yaw_accel * dt)
        return self.forward, self.yaw, state
