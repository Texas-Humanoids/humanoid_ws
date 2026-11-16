"""Pure math for the dashboard: battery estimate, base orientation, power. No ROS imports."""

import math
import statistics

# Resting voltage per cell (V) vs. state of charge (%) for a Li-ion/LiPo cell. Under load the
# voltage sags, so the estimate reads low while the robot is working hard.
CELL_CURVE = [
    (3.27, 0.0), (3.61, 5.0), (3.69, 10.0), (3.71, 15.0), (3.73, 20.0), (3.75, 25.0),
    (3.77, 30.0), (3.79, 35.0), (3.80, 40.0), (3.82, 45.0), (3.84, 50.0), (3.85, 55.0),
    (3.87, 60.0), (3.91, 65.0), (3.95, 70.0), (3.98, 75.0), (4.02, 80.0), (4.08, 85.0),
    (4.11, 90.0), (4.15, 95.0), (4.20, 100.0),
]

# Rotation taking NED world vectors into ENU: swap x and y, flip z.
NED_TO_ENU = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)


def battery_fraction(volts, cells):
    """State of charge 0..1 from pack voltage, or NaN when unknown (no cell count or reading)."""
    if not cells or cells <= 0 or volts is None or not math.isfinite(volts):
        return math.nan
    cell = volts / cells
    if cell <= CELL_CURVE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(CELL_CURVE, CELL_CURVE[1:]):
        if cell <= v1:
            return (p0 + (p1 - p0) * (cell - v0) / (v1 - v0)) / 100.0
    return 1.0


def pack_voltage(readings):
    """All motors share the supply, so take the median of their readings (robust to one bad)."""
    values = [v for v in readings if v is not None and math.isfinite(v)]
    return statistics.median(values) if values else math.nan


def mechanical_power(torques, velocities):
    """Total |torque * velocity| over the joints, in W: the work the motors are doing.

    Motor space works directly: direction is +/-1 on both torque and velocity, so it cancels.
    Electrical draw is higher (copper and driver losses), and braking does not recharge much,
    so the absolute value is the useful number.
    """
    return sum(abs(t * w) for t, w in zip(torques, velocities)
               if t is not None and w is not None and math.isfinite(t * w))


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('not a valid quaternion')
    return tuple(v / norm for v in q)


def quat_rotate(q, v):
    w, x, y, z = quat_multiply(quat_multiply(q, (0.0, *v)), quat_conjugate(q))
    return (x, y, z)


def base_orientation(imu_orientation, imu_to_base, world_ned=False):
    """Orientation of base_link in an ENU world from the IMU, all quaternions [w, x, y, z].

    Same conventions as policy_bridge: imu_orientation rotates IMU-frame vectors into the
    world, and imu_to_base rotates IMU-frame vectors into base_link. So
    world <- base = (world <- imu) * (imu <- base) = imu_orientation * conj(imu_to_base).
    """
    q = quat_multiply(quat_normalize(imu_orientation), quat_conjugate(quat_normalize(imu_to_base)))
    if world_ned:
        q = quat_multiply(NED_TO_ENU, q)
    return quat_normalize(q)


def tilt(orientation):
    """Angle (rad) between base_link's z axis and the world's up axis."""
    w, x, y, z = quat_normalize(orientation)
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))
