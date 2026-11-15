"""Motor and IMU emulation shared by the sim node and its tests (numpy only, no ROS/MuJoCo)."""

import numpy as np

# Peak torque per RobStride model, as kMotorLimitsByModel in robstride_motor_node.cpp.
TORQUE_MAX = {'RS02': 17.0, 'RS03': 60.0, 'RS06': 36.0}
KP_MAX = {'RS02': 500.0, 'RS03': 5000.0, 'RS06': 5000.0}
KD_MAX = {'RS02': 5.0, 'RS03': 100.0, 'RS06': 100.0}
POSITION_MAX = 4.0 * np.pi


class MitMotors:
    """N RobStride motors in MIT mode, as robstride_motor_node drives them.

    Positions are motor space: motor = direction * joint + offset. While a target
    is fresh (younger than target_timeout) each motor runs PD toward it with
    kp/kd; otherwise kp = 0 and kd = damping_kd. Torque saturates at
    torque_limit. The motor holds its last target between messages.

    command_delay holds each target back before the motor uses it, standing in
    for the bus and ROS transport (training's DelayedDCMotor delays 0-20 ms).
    """

    def __init__(self, directions, offsets, kp, kd, damping_kd, torque_limit,
                 target_timeout=0.1, command_delay=0.0):
        self.direction = np.asarray(directions, float)
        self.offset = np.asarray(offsets, float)
        self.kp = np.asarray(kp, float)
        self.kd = np.asarray(kd, float)
        self.damping_kd = np.asarray(damping_kd, float) * np.ones_like(self.kp)
        self.torque_limit = np.asarray(torque_limit, float)
        self.target_timeout = float(target_timeout)
        self.command_delay = float(command_delay)
        if not self.command_delay >= 0.0:
            raise ValueError('command_delay must be >= 0')
        n = len(self.direction)
        for name in ('offset', 'kp', 'kd', 'damping_kd', 'torque_limit'):
            if getattr(self, name).shape != (n,):
                raise ValueError(f'{name} needs {n} entries')
        if np.any(np.abs(self.direction) != 1.0):
            raise ValueError('direction must be +1 or -1')
        self.target = np.full(n, np.nan)  # motor space; NaN until the first command
        self.target_time = np.full(n, -np.inf)  # when the motor started using the target
        self.pending = []  # (time due, index, motor target), oldest first

    def set_gains(self, index, kp, kd):
        """A motor/gains message ([kp, kd]) arrived; the next PD step uses it."""
        if not (np.isfinite(kp) and np.isfinite(kd) and kp >= 0 and kd >= 0):
            return False
        self.kp[index] = kp
        self.kd[index] = kd
        return True

    def joint_to_motor(self, q):
        return self.direction * q + self.offset

    def motor_feedback(self, q, qd):
        """Joint-space state -> (motor position, motor velocity) as the motor reports it."""
        return self.joint_to_motor(q), self.direction * qd

    def command(self, index, motor_target, now):
        """A motor/target_position message (motor space) arrived at time `now`."""
        if not np.isfinite(motor_target):
            return
        self.pending.append(
            (now + self.command_delay, index, np.clip(motor_target, -POSITION_MAX, POSITION_MAX)))

    def apply_due(self, now):
        """Hands the motors every target whose delay has passed by `now`."""
        while self.pending and self.pending[0][0] <= now:
            due, index, target = self.pending.pop(0)
            self.target[index] = target
            self.target_time[index] = due

    def active(self, now):
        return (now - self.target_time) <= self.target_timeout

    def joint_torque(self, q, qd, now):
        """Joint-space torque the motors apply for joint state (q, qd)."""
        self.apply_due(now)
        position, velocity = self.motor_feedback(q, qd)
        active = self.active(now)
        kp = np.where(active, self.kp, 0.0)
        kd = np.where(active, self.kd, self.damping_kd)
        error = np.where(active, self.target - position, 0.0)
        torque = np.clip(kp * error - kd * velocity, -self.torque_limit, self.torque_limit)
        return self.direction * torque


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q = [w, x, y, z]."""
    return quat_mul(quat_mul(q, np.array([0.0, *v])), quat_conj(q))[1:]


# Takes ENU-world vectors into NED-world (x_n = y_e, y_n = x_e, z_n = -z_e).
NED_FROM_ENU = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])


class ImuMount:
    """Turns base_link state into what the real IMU would report.

    rotation is the robot yaml imu.rotation: [w, x, y, z] taking IMU-frame
    vectors into base_link. The bridge applies it, so the sim applies the
    inverse; with world_frame 'ned' the orientation is reported against NED.
    """

    def __init__(self, rotation=(1.0, 0.0, 0.0, 0.0), world_frame='enu'):
        q = np.asarray(rotation, float)
        if q.shape != (4,) or not np.linalg.norm(q) > 1e-6:
            raise ValueError('imu rotation is [w, x, y, z]')
        if world_frame not in ('enu', 'ned'):
            raise ValueError("imu world_frame is 'enu' or 'ned'")
        self.base_from_imu = q / np.linalg.norm(q)
        self.ned = world_frame == 'ned'

    def measure(self, world_from_base, gyro_base, accel_base):
        """-> (orientation [w,x,y,z], angular velocity, linear acceleration) in the IMU frame."""
        orientation = quat_mul(np.asarray(world_from_base, float), self.base_from_imu)
        if self.ned:
            orientation = quat_mul(NED_FROM_ENU, orientation)
        imu_from_base = quat_conj(self.base_from_imu)
        return (orientation / np.linalg.norm(orientation),
                quat_rotate(imu_from_base, gyro_base),
                quat_rotate(imu_from_base, accel_base))


def projected_gravity(orientation, world_z_down=False):
    """What policy_bridge computes from the IMU orientation (for tests)."""
    return quat_rotate(quat_conj(orientation), [0.0, 0.0, 1.0 if world_z_down else -1.0])
