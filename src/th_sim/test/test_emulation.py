"""Motor and IMU emulation against what robstride_motor_node and policy_bridge expect."""

import numpy as np
import pytest

from th_sim.emulation import ImuMount, MitMotors, projected_gravity, quat_rotate


def motors(direction=1.0, offset=0.0):
    return MitMotors([direction], [offset], kp=[60.0], kd=[3.0], damping_kd=1.0,
                     torque_limit=[60.0])


def test_damps_until_a_target_arrives():
    m = motors()
    # kp = 0, kd = damping_kd: only velocity matters.
    assert m.joint_torque(np.array([0.5]), np.array([2.0]), now=0.0) == pytest.approx([-2.0])


def test_pd_while_fresh_then_damping_after_timeout():
    m = motors()
    m.command(0, 0.3, now=10.0)
    tau = m.joint_torque(np.array([0.1]), np.array([0.5]), now=10.05)
    assert tau == pytest.approx([60.0 * 0.2 - 3.0 * 0.5])
    tau = m.joint_torque(np.array([0.1]), np.array([0.5]), now=10.11)
    assert tau == pytest.approx([-1.0 * 0.5])


def test_command_delay_holds_the_target_back():
    m = MitMotors([1.0], [0.0], kp=[60.0], kd=[3.0], damping_kd=1.0, torque_limit=[60.0],
                  command_delay=0.01)
    m.command(0, 0.3, now=10.0)
    still = np.array([0.0])
    assert m.joint_torque(still, still, now=10.005) == pytest.approx([0.0])  # still damping
    assert m.joint_torque(still, still, now=10.01) == pytest.approx([60.0 * 0.3])
    # Freshness counts from when the motor got the target, not when it was sent.
    assert m.joint_torque(still, still, now=10.105) == pytest.approx([60.0 * 0.3])


def test_torque_saturates():
    m = motors()
    m.command(0, 3.0, now=0.0)
    assert m.joint_torque(np.array([0.0]), np.array([0.0]), now=0.0) == pytest.approx([60.0])


def test_reversed_motor_with_offset():
    # motor = -joint + 1. The bridge sends joint target 0.4 as motor 0.6.
    m = motors(direction=-1.0, offset=1.0)
    position, velocity = m.motor_feedback(np.array([0.4]), np.array([0.2]))
    assert position == pytest.approx([0.6]) and velocity == pytest.approx([-0.2])
    m.command(0, 1.0 - 0.5, now=0.0)  # joint target 0.5
    tau = m.joint_torque(np.array([0.4]), np.array([0.0]), now=0.0)
    # Joint-space PD pulls the joint toward 0.5 regardless of the motor direction.
    assert tau == pytest.approx([60.0 * 0.1])


def test_imu_identity_mount_gives_base_gravity():
    imu = ImuMount()
    # Base pitched nose-down by 0.3 rad (positive rotation about y).
    q = np.array([np.cos(0.15), 0.0, np.sin(0.15), 0.0])
    orientation, gyro, _ = imu.measure(q, [0.0, 0.1, 0.0], [0.0, 0.0, 9.81])
    g = projected_gravity(orientation)
    assert g[0] == pytest.approx(np.sin(0.3))  # nose down reads +x, as the README checklist says
    assert gyro == pytest.approx([0.0, 0.1, 0.0])


@pytest.mark.parametrize('world_frame', ['enu', 'ned'])
def test_rotated_mount_round_trips_through_the_bridge(world_frame):
    # IMU mounted yawed 90 degrees and upside down; the bridge rotates back with imu.rotation.
    rotation = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])
    imu = ImuMount(rotation, world_frame)
    q_base = np.array([np.cos(0.1), np.sin(0.1), 0.0, 0.0])  # rolled 0.2 rad
    gyro_base = np.array([0.3, -0.2, 0.1])
    orientation, gyro, _ = imu.measure(q_base, gyro_base, [0.0, 0.0, 9.81])
    # policy_bridge: gravity_base = rotate(imu_rotation, projected_gravity(orientation, ned)).
    g = quat_rotate(rotation, projected_gravity(orientation, world_frame == 'ned'))
    assert g == pytest.approx(projected_gravity(q_base))
    assert quat_rotate(rotation, gyro) == pytest.approx(gyro_base)


def test_gain_change_takes_effect_on_the_next_step():
    m = MitMotors([1.0], [0.0], [30.0], [1.5], 1.0, [36.0])
    m.command(0, 0.1, now=0.0)
    assert m.joint_torque([0.0], [0.0], now=0.01)[0] == pytest.approx(3.0)  # 30 * 0.1
    assert m.set_gains(0, 80.0, 3.0)
    assert m.joint_torque([0.0], [0.0], now=0.02)[0] == pytest.approx(8.0)  # 80 * 0.1
    assert not m.set_gains(0, float('nan'), 1.0)
