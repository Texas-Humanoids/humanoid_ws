import math
import random

import pytest

from th_foxglove import telemetry as t
from th_foxglove.mjpeg import MjpegParser


def random_quat(rng):
    return t.quat_normalize(tuple(rng.gauss(0, 1) for _ in range(4)))


@pytest.mark.parametrize('ned', [False, True])
def test_base_orientation_matches_the_bridges_projected_gravity(ned):
    """policy_bridge computes g_base = imu_to_base * conj(q_imu) * g_world. The pose the
    dashboard shows must tilt the same way the policy sees it."""
    rng = random.Random(1)
    for _ in range(50):
        q_imu, imu_to_base = random_quat(rng), random_quat(rng)
        g_world = (0.0, 0.0, 1.0 if ned else -1.0)
        bridge = t.quat_rotate(imu_to_base, t.quat_rotate(t.quat_conjugate(q_imu), g_world))
        q_base = t.base_orientation(q_imu, imu_to_base, ned)
        ours = t.quat_rotate(t.quat_conjugate(q_base), (0.0, 0.0, -1.0))  # ENU world
        assert ours == pytest.approx(bridge, abs=1e-9)


def test_tilt():
    assert t.tilt((1.0, 0.0, 0.0, 0.0)) == pytest.approx(0.0)
    half = math.radians(30) / 2
    assert math.degrees(t.tilt((math.cos(half), math.sin(half), 0.0, 0.0))) == pytest.approx(30)
    # Yaw is not tilt.
    assert t.tilt((math.cos(1.0), 0.0, 0.0, math.sin(1.0))) == pytest.approx(0.0, abs=1e-7)


def test_lean_signs_match_projected_gravity():
    half = math.radians(10) / 2
    # Nose down = rotate +10 deg about +y.
    forward, left = t.lean((math.cos(half), 0.0, math.sin(half), 0.0))
    assert forward == pytest.approx(10.0)
    assert left == pytest.approx(0.0, abs=1e-9)
    # Left side down = rotate -10 deg about +x.
    forward, left = t.lean((math.cos(half), -math.sin(half), 0.0, 0.0))
    assert left == pytest.approx(10.0)


def test_battery_fraction():
    assert math.isnan(t.battery_fraction(48.0, 0))
    assert math.isnan(t.battery_fraction(math.nan, 12))
    assert t.battery_fraction(12 * 4.2, 12) == pytest.approx(1.0)
    assert t.battery_fraction(12 * 4.3, 12) == 1.0
    assert t.battery_fraction(12 * 2.9, 12) == 0.0
    # The team's 36 V 10S pack: nominal 36 V is about a third, 42 V full.
    assert t.battery_fraction(36.0, 10) == pytest.approx(0.30)
    assert t.battery_fraction(42.0, 10) == pytest.approx(1.0)
    assert t.battery_fraction(37.5, 10) == pytest.approx(0.525)


def test_pack_voltage_and_power():
    assert t.pack_voltage([47.9, 48.1, None, 10.0, 48.0]) == pytest.approx(47.95)
    assert math.isnan(t.pack_voltage([None, math.nan]))
    # A reversed motor flips both torque and velocity; power is |tau * w| either way.
    assert t.mechanical_power([2.0, -3.0, None], [1.5, 2.0, 1.0]) == pytest.approx(9.0)


def jpeg(body):
    return b'\xff\xd8' + body + b'\xff\xd9'


def test_mjpeg_parser_uses_content_length_across_chunks():
    # The first image contains an end-of-image marker inside, as an EXIF thumbnail would.
    images = [jpeg(b'thumb\xff\xd9more'), jpeg(b'second')]
    stream = b''.join(
        b'--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(i) + i +
        b'\r\n' for i in images)
    parser = MjpegParser()
    frames = []
    for k in range(0, len(stream), 7):
        frames += parser.feed(stream[k:k + 7])
    assert frames == images


def test_mjpeg_parser_falls_back_to_markers():
    parser = MjpegParser()
    assert parser.feed(b'--b\r\n\r\n' + jpeg(b'one')[:4]) == []
    assert parser.feed(jpeg(b'one')[4:] + b'\r\n--b\r\n\r\n' + jpeg(b'two')) == [
        jpeg(b'one'), jpeg(b'two')]


def test_diagnostic_levels_round_trip():
    from th_foxglove.robot_monitor import level_of, status, WARN
    msg = status('robot: battery', WARN, 'LOW', {'voltage_v': 40.1})
    assert msg.level == b'\x01'
    assert level_of(msg) == WARN
    assert [(kv.key, kv.value) for kv in msg.values] == [('voltage_v', '40.1')]


def test_dashboard_geometry():
    from th_foxglove import dashboard as d
    q = (math.cos(0.3), 0.0, 0.0, math.sin(0.3))  # 0.6 rad yaw
    assert d.yaw_of(q) == pytest.approx(0.6)
    arc = d.circle(1.0, start=0.0, sweep=math.pi / 2)
    assert arc[0] == pytest.approx((1.0, 0.0)) and arc[-1] == pytest.approx((0.0, 1.0), abs=1e-9)
    # Walking forward at 0.4 m/s while facing +y in the world reads as forward.
    v = d.VelocityEstimate()
    for i in range(20):
        out = v.update(i * 0.02, 0.0, 0.4 * i * 0.02, math.pi / 2)
    assert out == pytest.approx((0.4, 0.0), abs=1e-9)


def test_dashboard_markers_build():
    from th_foxglove import dashboard as d
    from builtin_interfaces.msg import Time
    cmd = d.command_markers('base_footprint', Time(), (0.3, 0.0, 0.4), (0.25, 0.01))
    from visualization_msgs.msg import Marker
    kinds = [m.type for m in cmd.markers if m.action == Marker.ADD]
    assert kinds.count(Marker.ARROW) == 2  # command and measured
    assert cmd.markers[0].action == Marker.DELETEALL  # clears last update's leftovers
    assert [m.id for m in cmd.markers[1:]] == list(range(len(cmd.markers) - 1))
    lean = d.lean_markers('base_footprint', Time(), (80.0, 0.0), [(0, 0), (80.0, 0.0)], 57.3)
    dot = [m for m in lean.markers if m.type == Marker.SPHERE][-1]
    assert dot.pose.position.x == pytest.approx(57.3 * 1.1 * d.LEAN_SCALE)  # a fall stays on screen


def test_measured_velocity_averages_out_step_sway():
    from th_foxglove import dashboard as d
    v = d.VelocityEstimate(window=1.4)
    # 0.3 m/s forward plus a 0.1 m side-to-side sway every 0.7 s step cycle.
    for i in range(200):
        t = i * 0.02
        out = v.update(t, 0.3 * t, 0.05 * math.sin(2 * math.pi * t / 0.7), 0.0)
    assert out[0] == pytest.approx(0.3, abs=0.01)
    assert abs(out[1]) < 0.02  # the sway (peak 0.45 m/s sideways) is gone


def test_smoothed_pose_glides_and_wraps_heading():
    from th_foxglove import dashboard as d
    s = d.SmoothedPose(tau=0.8)
    s.update(0.0, 0.0, 0.0, math.pi - 0.05)
    x, _, yaw = s.update(0.02, 1.0, 0.0, -math.pi + 0.05)  # heading crosses +/-pi
    assert 0.0 < x < 0.05  # a step jump moves the camera only a little
    assert abs(abs(yaw) - math.pi) < 0.06  # went the short way round, not through 0


def test_battery_current_estimate():
    # 2 motors doing 36 W of work on a 36 V bus at 85% efficiency.
    assert t.battery_current([6.0, -3.0], [3.0, -6.0], 36.0, 0.85) == pytest.approx(
        36.0 / (36.0 * 0.85))
    assert math.isnan(t.battery_current([1.0], [1.0], math.nan))


def test_health_summary():
    from th_foxglove.robot_monitor import ERROR, OK, STALE, WARN, health_summary, status
    ok = [status('motor: a', OK, ''), status('robot: imu', OK, '')]
    sim_battery = status('robot: battery', STALE, 'no voltage reading')
    disarmed = status('robot: policy_bridge', WARN, 'DISARMED')
    assert health_summary(ok + [sim_battery, disarmed]) == 'OK'
    assert health_summary(ok + [status('motor: b', ERROR, 'overtemp')]) == 'FAULT'
    assert health_summary(ok + [status('motor: b', STALE, 'no feedback')]) == 'WARNING'
    assert health_summary([sim_battery]) == 'NO DATA'


@pytest.mark.parametrize('torques,velocities', [
    ([], []), ([1.0, None], [1.0, 1.0]), ([1.0], [None]),
    ([math.nan], [1.0]), ([1.0], [math.inf]), ([1.0, 1.0], [1.0]),
])
def test_current_does_not_present_missing_feedback_as_low_load(torques, velocities):
    assert math.isnan(t.battery_current(torques, velocities, 36.0))


def test_telemetry_dials_missing_and_out_of_range():
    from builtin_interfaces.msg import Time
    from visualization_msgs.msg import Marker
    from th_foxglove.dashboard import telemetry_markers
    msg = telemetry_markers('dashboard', Time(), math.nan,
                            {'left': math.nan, 'right': 2.34, 'total': 35.67}, nominal=True)
    added = [m for m in msg.markers if m.action == Marker.ADD]
    text = [m.text for m in added if m.type == Marker.TEXT_VIEW_FACING]
    assert text.count('NO DATA') == 2
    assert '35.7 A' in text and '2.3 A' in text
    assert 'OUTSIDE DIAL RANGE' in text
    # Only available measurements get needle hubs; invalid needles do not sit at zero.
    assert len([m for m in added if m.type == Marker.SPHERE]) == 2
    assert all(m.lifetime.sec == 1 for m in added)
    assert all(m.action != Marker.DELETEALL for m in msg.markers)
    recovered = telemetry_markers('dashboard', Time(), 38.0,
                                  {'left': 1.0, 'right': 2.0, 'total': 3.0})
    assert [m.id for m in msg.markers] == [m.id for m in recovered.markers]
    # Range colors and arc positions stay fixed through value changes and dropouts.
    arcs = [m for m in msg.markers if m.type == Marker.LINE_STRIP and m.scale.x == 0.045]
    recovered_arcs = [m for m in recovered.markers
                      if m.type == Marker.LINE_STRIP and m.scale.x == 0.045]
    assert len(arcs) == len(recovered_arcs) == 12
    assert [(m.points, m.color) for m in arcs] == [
        (m.points, m.color) for m in recovered_arcs]


def test_summary_clears_stale_voltage_and_partial_current():
    from th_foxglove.robot_monitor import RobotMonitor, Motor

    class Publisher:
        def publish(self, msg):
            self.last = msg

    class Stamp:
        def to_msg(self):
            from builtin_interfaces.msg import Time
            return Time()

    class Clock:
        def now(self):
            return Stamp()

    # Exercise the real summary callback without a ROS executor or live topics.
    from types import SimpleNamespace
    monitor = SimpleNamespace(
        cells=10, nominal_voltage=36.0, efficiency=0.85, base_current=0.0,
        motors=[Motor('left_a', 'left'), Motor('right_a', 'right')],
        voltage=lambda: math.nan, get_clock=lambda: Clock(),
        base_pose=lambda: ((0, 0, 0), (1, 0, 0, 0), 'none'),
        current_pubs={s: Publisher() for s in ('left', 'right', 'total')},
        poll=lambda group: None)  # sampled inputs: nothing to take here
    for name in ('voltage', 'percent', 'power', 'battery', 'temperature', 'telemetry'):
        setattr(monitor, name + '_pub', Publisher())
    monitor.motors[0].torque.set(3.0)
    monitor.motors[0].velocity.set(2.0)
    RobotMonitor.publish_summary(monitor)
    assert math.isnan(monitor.voltage_pub.last.data)
    assert math.isnan(monitor.percent_pub.last.data)
    assert monitor.current_pubs['left'].last.data > 0
    assert math.isnan(monitor.current_pubs['right'].last.data)
    assert math.isnan(monitor.current_pubs['total'].last.data)
    assert math.isnan(monitor.power_pub.last.data)
    assert math.isnan(monitor.battery_pub.last.current)


def test_cpu_percent_from_proc_stat():
    a = t.cpu_times('cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 1 2 3 4\n')
    b = t.cpu_times('cpu  150 0 150 900 0 0 0 0 0 0\n')
    assert a == (200, 1000) and b == (300, 1200)
    assert t.cpu_percent(a, b) == pytest.approx(50.0)
    assert math.isnan(t.cpu_percent(b, b))
    with pytest.raises(ValueError):
        t.cpu_times('intr 1 2 3')
