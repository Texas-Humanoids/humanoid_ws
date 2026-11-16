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
