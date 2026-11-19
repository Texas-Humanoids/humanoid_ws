import math

import pytest

from th_autonomy.follow_controller import LOST, SEARCH, TRACKING, FollowController, FollowParams


def settled(params, distance, bearing, seconds=5.0, dt=0.05):
    """The command after following a fixed target long enough to finish ramping."""
    c = FollowController(params)
    t = 0.0
    while t < seconds:
        c.observe(distance, bearing, t)
        out = c.update(t)
        t += dt
    return out


def test_walks_toward_far_target_and_clamps():
    forward, yaw, state = settled(FollowParams(), 5.0, 0.0)
    assert state == TRACKING
    assert forward == pytest.approx(FollowParams().max_forward)
    assert yaw == 0.0


def test_stops_inside_deadband():
    p = FollowParams()
    assert settled(p, p.target_distance + 0.09, 0.02)[:2] == (0.0, 0.0)


def test_proportional_outside_deadband():
    p = FollowParams()
    forward, _, _ = settled(p, p.target_distance + 0.3, 0.0)
    assert forward == pytest.approx(p.k_forward * 0.2)


def test_backs_up_when_too_close():
    p = FollowParams()
    forward, _, _ = settled(p, 0.2, 0.0)
    assert forward == pytest.approx(-p.max_backward)
    p.max_backward = 0.0
    assert settled(p, 0.2, 0.0)[0] == 0.0


def test_turns_left_toward_left_target():
    p = FollowParams()
    _, yaw, _ = settled(p, 1.0, 0.2)
    assert yaw == pytest.approx(p.k_yaw * (0.2 - p.yaw_deadband))
    _, yaw, _ = settled(p, 1.0, -1.0)
    assert yaw == pytest.approx(-p.max_yaw_rate)


def test_turns_in_place_when_target_far_to_the_side():
    p = FollowParams()
    forward, yaw, _ = settled(p, 4.0, p.walk_bearing + 0.1)
    assert forward == 0.0 and yaw > 0.0
    half, _, _ = settled(p, 4.0, p.walk_bearing / 2)
    assert half == pytest.approx(p.max_forward / 2)


def test_output_ramps_from_standstill():
    p = FollowParams()
    c = FollowController(p)
    c.observe(5.0, 1.0, 0.0)
    assert c.update(0.0)[:2] == (0.0, 0.0)
    c.observe(5.0, 1.0, 0.1)
    forward, yaw, _ = c.update(0.1)
    assert yaw == pytest.approx(p.max_yaw_accel * 0.1)
    assert forward == 0.0  # bearing beyond walk_bearing


def test_long_gap_does_not_jump():
    c = FollowController(FollowParams())
    c.observe(5.0, 0.0, 0.0)
    c.update(0.0)
    c.observe(5.0, 0.0, 10.0)
    forward, _, _ = c.update(10.0)
    assert forward <= FollowParams().max_linear_accel * 0.2 + 1e-9


def test_holds_through_short_dropout_then_stops():
    p = FollowParams()
    c = FollowController(p)
    t = 0.0
    while t < 3.0:
        c.observe(5.0, 0.0, t)
        c.update(t)
        t += 0.05
    last_seen = t - 0.05
    assert c.update(last_seen + p.hold_time - 0.01)[2] == TRACKING
    assert c.update(last_seen + p.hold_time + 0.05)[2] == LOST
    t = last_seen + p.hold_time + 0.05
    while t < last_seen + 5.0:
        forward, yaw, state = c.update(t)
        t += 0.05
    assert (forward, yaw, state) == (0.0, 0.0, LOST)


def test_smoothing_averages_noisy_bearing():
    p = FollowParams(smoothing=0.25)
    c = FollowController(p)
    for i in range(200):
        c.observe(1.0, 0.3 if i % 2 else -0.3, i * 0.05)
    assert abs(c.bearing) < 0.1


def test_search_turns_toward_last_side():
    p = FollowParams(search_yaw_rate=0.25, search_after=1.0)
    c = FollowController(p)
    c.observe(2.0, -0.4, 0.0)
    assert c.desired(0.5) == (0.0, 0.0, LOST)
    assert c.desired(1.5) == (0.0, -0.25, SEARCH)


def test_reset_output_keeps_target():
    c = FollowController(FollowParams())
    c.observe(5.0, 0.0, 0.0)
    c.update(0.0)
    c.update(0.1)
    c.reset_output()
    assert (c.forward, c.yaw) == (0.0, 0.0)
    assert c.state(0.1) == TRACKING


@pytest.mark.parametrize('field,value', [('smoothing', 0.0), ('walk_bearing', 0.0),
                                         ('k_forward', -1.0), ('search_yaw_rate', 1.0)])
def test_rejects_bad_params(field, value):
    with pytest.raises(ValueError):
        FollowController(FollowParams(**{field: value}))


def test_limits_stay_inside_training_ranges():
    p = FollowParams()
    assert p.max_forward <= 0.5 and p.max_backward <= 0.3 and p.max_yaw_rate <= 0.5
    assert not math.isnan(settled(p, 1e6, 3.0)[0])
