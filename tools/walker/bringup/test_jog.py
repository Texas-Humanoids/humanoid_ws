"""jog.py against simulated RobStride motors (chirp.SimMotor). No hardware, no curses.

  python -m pytest tools/walker/bringup/test_jog.py -q
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import jog  # noqa: E402
import rsproto as rs  # noqa: E402

PERIOD = 1.0 / jog.RATE


def make(*extra):
    a = jog.parse_args(["--sim", *extra])
    motors, buses, clock, missing = jog.build(a)
    b = jog.Bringup(motors, buses, clock, a)
    b._t = clock.now() + PERIOD
    run(b, 0.2)  # first pings -> feedback
    return b


def run(b, seconds):
    end = b.clock.now() + seconds
    while b.clock.now() < end:
        b._t = jog.loop_once(b, b._t, PERIOD, sim=False)


def sim_of(b, m):
    return next(s for s in b.buses["sim"].motors if s.id == m.id)


def test_disabled_motors_report_position():
    b = make()
    assert len(b.motors) == 10
    assert all(m.st is not None and not m.enabled for m in b.motors)
    assert all(m.seen is not None for m in b.motors)


def test_enable_holds_and_jog_moves_within_window():
    b = make("--window", "0.3", "--max-speed", "1.0")
    m = b.cur
    b.enable(m)
    assert m.enabled and m.state == "hold"
    q0 = m.st.q
    run(b, 0.5)
    assert abs(m.st.q - q0) < 0.02                      # holds
    b.set_speed(0.5)
    run(b, 0.4)
    assert m.st.q - q0 == pytest.approx(0.2, abs=0.05)   # 0.5 rad/s for 0.4 s
    run(b, 1.5)                                          # runs into the +0.3 window
    assert m.q_ref == pytest.approx(q0 + 0.3, abs=1e-6)
    assert m.flag == "AT WINDOW"
    b.set_speed(-1.0)
    run(b, 1.0)
    assert m.q_ref == pytest.approx(q0 - 0.3, abs=1e-6)
    assert not b.estop_reason


def test_only_selected_motor_jogs():
    b = make()
    for m in b.motors[:2]:
        b.enable(m)
    other = b.motors[1]
    q_other = other.q_ref
    b.set_speed(0.5)
    run(b, 0.5)
    assert other.q_ref == q_other
    assert abs(b.motors[0].q_ref - b.motors[0].q_en) > 0.2


def test_pulse_then_answer_sets_direction():
    b = make()
    m = b.cur
    b.start_pulse()
    run(b, 0.9)
    assert m.st.q - m.q_en > 0.12                        # went + by ~0.15
    run(b, 1.5)
    assert b.asking is m and abs(m.st.q - m.q_en) < 0.03  # came back, asking
    b.answer(False)
    assert m.direction == -1.0 and m.dir_checked and b.asking is None


def test_calibrate_needs_checked_directions_then_maps_joint_frame():
    b = make()
    b.calibrate({})
    assert all(m.offset is None for m in b.motors)       # refused: directions unchecked
    for m in b.motors:
        m.dir_checked = True
    b.motors[3].direction = -1.0
    b.calibrate({m.joint: 0.0 for m in b.motors})
    for m in b.motors:
        assert m.to_joint(m.st.q) == pytest.approx(0.0, abs=1e-3)
    m = b.motors[3]
    assert m.to_motor(0.1) == pytest.approx(m.offset - 0.1)


def test_yaml_limits_clamp_jog_until_unlocked():
    b = make()
    m = b.cur
    for x in b.motors:
        x.dir_checked = True
    b.calibrate({x.joint: 0.0 for x in b.motors})
    m.lower, m.upper = -0.1, 0.1
    b.enable(m)
    b.set_speed(1.0)
    run(b, 0.6)
    assert m.to_joint(m.q_ref) == pytest.approx(0.1, abs=1e-6)
    b.unlocked = True
    run(b, 0.3)
    assert m.to_joint(m.q_ref) > 0.25


def test_stall_at_hard_stop_freezes_target():
    b = make("--window", "1.0")
    m = b.cur
    s = sim_of(b, m)
    stop_at = s.q + 0.2
    orig = s.advance

    def advance(t):                                       # a hard stop 0.2 rad above the start
        orig(t)
        if s.q > stop_at:
            s.q, s.v = stop_at, min(s.v, 0.0)
    s.advance = advance
    b.enable(m)
    b.set_speed(0.5)
    run(b, 1.5)
    assert m.flag == "STALL"
    assert b.speed == 0.0 and abs(m.q_ref - stop_at) < 0.05
    assert m.enabled and not b.estop_reason               # holds, doesn't trip
    assert m.seen[1] == pytest.approx(stop_at, abs=0.01)  # recorded as the range edge


def test_lost_feedback_trips_everything():
    b = make()
    for m in b.motors[:3]:
        b.enable(m)
    sim_of(b, b.motors[2]).mute = True
    run(b, 0.3)
    assert "no feedback" in (b.estop_reason or "")
    assert not any(m.enabled for m in b.motors)


def test_fault_trips_everything():
    b = make()
    for m in b.motors[:2]:
        b.enable(m)
    s = sim_of(b, b.motors[1])
    s.fault_at = (b.clock.now() + 0.05, 0b100)            # overtemperature
    run(b, 0.3)
    assert b.estop_reason and not any(m.enabled for m in b.motors)


def test_go_pose_and_squat_track():
    b = make("--window", "1.0")
    for m in b.motors:
        m.dir_checked = True
    b.calibrate({m.joint: 0.0 for m in b.motors})
    for m in b.motors:
        b.enable(m)
    pose = {m.joint: 0.0 for m in b.motors}
    pose.update({"left_knee_pitch_joint": 0.4, "right_knee_pitch_joint": 0.4,
                 "left_hip_pitch_joint": -0.2, "right_hip_pitch_joint": -0.2})
    b.go_pose(pose, duration=1.0)
    run(b, 1.5)
    knee = next(m for m in b.motors if m.joint == "left_knee_pitch_joint")
    assert knee.to_joint(knee.st.q) == pytest.approx(0.4, abs=0.03)
    b.squat(pose, depth=0.2, period=1.0, cycles=1)
    run(b, 0.5)
    assert knee.to_joint(knee.st.q) == pytest.approx(0.6, abs=0.05)
    run(b, 0.7)
    assert knee.to_joint(knee.st.q) == pytest.approx(0.4, abs=0.03) and knee.ramp is None
    assert not b.estop_reason


def test_write_updates_yaml(tmp_path):
    src = os.path.join(os.path.dirname(__file__), "..", "..", "..", "src", "th_bringup", "config", "walker_demo_robot.yaml")
    y = tmp_path / "robot.yaml"
    y.write_text(open(src).read())
    b = make("--robot", str(y))
    for m in b.motors:
        m.dir_checked = True
    b.motors[0].direction = -1.0
    b.calibrate({m.joint: 0.0 for m in b.motors})
    b.write()
    import yaml
    out = yaml.safe_load(y.read_text())
    m0 = next(m for m in out["motors"] if m["joint"] == b.motors[0].joint)
    assert m0["direction"] == -1.0 and m0["offset"] == pytest.approx(b.motors[0].offset, abs=1e-4)
    assert (tmp_path / "bringup_session.json").exists()
