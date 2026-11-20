"""Demo walker flat-ground velocity tracking (mjlab). Derived from the WalkerV2 v2 task, which
passed sim2sim 9/9, with the demo walker's model, measured actuators and a demo-sized command range.

Changes from walker_v2_velocity, and why:
  * robot: walker_demo (16.9 kg, CAD or wide joint limits), measured armature/friction/speed.
  * scalar action scale 0.25 (the deploy bridge only takes a scalar).
  * swing height 7 -> 5 cm: with the CAD knee stop at 60 deg, 7 cm needs ~1.02 rad of knee, which
    is past the 0.95 soft limit. 5 cm needs ~0.87 rad.
  * commands sized for a demo, not a speed run: vx -0.3..0.6, vy +-0.2, wz +-0.8. Narrower range,
    better tracking where we'll actually drive it.
  * domain randomization narrowed where sysid measured the value (armature, friction) and widened
    where the CAD is least trustworthy (base mass: wiring/fasteners aren't in CAD; encoder bias:
    our offset calibration is by eye).
"""
from __future__ import annotations

import dataclasses
import math

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RingPatternCfg, TerrainHeightSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from th_training.robots.walker_demo import ACTION_SCALE, FOOT_GEOMS, FOOT_SITES, get_walker_demo_robot_cfg
from th_training.tasks.walker_v2_velocity import mdp as wmdp

GAIT_PERIOD = 0.72      # s. COM height 0.556 m -> LIPM omega 4.2 rad/s, omega * T_step = 1.5
GAIT_DUTY = 0.55
SWING_HEIGHT = 0.05
VX = (-0.3, 0.6)
VY = (-0.2, 0.2)
WZ = (-0.8, 0.8)
MIN_FOOT_SEP = 0.18

# The deploy spec (export_spec.py) lists the actor terms in this order with these bridge names.
ACTOR_TERMS_TO_BRIDGE = {
    "base_ang_vel": "base_ang_vel",
    "projected_gravity": "projected_gravity",
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel",
    "actions": "last_action",
    "command": "velocity_command",
    "gait_clock": "gait_phase",
}


def walker_demo_flat_env_cfg(variant: str = "cad", play: bool = False, profile: str = "demo") -> ManagerBasedRlEnvCfg:
    """profile "demo": v1 recipe. profile "safe": reliability first (see _make_safe)."""
    cfg = make_velocity_env_cfg()

    cfg.sim.njmax = 300
    cfg.sim.nconmax = None
    cfg.sim.mujoco.ccd_iterations = 50
    cfg.sim.contact_sensor_maxmatch = 64

    cfg.scene.entities = {"robot": get_walker_demo_robot_cfg(variant, delay_max_lag=5 if profile == "safe" else 3)}
    assert cfg.scene.terrain is not None
    cfg.scene.terrain.terrain_type = "plane"
    cfg.scene.terrain.terrain_generator = None

    sensors = [s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"]
    for s in sensors:
        if s.name == "foot_height_scan":
            assert isinstance(s, TerrainHeightSensorCfg)
            s.frame = tuple(ObjRef(type="site", name=n, entity="robot") for n in FOOT_SITES)
            s.pattern = RingPatternCfg.single_ring(radius=0.03, num_samples=6)
    feet_contact = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(mode="subtree", pattern=r"^(left_foot_link|right_foot_link)$", entity="robot"),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="netforce",
        num_slots=1,
        track_air_time=True,
    )
    cfg.scene.sensors = tuple(sensors) + (feet_contact,)

    # ---------------------------------------------------------------- observations
    actor = cfg.observations["actor"].terms
    critic = cfg.observations["critic"].terms
    for t in ("height_scan",):
        actor.pop(t, None)
        critic.pop(t, None)
    actor.pop("base_lin_vel")  # VN-100 can't measure it; critic keeps it
    clock = ObservationTermCfg(func=wmdp.gait_clock, params={"period": GAIT_PERIOD})
    actor["gait_clock"] = clock
    critic["gait_clock"] = clock
    assert list(actor) == list(ACTOR_TERMS_TO_BRIDGE), f"actor term order changed: {list(actor)}"

    # ---------------------------------------------------------------- actions
    act = cfg.actions["joint_pos"]
    assert isinstance(act, JointPositionActionCfg)
    act.scale = ACTION_SCALE

    # ---------------------------------------------------------------- commands
    twist = cfg.commands["twist"]
    assert isinstance(twist, UniformVelocityCommandCfg)
    twist.ranges.lin_vel_x = VX
    twist.ranges.lin_vel_y = VY
    twist.ranges.ang_vel_z = WZ
    twist.rel_standing_envs = 0.2
    twist.viz.z_offset = 0.9
    cfg.curriculum = {}

    # ---------------------------------------------------------------- events / domain randomization
    ev = cfg.events
    ev["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
    ev["foot_friction"].params["ranges"] = (0.4, 1.2)
    ev["base_com"].params["asset_cfg"].body_names = ("base_link",)
    ev["base_com"].params["ranges"] = {0: (-0.03, 0.03), 1: (-0.02, 0.02), 2: (-0.03, 0.03)}
    ev["encoder_bias"].params["bias_range"] = (-0.03, 0.03)
    ev["reset_robot_joints"].params["position_range"] = (-0.05, 0.05)
    ev["push_robot"].interval_range_s = (3.0, 6.0)
    ev["push_robot"].params["velocity_range"] = {
        "x": (-0.4, 0.4), "y": (-0.3, 0.3), "z": (-0.2, 0.2),
        "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.5, 0.5),
    }
    ev["base_mass"] = EventTermCfg(
        mode="startup", func=dr.body_mass,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=("base_link",)), "ranges": (0.9, 1.25)},
    )
    ev["leg_mass"] = EventTermCfg(
        mode="startup", func=dr.body_mass,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=(".*_(hip_pitch|hip_roll|thigh|shank|foot)_link",)),
                "ranges": (0.9, 1.15)},
    )
    # kp/kd as seen at the joint. Also covers torque-constant error (RS06 measured ~1.1x).
    ev["pd_gains"] = EventTermCfg(
        mode="startup", func=dr.pd_gains,
        params={"asset_cfg": SceneEntityCfg("robot", actuator_names=(".*",)),
                "kp_range": (0.8, 1.2), "kd_range": (0.7, 1.3)},
    )
    # Measured now (one unit per model), so narrower than v2's x0.5-2 / x0-2.
    ev["armature"] = EventTermCfg(
        mode="startup", func=dr.joint_armature,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_joint",)),
                "ranges": (0.7, 1.4), "operation": "scale", "distribution": "log_uniform"},
    )
    ev["joint_friction"] = EventTermCfg(
        mode="startup", func=dr.joint_friction,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_joint",)),
                "ranges": (0.5, 1.5), "operation": "scale"},
    )
    ev["effort_limits"] = EventTermCfg(
        mode="startup", func=dr.effort_limits,
        params={"asset_cfg": SceneEntityCfg("robot", actuator_names=(".*",)), "effort_limit_range": (0.8, 1.0)},
    )

    # ---------------------------------------------------------------- rewards
    r = cfg.rewards
    r["track_linear_velocity"].params["std"] = math.sqrt(0.1)
    r["track_angular_velocity"].params["std"] = math.sqrt(0.25)
    r["upright"].params["asset_cfg"].body_names = ("base_link",)
    r["body_ang_vel"].params["asset_cfg"].body_names = ("base_link",)
    r["body_ang_vel"].weight = -0.05
    r["angular_momentum"].weight = -0.02
    r["air_time"].weight = 0.0
    r["pose"].params["std_standing"] = {".*": 0.05}
    r["pose"].params["std_walking"] = {
        r".*hip_pitch.*": 0.3, r".*hip_roll.*": 0.12, r".*hip_yaw.*": 0.12,
        r".*knee.*": 0.35, r".*ankle_pitch.*": 0.25,
    }
    r["pose"].params["std_running"] = {
        r".*hip_pitch.*": 0.5, r".*hip_roll.*": 0.2, r".*hip_yaw.*": 0.2,
        r".*knee.*": 0.5, r".*ankle_pitch.*": 0.35,
    }
    r["pose"].params["running_threshold"] = 0.9
    for name in ("foot_clearance", "foot_slip"):
        r[name].params["asset_cfg"].site_names = FOOT_SITES
    r["foot_clearance"].params["target_height"] = SWING_HEIGHT
    r["foot_swing_height"].params["target_height"] = SWING_HEIGHT
    r["dof_pos_limits"].weight = -2.0  # the CAD limits are tight; stay off them

    r["gait_contact"] = RewardTermCfg(
        func=wmdp.gait_contact, weight=0.6,
        params={"sensor_name": "feet_ground_contact", "period": GAIT_PERIOD, "duty": GAIT_DUTY,
                "command_name": "twist", "command_threshold": 0.1},
    )
    r["swing_height"] = RewardTermCfg(
        func=wmdp.swing_height_tracking, weight=1.0,
        params={"height_sensor_name": "foot_height_scan", "period": GAIT_PERIOD, "duty": GAIT_DUTY,
                "swing_height": SWING_HEIGHT, "std": 0.03, "command_name": "twist", "command_threshold": 0.1},
    )
    r["foot_slip"].weight = -1.0
    r["foot_flat"] = RewardTermCfg(
        func=wmdp.foot_flat, weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=("left_foot_link", "right_foot_link"))},
    )
    r["feet_too_close"] = RewardTermCfg(
        func=wmdp.feet_too_close, weight=-10.0,
        params={"min_dist": MIN_FOOT_SEP, "asset_cfg": SceneEntityCfg("robot", site_names=FOOT_SITES)},
    )
    r["joint_torques"] = RewardTermCfg(func=mdp.joint_torques_l2, weight=-2e-5)
    r["joint_acc"] = RewardTermCfg(func=mdp.joint_acc_l2, weight=-2.5e-7)

    # ---------------------------------------------------------------- terminations
    cfg.terminations.pop("out_of_terrain_bounds", None)
    cfg.terminations["fell_over"].params["limit_angle"] = math.radians(60.0)
    cfg.terminations["base_too_low"] = TerminationTermCfg(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.35},
    )

    if profile == "safe":
        _make_safe(cfg)
    elif profile in ("v2", "v2b"):
        _make_v2(cfg, variant, revision=profile)
    elif profile == "v3":
        _make_v2(cfg, variant, revision="v2b")
        _make_real_foot(cfg)

    cfg.viewer.body_name = "base_link"
    cfg.viewer.distance = 2.0

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
    return cfg


def _make_v2(cfg: ManagerBasedRlEnvCfg, variant: str, revision: str = "v2") -> None:
    """v1 recipe + fixes from the 2026-11-19 real standing tests (cad_v1 held full weight ~19 s):
      * targets clipped to the joint limits, exactly as the bridge clamps them. Without it, the
        policy pressed hip roll ~0.12 rad past the inward stop (~19 Nm into it, in sim); the robot
        never gets that torque (real hip-roll target error 0.09-0.13 rad, torque ~1.5 Nm).
      * held gait clock (demo mdp.py): the clock stops at a stand command, so standing is still
        instead of swaying at the stride rate. Deploy: policy_spec gait_clock = "held".
      * stand still: joint speed cost at a stand command; 30% standing envs.
      * base COM range shifted back (x -5..+2 cm): the real robot leaned ~2.5 deg back, which a
        COM ~3 cm behind the CAD reproduces in sim.
      * stale motor feedback: actor joint_pos/joint_vel freeze for 1-15 steps (<= 0.3 s, the
        bridge's tolerance) about every 7 s, like the USB-CAN stalls seen on the robot.
    Same observation layout, gains and action scale as v1.

    v2b (revision "v2b"), after cad_v2 refused to walk backward at any speed and stood ~3 deg
    nose-up: COM range symmetric and wider (x +-4 cm; a backward-biased range made stepping back
    the quickest way to fall, so the policy stopped doing it) and a tilt cost while standing."""
    from th_training.robots.walker_demo import JOINT_ORDER, limits_of

    from . import mdp as dmdp

    act = cfg.actions["joint_pos"]
    act.clip = {j: tuple(float(x) for x in limits_of(j, variant)) for j in JOINT_ORDER}

    actor = cfg.observations["actor"].terms
    critic = cfg.observations["critic"].terms
    held = ObservationTermCfg(func=dmdp.gait_clock_held, params={"period": GAIT_PERIOD, "command_name": "twist"})
    actor["gait_clock"] = held
    critic["gait_clock"] = held
    stall = {"p_start": 0.003, "max_steps": 15}
    for name, fn in (("joint_pos", dmdp.stale_joint_pos_rel), ("joint_vel", dmdp.stale_joint_vel_rel)):
        actor[name] = dataclasses.replace(actor[name], func=fn, params={**actor[name].params, **stall})
    assert list(actor) == list(ACTOR_TERMS_TO_BRIDGE), f"actor term order changed: {list(actor)}"

    cfg.commands["twist"].rel_standing_envs = 0.3
    com_x = (-0.04, 0.04) if revision == "v2b" else (-0.05, 0.02)
    cfg.events["base_com"].params["ranges"] = {0: com_x, 1: (-0.02, 0.02), 2: (-0.03, 0.03)}

    r = cfg.rewards
    r["gait_contact"].func = dmdp.gait_contact_held
    r["swing_height"].func = dmdp.swing_height_tracking_held
    r["stand_still"] = RewardTermCfg(func=dmdp.stand_still_joint_vel, weight=-0.05, params={"command_name": "twist"})
    if revision == "v2b":
        # 3 deg -> 0.0027; x30 = 0.08 per step, comparable to the pose reward's pull
        r["stand_tilt"] = RewardTermCfg(func=dmdp.stand_tilt, weight=-30.0, params={"command_name": "twist"})


def _make_real_foot(cfg: ManagerBasedRlEnvCfg) -> None:
    """v3 = v2b on the foot the real robot actually stands on (2026-11-19 lab): it tipped back at
    the CAD stand pose, stood at a -0.05 rad ankle lean with +1.4..+2.0 N m per ankle, and cad_v1
    tripped the speed limit within ~1 s of taking over. A sim foot with usable sole ~1.5 cm behind
    to ~6 cm ahead of the ankle (CAD: 7.6 / 7.6 cm) and the COM ~2.5 cm behind the CAD reproduces
    all three. Randomized per env around that, so the policy learns to balance on it."""
    from mjlab.managers.scene_entity_config import SceneEntityCfg as _S
    ev = cfg.events
    feet = _S("robot", geom_names=FOOT_GEOMS)
    ev["foot_length"] = EventTermCfg(mode="startup", func=dr.geom_size,
                                     params={"asset_cfg": feet, "ranges": {0: (0.03, 0.06)}, "operation": "abs",
                                             "axes": [0], "shared_random": True})
    ev["foot_shift"] = EventTermCfg(mode="startup", func=dr.geom_pos,
                                    params={"asset_cfg": _S("robot", geom_names=FOOT_GEOMS), "ranges": {0: (0.005, 0.035)},
                                            "operation": "add", "axes": [0], "shared_random": True})
    ev["base_com"].params["ranges"] = {0: (-0.05, 0.0), 1: (-0.02, 0.02), 2: (-0.03, 0.03)}


def _make_safe(cfg: ManagerBasedRlEnvCfg) -> None:
    """Reliability over speed, for a robot that is probably worse than its CAD:
      * slow commands only: vx -0.15..0.25 m/s, no sideways, wz +-0.4 rad/s; 30% standing.
      * wider randomization: kp x0.7-1.3 / kd x0.6-1.4, 0-25 ms delay, foot friction 0.3-1.2,
        base mass x0.85-1.35, base COM +-4 cm, encoder bias +-0.05 rad (offset calibration by eye),
        joint friction x0.5-2, torque limit x0.7-1.0, harder and more frequent pushes.
      * gentler motion: 3x action-rate, 4x joint-acceleration and 2.5x torque penalties, 2x upright,
        2x body angular velocity, and a -100 penalty on falling.
    The gains, observation and deploy spec are unchanged, so it drops into the same bridge config."""
    twist = cfg.commands["twist"]
    twist.ranges.lin_vel_x = (-0.15, 0.25)
    twist.ranges.lin_vel_y = (0.0, 0.0)
    twist.ranges.ang_vel_z = (-0.4, 0.4)
    twist.rel_standing_envs = 0.3
    ev = cfg.events
    ev["pd_gains"].params.update(kp_range=(0.7, 1.3), kd_range=(0.6, 1.4))
    ev["foot_friction"].params["ranges"] = (0.3, 1.2)
    ev["base_mass"].params["ranges"] = (0.85, 1.35)
    ev["leg_mass"].params["ranges"] = (0.85, 1.2)
    ev["base_com"].params["ranges"] = {0: (-0.04, 0.04), 1: (-0.03, 0.03), 2: (-0.04, 0.04)}
    ev["encoder_bias"].params["bias_range"] = (-0.05, 0.05)
    ev["joint_friction"].params["ranges"] = (0.5, 2.0)
    ev["effort_limits"].params["effort_limit_range"] = (0.7, 1.0)
    if "push_robot" in ev:
        ev["push_robot"].interval_range_s = (2.0, 5.0)
        ev["push_robot"].params["velocity_range"] = {
            "x": (-0.5, 0.5), "y": (-0.4, 0.4), "z": (-0.2, 0.2),
            "roll": (-0.4, 0.4), "pitch": (-0.4, 0.4), "yaw": (-0.5, 0.5),
        }
    r = cfg.rewards
    r["action_rate_l2"].weight = -0.3
    r["joint_acc"].weight = -1e-6
    r["joint_torques"].weight = -5e-5
    r["upright"].weight = 2.0
    r["body_ang_vel"].weight = -0.1
    r["termination"] = RewardTermCfg(func=mdp.is_terminated, weight=-100.0)
