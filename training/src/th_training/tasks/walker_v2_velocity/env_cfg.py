"""WalkerV2 flat-ground velocity tracking, built on mjlab's velocity task factory.

The design choices (and where each number comes from) are written up in
training/docs/walker_v2_policy_plan.md. Gait numbers come from docs/walker_v2_first_principles.md.
"""
from __future__ import annotations

import math

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg, ObjRef, RingPatternCfg, TerrainHeightSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

from th_training.robots.walker_v2 import FOOT_GEOMS, FOOT_SITES, action_scale, get_walker_v2_robot_cfg
from th_training.tasks.walker_v2_velocity import mdp as wmdp

# ---- From docs/walker_v2_first_principles.md ----
GAIT_PERIOD = 0.72      # s. LIPM omega = 4.16 rad/s, omega * T_step = 1.5
GAIT_DUTY = 0.55        # stance fraction per foot -> 5% double support at each switch
SWING_HEIGHT = 0.07     # m. ~10% of the 0.66 m leg
VX = (-0.4, 0.8)        # m/s. Reach-limited to ~0.6 m/s at standing height; past that the robot crouches
VX_LATE = (-0.5, 1.0)   # unlocked after STAGE2_ITERS
VY = (-0.3, 0.3)
WZ = (-1.0, 1.0)
STAGE2_ITERS = 2000
MIN_FOOT_SEP = 0.18     # m. Stance width is 0.29 m; feet touch at ~0.10 m


def walker_v2_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    cfg = make_velocity_env_cfg()

    # ---------------------------------------------------------------- sim / scene
    cfg.sim.njmax = 300
    cfg.sim.nconmax = None
    cfg.sim.mujoco.ccd_iterations = 50
    cfg.sim.contact_sensor_maxmatch = 64

    cfg.scene.entities = {"robot": get_walker_v2_robot_cfg()}
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
    # No linear-velocity sensor on the robot (VectorNav IMU only), so the actor doesn't get it.
    # The critic keeps it as privileged information.
    actor.pop("base_lin_vel")
    clock = ObservationTermCfg(func=wmdp.gait_clock, params={"period": GAIT_PERIOD})
    actor["gait_clock"] = clock
    critic["gait_clock"] = clock

    # ---------------------------------------------------------------- actions
    act = cfg.actions["joint_pos"]
    assert isinstance(act, JointPositionActionCfg)
    act.scale = action_scale()

    # ---------------------------------------------------------------- commands
    twist = cfg.commands["twist"]
    assert isinstance(twist, UniformVelocityCommandCfg)
    twist.ranges.lin_vel_x = VX
    twist.ranges.lin_vel_y = VY
    twist.ranges.ang_vel_z = WZ
    twist.rel_standing_envs = 0.15
    twist.viz.z_offset = 0.9
    cfg.curriculum = {
        "command_vel": CurriculumTermCfg(
            func=mdp.commands_vel,
            params={
                "command_name": "twist",
                "velocity_stages": [
                    {"step": 0, "lin_vel_x": VX, "ang_vel_z": WZ},
                    {"step": STAGE2_ITERS * 24, "lin_vel_x": VX_LATE},
                ],
            },
        )
    }

    # ---------------------------------------------------------------- events / domain randomization
    ev = cfg.events
    ev["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
    ev["foot_friction"].params["ranges"] = (0.4, 1.2)
    ev["base_com"].params["asset_cfg"].body_names = ("base_link",)
    ev["reset_robot_joints"].params["position_range"] = (-0.05, 0.05)
    ev["push_robot"].interval_range_s = (3.0, 6.0)
    ev["push_robot"].params["velocity_range"] = {
        "x": (-0.4, 0.4), "y": (-0.3, 0.3), "z": (-0.2, 0.2),
        "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.5, 0.5),
    }
    # CAD masses are unverified (CAD_REVIEW.md: weigh the robot; a 10x mass typo was already found).
    ev["base_mass"] = EventTermCfg(
        mode="startup", func=dr.body_mass,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=("base_link",)), "ranges": (0.85, 1.2)},
    )
    ev["leg_mass"] = EventTermCfg(
        mode="startup", func=dr.body_mass,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=(".*_(hip_pitch|hip_roll|thigh|shank|foot)_link",)),
                "ranges": (0.9, 1.1)},
    )
    # Actuator unknowns until the real2sim characterization: gains as seen by the joint,
    # rotor inertia, friction, torque limit. Command delay (0-10 ms) is in the actuator cfg.
    ev["pd_gains"] = EventTermCfg(
        mode="startup", func=dr.pd_gains,
        params={"asset_cfg": SceneEntityCfg("robot", actuator_names=(".*",)),
                "kp_range": (0.8, 1.2), "kd_range": (0.7, 1.3)},
    )
    ev["armature"] = EventTermCfg(
        mode="startup", func=dr.joint_armature,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_joint",)),
                "ranges": (0.5, 2.0), "operation": "scale", "distribution": "log_uniform"},
    )
    ev["joint_friction"] = EventTermCfg(
        mode="startup", func=dr.joint_friction,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_joint",)),
                "ranges": (0.0, 2.0), "operation": "scale"},
    )
    ev["effort_limits"] = EventTermCfg(
        mode="startup", func=dr.effort_limits,
        params={"asset_cfg": SceneEntityCfg("robot", actuator_names=(".*",)), "effort_limit_range": (0.8, 1.0)},
    )

    # ---------------------------------------------------------------- rewards
    r = cfg.rewards
    r["track_linear_velocity"].params["std"] = math.sqrt(0.15)
    r["track_angular_velocity"].params["std"] = math.sqrt(0.3)
    r["upright"].params["asset_cfg"].body_names = ("base_link",)
    r["body_ang_vel"].params["asset_cfg"].body_names = ("base_link",)
    r["body_ang_vel"].weight = -0.05
    r["angular_momentum"].weight = -0.02
    r["air_time"].weight = 0.0  # the gait clock sets swing timing instead
    r["pose"].params["std_standing"] = {".*": 0.05}
    r["pose"].params["std_walking"] = {
        r".*hip_pitch.*": 0.3, r".*hip_roll.*": 0.12, r".*hip_yaw.*": 0.12,
        r".*knee.*": 0.35, r".*ankle_pitch.*": 0.25,
    }
    r["pose"].params["std_running"] = {
        r".*hip_pitch.*": 0.5, r".*hip_roll.*": 0.2, r".*hip_yaw.*": 0.2,
        r".*knee.*": 0.6, r".*ankle_pitch.*": 0.35,
    }
    r["pose"].params["running_threshold"] = 0.9
    for name in ("foot_clearance", "foot_slip"):
        r[name].params["asset_cfg"].site_names = FOOT_SITES
    r["foot_clearance"].params["target_height"] = SWING_HEIGHT
    r["foot_swing_height"].params["target_height"] = SWING_HEIGHT

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
    r["foot_slip"].weight = -1.0  # v1: -0.1 made sliding the feet nearly free
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

    cfg.viewer.body_name = "base_link"
    cfg.viewer.distance = 2.0

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}
        twist.ranges.lin_vel_x = VX_LATE
    return cfg
