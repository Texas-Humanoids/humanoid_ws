"""Demo walker (Onshape export pkg_002, 2026-11-17) robot config for mjlab.

Same 10-DOF layout and joint names as WalkerV2. Differences that matter for training:
  * 16.87 kg (WalkerV2 14.8): battery, Jetson, DC-DC and power distribution are in the base now.
  * The CAD joint limits are tighter: hip roll is abduction-only (0 adduction) and the knee stops
    at 60 deg. Nobody has confirmed whether those are real hard stops or just Onshape mate limits,
    so there are two limit sets (LIMITS["cad"], LIMITS["wide"]) and one task per set. Measure the
    real stops with tools/walker/bringup/jog.py and deploy the policy whose limits fit inside them.
  * Actuator numbers are MEASURED (sysid 2026-11-15, sysid_data/README.md), not datasheet guesses:
    armature, Coulomb friction, 36 V top speed, CAN delay. Peak torque is still the datasheet value
    (no torque test bed), and the RS06 measured ~1.1x commanded torque, so strength is randomized.

The deploy stack (th_policy_bridge) takes ONE scalar action scale and the observation terms by name,
so this config uses a scalar ACTION_SCALE; see export_spec.py for the policy_spec.json it produces.
"""
from __future__ import annotations

import os
from dataclasses import dataclass

from th_training.paths import WALKER_DEMO_XML

JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_pitch_joint", "left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_pitch_joint", "right_ankle_pitch_joint",
]

# --------------------------------------------------------------------------------------
# Joint limits (clean convention: pitch + = foot back / knee flex / toes down, roll + = foot left)
# --------------------------------------------------------------------------------------
LIMITS = {
    # Straight from pkg_002.urdf after the sign fix.
    "cad": {
        "hip_pitch": (-0.7854, 0.7854),
        "left_hip_roll": (0.0, 0.7854), "right_hip_roll": (-0.7854, 0.0),
        "hip_yaw": (-0.7854, 0.7854),
        "knee_pitch": (0.0, 1.0472),
        "ankle_pitch": (-1.5708, 1.5708),
    },
    # WalkerV2's verified limits (hip roll inner limit from foot collision at ~17-20 deg).
    "wide": {
        "hip_pitch": (-0.7854, 0.7854),
        "left_hip_roll": (-0.35, 0.7854), "right_hip_roll": (-0.7854, 0.35),
        "hip_yaw": (-0.7854, 0.7854),
        "knee_pitch": (0.0, 2.0944),
        "ankle_pitch": (-1.5708, 1.5708),
    },
}
# Hip roll default per limit set. With "cad" the inner stop is AT 0, so stand slightly abducted
# (+-0.05 rad = feet ~6 cm wider) to keep the default inside the limits.
HIP_ROLL_DEFAULT = {"cad": 0.05, "wide": 0.0}


def limits_of(j: str, variant: str) -> tuple[float, float]:
    L = LIMITS[variant]
    side_key = j.replace("_joint", "")
    return L[side_key] if side_key in L else L[joint_type(j)]


def default_joint_pos(variant: str) -> dict[str, float]:
    hr = HIP_ROLL_DEFAULT[variant]
    return {
        "left_hip_pitch_joint": -0.2, "right_hip_pitch_joint": -0.2,
        "left_hip_roll_joint": hr, "right_hip_roll_joint": -hr,
        "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
        "left_knee_pitch_joint": 0.4, "right_knee_pitch_joint": 0.4,
        "left_ankle_pitch_joint": -0.2, "right_ankle_pitch_joint": -0.2,
    }


# Base (hip-pitch axis) height above the soles at the ready pose: 0.6605 m (build_report.json / FK).
STAND_BASE_HEIGHT = 0.661

# --------------------------------------------------------------------------------------
# Motors. Joint-side values (the gearbox is inside the actuator).
# --------------------------------------------------------------------------------------
BUS_VOLTAGE = 36.0


@dataclass(frozen=True)
class Motor:
    peak_torque: float      # Nm, datasheet peak (current-limited, same at 36 V)
    rated_torque: float     # Nm, continuous
    speed_36v: float        # rad/s, measured top speed at our bus, scaled to 36 V (sysid spin test)
    armature: float         # kg m^2, measured (Bode / time-domain fit)
    friction: float         # Nm Coulomb, measured


MOTORS = {
    "RS03": Motor(peak_torque=60.0, rated_torque=20.0, speed_36v=16.4, armature=0.07, friction=0.40),
    "RS02": Motor(peak_torque=17.0, rated_torque=6.0, speed_36v=33.3, armature=0.013, friction=0.12),
    "RS06": Motor(peak_torque=36.0, rated_torque=11.0, speed_36v=37.7, armature=0.018, friction=0.18),
}

MOTOR_OF_JOINT = {
    **{f"{s}_hip_pitch_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_hip_roll_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_hip_yaw_joint": "RS02" for s in ("left", "right")},
    **{f"{s}_knee_pitch_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_ankle_pitch_joint": "RS06" for s in ("left", "right")},
}

# MIT-mode kp (Nm/rad), kd (Nm s/rad). These go to the drives unchanged (policy_spec.json).
# Chosen so the robot stands on PD ALONE (zero action) down to 0.7x kp with 15 ms delay and a
# 0.3 m/s push (sim2sim_demo stand). That's the safety net on hardware: if the policy output is
# garbage, the default pose still holds. The ankle sets it: 16.9 kg at COM height 0.556 m needs
# m*g*h = 92 Nm/rad of total ankle stiffness to stand, and ankle 60 fell at 0.8x. kd gives a
# damping ratio of ~0.6-0.9 with the measured rotor inertia.
GAINS = {
    "hip_pitch": (80.0, 4.0),
    "hip_roll": (100.0, 5.0),
    "hip_yaw": (30.0, 1.0),
    "knee_pitch": (120.0, 4.0),
    "ankle_pitch": (100.0, 2.5),
}

# One scalar for all joints (the bridge only supports a scalar). target = default + ACTION_SCALE * a.
ACTION_SCALE = 0.25

# CAN round trip 1.2-1.6 ms + ROS pipeline 3-13 ms (sysid latency run) -> 0..3 physics steps of 5 ms.
DELAY_MAX_LAG = 3


def no_load_speed_at(motor: str, voltage: float = BUS_VOLTAGE) -> float:
    return MOTORS[motor].speed_36v * voltage / 36.0


def joint_type(j: str) -> str:
    return j.replace("left_", "").replace("right_", "").replace("_joint", "")


def gains_of(j: str) -> tuple[float, float]:
    return GAINS[joint_type(j)]


def action_scale() -> dict[str, float]:
    return {j: ACTION_SCALE for j in JOINT_ORDER}


# --------------------------------------------------------------------------------------
# mjlab entity config
# --------------------------------------------------------------------------------------
def make_spec_fn(variant: str, visual_meshes: bool | None = None):
    """MjSpec loader that applies the joint-limit set. Visual meshes are dropped for training
    (they never collide and only slow the compile); set TH_VISUAL=1 to keep them for play."""
    if visual_meshes is None:
        visual_meshes = os.environ.get("TH_VISUAL", "0") == "1"

    def spec_fn():
        import mujoco  # lazy: Isaac's isaac_check_demo imports this module without MuJoCo

        spec = mujoco.MjSpec.from_file(str(WALKER_DEMO_XML))
        for j in JOINT_ORDER:
            spec.joint(j).range = list(limits_of(j, variant))
        if not visual_meshes:
            for g in list(spec.geoms):
                if g.type == mujoco.mjtGeom.mjGEOM_MESH:
                    spec.delete(g)
            for m in list(spec.meshes):
                spec.delete(m)
        return spec

    return spec_fn


def _actuator_cfgs(delay_max_lag: int = DELAY_MAX_LAG):
    from mjlab.actuator import DcMotorActuatorCfg

    cfgs = []
    for j in JOINT_ORDER:
        mot = MOTORS[MOTOR_OF_JOINT[j]]
        kp, kd = gains_of(j)
        cfgs.append(
            DcMotorActuatorCfg(
                target_names_expr=(j,),
                stiffness=kp,
                damping=kd,
                # Linear torque-speed line from peak at stall to 0 at the 36 V top speed. This is
                # below RobStride's published curves (they hold peak to ~1/3 of top speed), i.e.
                # conservative: the policy never counts on torque the motor might not have.
                saturation_effort=mot.peak_torque,
                effort_limit=mot.peak_torque,
                velocity_limit=no_load_speed_at(MOTOR_OF_JOINT[j]),
                armature=mot.armature,
                frictionloss=mot.friction,
                delay_min_lag=0,
                delay_max_lag=delay_max_lag,
            )
        )
    return tuple(cfgs)


FOOT_GEOMS = ("left_foot_collision", "right_foot_collision")
FOOT_SITES = ("left_foot", "right_foot")


def get_walker_demo_robot_cfg(variant: str = "cad", delay_max_lag: int = DELAY_MAX_LAG):
    from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
    from mjlab.utils.spec_config import CollisionCfg

    collision = CollisionCfg(
        geom_names_expr=(".*_collision",),
        contype={r"^(left|right)_foot_collision$": 1, ".*": 0},
        conaffinity=1,
        condim={r"^(left|right)_foot_collision$": 3, ".*": 1},
        priority={r"^(left|right)_foot_collision$": 1, ".*": 0},
        friction={r"^(left|right)_foot_collision$": (0.8,), ".*": (0.6,)},
    )
    return EntityCfg(
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, STAND_BASE_HEIGHT + 0.01),
            joint_pos=default_joint_pos(variant),
            joint_vel={".*": 0.0},
        ),
        collisions=(collision,),
        spec_fn=make_spec_fn(variant),
        articulation=EntityArticulationInfoCfg(
            actuators=_actuator_cfgs(delay_max_lag),
            soft_joint_pos_limit_factor=0.95,
        ),
    )
