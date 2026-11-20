"""WalkerV2 robot config for mjlab.

This is the ONE place that holds actuator numbers. When the real2sim motor characterization
lands, update `MOTORS` (armature, friction, torque-speed) and `GAINS` here. The task and the
deploy-side controller both read them from here, so nothing else has to change.

Numbers marked ESTIMATE are not measured. Each one is domain-randomized in the task
(see tasks/walker_v2_velocity/env_cfg.py) wide enough to cover the likely truth.
Derivations: training/docs/walker_v2_first_principles.md.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco

from th_training.paths import WALKER_V2_XML

# --------------------------------------------------------------------------------------
# Joints. Same order everywhere: obs, actions, ONNX, and the hardware driver (map by name).
# --------------------------------------------------------------------------------------
JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_pitch_joint", "left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_pitch_joint", "right_ankle_pitch_joint",
]

# Ready pose: knees bent 0.4 rad, feet flat (hip = ankle = -knee/2). Base height is computed by
# the first-principles script; 0.659 m from CAD_REVIEW.md.
DEFAULT_JOINT_POS = {
    "left_hip_pitch_joint": -0.2, "right_hip_pitch_joint": -0.2,
    "left_hip_roll_joint": 0.0, "right_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0, "right_hip_yaw_joint": 0.0,
    "left_knee_pitch_joint": 0.4, "right_knee_pitch_joint": 0.4,
    "left_ankle_pitch_joint": -0.2, "right_ankle_pitch_joint": -0.2,
}
STAND_BASE_HEIGHT = 0.659


# --------------------------------------------------------------------------------------
# Motors (Robstride datasheets, quoted @ 48 V). Joint-side values; the gearbox is inside the actuator.
# The robot runs the motors at BUS_VOLTAGE. Peak torque is current-limited (same at 36 V); the
# no-load speed is back-EMF-limited and scales with voltage -> use no_load_speed_at().
# --------------------------------------------------------------------------------------
DATASHEET_VOLTAGE = 48.0  # V, the voltage the Robstride datasheet numbers are quoted at
BUS_VOLTAGE = 36.0        # V, what the robot actually runs the motors at (2026-11-15)


@dataclass(frozen=True)
class Motor:
    peak_torque: float     # Nm, datasheet peak
    rated_torque: float    # Nm, continuous
    no_load_speed: float   # rad/s at the output, @ DATASHEET_VOLTAGE
    armature: float        # kg m^2 reflected rotor inertia. ESTIMATE until real2sim.
    friction: float        # Nm Coulomb friction. ESTIMATE until real2sim.


MOTORS = {
    "RS03": Motor(peak_torque=60.0, rated_torque=20.0, no_load_speed=20.4, armature=0.015, friction=0.2),
    "RS02": Motor(peak_torque=17.0, rated_torque=6.0, no_load_speed=42.9, armature=0.004, friction=0.1),
    "RS06": Motor(peak_torque=36.0, rated_torque=11.0, no_load_speed=50.3, armature=0.008, friction=0.15),
}

MOTOR_OF_JOINT = {
    **{f"{s}_hip_pitch_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_hip_roll_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_hip_yaw_joint": "RS02" for s in ("left", "right")},
    **{f"{s}_knee_pitch_joint": "RS03" for s in ("left", "right")},
    **{f"{s}_ankle_pitch_joint": "RS06" for s in ("left", "right")},
}

# (kp Nm/rad, kd Nm s/rad) per joint type. Derived in walker_v2_first_principles.md (section 5).
# These MUST be the exact MIT-mode kp/kd sent to the Robstride drivers on hardware.
GAINS = {
    "hip_pitch": (60.0, 5.5),
    "hip_roll": (120.0, 7.5),   # carries the lateral balance (no ankle roll): ~24 Nm at foot switch
    "hip_yaw": (20.0, 0.5),
    "knee_pitch": (115.0, 3.0), # knee sag = torso pitch; 60 diverged in the PD-only stand at 0.8x kp
    "ankle_pitch": (85.0, 1.5),
}


def no_load_speed_at(motor: str, voltage: float = BUS_VOLTAGE) -> float:
    """Output no-load speed at a bus voltage (back-EMF scaling). 36 V -> 0.75x datasheet."""
    return MOTORS[motor].no_load_speed * voltage / DATASHEET_VOLTAGE


def joint_type(j: str) -> str:
    return j.replace("left_", "").replace("right_", "").replace("_joint", "")


def gains_of(j: str) -> tuple[float, float]:
    return GAINS[joint_type(j)]


def action_scale() -> dict[str, float]:
    """0.25 * peak / kp: action = 1 asks for 25% of peak torque at zero velocity."""
    return {j: 0.25 * MOTORS[MOTOR_OF_JOINT[j]].peak_torque / gains_of(j)[0] for j in JOINT_ORDER}


# --------------------------------------------------------------------------------------
# mjlab entity config
# --------------------------------------------------------------------------------------
def get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(WALKER_V2_XML))


def _actuator_cfgs(delay_max_lag: int = 2):
    """One DC-motor actuator group per joint type. Explicit PD + linear torque-speed clip,
    which is what MIT mode does on the drive (PD at kHz; we run it at the 200 Hz physics rate).
    delay: 0..delay_max_lag physics steps (5 ms each) of command latency for CAN."""
    from mjlab.actuator import DcMotorActuatorCfg

    # One cfg per joint, in JOINT_ORDER, so the action vector order is exactly JOINT_ORDER
    # (grouping by type would interleave left/right).
    cfgs = []
    for j in JOINT_ORDER:
        mot = MOTORS[MOTOR_OF_JOINT[j]]
        kp, kd = gains_of(j)
        cfgs.append(
            DcMotorActuatorCfg(
                target_names_expr=(j,),
                stiffness=kp,
                damping=kd,
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


def get_walker_v2_robot_cfg():
    from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
    from mjlab.utils.spec_config import CollisionCfg

    # Feet collide with the floor and with the other leg (contype=1); other links only with the
    # floor (contype=0, conaffinity=1). Feet get condim 3 + priority so their friction wins.
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
            joint_pos=dict(DEFAULT_JOINT_POS),
            joint_vel={".*": 0.0},
        ),
        collisions=(collision,),
        spec_fn=get_spec,
        articulation=EntityArticulationInfoCfg(
            actuators=_actuator_cfgs(),
            soft_joint_pos_limit_factor=0.95,
        ),
    )
