"""Demo-walker MDP terms for the v2 recipe, from the 2026-11-19 lab standing tests.

held gait clock    cad_v1's clock ran while standing, so the robot swayed at the 1.39 Hz stride
                   rate (real bag; reproduced in sim, and freezing the clock in sim cut that motion
                   10-20x). v2's phase is a recurrence the bridge copies exactly, once per policy step:
                       phase = 0                              on the first step of an episode / WALK
                       phase = (phase + dt / period) mod 1    if |v_xy| + |w_z| > threshold
                       phase = 0                              otherwise (standing: clock held)
                   It advances in the observation term, i.e. with the command the policy sees that
                   step (mjlab resamples commands after rewards, before observations). Rewards read
                   the phase the policy acted under.
stale joints       USB-CAN stalls on the robot froze motor feedback for up to ~1 s; the bridge keeps
                   running on the last readings for 0.3 s, then disarms. In training, the actor's
                   joint_pos / joint_vel occasionally freeze for 1..max_steps steps (IMU stays live,
                   as on the robot). The critic always sees the truth.
stand still        joint speed cost under a stand command.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from mjlab.envs.mdp import observations as obs_mdp
from mjlab.managers.scene_entity_config import SceneEntityCfg

from th_training.tasks.walker_v2_velocity.mdp import _moving

if TYPE_CHECKING:
    from mjlab.entity import Entity
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.sensor import ContactSensor

_ROBOT = SceneEntityCfg("robot")
STAND_THRESHOLD = 0.1  # rad/s + m/s, same as the gait rewards' command_threshold


def _state(env) -> dict:
    st = getattr(env, "_th_demo_state", None)
    if st is None:
        st = {}
        env._th_demo_state = st
    return st


def _phase_buf(env, period: float, command_name: str, threshold: float) -> dict:
    key = ("phase", period, command_name, threshold)
    st = _state(env)
    if key not in st:
        st[key] = {"phase": torch.zeros(env.num_envs, device=env.device), "step": None}
    return st[key]


def advance_held_phase(env: ManagerBasedRlEnv, period: float, command_name: str, threshold: float) -> torch.Tensor:
    """The recurrence in the module docstring; advances at most once per env step."""
    buf = _phase_buf(env, period, command_name, threshold)
    step = int(env.common_step_counter)
    if buf["step"] != step:
        moving = _moving(env, command_name, threshold)
        nxt = torch.remainder(buf["phase"] + env.step_dt / period, 1.0)
        keep = moving & (env.episode_length_buf > 0)
        buf["phase"] = torch.where(keep, nxt, torch.zeros_like(nxt))
        buf["step"] = step
    return buf["phase"]


def held_phase(env: ManagerBasedRlEnv, period: float, command_name: str, threshold: float) -> torch.Tensor:
    """The phase the policy last saw (rewards: no advance)."""
    return _phase_buf(env, period, command_name, threshold)["phase"]


# ----------------------------------------------------------------------------- observations
def gait_clock_held(env: ManagerBasedRlEnv, period: float, command_name: str = "twist",
                    threshold: float = STAND_THRESHOLD) -> torch.Tensor:
    ph = 2 * math.pi * advance_held_phase(env, period, command_name, threshold)
    return torch.stack([torch.sin(ph), torch.cos(ph)], dim=-1)


def _stalled(env: ManagerBasedRlEnv, p_start: float, max_steps: int) -> torch.Tensor:
    st = _state(env)
    buf = st.get("stall")
    if buf is None:
        buf = st["stall"] = {"left": torch.zeros(env.num_envs, dtype=torch.long, device=env.device), "step": None}
    step = int(env.common_step_counter)
    if buf["step"] != step:
        left = torch.clamp(buf["left"] - 1, min=0)
        start = (left == 0) & (torch.rand(env.num_envs, device=env.device) < p_start)
        length = torch.randint(1, max_steps + 1, (env.num_envs,), device=env.device)
        left = torch.where(start, length, left)
        left = torch.where(env.episode_length_buf > 0, left, torch.zeros_like(left))
        buf["left"], buf["step"] = left, step
    return buf["left"] > 0


def _hold(env, name: str, fresh: torch.Tensor, stalled: torch.Tensor) -> torch.Tensor:
    st = _state(env)
    key = ("held", name)
    step = int(env.common_step_counter)
    prev = st.get(key)
    if prev is not None and prev[1] == step and step > 0:  # step 0: reset() may run more than once
        return prev[0]
    out = fresh if prev is None else torch.where(stalled.unsqueeze(-1), prev[0], fresh)
    st[key] = (out.clone(), step)
    return out


def stale_joint_pos_rel(env: ManagerBasedRlEnv, p_start: float = 0.003, max_steps: int = 15, biased: bool = True,
                        asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    fresh = obs_mdp.joint_pos_rel(env, biased=biased, asset_cfg=asset_cfg)
    return _hold(env, "joint_pos", fresh, _stalled(env, p_start, max_steps))


def stale_joint_vel_rel(env: ManagerBasedRlEnv, p_start: float = 0.003, max_steps: int = 15,
                        asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    fresh = obs_mdp.joint_vel_rel(env, asset_cfg=asset_cfg)
    return _hold(env, "joint_vel", fresh, _stalled(env, p_start, max_steps))


# ----------------------------------------------------------------------------- rewards
def gait_contact_held(env: ManagerBasedRlEnv, sensor_name: str, period: float, duty: float, command_name: str,
                      command_threshold: float = STAND_THRESHOLD) -> torch.Tensor:
    """walker_v2 gait_contact on the held clock."""
    sensor: ContactSensor = env.scene[sensor_name]
    contact = sensor.data.found > 0
    ph = held_phase(env, period, command_name, command_threshold)
    expected = torch.stack([ph < duty, torch.remainder(ph + 0.5, 1.0) < duty], dim=-1)
    moving = _moving(env, command_name, command_threshold).unsqueeze(-1)
    expected = torch.where(moving, expected, torch.ones_like(expected))
    return (contact == expected).float().mean(dim=-1)


def swing_height_tracking_held(env: ManagerBasedRlEnv, height_sensor_name: str, period: float, duty: float,
                               swing_height: float, std: float, command_name: str,
                               command_threshold: float = STAND_THRESHOLD) -> torch.Tensor:
    """walker_v2 swing_height_tracking on the held clock."""
    heights = env.scene[height_sensor_name].data.heights
    ph = held_phase(env, period, command_name, command_threshold)
    phases = torch.stack([ph, torch.remainder(ph + 0.5, 1.0)], dim=-1)
    s = torch.clamp((phases - duty) / (1.0 - duty), min=0.0)
    target = torch.where(phases >= duty, swing_height * torch.sin(math.pi * s), torch.zeros_like(s))
    moving = _moving(env, command_name, command_threshold).unsqueeze(-1)
    target = torch.where(moving, target, torch.zeros_like(target))
    return torch.exp(-torch.sum(torch.square(heights - target), dim=-1) / std**2)


def stand_still_joint_vel(env: ManagerBasedRlEnv, command_name: str = "twist", threshold: float = STAND_THRESHOLD,
                          asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Sum of squared joint speeds, only under a stand command."""
    asset: Entity = env.scene[asset_cfg.name]
    cost = torch.sum(torch.square(asset.data.joint_vel[:, asset_cfg.joint_ids]), dim=-1)
    return cost * (~_moving(env, command_name, threshold)).float()


def stand_tilt(env: ManagerBasedRlEnv, command_name: str = "twist", threshold: float = STAND_THRESHOLD,
               asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Squared body tilt (projected-gravity xy) under a stand command only. v2 stood ~3 deg nose-up
    (both ankles ~2 deg toes-down): the generic upright reward barely notices 3 deg."""
    asset: Entity = env.scene[asset_cfg.name]
    tilt = torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=-1)
    return tilt * (~_moving(env, command_name, threshold)).float()
