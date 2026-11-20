"""WalkerV2-specific MDP terms on top of mjlab's velocity task.

Why each one exists (numbers from docs/walker_v2_first_principles.md):
  gait_clock / gait_contact   The LIPM says this robot should step every ~0.36 s. A clock gives the
                              policy that rhythm directly, so it doesn't have to discover it.
                              Cheap to reproduce on the robot: phase = (t mod T) / T.
  foot_flat                   No ankle roll, so foot roll = torso roll + hip roll. Without a cost the
                              policy happily walks on the sole edges, which won't transfer.
  feet_too_close              Stance width is 0.29 m. Crossing the feet is how a hip-roll-only
                              biped falls sideways.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.lab_api.math import quat_apply_inverse, yaw_quat

if TYPE_CHECKING:
    from mjlab.entity import Entity
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.sensor import ContactSensor

_ROBOT = SceneEntityCfg("robot")


def _phase(env: ManagerBasedRlEnv, period: float) -> torch.Tensor:
    t = env.episode_length_buf.float() * env.step_dt
    return torch.remainder(t / period, 1.0)


def _moving(env: ManagerBasedRlEnv, command_name: str, threshold: float) -> torch.Tensor:
    cmd = env.command_manager.get_command(command_name)
    return (torch.norm(cmd[:, :2], dim=1) + torch.abs(cmd[:, 2])) > threshold


# ----------------------------------------------------------------------------- observations
def gait_clock(env: ManagerBasedRlEnv, period: float) -> torch.Tensor:
    """[sin, cos] of the gait phase. On hardware: phase = ((t - t_start) mod period) / period."""
    ph = 2 * math.pi * _phase(env, period)
    return torch.stack([torch.sin(ph), torch.cos(ph)], dim=-1)


# ----------------------------------------------------------------------------- rewards
def gait_contact(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    period: float,
    duty: float,
    command_name: str,
    command_threshold: float = 0.1,
) -> torch.Tensor:
    """Fraction of feet whose contact state matches the clock (left foot leads, right is half a
    period behind). Under a stand command both feet should be down."""
    sensor: ContactSensor = env.scene[sensor_name]
    contact = sensor.data.found > 0  # [B, 2] (left, right)
    ph = _phase(env, period)
    stance_l = ph < duty
    stance_r = torch.remainder(ph + 0.5, 1.0) < duty
    expected = torch.stack([stance_l, stance_r], dim=-1)
    moving = _moving(env, command_name, command_threshold).unsqueeze(-1)
    expected = torch.where(moving, expected, torch.ones_like(expected))
    return (contact == expected).float().mean(dim=-1)


def swing_height_tracking(
    env: ManagerBasedRlEnv,
    height_sensor_name: str,
    period: float,
    duty: float,
    swing_height: float,
    std: float,
    command_name: str,
    command_threshold: float = 0.1,
) -> torch.Tensor:
    """Positive, dense swing reward driven by the clock: each foot tracks
    h*(s) = swing_height * sin(pi s) over its swing phase s in [0, 1), and 0 in stance.
    Under a stand command both targets are 0.

    v1 lesson: the swing terms were all penalties that fire only when a foot moves or lands
    (feet_clearance, feet_swing_height), so the policy avoided them by shuffling with ~1 cm lift."""
    heights = env.scene[height_sensor_name].data.heights  # [B, 2] sole height above terrain
    ph = _phase(env, period)
    phases = torch.stack([ph, torch.remainder(ph + 0.5, 1.0)], dim=-1)
    s = torch.clamp((phases - duty) / (1.0 - duty), min=0.0)
    target = torch.where(phases >= duty, swing_height * torch.sin(math.pi * s), torch.zeros_like(s))
    moving = _moving(env, command_name, command_threshold).unsqueeze(-1)
    target = torch.where(moving, target, torch.zeros_like(target))
    err = torch.sum(torch.square(heights - target), dim=-1)
    return torch.exp(-err / std**2)


def foot_flat(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Sum over feet of the squared xy of gravity in the foot frame (0 when the sole is level)."""
    asset: Entity = env.scene[asset_cfg.name]
    q = asset.data.body_link_quat_w[:, asset_cfg.body_ids]  # [B, F, 4]
    g = torch.zeros(q.shape[:-1] + (3,), device=q.device)
    g[..., 2] = -1.0
    g_foot = quat_apply_inverse(q, g)
    return torch.sum(torch.square(g_foot[..., :2]), dim=(-1, -2))


def feet_too_close(env: ManagerBasedRlEnv, min_dist: float, asset_cfg: SceneEntityCfg = _ROBOT) -> torch.Tensor:
    """Linear cost when the lateral foot separation (in the heading frame) drops below min_dist."""
    asset: Entity = env.scene[asset_cfg.name]
    p = asset.data.site_pos_w[:, asset_cfg.site_ids]  # [B, 2, 3] (left, right)
    rel = p[:, 0] - p[:, 1]
    rel_b = quat_apply_inverse(yaw_quat(asset.data.root_link_quat_w), rel)
    return torch.clamp(min_dist - rel_b[:, 1], min=0.0)
