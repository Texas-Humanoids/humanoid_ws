from mjlab.tasks.registry import register_mjlab_task

from th_training.export import make_runner_cls
from th_training.tasks.walker_v2_velocity import walker_v2_ppo_runner_cfg

from .env_cfg import walker_demo_flat_env_cfg


def _runner_cfg(variant: str):
    cfg = walker_v2_ppo_runner_cfg()
    cfg.experiment_name = f"walker_demo_{variant}"
    cfg.max_iterations = 3000
    return cfg


# CAD = joint limits straight from pkg_002.urdf (hip roll abduction-only, knee <= 60 deg).
# Wide = WalkerV2's limits (hip roll 20 deg adduction, knee 120 deg). Deploy whichever fits
# inside the hard stops measured on the robot (tools/walker/bringup/jog.py).
for _variant, _suffix in (("cad", "Cad"), ("wide", "Wide")):
    register_mjlab_task(
        task_id=f"TH-Velocity-Flat-WalkerDemo-{_suffix}",
        env_cfg=walker_demo_flat_env_cfg(_variant),
        play_env_cfg=walker_demo_flat_env_cfg(_variant, play=True),
        rl_cfg=_runner_cfg(_variant),
        runner_cls=make_runner_cls("walker_demo"),
    )

# Reliability-first: slow commands, wider randomization, gentler motion. CAD limits.
_safe = _runner_cfg("cad")
_safe.experiment_name = "walker_demo_cad_safe"
register_mjlab_task(
    task_id="TH-Velocity-Flat-WalkerDemo-Cad-Safe",
    env_cfg=walker_demo_flat_env_cfg("cad", profile="safe"),
    play_env_cfg=walker_demo_flat_env_cfg("cad", play=True, profile="safe"),
    rl_cfg=_safe,
    runner_cls=make_runner_cls("walker_demo"),
)

# v2: v1 + fixes from the 2026-11-19 real standing tests (env_cfg._make_v2). Needs a bridge
# that supports policy_spec gait_clock "held".
_v2 = _runner_cfg("cad")
_v2.experiment_name = "walker_demo_cad_v2"
register_mjlab_task(
    task_id="TH-Velocity-Flat-WalkerDemo-Cad-V2",
    env_cfg=walker_demo_flat_env_cfg("cad", profile="v2"),
    play_env_cfg=walker_demo_flat_env_cfg("cad", play=True, profile="v2"),
    rl_cfg=_v2,
    runner_cls=make_runner_cls("walker_demo", gait_clock="held"),
)

# v2b: v2 with a symmetric COM range and a standing tilt cost (see env_cfg._make_v2).
_v2b = _runner_cfg("cad")
_v2b.experiment_name = "walker_demo_cad_v2b"
register_mjlab_task(
    task_id="TH-Velocity-Flat-WalkerDemo-Cad-V2b",
    env_cfg=walker_demo_flat_env_cfg("cad", profile="v2b"),
    play_env_cfg=walker_demo_flat_env_cfg("cad", play=True, profile="v2b"),
    rl_cfg=_v2b,
    runner_cls=make_runner_cls("walker_demo", gait_clock="held"),
)

# v3: v2b on the measured real foot (smaller, forward sole; COM behind CAD): env_cfg._make_real_foot.
_v3 = _runner_cfg("cad")
_v3.experiment_name = "walker_demo_cad_v3"
register_mjlab_task(
    task_id="TH-Velocity-Flat-WalkerDemo-Cad-V3",
    env_cfg=walker_demo_flat_env_cfg("cad", profile="v3"),
    play_env_cfg=walker_demo_flat_env_cfg("cad", play=True, profile="v3"),
    rl_cfg=_v3,
    runner_cls=make_runner_cls("walker_demo", gait_clock="held"),
)
