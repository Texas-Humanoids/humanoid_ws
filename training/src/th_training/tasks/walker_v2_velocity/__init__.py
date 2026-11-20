from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.tasks.registry import register_mjlab_task
from th_training.export import make_runner_cls

from .env_cfg import walker_v2_flat_env_cfg


def walker_v2_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    """Same PPO recipe as mjlab's G1 velocity task. 10 DOF and ~50 obs don't need a bigger net."""
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(256, 128, 128),
            activation="elu",
            obs_normalization=True,
            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"},
        ),
        critic=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.01,
            num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-3, schedule="adaptive",
            gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
        ),
        experiment_name="walker_v2_velocity",
        save_interval=100,
        num_steps_per_env=24,
        max_iterations=4000,
    )


register_mjlab_task(
    task_id="TH-Velocity-Flat-WalkerV2",
    env_cfg=walker_v2_flat_env_cfg(),
    play_env_cfg=walker_v2_flat_env_cfg(play=True),
    rl_cfg=walker_v2_ppo_runner_cfg(),
    runner_cls=make_runner_cls(),
)
