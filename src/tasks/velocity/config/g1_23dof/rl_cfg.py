"""RL configuration for Unitree G1-23DOF velocity task."""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def unitree_g1_23dof_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Create RL runner configuration for Unitree G1-23DOF velocity task."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 1.0,
        "std_type": "scalar",
      },
    ),
    critic=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="g1_23dof_velocity",
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=10001,
  )


def unitree_g1_23dof_depth_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Stairs student (depth camera): CNN actor on (actor, depth), MLP critic on
  the privileged critic group. Same PPO settings as the blind policy."""
  cfg = unitree_g1_23dof_ppo_runner_cfg()
  cfg.actor = RslRlModelCfg(
    hidden_dims=(512, 256, 128),
    activation="elu",
    obs_normalization=True,
    class_name="CNNModel",
    cnn_cfg={
      # Input 3 x 24 x 40 -> 16 x 10 x 18 -> 32 x 4 x 8 -> 32 x 2 x 6, flattened (384).
      "output_channels": [16, 32, 32],
      "kernel_size": [5, 3, 3],
      "stride": [2, 2, 1],
      "padding": "none",
      "activation": "elu",
      "max_pool": False,
      "global_pool": "none",
      "flatten": True,
    },
    distribution_cfg={
      "class_name": "GaussianDistribution",
      "init_std": 1.0,
      "std_type": "scalar",
    },
  )
  cfg.obs_groups = {"actor": ("actor", "depth"), "critic": ("critic",)}
  return cfg
