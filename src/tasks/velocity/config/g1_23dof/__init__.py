from mjlab.tasks.registry import register_mjlab_task
from src.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  unitree_g1_23dof_flat_env_cfg,
  unitree_g1_23dof_rough_blind_env_cfg,
  unitree_g1_23dof_rough_env_cfg,
)
from .env_cfgs_stairs import (
  unitree_g1_23dof_open_stairs_blind_env_cfg,
  unitree_g1_23dof_open_stairs_teacher_env_cfg,
)
from .rl_cfg import unitree_g1_23dof_ppo_runner_cfg

register_mjlab_task(
  task_id="Unitree-G1-23Dof-Rough",
  env_cfg=unitree_g1_23dof_rough_env_cfg(),
  play_env_cfg=unitree_g1_23dof_rough_env_cfg(play=True),
  rl_cfg=unitree_g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Unitree-G1-23Dof-Flat",
  env_cfg=unitree_g1_23dof_flat_env_cfg(),
  play_env_cfg=unitree_g1_23dof_flat_env_cfg(play=True),
  rl_cfg=unitree_g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Unitree-G1-23Dof-Rough-Blind",
  env_cfg=unitree_g1_23dof_rough_blind_env_cfg(),
  play_env_cfg=unitree_g1_23dof_rough_blind_env_cfg(play=True),
  rl_cfg=unitree_g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Unitree-G1-23Dof-OpenStairs-Blind",
  env_cfg=unitree_g1_23dof_open_stairs_blind_env_cfg(),
  play_env_cfg=unitree_g1_23dof_open_stairs_blind_env_cfg(play=True),
  rl_cfg=unitree_g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Unitree-G1-23Dof-OpenStairs-Teacher",
  env_cfg=unitree_g1_23dof_open_stairs_teacher_env_cfg(),
  play_env_cfg=unitree_g1_23dof_open_stairs_teacher_env_cfg(play=True),
  rl_cfg=unitree_g1_23dof_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
