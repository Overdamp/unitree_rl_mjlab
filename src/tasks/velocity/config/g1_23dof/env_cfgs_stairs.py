"""Phase 4 open-riser stairs environments for G1-23DoF.

Step 1/2 of doc/g1_23dof_stairs_plan.md: a blind task on open-riser stairs.
Its observations match Rough-Blind / Flat (actor 80, critic 95), so existing
blind checkpoints play on it unchanged; that is the step-2 baseline.
"""

from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.observation_manager import (
  ObservationGroupCfg,
  ObservationTermCfg,
)
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.terrains import BoxFlatTerrainCfg

from .env_cfgs import (
  unitree_g1_23dof_rough_blind_env_cfg,
  unitree_g1_23dof_rough_env_cfg,
)
from .depth_camera import d455_sensor_cfg, depth_image, get_spec_with_d455
from .rewards_stairs import feet_clearance_terrain, foot_ground_ray_cfgs
from .terrains_stairs import OpenRiserStairsTerrainCfg

# Placeholders until the target staircase is measured (plan step 0).
# The rise starts at 2 cm: the flat-trained P3b already falls on 86% of 3 cm
# steps going up (step-2 baseline), so the easiest curriculum row must be
# lower than that for the terrain curriculum to get going.
OPEN_STAIRS_RISE = (0.02, 0.18)
OPEN_STAIRS_TREAD = (0.25, 0.32)
OPEN_STAIRS_THICKNESS = (0.03, 0.06)
OPEN_STAIRS_NOSING = (0.0, 0.03)


def open_stairs_sub_terrains() -> dict:
  common = dict(
    step_height_range=OPEN_STAIRS_RISE,
    tread_depth_range=OPEN_STAIRS_TREAD,
    tread_thickness_range=OPEN_STAIRS_THICKNESS,
    nosing_range=OPEN_STAIRS_NOSING,
    platform_width=2.0,
    border_width=1.0,
  )
  return {
    "flat": BoxFlatTerrainCfg(proportion=0.1),
    "open_stairs_down": OpenRiserStairsTerrainCfg(proportion=0.45, inverted=False, **common),
    "open_stairs_up": OpenRiserStairsTerrainCfg(proportion=0.45, inverted=True, **common),
  }


def unitree_g1_23dof_open_stairs_blind_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Blind G1-23DoF on open-riser stairs (pyramid = down, bowl = up)."""
  cfg = unitree_g1_23dof_rough_blind_env_cfg(play=play)

  assert cfg.scene.terrain is not None
  terrain_gen = cfg.scene.terrain.terrain_generator
  assert terrain_gen is not None
  cfg.scene.terrain.terrain_generator = replace(
    terrain_gen, sub_terrains=open_stairs_sub_terrains()
  )
  return cfg


def unitree_g1_23dof_open_stairs_teacher_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Teacher for doc/g1_23dof_stairs_camera_plan.md part A: open-riser stairs
  with the privileged height scan (terrain_scan, 17 x 11 rays under the pelvis)
  in actor and critic. The real robot has no such scan; the student (depth
  camera + lidar) learns from this teacher. Actor obs = 80 + 187, so it trains
  from scratch (P3b checkpoints do not load).
  """
  cfg = unitree_g1_23dof_rough_env_cfg(play=play)

  assert cfg.scene.terrain is not None
  terrain_gen = cfg.scene.terrain.terrain_generator
  assert terrain_gen is not None
  cfg.scene.terrain.terrain_generator = replace(
    terrain_gen, sub_terrains=open_stairs_sub_terrains()
  )

  # Fixes after the first teacher run (12/11, iteration 2428: tracking 0.47,
  # error_vel_xy 1.03 m/s, terrain_levels 0.08), the same changes that made
  # Rough-Blind track:
  # 1. Start every env on the easiest rows (the base config spreads them over
  #    rows 0-5, so a robot that cannot walk yet is demoted from row 5 at once).
  cfg.scene.terrain.max_init_terrain_level = 1
  # 2. Double the linear tracking reward: standing still was paying too well.
  cfg.rewards["track_linear_velocity"].weight = 2.0
  # 3. One command stage inside what deploy can send (vx -0.5..1.0). The base
  #    curriculum widens vx to -1..2 m/s after 5000 iterations, and its step
  #    counter restarts on resume, so it would fire 5000 iterations later.
  stage = {
    "step": 0,
    "lin_vel_x": (-0.5, 1.0),
    "lin_vel_y": (-0.5, 0.5),
    "ang_vel_z": (-1.0, 1.0),
  }
  if "command_vel" in cfg.curriculum:
    cfg.curriculum["command_vel"].params["velocity_stages"] = [stage]
  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.ranges.lin_vel_x = stage["lin_vel_x"]
  twist_cmd.ranges.lin_vel_y = stage["lin_vel_y"]

  # 4. (13/11) Foot clearance from the ground under each foot, not from world
  #    z = 0: the shared term made every step on a tall pyramid/bowl costly and
  #    the teacher stopped in front of stairs from 8 cm (rewards_stairs.py).
  #    Same weight and 0.10 m target; obs unchanged, so checkpoints still load.
  cfg.scene.sensors = (cfg.scene.sensors or ()) + foot_ground_ray_cfgs()
  clearance = cfg.rewards["foot_clearance"]
  cfg.rewards["foot_clearance"] = replace(
    clearance,
    func=feet_clearance_terrain,
    params={
      "target_height": clearance.params["target_height"],
      "command_name": clearance.params["command_name"],
      "command_threshold": clearance.params["command_threshold"],
      "asset_cfg": clearance.params["asset_cfg"],
    },
  )
  return cfg


# Depth frames stacked as CNN channels (current + 2 past): the step under the
# feet leaves the image, so the policy needs a short memory of it.
DEPTH_HISTORY = 3
# Camera latency in policy steps (20 ms each): D455 + transport ~20-60 ms.
DEPTH_DELAY_STEPS = (1, 3)


def unitree_g1_23dof_open_stairs_depth_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Student for camera plan part B (depth only): open-riser stairs where the
  actor sees proprioception (80) + a simulated head D455 depth image, and the
  critic keeps the privileged height scan (asymmetric actor-critic, PPO).
  Observation groups: actor (80), depth (3 x 24 x 40), critic (95 + 187).
  """
  cfg = unitree_g1_23dof_open_stairs_teacher_env_cfg(play=play)

  robot = cfg.scene.entities["robot"]
  cfg.scene.entities["robot"] = replace(robot, spec_fn=get_spec_with_d455)
  cam = d455_sensor_cfg()
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (cam,)

  # The real robot has no height scan: the actor loses it, the critic keeps it.
  del cfg.observations["actor"].terms["height_scan"]
  cfg.observations["depth"] = ObservationGroupCfg(
    terms={
      "depth_image": ObservationTermCfg(
        func=depth_image,
        params={"sensor_name": cam.name},
        delay_min_lag=DEPTH_DELAY_STEPS[0],
        delay_max_lag=DEPTH_DELAY_STEPS[1],
        history_length=DEPTH_HISTORY,
        flatten_history_dim=False,
      )
    },
    enable_corruption=False,
    concatenate_terms=True,
  )
  return cfg
