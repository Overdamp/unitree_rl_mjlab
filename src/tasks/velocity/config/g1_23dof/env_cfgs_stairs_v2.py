"""teacher-v2: the G1-23DoF open-riser stairs teacher, trained FROM SCRATCH with
the rules learned from teacher1-9b (plan step 4, 25/11/2026).

teacher1-9b were one lineage of resumes, each a patch on the last
(env_cfgs_stairs.py). All of them, at wz = 0, turned ~90 deg at the first
step and walked along it, because "walk forward" was scored in the body frame
(heading_lock.py). Resuming could not unlearn that, and every resume
restarted the terrain curriculum. This config writes the whole rule set in
one place, starting again from the rough env.

Observations (obs layout fixed from the start, no checkpoint surgery):
  actor  = rough-env proprioception + height scan + stair token (4)
  critic = rough-env privileged obs + height scan + clean stair token (4)

Rules, with the run that showed each one:
  KEPT
  R1  start on the easiest terrain rows                             teacher1
  R2  one command stage inside what deploy can send                 teacher1
  R3  foot clearance from the ground under the foot, below-only     teacher2/3
  R4  angular tracking weight 2.0                                   teacher3
  R5  pose std 1.0 rad for hip pitch / knee while walking           teacher4
  R6  falling costs -100, not -200                                  teacher5
  R7  linear progress reward (pays nothing for standing)            teacher6
  R8  stair token in obs, actor copy noisy + 10% dropout            teacher8/9b
  R9  half-wall-free terrain mix incl. straight flights, Chebyshev
      promotion, demotion capped at 2 m                             teacher9/9b
  NEW (v2b adds N4, N5 below)
  N1  linear tracking and progress in a heading-locked world frame,
      no vertical-velocity error                                     diag 25/11
  N2  heading drift penalty (small)                                 diag 25/11
  N3  80% heading-command envs (mjlab default 30%)                  diag 25/11
  DROPPED
  foothold / stumble penalties: stalling in front of steps    teacher7_foothold, teacher9
  AMP with a flat-ground clip: discriminator collapse          teacher7_amp
"""

import os
from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.terrains import BoxFlatTerrainCfg
from mjlab.utils.noise import GaussianNoiseCfg

from .curriculum_stairs import terrain_levels_chebyshev
from .env_cfgs import unitree_g1_23dof_rough_env_cfg
from .env_cfgs_stairs import (
  OPEN_STAIRS_NOSING,
  OPEN_STAIRS_RISE,
  OPEN_STAIRS_THICKNESS,
  OPEN_STAIRS_TREAD,
)
from .heading_lock import (
  heading_drift,
  heading_locked_from,
  progress_locked,
  track_linear_velocity_locked,
)
from .posture import joint_deviation_l2, pelvis_height_below
from .rewards_stairs import feet_clearance_terrain, foot_ground_ray_cfgs
from .stair_token import TOKEN_NOISE_STD, stair_profile_ray_cfg, stair_token
from .terrains_stairs import OpenRiserStairsTerrainCfg

# N2: 1 - cos(drift) is 1 at 90 deg; progress pays up to 1.0 per second.
V2_HEADING_DRIFT_W = -0.5
# N3
V2_REL_HEADING_ENVS = 0.8
# R8
V2_TOKEN_DROPOUT = 0.1
# R9
V2_DEMOTE_MAX_M = 2.0


def open_stairs_v2_sub_terrains() -> dict:
  """R9: rings (walk out in any direction, turn on stairs) + 30% straight
  flights between side walls (like real stairs, no corners) + 10% flat."""
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
    "rings_down": OpenRiserStairsTerrainCfg(proportion=0.3, inverted=False, **common),
    "rings_up": OpenRiserStairsTerrainCfg(proportion=0.3, inverted=True, **common),
    "straight_down": OpenRiserStairsTerrainCfg(proportion=0.15, inverted=False, straight=True, **common),
    "straight_up": OpenRiserStairsTerrainCfg(proportion=0.15, inverted=True, straight=True, **common),
  }


def unitree_g1_23dof_open_stairs_teacher_v2_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  cfg = unitree_g1_23dof_rough_env_cfg(play=play)

  # Terrain + curriculum (R1, R9).
  assert cfg.scene.terrain is not None
  gen = cfg.scene.terrain.terrain_generator
  assert gen is not None
  cfg.scene.terrain.terrain_generator = replace(gen, sub_terrains=open_stairs_v2_sub_terrains())
  cfg.scene.terrain.max_init_terrain_level = 1
  if "terrain_levels" in cfg.curriculum:
    cfg.curriculum["terrain_levels"].func = terrain_levels_chebyshev
    cfg.curriculum["terrain_levels"].params = {
      **cfg.curriculum["terrain_levels"].params,
      "demote_max_m": V2_DEMOTE_MAX_M,
    }

  # Command (R2, N1, N3). The base curriculum would widen vx to -1..2 m/s
  # after 5000 iterations; deploy sends -0.5..1.0.
  stage = {"step": 0, "lin_vel_x": (-0.5, 1.0), "lin_vel_y": (-0.5, 0.5), "ang_vel_z": (-1.0, 1.0)}
  if "command_vel" in cfg.curriculum:
    cfg.curriculum["command_vel"].params["velocity_stages"] = [stage]
  twist = cfg.commands["twist"]
  assert isinstance(twist, UniformVelocityCommandCfg)
  ranges = replace(twist.ranges, lin_vel_x=stage["lin_vel_x"], lin_vel_y=stage["lin_vel_y"])
  cfg.commands["twist"] = heading_locked_from(
    twist, ranges=ranges, rel_heading_envs=V2_REL_HEADING_ENVS
  )

  # Sensors: ground under each foot (R3), stair profile for the token (R8).
  cfg.scene.sensors = (cfg.scene.sensors or ()) + foot_ground_ray_cfgs() + (stair_profile_ray_cfg(),)

  # Rewards.
  lin = cfg.rewards["track_linear_velocity"]
  cfg.rewards["track_linear_velocity"] = RewardTermCfg(          # N1
    func=track_linear_velocity_locked,
    weight=2.0,
    params={"command_name": "twist", "std": lin.params["std"]},
  )
  cfg.rewards["track_angular_velocity"].weight = 2.0             # R4
  clearance = cfg.rewards["foot_clearance"]
  cfg.rewards["foot_clearance"] = replace(                       # R3
    clearance,
    func=feet_clearance_terrain,
    params={
      "target_height": clearance.params["target_height"],
      "command_name": clearance.params["command_name"],
      "command_threshold": clearance.params["command_threshold"],
      "asset_cfg": clearance.params["asset_cfg"],
      "below_only": True,
    },
  )
  pose = cfg.rewards["pose"]                                     # R5
  std_walking = dict(pose.params["std_walking"])
  std_walking[".*hip_pitch.*"] = 1.0
  std_walking[".*knee.*"] = 1.0
  pose.params = {**pose.params, "std_walking": std_walking}
  cfg.rewards["is_terminated"].weight = -100.0                   # R6
  cfg.rewards["progress"] = RewardTermCfg(                       # R7 + N1
    func=progress_locked,
    weight=1.0,
    params={"command_name": "twist", "command_threshold": 0.1},
  )
  cfg.rewards["heading_drift"] = RewardTermCfg(                  # N2
    func=heading_drift, weight=V2_HEADING_DRIFT_W, params={"command_name": "twist"}
  )

  # Observations: stair token last in actor and critic (R8).
  cfg.observations["critic"].terms["stair_token"] = ObservationTermCfg(func=stair_token)
  if play:
    cfg.observations["actor"].terms["stair_token"] = ObservationTermCfg(func=stair_token)
  else:
    cfg.observations["actor"].terms["stair_token"] = ObservationTermCfg(
      func=stair_token,
      params={"dropout": V2_TOKEN_DROPOUT},
      noise=GaussianNoiseCfg(std=TOKEN_NOISE_STD),
    )
  return cfg


# teacher-v2b (plan step 4, 27/11/2026). Measured walking at vx 0.5 (flat /
# up 12 cm, 64 envs): model_2100 (posture still normal) upper-body sum dq^2
# 0.14 / 0.28, pelvis above the feet 0.755 / 0.734 m; model_15000 (twisted,
# arms up, crouched) 11.9 / 12.2 and 0.640 / 0.636 m.
# N4: -0.5 x 0.14 = -0.07 when normal, -6 when twisted (progress pays <= 1).
V2B_UPPER_BODY_W = float(os.environ.get("V2B_UPPER_BODY_W", "-0.5"))
# N5: between normal (0.73-0.755) and crouched (0.64); -20 x 0.06 m = -1.2.
V2B_MIN_PELVIS_H = 0.70
V2B_PELVIS_H_W = float(os.environ.get("V2B_PELVIS_H_W", "-20.0"))
# Fine-tuning a robot that already climbs: start a little higher than R1 so
# the reset curriculum (levels are not in the checkpoint) has less to redo.
V2B_INIT_LEVEL = int(os.environ.get("V2B_INIT_LEVEL", "1"))


def unitree_g1_23dof_open_stairs_teacher_v2b_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """teacher-v2 + N4 upper-body deviation penalty + N5 pelvis height floor
  (posture.py). Same observations, so a teacher-v2 checkpoint loads as is."""
  cfg = unitree_g1_23dof_open_stairs_teacher_v2_env_cfg(play=play)
  assert cfg.scene.terrain is not None
  cfg.scene.terrain.max_init_terrain_level = V2B_INIT_LEVEL
  cfg.rewards["upper_body_deviation"] = RewardTermCfg(            # N4
    func=joint_deviation_l2, weight=V2B_UPPER_BODY_W, params={}
  )
  cfg.rewards["pelvis_height"] = RewardTermCfg(                   # N5
    func=pelvis_height_below,
    weight=V2B_PELVIS_H_W,
    params={"min_height": V2B_MIN_PELVIS_H},
  )
  return cfg
