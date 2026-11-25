"""Phase 4 open-riser stairs environments for G1-23DoF.

Step 1/2 of doc/g1_23dof_stairs_plan.md: a blind task on open-riser stairs.
Its observations match Rough-Blind / Flat (actor 80, critic 95), so existing
blind checkpoints play on it unchanged; that is the step-2 baseline.
"""

import os
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
from .amp_ppo import AMP_OBS_GROUP, amp_state
from .depth_camera import d455_sensor_cfg, depth_image, get_spec_with_d455
from mjlab.managers.reward_manager import RewardTermCfg

from .rewards_stairs import (
  feet_clearance_terrain,
  feet_stumble,
  foot_ground_ray_cfgs,
  foot_sole_ray_cfgs,
  foothold_unsupported,
  progress_along_command,
)
from mjlab.utils.noise import GaussianNoiseCfg

from .stair_token import TOKEN_NOISE_STD, stair_profile_ray_cfg, stair_token
from .curriculum_stairs import terrain_levels_chebyshev
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


def open_stairs_straight_sub_terrains() -> dict:
  """(24/11, teacher9) Half the stairs as straight flights between side walls
  (no corners to climb at), half as the old rings so turning on stairs and
  walking out in any direction stay in training."""
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
    "open_stairs_down": OpenRiserStairsTerrainCfg(proportion=0.2, inverted=False, **common),
    "open_stairs_up": OpenRiserStairsTerrainCfg(proportion=0.2, inverted=True, **common),
    "straight_stairs_down": OpenRiserStairsTerrainCfg(
      proportion=0.25, inverted=False, straight=True, **common
    ),
    "straight_stairs_up": OpenRiserStairsTerrainCfg(
      proportion=0.25, inverted=True, straight=True, **common
    ),
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
      # 5. (16/11) Charge only feet below the target: the two-sided cost also
      #    charged the extra lift needed to step UP (teacher3: down 8 cm
      #    2% -> 95%, but up 8 cm 31% -> 5%).
      "below_only": True,
    },
  )
  # 6. (16/11) Angular tracking as strong as linear: with wz = 0 commanded
  #    teacher3 turned up to 2.3 rad off course and walked around the stairs
  #    (error_vel_yaw ~2.5 through teacher2/3; heading control hid it in
  #    training).
  cfg.rewards["track_angular_velocity"].weight = 2.0
  # 7. (17/11) Let the knee and hip pitch bend far from the home pose while
  #    walking: a 12-15 cm step needs ~0.9 rad more knee than standing, and
  #    with std 0.5 rad the pose reward was nearly gone there (teacher4
  #    stopped at every edge from 10 cm; curriculum stuck near 7 cm).
  #    Copy the dict so the base config's (shared) one is left as it was.
  pose = cfg.rewards["pose"]
  std_walking = dict(pose.params["std_walking"])
  std_walking[".*hip_pitch.*"] = 1.0
  std_walking[".*knee.*"] = 1.0
  pose.params = {**pose.params, "std_walking": std_walking}
  # 8. (18/11) Make falling cheaper: is_terminated -200 -> -100. teacher5
  #    fell half as often as at mid-run but its curriculum fell back to ~7 cm
  #    (terrain_levels 3.04 -> 2.74): a fall (-200 x 0.02 = -4 plus the lost
  #    future reward) cost more than standing at the edge (~-1 over 2 s).
  cfg.rewards["is_terminated"].weight = -100.0
  # 9. (18/11) Pay for actual progress along the command (rewards_stairs.py):
  #    linear, 0 when standing, so stopping at an edge now loses up to
  #    1.0 x dt per step on top of the tracking reward.
  cfg.rewards["progress"] = RewardTermCfg(
    func=progress_along_command,
    weight=1.0,
    params={"command_name": "twist", "command_threshold": 0.1},
  )
  # 10. (19/11, teacher7) Land the stance foot on the tread. teacher6 lifted
  #     its feet over the edge but kept its step length: going up the sole
  #     centre landed 6-13 cm behind the tread centre with the heel over the
  #     riser gap (foothold_open_stairs.py, plan step 3f). Charge the share of
  #     six sole points whose ground is > 1.5 cm below the highest ground under
  #     that foot, per foot in contact (0..2 per step).
  cfg.scene.sensors = cfg.scene.sensors + foot_sole_ray_cfgs()
  cfg.rewards["foothold"] = RewardTermCfg(
    func=foothold_unsupported, weight=-1.0, params={"sensor_name": "feet_ground_contact"}
  )
  # 11. (19/11, teacher7) Charge feet pushing on an edge (contact force more
  #     horizontal than vertical, > 20 N): 3-30x more of these steps in
  #     robots that failed than in robots that made it.
  cfg.rewards["stumble"] = RewardTermCfg(
    func=feet_stumble, weight=-0.5, params={"sensor_name": "feet_ground_contact"}
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


def _drop_foothold_stumble(cfg: ManagerBasedRlEnvCfg) -> ManagerBasedRlEnvCfg:
  """(20/11) Remove the foothold / stumble rewards and their sole rays: with
  them the robot stalled in front of steps (teacher7_foothold, plan step 3g)."""
  for name in ("foothold", "stumble"):
    del cfg.rewards[name]
  cfg.scene.sensors = tuple(s for s in cfg.scene.sensors if not s.name.endswith("_sole_rays"))
  return cfg


def unitree_g1_23dof_open_stairs_teacher_amp_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """Teacher + AMP style reward (amp_ppo.py): the teacher env plus an "amp"
  observation group (61 features of the current frame, no noise, no history)
  that only the AMP discriminator reads. Actor and critic obs are unchanged,
  so teacher checkpoints resume here and the exported policy is the same.
  """
  # The AMP run resumes the clip-only teacher7, so this is teacher7 + style.
  cfg = _drop_foothold_stumble(unitree_g1_23dof_open_stairs_teacher_env_cfg(play=play))
  cfg.observations[AMP_OBS_GROUP] = ObservationGroupCfg(
    terms={"amp_state": ObservationTermCfg(func=amp_state)},
    enable_corruption=False,
    concatenate_terms=True,
  )
  return cfg


def unitree_g1_23dof_open_stairs_teacher8_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """teacher8 (plan step 3h): clip-only teacher7 + an explicit stair geometry
  token (stair_token.py: rise, tread depth, yaw to the edge, edge distance)
  appended LAST to actor and critic obs (actor 267 + 4, critic 282 + 4), so a
  teacher7 checkpoint widened by expand_checkpoint_obs.py resumes here and
  starts out acting exactly like teacher7. The height scan stays: the token
  only describes regular stairs.
  """
  cfg = _drop_foothold_stumble(unitree_g1_23dof_open_stairs_teacher_env_cfg(play=play))
  cfg.scene.sensors = cfg.scene.sensors + (stair_profile_ray_cfg(),)
  for group in ("actor", "critic"):
    cfg.observations[group].terms["stair_token"] = ObservationTermCfg(func=stair_token)
  return cfg


def unitree_g1_23dof_open_stairs_teacher7_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """(24/11) Clean control for teacher8 (plan step 3i): the clip-only teacher7
  env as it was trained (teacher env minus foothold / stumble), plain PPO, no
  AMP, no token. Resuming teacher7 here for as many iterations as teacher8
  tells whether teacher8's gain came from the token or from more training
  (the AMP run, the first control, used a different runner)."""
  return _drop_foothold_stumble(unitree_g1_23dof_open_stairs_teacher_env_cfg(play=play))


# teacher9 foothold weight; override with TEACHER9_FOOTHOLD_W (read when the
# task registry imports this module, i.e. at train / play start).
TEACHER9_FOOTHOLD_W = -3.0
TEACHER9_TOKEN_DROPOUT = 0.1


def unitree_g1_23dof_open_stairs_teacher9_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """teacher9 (plan step 3i): teacher8 + two changes, resumed from teacher8
  (obs unchanged: actor 271, critic 286).
  1. Foothold reward back at 3x teacher7_foothold's weight, WITHOUT the
     stumble term. teacher8 still lands its sole centre 9.4 cm behind the
     tread centre with the heel over the gap on 44% of touchdowns going up
     10 cm (target: +-5 cm, < 15%). At -1 the term stayed flat at -0.072 per
     episode; teacher7_foothold also had stumble -0.5, the likelier cause of
     its stalling at edges.
  2. Actor token noise (TOKEN_NOISE_STD) and 10% dropout to the flat token,
     training only, to prepare for a student that estimates the token from
     depth. The critic keeps the clean token.
  3. Climb straight: teacher8 climbed at the ring corners (diagonal = sqrt(2)
     longer treads, gentler slope) and drifted off the straight eval stairs
     with wz = 0 (up 10 cm: 23%). Half the stair patches are now straight
     flights between walls, and the curriculum promotes by Chebyshev
     distance, so a diagonal walk no longer reaches the next level with
     fewer steps climbed (curriculum_stairs.py).
  """
  cfg = unitree_g1_23dof_open_stairs_teacher8_env_cfg(play=play)
  assert cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None
  cfg.scene.terrain.terrain_generator = replace(
    cfg.scene.terrain.terrain_generator, sub_terrains=open_stairs_straight_sub_terrains()
  )
  if "terrain_levels" in cfg.curriculum:
    cfg.curriculum["terrain_levels"].func = terrain_levels_chebyshev
  cfg.scene.sensors = cfg.scene.sensors + foot_sole_ray_cfgs()
  weight = float(os.environ.get("TEACHER9_FOOTHOLD_W", TEACHER9_FOOTHOLD_W))
  cfg.rewards["foothold"] = RewardTermCfg(
    func=foothold_unsupported, weight=weight, params={"sensor_name": "feet_ground_contact"}
  )
  if not play:
    cfg.observations["actor"].terms["stair_token"] = ObservationTermCfg(
      func=stair_token,
      params={"dropout": TEACHER9_TOKEN_DROPOUT},
      noise=GaussianNoiseCfg(std=TOKEN_NOISE_STD),
    )
  return cfg


# teacher9b: robots on a straight flight whose command points at a side wall
# stop ~3 m out; teacher9 demoted them (mjlab wants half the commanded
# distance, ~5 m), so its curriculum sat at ~2.5 (~6 cm rises).
TEACHER9B_DEMOTE_MAX_M = 2.0


def unitree_g1_23dof_open_stairs_teacher9b_env_cfg(
  play: bool = False,
) -> ManagerBasedRlEnvCfg:
  """teacher9b (plan step 3i): teacher9 without the foothold reward and with a
  capped demotion distance; resumed from teacher8 (obs unchanged).
  teacher9 (25/11) stalled in front of the first step (up 10 cm hold: 97% ->
  4%, steps median 0; wz0 up 3 cm 12.5%): the foothold penalty is only paid
  standing ON stairs, so standing before them was cheaper, the same failure
  as teacher7_foothold (so not the stumble term), and its curriculum sat at
  ~2.5 (see TEACHER9B_DEMOTE_MAX_M). Kept: straight flights + Chebyshev
  promotion, actor token noise + dropout.
  """
  cfg = unitree_g1_23dof_open_stairs_teacher9_env_cfg(play=play)
  del cfg.rewards["foothold"]
  cfg.scene.sensors = tuple(s for s in cfg.scene.sensors if not s.name.endswith("_sole_rays"))
  if "terrain_levels" in cfg.curriculum:
    cfg.curriculum["terrain_levels"].params = {
      **cfg.curriculum["terrain_levels"].params,
      "demote_max_m": TEACHER9B_DEMOTE_MAX_M,
    }
  return cfg
