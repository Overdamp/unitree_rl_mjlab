"""Stairs rewards for G1-23DoF that measure foot height from the ground below.

The shared `foot_clearance` (src/tasks/velocity/mdp/rewards.py) compares the
foot with z = target_height in WORLD coordinates. On the open-riser terrain
the walking surface sits far from z = 0 (pyramid top at (N+1) * rise, e.g.
0.64 m at 8 cm; bowl floor at -N * rise), so every moving foot pays a large
cost there and standing still wins: the 13/11 teacher stopped in front of
every staircase from 8 cm up (doc/g1_23dof_stairs_plan.md, step 3).

Here each foot casts one ray straight down (world frame) from its ankle roll
link origin, 3.7 cm above the sole site; distance minus that offset is the
sole's height above whatever is under it, tread or floor. (A ray from the sole
site itself starts below the surface whenever the foot sinks a few mm into the
ground and then misses it.) The shared reward and mdp/* stay untouched.
"""

from __future__ import annotations

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ObjRef, RayCastSensorCfg
from mjlab.sensor import GridPatternCfg
from mjlab.sensor.raycast_sensor import RayCastSensor

FOOT_SITES = ("left_foot", "right_foot")
FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")
# The foot sites sit at z = -0.037 in the ankle roll link (g1_23dof.xml).
SOLE_BELOW_ANKLE_M = 0.037
FOOT_RAY_MAX_M = 2.0


def foot_ground_ray_cfgs() -> tuple[RayCastSensorCfg, ...]:
  """One downward ray per foot, hitting terrain geoms only (group 0), so the
  other leg's visual (group 2) or collision (group 3) geoms are never read
  as ground."""
  return tuple(
    RayCastSensorCfg(
      name=f"{site}_ground_ray",
      frame=ObjRef(type="body", name=body, entity="robot"),
      pattern=GridPatternCfg(size=(0.0, 0.0), resolution=0.1),  # a single ray
      ray_alignment="world",
      max_distance=FOOT_RAY_MAX_M,
      exclude_parent_body=True,
      include_geom_groups=(0,),
    )
    for site, body in zip(FOOT_SITES, FOOT_BODIES)
  )


def foot_height_above_ground(env: ManagerBasedRlEnv) -> torch.Tensor:
  """(B, 2) height of each sole above the surface below it, in meters."""
  heights = []
  for site in FOOT_SITES:
    sensor: RayCastSensor = env.scene[f"{site}_ground_ray"]
    d = sensor.data.distances[:, 0]
    # A miss means nothing within FOOT_RAY_MAX_M below (should not happen on
    # this terrain); read it as far above ground rather than as zero.
    h = d - SOLE_BELOW_ANKLE_M
    heights.append(torch.where(d < 0, torch.full_like(d, FOOT_RAY_MAX_M), h))
  return torch.stack(heights, dim=1)


def feet_clearance_terrain(
  env: ManagerBasedRlEnv,
  target_height: float,
  command_name: str | None = None,
  command_threshold: float = 0.1,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", site_names=FOOT_SITES),
) -> torch.Tensor:
  """Same cost as the shared foot_clearance (|h - target| x foot xy speed),
  with h measured from the ground under the foot instead of from z = 0.
  Over a step edge the ground below rises, so the swing foot must lift above
  the next tread, not just 10 cm above the floor."""
  asset = env.scene[asset_cfg.name]
  foot_vel_xy = asset.data.site_lin_vel_w[:, asset_cfg.site_ids, :2]
  vel_norm = torch.norm(foot_vel_xy, dim=-1)
  delta = torch.abs(foot_height_above_ground(env) - target_height)
  cost = torch.sum(delta * vel_norm, dim=1)
  if command_name is not None:
    command = env.command_manager.get_command(command_name)
    if command is not None:
      total = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
      cost = cost * (total > command_threshold).float()
  return cost
