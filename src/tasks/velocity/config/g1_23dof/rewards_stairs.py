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

from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor, ObjRef, RayCastSensorCfg
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
  below_only: bool = False,
) -> torch.Tensor:
  """Same cost as the shared foot_clearance (|h - target| x foot xy speed),
  with h measured from the ground under the foot instead of from z = 0.
  Over a step edge the ground below rises, so the swing foot must lift above
  the next tread, not just 10 cm above the floor.

  below_only=True charges only a foot lower than the target,
  max(0, target - h): stepping UP the swing foot must pass rise + clearance
  above the lower tread before the ground under it rises, and the two-sided
  cost charged exactly that lift (teacher3, 16/11: up 8 cm 31% -> 5%)."""
  asset = env.scene[asset_cfg.name]
  foot_vel_xy = asset.data.site_lin_vel_w[:, asset_cfg.site_ids, :2]
  vel_norm = torch.norm(foot_vel_xy, dim=-1)
  diff = target_height - foot_height_above_ground(env)
  delta = torch.clamp(diff, min=0.0) if below_only else torch.abs(diff)
  cost = torch.sum(delta * vel_norm, dim=1)
  if command_name is not None:
    command = env.command_manager.get_command(command_name)
    if command is not None:
      total = torch.norm(command[:, :2], dim=1) + torch.abs(command[:, 2])
      cost = cost * (total > command_threshold).float()
  return cost


def progress_along_command(
  env: ManagerBasedRlEnv,
  command_name: str,
  command_threshold: float = 0.1,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
  """Linear reward in [0, 1] for base speed along the commanded xy direction,
  as a fraction of the commanded speed (body frame, capped at the command so
  overspeed earns nothing; 0 while the command is below the threshold).

  The shared tracking reward exp(-err^2 / 0.25) still pays 37% when a robot
  told to walk at 0.5 m/s stands still, so stopping in front of a tall step
  costs little against the fall it avoids (teacher4/5 stopped at every edge
  from 10 cm). This term pays nothing for standing."""
  asset = env.scene[asset_cfg.name]
  command = env.command_manager.get_command(command_name)
  assert command is not None, f"Command '{command_name}' not found."
  cmd_xy = command[:, :2]
  speed_cmd = torch.norm(cmd_xy, dim=1)
  active = speed_cmd > command_threshold
  direction = cmd_xy / speed_cmd.clamp(min=1e-6).unsqueeze(1)
  along = torch.sum(asset.data.root_link_lin_vel_b[:, :2] * direction, dim=1)
  frac = torch.clamp(along, min=0.0) / speed_cmd.clamp(min=1e-6)
  return torch.clamp(frac, max=1.0) * active.float()


# --- teacher7 (19/11): where the stance foot stands -------------------------
# foothold_open_stairs.py on teacher6 model_24500 (doc/g1_23dof_stairs_plan.md,
# step 3f): going up the feet land on the back of the tread, sole centre
# -6 cm (5 cm rise) to -13 cm (10 cm) behind the tread centre, heel over the
# riser gap on 40-52% of touchdowns of robots that made it and 68-70% of
# those that did not (support 29-44% of the sole). Nothing rewarded where the
# foot lands; these two terms do.

# Sole points in the ankle roll link (g1_23dof.xml foot capsules span
# x -0.054..0.132, y +-0.026, bottom z -0.035): heel / middle / toe x
# left / right edge.
SOLE_POINTS_X = (-0.045, 0.04, 0.125)
SOLE_POINTS_Y = (-0.02, 0.02)
# A sole point whose ground is this much below the foot's highest ground is
# over an edge (the open stairs start at 2 cm rise).
FOOTHOLD_GAP_M = 0.015


@dataclass
class SolePointsPatternCfg(GridPatternCfg):
  """Parallel rays from fixed points of the sole (frame-local x, y; z = 0),
  instead of a grid centred on the frame."""

  xs: tuple[float, ...] = SOLE_POINTS_X
  ys: tuple[float, ...] = SOLE_POINTS_Y

  def generate_rays(self, mj_model, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    pts = [(x, y, 0.0) for x in self.xs for y in self.ys]
    offsets = torch.tensor(pts, device=device, dtype=torch.float32)
    d = torch.tensor(self.direction, device=device, dtype=torch.float32)
    return offsets, (d / d.norm()).expand(len(pts), 3).clone()


def foot_sole_ray_cfgs() -> tuple[RayCastSensorCfg, ...]:
  """Six downward rays per foot from the ankle roll link origin height (3.5 cm
  above the sole, so a foot sunk a few mm still starts above the ground),
  laid out along the sole and following the foot's yaw; terrain only."""
  return tuple(
    RayCastSensorCfg(
      name=f"{site}_sole_rays",
      frame=ObjRef(type="body", name=body, entity="robot"),
      pattern=SolePointsPatternCfg(),
      ray_alignment="yaw",
      max_distance=FOOT_RAY_MAX_M,
      exclude_parent_body=True,
      include_geom_groups=(0,),
    )
    for site, body in zip(FOOT_SITES, FOOT_BODIES)
  )


def foothold_unsupported(
  env: ManagerBasedRlEnv,
  sensor_name: str = "feet_ground_contact",
  gap: float = FOOTHOLD_GAP_M,
) -> torch.Tensor:
  """(B,) sum over feet in contact of the share of sole points whose ground
  lies more than `gap` below the highest ground under that foot (the surface
  it stands on): heel over the riser gap, toe past the nosing. 0 for a foot
  on one level, also while it rolls heel-to-toe (comparing with the sole
  height instead charged ~0.18 per step on flat ground). Up to 1 per foot.
  Uses only the geometry under the foot, so it works on any terrain without
  knowing where the edges are."""
  contact: ContactSensor = env.scene[sensor_name]
  found = contact.data.found[:, :2] > 0
  shares = []
  for site in FOOT_SITES:
    data = env.scene[f"{site}_sole_rays"].data
    z = torch.where(data.distances < 0, torch.full_like(data.distances, -1e3), data.hit_pos_w[..., 2])
    top = z.max(dim=1, keepdim=True).values
    shares.append((top - z > gap).float().mean(dim=1))
  return torch.sum(torch.stack(shares, dim=1) * found.float(), dim=1)


def feet_stumble(
  env: ManagerBasedRlEnv,
  sensor_name: str = "feet_ground_contact",
  ratio: float = 1.0,
  min_force: float = 20.0,
) -> torch.Tensor:
  """(B,) number of feet whose ground contact force is more horizontal than
  vertical (|F_xy| > ratio * |F_z| and > min_force N): toe or heel pushing
  on an edge. Robots that failed in foothold_open_stairs.py had 3-30x more
  such steps than robots that made it."""
  contact: ContactSensor = env.scene[sensor_name]
  force = contact.data.force[:, :2]
  found = contact.data.found[:, :2] > 0
  horiz = torch.norm(force[..., :2], dim=-1)
  hit = found & (horiz > ratio * force[..., 2].abs()) & (horiz > min_force)
  return hit.float().sum(dim=1)
