"""Terrain curriculum for the G1-23DoF stairs teacher9 (plan step 3i).

mjlab's terrain_levels_vel promotes a robot that ends its episode farther
than half a patch from its spawn point in straight-line (Euclidean) distance.
On the square stair rings a diagonal walk reaches that distance while
climbing fewer steps (the rings are square, the distance is round), so the
curriculum paid the policy for heading to the corners. Here the walked
distance is the Chebyshev distance max(|dx|, |dy|): on square rings it says
which ring the robot reached, on straight flights between walls it is the
distance along the flight. Demotion is Euclidean as in mjlab (walked less
than half the commanded distance), optionally capped at demote_max_m
(teacher9b): between the walls of a straight flight a robot commanded
sideways cannot walk the ~5 m mjlab asks for and was demoted for it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


def terrain_levels_chebyshev(
  env: ManagerBasedRlEnv,
  env_ids: torch.Tensor,
  command_name: str,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
  demote_max_m: float | None = None,
) -> torch.Tensor:
  asset = env.scene[asset_cfg.name]
  terrain = env.scene.terrain
  assert terrain is not None
  terrain_generator = terrain.cfg.terrain_generator
  assert terrain_generator is not None
  command = env.command_manager.get_command(command_name)
  assert command is not None

  d = asset.data.root_link_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2]
  move_up = d.abs().max(dim=1).values > terrain_generator.size[0] / 2
  need = torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
  if demote_max_m is not None:
    need = need.clamp(max=demote_max_m)
  move_down = torch.norm(d, dim=1) < need
  move_down *= ~move_up
  terrain.update_env_origins(env_ids, move_up, move_down)
  return torch.mean(terrain.terrain_levels.float())
