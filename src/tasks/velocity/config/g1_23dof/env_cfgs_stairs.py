"""Phase 4 open-riser stairs environments for G1-23DoF.

Step 1/2 of doc/g1_23dof_stairs_plan.md: a blind task on open-riser stairs.
Its observations match Rough-Blind / Flat (actor 80, critic 95), so existing
blind checkpoints play on it unchanged; that is the step-2 baseline.
"""

from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.terrains import BoxFlatTerrainCfg

from .env_cfgs import unitree_g1_23dof_rough_blind_env_cfg
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
