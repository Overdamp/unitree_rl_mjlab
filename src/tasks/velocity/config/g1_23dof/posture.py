"""Non-saturating posture terms for the G1-23DoF stairs teacher-v2b (plan step 4).

Why: teacher-v2 (from scratch) climbed straight stairs at wz = 0 (model_15000:
up 10 cm 84%), but by then it walked with the torso twisted ~100 deg
(waist_yaw +1.78 rad), both arms raised (shoulder pitch +1.2 rad) and the knees
bent 1.1 rad (pelvis 0.66 m instead of 0.77 m on flat ground). At 2100 the
posture was still normal. The shared pose reward is exp(-mean(err^2 / std^2)):
with waist_yaw 1.78 rad off at std 0.15 it is already ~exp(-6), so once the
posture leaves its narrow basin the gradient pulling it back is ~0. teacher1-9b
never met this because they started from a checkpoint with a good posture.

These two penalties grow with the error instead of flattening out.
"""

from __future__ import annotations

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg

UPPER_BODY_JOINTS = (r".*waist.*", r".*shoulder.*", r".*elbow.*", r".*wrist.*")
FOOT_SITES = ("left_foot", "right_foot")


def joint_deviation_l2(
  env: ManagerBasedRlEnv,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", joint_names=UPPER_BODY_JOINTS),
) -> torch.Tensor:
  """sum over the selected joints of (q - q_default)^2, rad^2."""
  asset = env.scene[asset_cfg.name]
  ids = asset_cfg.joint_ids
  dq = asset.data.joint_pos[:, ids] - asset.data.default_joint_pos[:, ids]
  return torch.sum(torch.square(dq), dim=1)


def pelvis_height_below(
  env: ManagerBasedRlEnv,
  min_height: float,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", site_names=FOOT_SITES),
) -> torch.Tensor:
  """max(0, min_height - (pelvis z - mean foot-site z)), m. Measured from the
  feet, not the terrain origin, so it means the same on a stair as on flat
  ground (a foot one step higher lowers the mean by half a rise)."""
  asset = env.scene[asset_cfg.name]
  feet_z = asset.data.site_pos_w[:, asset_cfg.site_ids, 2].mean(dim=1)
  h = asset.data.root_link_pos_w[:, 2] - feet_z
  return torch.clamp(min_height - h, min=0.0)
