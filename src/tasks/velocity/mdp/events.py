"""Event terms that extend mjlab's domain randomization."""

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import torch

from mjlab.actuator import BuiltinPositionActuator, XmlPositionActuator
from mjlab.actuator.delayed_actuator import DelayedActuator
from mjlab.envs.mdp.dr._types import resolve_distribution
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv


@requires_model_fields("actuator_forcerange")
def delayed_effort_limits(
  env: ManagerBasedRlEnv,
  env_ids: torch.Tensor | None,
  effort_limit_range: tuple[float, float],
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
  distribution: Literal["uniform", "log_uniform"] = "uniform",
  operation: Literal["scale", "abs"] = "scale",
) -> None:
  """Randomize actuator effort limits, unwrapping DelayedActuator.

  Same behaviour as ``mjlab.envs.mdp.dr.effort_limits`` for built-in position
  actuators. mjlab 1.2.0's version rejects DelayedActuator wrappers, while
  ``pd_gains`` already unwraps them.
  """
  asset = env.scene[asset_cfg.name]

  if env_ids is None:
    env_ids = torch.arange(env.num_envs, device=env.device, dtype=torch.int)
  else:
    env_ids = env_ids.to(env.device, dtype=torch.int)

  if isinstance(asset_cfg.actuator_ids, list):
    actuators = [asset.actuators[i] for i in asset_cfg.actuator_ids]
  else:
    actuators = asset.actuators[asset_cfg.actuator_ids]
  if not isinstance(actuators, list):
    actuators = [actuators]

  default_forcerange = env.sim.get_default_field("actuator_forcerange")
  dist = resolve_distribution(distribution)

  for actuator in actuators:
    base = actuator.base_actuator if isinstance(actuator, DelayedActuator) else actuator
    if not isinstance(base, (BuiltinPositionActuator, XmlPositionActuator)):
      raise TypeError(
        f"delayed_effort_limits only supports BuiltinPositionActuator and "
        f"XmlPositionActuator (optionally wrapped with DelayedActuator), "
        f"got {type(base).__name__}"
      )

    ctrl_ids = actuator.global_ctrl_ids
    samples = dist.sample(
      torch.tensor(effort_limit_range[0], device=env.device),
      torch.tensor(effort_limit_range[1], device=env.device),
      (len(env_ids), len(ctrl_ids)),
      env.device,
    )
    if operation == "scale":
      low = default_forcerange[ctrl_ids, 0] * samples
      high = default_forcerange[ctrl_ids, 1] * samples
    else:
      low, high = -samples, samples
    env.sim.model.actuator_forcerange[env_ids[:, None], ctrl_ids, 0] = low
    env.sim.model.actuator_forcerange[env_ids[:, None], ctrl_ids, 1] = high
