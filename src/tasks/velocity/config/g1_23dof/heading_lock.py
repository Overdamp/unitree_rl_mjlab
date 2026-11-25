"""Heading-locked velocity command and rewards for G1-23DoF teacher-v2
(plan step 4).

Why: every stairs teacher up to teacher9b turned ~90 deg at the first step
and walked along it when the command had wz = 0 (diag 25/11: x stops at the
first edge, |yaw| median 15 -> 91 deg between 2 s and 4 s, forward speed
still 0.5 m/s). The shared tracking reward and our progress reward both
measure velocity in the BODY frame, so "walk forward" is fully paid in
whatever direction the robot faces: turning away from a step costs only a
moment of yaw-rate error, climbing it risks a fall.

Fix: score walking against a reference heading that the ROBOT cannot move.
  heading envs  ref = heading_target (world, fixed until the next resample);
                the command's wz = k * (target - yaw) also keeps pushing
                back while the robot faces away, so turning away is costly
                for as long as it lasts.
  wz envs       ref = robot heading at the resample + the integral of the
                commanded wz since then.
The intended world velocity is the body-frame command rotated by ref; the
rewards compare the world velocity with that. The policy obs (the 3-value
command) is unchanged, so deploy is unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand
from mjlab.utils.lab_api.math import wrap_to_pi


class HeadingLockedVelocityCommand(UniformVelocityCommand):
  cfg: HeadingLockedVelocityCommandCfg

  def __init__(self, cfg: HeadingLockedVelocityCommandCfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    self.ref_yaw = torch.zeros(self.num_envs, device=self.device)
    self.metrics["heading_drift"] = torch.zeros(self.num_envs, device=self.device)

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    super()._resample_command(env_ids)
    self.ref_yaw[env_ids] = self.robot.data.heading_w[env_ids]

  def _update_command(self) -> None:
    super()._update_command()
    # wz envs: follow the commanded turn rate (standing envs: wz = 0).
    self.ref_yaw = wrap_to_pi(self.ref_yaw + self.vel_command_b[:, 2] * self._env.step_dt)
    if self.cfg.heading_command:
      # A standing env is told wz = 0, so it keeps its resample heading.
      locked = self.is_heading_env & ~self.is_standing_env
      self.ref_yaw = torch.where(locked, self.heading_target, self.ref_yaw)

  def _update_metrics(self) -> None:
    super()._update_metrics()
    max_command_step = self.cfg.resampling_time_range[1] / self._env.step_dt
    self.metrics["heading_drift"] += (
      wrap_to_pi(self.robot.data.heading_w - self.ref_yaw).abs() / max_command_step
    )

  def command_w(self) -> torch.Tensor:
    """(B, 2) intended world xy velocity: the body command rotated by ref."""
    c, s = torch.cos(self.ref_yaw), torch.sin(self.ref_yaw)
    vx, vy = self.vel_command_b[:, 0], self.vel_command_b[:, 1]
    return torch.stack([c * vx - s * vy, s * vx + c * vy], dim=1)


@dataclass(kw_only=True)
class HeadingLockedVelocityCommandCfg(UniformVelocityCommandCfg):
  def build(self, env: ManagerBasedRlEnv) -> HeadingLockedVelocityCommand:
    return HeadingLockedVelocityCommand(self, env)


def heading_locked_from(cfg: UniformVelocityCommandCfg, **overrides) -> HeadingLockedVelocityCommandCfg:
  """Same settings as an existing UniformVelocityCommandCfg."""
  from dataclasses import fields

  kw = {f.name: getattr(cfg, f.name) for f in fields(cfg) if f.init}
  kw.update(overrides)
  return HeadingLockedVelocityCommandCfg(**kw)


def _term(env: ManagerBasedRlEnv, command_name: str) -> HeadingLockedVelocityCommand:
  term = env.command_manager.get_term(command_name)
  assert isinstance(term, HeadingLockedVelocityCommand), (
    f"command '{command_name}' must be a HeadingLockedVelocityCommandCfg"
  )
  return term


def track_linear_velocity_locked(
  env: ManagerBasedRlEnv,
  std: float,
  command_name: str,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
  """exp(-|v_xy_world - command_w|^2 / std^2). Unlike the shared term there is
  no vertical-velocity error: climbing a step needs vertical speed, and the
  shared term charged it."""
  asset = env.scene[asset_cfg.name]
  err = torch.sum(torch.square(asset.data.root_link_lin_vel_w[:, :2] - _term(env, command_name).command_w()), dim=1)
  return torch.exp(-err / std**2)


def progress_locked(
  env: ManagerBasedRlEnv,
  command_name: str,
  command_threshold: float = 0.1,
  asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
  """Linear reward in [0, 1]: world speed along command_w as a fraction of the
  commanded speed (capped at 1, 0 below the threshold). Pays nothing for
  standing (rewards_stairs.progress_along_command) and, unlike it, nothing for
  walking in a direction the robot turned to by itself."""
  asset = env.scene[asset_cfg.name]
  cmd_w = _term(env, command_name).command_w()
  speed = torch.norm(cmd_w, dim=1)
  active = speed > command_threshold
  along = torch.sum(asset.data.root_link_lin_vel_w[:, :2] * cmd_w, dim=1) / speed.clamp(min=1e-6)
  frac = torch.clamp(along, min=0.0) / speed.clamp(min=1e-6)
  return torch.clamp(frac, max=1.0) * active.float()


def heading_drift(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
  """1 - cos(yaw - ref) in [0, 2]: 0.13 at 30 deg, 1 at 90 deg. Use a negative
  weight well below the progress weight, or standing still in front of a step
  (no drift, no progress) could beat climbing it."""
  term = _term(env, command_name)
  return 1.0 - torch.cos(term.robot.data.heading_w - term.ref_yaw)
