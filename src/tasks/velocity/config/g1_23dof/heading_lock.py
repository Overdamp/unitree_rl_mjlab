"""Heading-locked velocity command and rewards for G1-23DoF teacher-v2
(plan step 4).

Why: every stairs teacher up to teacher9b turned ~90 deg at the first step
and walked along it when the command had wz = 0 (diag 25/11: x stops at the
first edge, |yaw| median 15 -> 91 deg between 2 s and 4 s, forward speed
still 0.5 m/s). The shared tracking reward and our progress reward both
measure velocity in the BODY frame, so "walk forward" is fully paid in
whatever direction the robot faces: turning away from a step costs only a
moment of yaw-rate error, climbing it risks a fall.

Fix: score walking against a reference heading `ref` that the ROBOT cannot
move once it is set.
  heading envs  ref follows the robot while it turns toward the new target;
                once |target - yaw| < LOCK_TOL, ref = heading_target until
                the next resample. Turning away after that is not paid, and
                the command's wz = k * (target - yaw) keeps pushing back.
  wz envs and   ref = robot heading at the resample + the integral of the
  standing      commanded wz, pulled toward the robot heading with time
                constant LEAK_TAU_S (imperfect wz tracking must not pile up
                into an error the policy cannot see; a fast turn away still
                costs).
The intended world velocity is the body-frame command rotated by ref; the
rewards compare the world velocity with that. The policy obs (the 3-value
command) is unchanged, so deploy is unchanged.

v2 run 1 (25-26/11, stopped at 10k): ref jumped to a new heading target AT
the resample, so with the target behind it "walk forward 0.8 m/s" was scored
as walking backward while the robot turned (up to ~3 s), toward a direction
the policy cannot see. Falls outnumbered timeouts 6.25 : 1.54, episodes 522
steps (teacher2/3 at the same stage: 0.75 : 3.8, ~930), pose reward 0.001.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand
from mjlab.utils.lab_api.math import wrap_to_pi


# Heading envs lock ref to the target once the robot faces it within this.
LOCK_TOL = 0.26  # rad, ~15 deg
# wz / standing envs: ref relaxes toward the robot heading with this constant.
LEAK_TAU_S = 2.0


class HeadingLockedVelocityCommand(UniformVelocityCommand):
  cfg: HeadingLockedVelocityCommandCfg

  def __init__(self, cfg: HeadingLockedVelocityCommandCfg, env: ManagerBasedRlEnv):
    super().__init__(cfg, env)
    self.ref_yaw = torch.zeros(self.num_envs, device=self.device)
    self.locked = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
    self.metrics["heading_drift"] = torch.zeros(self.num_envs, device=self.device)

  def _resample_command(self, env_ids: torch.Tensor) -> None:
    super()._resample_command(env_ids)
    self.ref_yaw[env_ids] = self.robot.data.heading_w[env_ids]
    self.locked[env_ids] = False

  def _update_command(self) -> None:
    super()._update_command()
    dt = self._env.step_dt
    yaw = self.robot.data.heading_w
    # wz / standing envs: commanded turn rate, leaking toward the robot heading.
    ref = self.ref_yaw + self.vel_command_b[:, 2] * dt
    ref = ref + wrap_to_pi(yaw - ref) * (dt / LEAK_TAU_S)
    if self.cfg.heading_command:
      # A standing env is told wz = 0, so it is treated like a wz env.
      steer = self.is_heading_env & ~self.is_standing_env
      faces = wrap_to_pi(self.heading_target - yaw).abs() < LOCK_TOL
      self.locked |= steer & faces
      ref = torch.where(steer, torch.where(self.locked, self.heading_target, yaw), ref)
    self.ref_yaw = wrap_to_pi(ref)

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
