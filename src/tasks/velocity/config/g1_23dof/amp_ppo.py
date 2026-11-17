"""AMP (adversarial motion prior) on top of rsl_rl PPO for the G1-23DoF stairs teacher.

Reference: Peng et al., "AMP: Adversarial Motion Priors for Stylized Physics-Based
Character Control" (SIGGRAPH 2021). A discriminator D learns to tell state
transitions (s, s') of reference motion clips from those of the policy, and the
policy gets a style reward for transitions that D rates as "like the clips":

    r = r_task + w * dt * max(0, 1 - 0.25 * (D(s, s') - 1)^2)

Only training changes. The actor and critic observations stay the same as the
teacher's (actor 267, critic 95 + 187), so teacher checkpoints resume into the
AMP task, and the exported policy (ONNX / deploy.yaml / g1_ctrl) is unchanged.
D is used during training only.

Pieces (all in this file, nothing shared is edited):
- ``amp_features``: the per-frame feature vector, one function for both sides.
- ``amp_state``: observation term; the env's "amp" group (no noise, no history).
- ``AmpMotionData``: loads reference clips (.npz) and precomputes (s, s') pairs.
- ``AmpPPO``: PPO subclass picked by ``class_name`` in rl_cfg.py; adds the style
  reward in ``process_env_step`` and trains D after the PPO update.

Reference clip format (.npz, the one mjlab's csv_to_npz.py writes, recorded
with the G1-23DoF robot so joint and body order match this env):
  fps (1,), joint_pos (T, 23), joint_vel (T, 23), body_pos_w (T, B, 3),
  body_quat_w (T, B, 4, wxyz), body_lin_vel_w (T, B, 3), body_ang_vel_w (T, B, 3)
fps must equal the policy rate (50 Hz).

Settings come from environment variables, because the clip path differs per
machine and the task config goes through tyro (an extra algorithm field would
not reach this class):
  AMP_MOTION_FILES   required: .npz files and/or directories, comma separated
  AMP_REWARD_WEIGHT  style weight w (default 0.5; task reward is ~0.034/step)
  AMP_DISC_LR        discriminator learning rate (default 1e-4)
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field

import numpy as np
import torch
import torch.nn as nn
from tensordict import TensorDict

from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.lab_api.math import quat_apply_inverse
from rsl_rl.algorithms import PPO
from rsl_rl.modules.normalization import EmpiricalNormalization

AMP_OBS_GROUP = "amp"
AMP_ROOT_BODY = "pelvis"
AMP_FOOT_BODIES = ("left_ankle_roll_link", "right_ankle_roll_link")
_GRAVITY_DIR = (0.0, 0.0, -1.0)


# ---------------------------------------------------------------------------
# Features
# ---------------------------------------------------------------------------


def amp_features(
  joint_pos_rel: torch.Tensor,  # (N, J) joint position minus the default pose
  joint_vel: torch.Tensor,  # (N, J)
  root_quat_w: torch.Tensor,  # (N, 4) wxyz
  root_pos_w: torch.Tensor,  # (N, 3)
  root_lin_vel_w: torch.Tensor,  # (N, 3)
  root_ang_vel_w: torch.Tensor,  # (N, 3)
  feet_pos_w: torch.Tensor,  # (N, F, 3)
) -> torch.Tensor:
  """One frame of AMP features, all in the pelvis frame: no world position or
  heading, so the style does not depend on where the robot is or which way it
  faces. Size = 2J + 9 + 3F (23 DoF, 2 feet: 61)."""
  n = root_quat_w.shape[0]
  lin_b = quat_apply_inverse(root_quat_w, root_lin_vel_w)
  ang_b = quat_apply_inverse(root_quat_w, root_ang_vel_w)
  gravity = torch.tensor(_GRAVITY_DIR, device=root_quat_w.device).expand(n, 3)
  gravity_b = quat_apply_inverse(root_quat_w, gravity)
  num_feet = feet_pos_w.shape[1]
  feet_rel_w = feet_pos_w - root_pos_w.unsqueeze(1)
  feet_b = quat_apply_inverse(
    root_quat_w.unsqueeze(1).expand(n, num_feet, 4), feet_rel_w
  ).reshape(n, 3 * num_feet)
  return torch.cat([joint_pos_rel, joint_vel, lin_b, ang_b, gravity_b, feet_b], dim=-1)


def amp_state(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
  """Observation term for the "amp" group: ``amp_features`` of the robot now."""
  robot = env.scene[asset_cfg.name]
  data = robot.data
  feet_ids = _body_ids(robot.body_names, AMP_FOOT_BODIES)
  return amp_features(
    data.joint_pos - data.default_joint_pos,
    data.joint_vel,
    data.root_link_quat_w,
    data.root_link_pos_w,
    data.root_link_lin_vel_w,
    data.root_link_ang_vel_w,
    data.body_link_pos_w[:, feet_ids],
  )


def _body_ids(body_names, wanted) -> list[int]:
  missing = [b for b in wanted if b not in body_names]
  if missing:
    raise ValueError(f"AMP: bodies {missing} not in robot bodies {list(body_names)}")
  return [list(body_names).index(b) for b in wanted]


# ---------------------------------------------------------------------------
# Reference motion
# ---------------------------------------------------------------------------


def _expand_motion_files(spec: str) -> list[str]:
  files: list[str] = []
  for item in (s.strip() for s in spec.split(",")):
    if not item:
      continue
    if os.path.isdir(item):
      files.extend(sorted(glob.glob(os.path.join(item, "*.npz"))))
    else:
      files.append(item)
  if not files:
    raise ValueError(f"AMP: no .npz files found in AMP_MOTION_FILES={spec!r}")
  return files


class AmpMotionData:
  """All (s, s') feature pairs of the reference clips, kept on the device."""

  def __init__(
    self,
    files: list[str],
    body_names: list[str],
    num_joints: int,
    default_joint_pos: torch.Tensor,  # (J,)
    policy_dt: float,
    device: str,
  ) -> None:
    root_id = _body_ids(body_names, (AMP_ROOT_BODY,))[0]
    feet_ids = _body_ids(body_names, AMP_FOOT_BODIES)
    pairs = []
    self.num_frames = 0
    for path in files:
      data = np.load(path)
      fps = float(np.asarray(data["fps"]).reshape(-1)[0])
      if abs(fps * policy_dt - 1.0) > 1e-3:
        raise ValueError(f"AMP: {path} is {fps} fps; the policy runs at {1 / policy_dt:.0f} Hz")
      joint_pos = torch.as_tensor(data["joint_pos"], dtype=torch.float32, device=device)
      if joint_pos.shape[1] != num_joints:
        raise ValueError(f"AMP: {path} has {joint_pos.shape[1]} joints, robot has {num_joints}")
      body_pos = torch.as_tensor(data["body_pos_w"], dtype=torch.float32, device=device)
      if body_pos.shape[1] != len(body_names):
        raise ValueError(f"AMP: {path} has {body_pos.shape[1]} bodies, robot has {len(body_names)}")
      if joint_pos.shape[0] < 2:
        continue

      def body(key: str, idx):
        return torch.as_tensor(data[key], dtype=torch.float32, device=device)[:, idx]

      feats = amp_features(
        joint_pos - default_joint_pos.to(device),
        torch.as_tensor(data["joint_vel"], dtype=torch.float32, device=device),
        body("body_quat_w", root_id),
        body_pos[:, root_id],
        body("body_lin_vel_w", root_id),
        body("body_ang_vel_w", root_id),
        body_pos[:, feet_ids],
      )
      pairs.append(torch.cat([feats[:-1], feats[1:]], dim=-1))
      self.num_frames += joint_pos.shape[0]
    if not pairs:
      raise ValueError("AMP: the reference clips have no usable transitions")
    self.pairs = torch.cat(pairs, dim=0)
    self.device = device

  def sample(self, n: int) -> torch.Tensor:
    idx = torch.randint(0, self.pairs.shape[0], (n,), device=self.device)
    return self.pairs[idx]


# ---------------------------------------------------------------------------
# Discriminator
# ---------------------------------------------------------------------------


class AmpDiscriminator(nn.Module):
  def __init__(self, pair_dim: int, hidden_dims=(1024, 512)) -> None:
    super().__init__()
    self.normalizer = EmpiricalNormalization(pair_dim)
    layers: list[nn.Module] = []
    last = pair_dim
    for h in hidden_dims:
      layers += [nn.Linear(last, h), nn.ReLU()]
      last = h
    self.trunk = nn.Sequential(*layers)
    self.head = nn.Linear(last, 1)

  def logits_normalized(self, x_norm: torch.Tensor) -> torch.Tensor:
    return self.head(self.trunk(x_norm)).squeeze(-1)

  def forward(self, x: torch.Tensor) -> torch.Tensor:
    return self.logits_normalized(self.normalizer(x))

  def style_reward(self, x: torch.Tensor) -> torch.Tensor:
    """LSGAN style reward in [0, 1] (expert target +1, policy target -1)."""
    d = self.forward(x)
    return torch.clamp(1.0 - 0.25 * (d - 1.0) ** 2, min=0.0)


@dataclass
class AmpCfg:
  motion_files: list[str] = field(default_factory=list)
  reward_weight: float = 0.5
  disc_lr: float = 1e-4
  disc_hidden_dims: tuple[int, ...] = (1024, 512)
  disc_epochs: int = 2
  disc_mini_batches: int = 8
  grad_penalty: float = 5.0
  logit_reg: float = 0.05  # weight decay on the output layer only
  weight_decay: float = 1e-4

  @staticmethod
  def from_environ() -> "AmpCfg":
    spec = os.environ.get("AMP_MOTION_FILES", "")
    if not spec:
      raise ValueError("AMP: set AMP_MOTION_FILES to the reference .npz files/directories")
    cfg = AmpCfg(motion_files=_expand_motion_files(spec))
    if "AMP_REWARD_WEIGHT" in os.environ:
      cfg.reward_weight = float(os.environ["AMP_REWARD_WEIGHT"])
    if "AMP_DISC_LR" in os.environ:
      cfg.disc_lr = float(os.environ["AMP_DISC_LR"])
    return cfg


# ---------------------------------------------------------------------------
# Algorithm
# ---------------------------------------------------------------------------


class AmpPPO(PPO):
  """PPO + AMP style reward. Select with algorithm.class_name in rl_cfg.py."""

  amp: AmpCfg
  disc: AmpDiscriminator

  @staticmethod
  def construct_algorithm(obs: TensorDict, env, cfg: dict, device: str) -> "AmpPPO":
    alg = PPO.construct_algorithm(obs, env, cfg, device)
    assert isinstance(alg, AmpPPO), "algorithm.class_name must point at AmpPPO"
    alg._setup_amp(obs, env)
    return alg

  def _setup_amp(self, obs: TensorDict, env) -> None:
    if self.is_multi_gpu:
      raise NotImplementedError("AMP: multi-GPU training is not supported")
    if AMP_OBS_GROUP not in obs:
      raise ValueError(f"AMP: the env has no '{AMP_OBS_GROUP}' observation group (use the AMP task)")
    self.amp = AmpCfg.from_environ()
    unwrapped = env.unwrapped
    robot = unwrapped.scene["robot"]
    self.step_dt = float(unwrapped.step_dt)
    self.motion = AmpMotionData(
      self.amp.motion_files,
      list(robot.body_names),
      robot.data.joint_pos.shape[1],
      robot.data.default_joint_pos[0],
      self.step_dt,
      self.device,
    )
    feat_dim = obs[AMP_OBS_GROUP].shape[-1]
    if self.motion.pairs.shape[1] != 2 * feat_dim:
      raise ValueError("AMP: reference features do not match the env's amp group")
    self.disc = AmpDiscriminator(2 * feat_dim, self.amp.disc_hidden_dims).to(self.device)
    self.disc_optimizer = torch.optim.Adam(
      [
        {"params": self.disc.trunk.parameters(), "weight_decay": self.amp.weight_decay},
        {"params": self.disc.head.parameters(), "weight_decay": self.amp.logit_reg},
      ],
      lr=self.amp.disc_lr,
    )
    self._amp_prev: torch.Tensor | None = None
    self._amp_pairs: list[torch.Tensor] = []
    self._amp_style_sum = 0.0
    self._amp_style_count = 0
    print(
      f"[AMP] {len(self.amp.motion_files)} clip file(s), {self.motion.num_frames} frames, "
      f"{self.motion.pairs.shape[0]} pairs, feature {feat_dim}, weight {self.amp.reward_weight}, "
      f"disc lr {self.amp.disc_lr}"
    )

  # --- rollout ---------------------------------------------------------------

  def act(self, obs: TensorDict) -> torch.Tensor:
    self._amp_prev = obs[AMP_OBS_GROUP].clone()
    return super().act(obs)

  def process_env_step(self, obs, rewards, dones, extras) -> None:
    s_next = obs[AMP_OBS_GROUP]
    assert self._amp_prev is not None
    pair = torch.cat([self._amp_prev, s_next], dim=-1)
    # After a reset the next obs belongs to the new episode, so the pair is not
    # a real transition: no style reward and not used to train D.
    valid = ~dones.bool()
    style = self.disc.style_reward(pair) * valid.float()
    self._amp_pairs.append(pair[valid])
    self._amp_style_sum += float(style.sum())
    self._amp_style_count += int(valid.sum())
    rewards = rewards + self.amp.reward_weight * self.step_dt * style
    super().process_env_step(obs, rewards, dones, extras)

  # --- update ----------------------------------------------------------------

  def update(self) -> dict[str, float]:
    loss_dict = super().update()
    loss_dict.update(self._update_discriminator())
    return loss_dict

  def _update_discriminator(self) -> dict[str, float]:
    # Rollout tensors were made under inference mode; clone to use them in autograd.
    policy_pairs = torch.cat(self._amp_pairs, dim=0).clone()
    self._amp_pairs = []
    style_mean = self._amp_style_sum / max(self._amp_style_count, 1)
    self._amp_style_sum, self._amp_style_count = 0.0, 0
    stats = {"amp_disc": 0.0, "amp_grad_pen": 0.0, "amp_d_expert": 0.0, "amp_d_policy": 0.0}
    if policy_pairs.shape[0] == 0:
      return {**stats, "amp_style": style_mean}

    with torch.no_grad():
      self.disc.normalizer.update(torch.cat([policy_pairs, self.motion.sample(policy_pairs.shape[0])]))

    batch = max(policy_pairs.shape[0] // self.amp.disc_mini_batches, 1)
    updates = 0
    for _ in range(self.amp.disc_epochs):
      perm = torch.randperm(policy_pairs.shape[0], device=self.device)
      for start in range(0, perm.numel() - batch + 1, batch):
        pol = policy_pairs[perm[start : start + batch]]
        exp = self.motion.sample(batch)
        d_pol = self.disc(pol)
        exp_norm = self.disc.normalizer(exp).detach().requires_grad_(True)
        d_exp = self.disc.logits_normalized(exp_norm)
        lsgan = 0.5 * ((d_exp - 1.0) ** 2).mean() + 0.5 * ((d_pol + 1.0) ** 2).mean()
        grad = torch.autograd.grad(d_exp.sum(), exp_norm, create_graph=True)[0]
        grad_pen = grad.pow(2).sum(dim=-1).mean()
        loss = lsgan + self.amp.grad_penalty * grad_pen
        self.disc_optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.disc.parameters(), self.max_grad_norm)
        self.disc_optimizer.step()
        stats["amp_disc"] += lsgan.item()
        stats["amp_grad_pen"] += grad_pen.item()
        stats["amp_d_expert"] += d_exp.mean().item()
        stats["amp_d_policy"] += d_pol.mean().item()
        updates += 1
    stats = {k: v / max(updates, 1) for k, v in stats.items()}
    stats["amp_style"] = style_mean
    return stats

  # --- modes / checkpoints -----------------------------------------------------

  def train_mode(self) -> None:
    super().train_mode()
    self.disc.train()

  def eval_mode(self) -> None:
    super().eval_mode()
    self.disc.eval()

  def save(self) -> dict:
    saved = super().save()
    saved["amp_disc_state_dict"] = self.disc.state_dict()
    saved["amp_disc_optimizer_state_dict"] = self.disc_optimizer.state_dict()
    return saved

  def load(self, loaded_dict: dict, load_cfg: dict | None, strict: bool) -> bool:
    load_iteration = super().load(loaded_dict, load_cfg, strict)
    # A plain teacher checkpoint has no discriminator: D starts fresh.
    if "amp_disc_state_dict" in loaded_dict:
      self.disc.load_state_dict(loaded_dict["amp_disc_state_dict"], strict=strict)
      self.disc_optimizer.load_state_dict(loaded_dict["amp_disc_optimizer_state_dict"])
    else:
      print("[AMP] checkpoint has no discriminator; it starts from scratch")
    return load_iteration
