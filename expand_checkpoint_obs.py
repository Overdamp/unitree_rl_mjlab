"""Widen an rsl_rl checkpoint for observations appended at the END of the actor
and critic inputs (plan step 3h: teacher7 -> teacher8 + stair token).

For each of actor / critic:
  mlp.0.weight          (H, n) -> (H, n + k), new columns = 0 (or tiny noise)
  obs_normalizer        mean 0 / var 1 / std 1 for the new entries
  Adam exp_avg(_sq)     zero-padded the same way, so the optimizer keeps its state
With zero columns the widened policy outputs exactly what the old one did, for
any value of the new inputs; PPO then grows those weights if they pay.

The new run folder goes next to the source run, so train.py --agent.load-run
can resume it:
  python expand_checkpoint_obs.py \
    --src logs/rsl_rl/g1_23dof_velocity/2026-11-19_11-02-54_stairs_teacher7/model_27499.pt \
    --add 4 --out-run 2026-11-19_11-02-54_stairs_teacher7_t8init
"""

from __future__ import annotations

import argparse
import os

import torch


def _pad_cols(t: torch.Tensor, k: int, fill: float, noise: float = 0.0) -> torch.Tensor:
  extra = torch.full((*t.shape[:-1], k), fill, dtype=t.dtype)
  if noise > 0.0:
    extra = extra + noise * torch.randn_like(extra)
  return torch.cat([t, extra], dim=-1)


def expand(ckpt: dict, k: int, noise: float) -> dict:
  old_w = {}
  for part in ("actor_state_dict", "critic_state_dict"):
    sd = ckpt[part]
    n = sd["mlp.0.weight"].shape[1]
    old_w[part] = sd["mlp.0.weight"]
    sd["mlp.0.weight"] = _pad_cols(sd["mlp.0.weight"], k, 0.0, noise)
    sd["obs_normalizer._mean"] = _pad_cols(sd["obs_normalizer._mean"], k, 0.0)
    sd["obs_normalizer._var"] = _pad_cols(sd["obs_normalizer._var"], k, 1.0)
    sd["obs_normalizer._std"] = _pad_cols(sd["obs_normalizer._std"], k, 1.0)
    print(f"{part}: input {n} -> {n + k}")

  # Adam state is keyed by parameter index; match the first-layer weights by shape.
  shapes = {tuple(w.shape) for w in old_w.values()}
  padded = 0
  for st in ckpt["optimizer_state_dict"]["state"].values():
    for key in ("exp_avg", "exp_avg_sq"):
      t = st.get(key)
      if t is not None and tuple(t.shape) in shapes:
        st[key] = _pad_cols(t, k, 0.0)
        padded += 1
  if padded != 2 * len(old_w):
    raise RuntimeError(f"padded {padded} optimizer tensors, expected {2 * len(old_w)}")
  return ckpt


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--src", required=True, help="model_*.pt to widen")
  ap.add_argument("--add", type=int, default=4, help="number of obs appended to actor and critic")
  ap.add_argument("--out-run", required=True, help="new run folder name, created next to the source run")
  ap.add_argument("--noise", type=float, default=0.0,
                  help="std of the new weights (0 = exact copy of the old policy)")
  args = ap.parse_args()

  ckpt = torch.load(args.src, map_location="cpu", weights_only=False)
  expand(ckpt, args.add, args.noise)
  out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(args.src))), args.out_run)
  os.makedirs(out_dir, exist_ok=True)
  out = os.path.join(out_dir, os.path.basename(args.src))
  torch.save(ckpt, out)
  print("wrote", out)


if __name__ == "__main__":
  main()
