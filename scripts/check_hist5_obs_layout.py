"""Check the actor history layout of Unitree-G1-23Dof-Rough-Blind-Hist5.

Deploy (deploy/include/isaaclab/manager/observation_manager.h, use_gym_history=false)
builds the actor input per term as [frame t-4, ..., frame t] and concatenates terms
in deploy.yaml order; on reset it fills every frame with the first observation.
This script verifies mjlab produces the same layout, so deploy.yaml only needs
history_length: 5 per term.

Usage: python scripts/check_hist5_obs_layout.py
"""

import torch

import mjlab.tasks  # noqa: F401
import src.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

TASK = "Unitree-G1-23Dof-Rough-Blind-Hist5"
FLAT = "Unitree-G1-23Dof-Flat"
H = 5
STEPS = 8


def main() -> None:
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  cfg = load_env_cfg(TASK)
  cfg.scene.num_envs = 2
  env = ManagerBasedRlEnv(cfg, device=device)
  om = env.observation_manager

  names = om.active_terms["actor"]
  flat_names = list(load_env_cfg(FLAT).observations["actor"].terms)
  assert names == flat_names, f"term order differs from Flat/deploy: {names}"
  dims = [d[0] // H for d in om.group_obs_term_dim["actor"]]
  assert sum(dims) == 80, dims
  print("terms:", list(zip(names, dims)), "actor dim:", sum(dims) * H)

  def split(obs: torch.Tensor) -> dict[str, torch.Tensor]:
    out, i = {}, 0
    for n, d in zip(names, dims):
      out[n] = obs[:, i : i + d * H].reshape(obs.shape[0], H, d)
      i += d * H
    assert i == obs.shape[1]
    return out

  obs, _ = env.reset()
  first = split(obs["actor"])
  for n in names:
    assert torch.allclose(first[n], first[n][:, -1:].expand_as(first[n])), (
      f"{n}: reset does not backfill history with the first frame"
    )

  newest: list[dict[str, torch.Tensor]] = [{n: first[n][:, -1] for n in names}]
  action = torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=device)
  for _ in range(STEPS):
    action.uniform_(-0.2, 0.2)
    obs, *_ = env.step(action)
    cur = split(obs["actor"])
    newest.append({n: cur[n][:, -1] for n in names})
    for n in names:
      for k in range(H):  # slot k holds the frame from (H - 1 - k) steps ago
        lag = H - 1 - k
        ref = newest[max(len(newest) - 1 - lag, 0)][n]
        assert torch.allclose(cur[n][:, k], ref, atol=1e-6), f"{n} slot {k} mismatch"
  print(f"OK: per-term, oldest->newest, backfilled on reset ({STEPS} steps checked)")
  env.close()


if __name__ == "__main__":
  main()
