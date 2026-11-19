"""Watch a checkpoint on ONE open-riser staircase case (fixed rise and
direction), the same setup eval_open_stairs.py scores: robots start on the
centre platform facing +x and are commanded straight ahead at --vx.

Usage (from the worktree root; the script sets PYTHONPATH itself):
  python view_open_stairs_case.py --checkpoint path/model.pt --rise 0.10 --direction up --heading-hold
  python view_open_stairs_case.py --checkpoint path/model.pt --rise 0.15 --direction down --viewer viser

Each robot gets its own staircase (eval puts several on one: the sim worlds
are separate, so they only overlap on screen). Episodes reset by themselves
(fall or time-out), so the case repeats.
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
os.environ["PYTHONPATH"] = f"{ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--checkpoint", required=True)
  ap.add_argument("--task", default="Unitree-G1-23Dof-OpenStairs-Teacher",
                  help="task whose obs match the checkpoint")
  ap.add_argument("--rise", type=float, default=0.10, help="step rise in m")
  ap.add_argument("--direction", choices=["up", "down"], default="up")
  ap.add_argument("--vx", type=float, default=0.5)
  ap.add_argument("--num-envs", type=int, default=8)
  ap.add_argument("--heading-hold", action="store_true",
                  help="steer back to heading 0 (wz up to +-1 rad/s) instead of commanding wz = 0")
  ap.add_argument("--clip-actions", type=float, default=None,
                  help="clamp actions like training with --agent.clip-actions (default: task cfg)")
  ap.add_argument("--viewer", choices=["native", "viser"], default="native")
  args = ap.parse_args()
  os.chdir(ROOT)

  import torch

  import mjlab.tasks  # noqa: F401
  import src.tasks  # noqa: F401
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_rl_cfg, load_runner_cls
  from mjlab.viewer import NativeMujocoViewer, ViserPlayViewer

  from eval_open_stairs import make_env_cfg

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  cfg = make_env_cfg(args.task, args.rise, args.direction == "up", args.vx,
                     args.heading_hold, args.num_envs, num_cols=args.num_envs)
  agent_cfg = load_rl_cfg(args.task)
  clip = args.clip_actions if args.clip_actions is not None else agent_cfg.clip_actions
  env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg=cfg, device=device), clip_actions=clip)
  runner = (load_runner_cls(args.task) or MjlabOnPolicyRunner)(env, asdict(agent_cfg), device=device)
  runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location=device)
  policy = runner.get_inference_policy(device=device)
  print(f"[view] {Path(args.checkpoint).name}: {args.direction} {args.rise * 100:.0f} cm, "
        f"vx {args.vx}, heading hold {args.heading_hold}, clip {clip}")
  (NativeMujocoViewer if args.viewer == "native" else ViserPlayViewer)(env, policy).run()
  env.close()


if __name__ == "__main__":
  main()
