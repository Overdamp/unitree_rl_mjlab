"""Step 2 of doc/g1_23dof_stairs_plan.md: how high an open-riser step can a
blind policy take?

For one rise and one direction it puts N robots on the centre platform of an
open-riser staircase (all facing +x, commanded straight ahead), runs one
episode and reports how many fell and how many steps they got past.

  down: pyramid, start on the top platform, 8 steps down to the floor.
  up:   bowl, start on the bottom platform, 7 steps up to the border.

Usage (from the worktree root; the script sets PYTHONPATH itself):
  python eval_open_stairs.py                       # sweep, P3b checkpoint
  python eval_open_stairs.py --checkpoint path/model.pt --rises 0.05,0.10
  python eval_open_stairs.py --rise 0.10 --direction up    # one case, JSON line

Tread 0.28 m, plate 0.04 m, nosing 0.02 m are fixed here (placeholders until
the target staircase is measured, plan step 0).
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import asdict, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parent
# The conda env has the main checkout pip-installed (editable): make `import src`
# load THIS folder even from a worktree.
sys.path.insert(0, str(ROOT))
os.environ["PYTHONPATH"] = f"{ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"

TASK = "Unitree-G1-23Dof-OpenStairs-Blind"
P3B = "deploy/robots/g1_23dof/config/policy/velocity/v0/checkpoints/model_phase3b_15998.pt"
TREAD, THICK, NOSING = 0.28, 0.04, 0.02
PATCH, BORDER, PLATFORM = 8.0, 1.0, 2.0
NUM_STEPS = int((PATCH - 2 * BORDER - PLATFORM) / (2 * TREAD))  # 7 rings
FIRST_EDGE = 0.5 * PATCH - BORDER - NUM_STEPS * TREAD  # x of the step edge nearest the platform
STAIRS_END = 0.5 * PATCH - BORDER + 0.3  # x from the patch centre where the stairs are behind


def run_case(args) -> dict:
  import torch

  import mjlab.tasks  # noqa: F401
  import src.tasks  # noqa: F401
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls
  from mjlab.utils.torch import configure_torch_backends
  from src.tasks.velocity.config.g1_23dof.terrains_stairs import (
    OpenRiserStairsTerrainCfg,
  )

  configure_torch_backends()
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  up = args.direction == "up"

  cfg = load_env_cfg(args.task, play=True)
  cfg.scene.num_envs = args.num_envs
  gen = cfg.scene.terrain.terrain_generator
  cfg.scene.terrain.terrain_generator = replace(
    gen,
    curriculum=False,
    num_rows=1,
    num_cols=4,
    size=(PATCH, PATCH),
    sub_terrains={
      "stairs": OpenRiserStairsTerrainCfg(
        proportion=1.0,
        inverted=up,
        step_height_range=(args.rise, args.rise),
        tread_depth_range=(TREAD, TREAD),
        tread_thickness_range=(THICK, THICK),
        nosing_range=(NOSING, NOSING),
        platform_width=PLATFORM,
        border_width=BORDER,
      )
    },
  )
  cfg.scene.terrain.max_init_terrain_level = 0
  cfg.curriculum = {}
  cfg.events.pop("randomize_terrain", None)
  cfg.events.pop("push_robot", None)
  # Everyone faces +x from near the platform centre and is told to walk straight.
  cfg.events["reset_base"].params["pose_range"] = {
    "x": (-0.2, 0.2), "y": (-0.3, 0.3), "z": (0.0, 0.0), "yaw": (0.0, 0.0),
  }
  twist = cfg.commands["twist"]
  twist.rel_standing_envs = 0.0
  twist.resampling_time_range = (1e6, 1e6)
  twist.ranges.lin_vel_x = (args.vx, args.vx)
  twist.ranges.lin_vel_y = (0.0, 0.0)
  twist.ranges.ang_vel_z = (0.0, 0.0)
  twist.ranges.heading = (0.0, 0.0)

  agent_cfg = load_rl_cfg(args.task)
  env = RslRlVecEnvWrapper(
    ManagerBasedRlEnv(cfg=cfg, device=device), clip_actions=agent_cfg.clip_actions
  )
  runner_cls = load_runner_cls(args.task) or MjlabOnPolicyRunner
  runner = runner_cls(env, asdict(agent_cfg), device=device)
  runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location=device)
  policy = runner.get_inference_policy(device=device)

  base = env.unwrapped
  robot = base.scene["robot"]
  origins = base.scene.env_origins
  n = args.num_envs
  steps = int(args.seconds / base.step_dt)

  obs, _ = env.reset()
  alive = torch.ones(n, dtype=torch.bool, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  last = (robot.data.root_link_pos_w - origins).clone()
  with torch.inference_mode():
    for _ in range(steps):
      obs, _, dones, extras = env.step(policy(obs))
      timeouts = extras.get("time_outs", torch.zeros_like(dones)).bool()
      newly = alive & dones.bool() & ~timeouts
      fell |= newly
      alive &= ~dones.bool()
      # A finished env is already reset here, so keep its last position.
      rel = robot.data.root_link_pos_w - origins
      last[alive] = rel[alive]
      # Past the last step (on the flat border): stop judging this robot, or it
      # walks on into the next patch and a fall there would count against it.
      alive &= ~(rel[:, 0].abs() > STAIRS_END)
  env.close()

  # Steps passed, from how far out the base got (a fallen base also drops in z,
  # so height would over-count). Step edges sit at FIRST_EDGE + i * TREAD.
  total_steps = NUM_STEPS if up else NUM_STEPS + 1
  done_steps = torch.clamp(
    torch.floor((last[:, 0] - FIRST_EDGE) / TREAD) + 1, 0, total_steps
  )
  success = (~fell) & (done_steps >= total_steps)
  return {
    "direction": args.direction,
    "rise_m": args.rise,
    "vx_cmd": args.vx,
    "envs": n,
    "fell_pct": round(100 * fell.float().mean().item(), 1),
    "success_pct": round(100 * success.float().mean().item(), 1),
    "steps_median": float(done_steps.median().item()),
    "steps_mean": round(done_steps.mean().item(), 2),
    "steps_total": total_steps,
    "x_mean_m": round(last[:, 0].mean().item(), 2),
    "x_end_stairs_m": STAIRS_END,
    "checkpoint": Path(args.checkpoint).name,
    "task": args.task,
  }


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--checkpoint", default=P3B)
  ap.add_argument("--task", default=TASK, help="task whose obs match the checkpoint (e.g. ...-OpenStairs-Teacher)")
  ap.add_argument("--rises", default="0.0,0.03,0.05,0.08,0.10,0.12,0.15,0.18", help="sweep: rises in m")
  ap.add_argument("--directions", default="down,up")
  ap.add_argument("--rise", type=float, help="run ONE case and print a JSON line")
  ap.add_argument("--direction", choices=["up", "down"], default="up")
  ap.add_argument("--vx", type=float, default=0.5)
  ap.add_argument("--num-envs", type=int, default=64)
  ap.add_argument("--seconds", type=float, default=14.0)
  ap.add_argument("--out", help="sweep: also write the JSON lines to this file")
  args = ap.parse_args()
  os.chdir(ROOT)

  if args.rise is not None:
    print("RESULT " + json.dumps(run_case(args)), flush=True)
    return

  # One process per case: each builds its own terrain and simulator.
  rows = []
  for direction in args.directions.split(","):
    for rise in (float(r) for r in args.rises.split(",")):
      cmd = [sys.executable, __file__, "--rise", str(rise), "--direction", direction,
             "--checkpoint", args.checkpoint, "--task", args.task, "--vx", str(args.vx),
             "--num-envs", str(args.num_envs), "--seconds", str(args.seconds)]
      p = subprocess.run(cmd, capture_output=True, text=True)
      line = next((ln for ln in p.stdout.splitlines() if ln.startswith("RESULT ")), None)
      if line is None:
        print(f"FAILED {direction} rise {rise}:\n{p.stderr[-1500:]}", file=sys.stderr)
        sys.exit(1)
      rows.append(json.loads(line[7:]))
      r = rows[-1]
      print(f"{r['direction']:4s} rise {r['rise_m']*100:4.0f} cm  success {r['success_pct']:5.1f}%  "
            f"fell {r['fell_pct']:5.1f}%  steps {r['steps_median']:.0f}/{r['steps_total']} (median)", flush=True)
  if args.out:
    Path(args.out).write_text("\n".join(json.dumps(r) for r in rows) + "\n")


if __name__ == "__main__":
  main()
