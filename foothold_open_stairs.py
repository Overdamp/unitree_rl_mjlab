"""Where do the feet land on the open-riser stairs? (doc/g1_23dof_stairs_plan.md)

Same staircase and start as eval_open_stairs.py (robots on the centre
platform, facing +x, commanded straight ahead). At every touchdown of a foot
it records the sole (heel..toe along x, from the foot collision capsules) and
the step the foot landed on, then reports:

  support    share of the sole length that is over the tread it stands on
  off_front  sole sticking out past the tread's front edge (m)
  off_back   sole sticking out past the tread's back edge (m)
  centre     sole centre minus tread centre along x (m; + = towards the
             walking direction)
  under      sole length under the next higher tread (up: toe under the
             tread ahead; down: heel under the tread behind) - a foot there
             was placed too close to the riser gap
  stumble    control steps where a foot's contact force is more horizontal
             than vertical (toe or heel pushing on an edge)

split by robots that made it to the top/bottom and robots that did not.

Usage (from the worktree root; the script sets PYTHONPATH itself):
  python foothold_open_stairs.py --checkpoint path/model.pt --rise 0.10 --direction up --heading-hold
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
os.environ["PYTHONPATH"] = f"{ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"

# Sole of ankle_roll_link (g1_23dof.xml foot capsules, radius 0.01).
HEEL_X, TOE_X, SOLE_Z = -0.064, 0.142, -0.035


def surfaces(rise: float, up: bool, run: float, nosing: float, half_inner: float, n: int):
  """Walking surfaces met going +x from the centre: list of (z, x_back, x_front),
  z relative to the start platform. x_front of the last one is +inf."""
  out = []
  if up:  # bowl: platform at -n*rise, ring k top -k*rise, nosing points inward
    out.append((0.0, -1e9, half_inner - n * run - nosing))
    for j in range(1, n + 1):
      k = n - j
      out.append((j * rise, half_inner - (k + 1) * run - nosing, half_inner - k * run))
    out[-1] = (out[-1][0], out[-1][1], 1e9)  # ring 0 is level with the border
  else:  # pyramid: platform at (n+1)*rise, ring k top (k+1)*rise, nosing outward
    out.append((0.0, -1e9, half_inner - n * run + nosing))
    for j in range(1, n + 1):
      k = n - j
      out.append((-j * rise, half_inner - (k + 1) * run, half_inner - k * run + (nosing if k > 0 else 0.0)))
    out.append((-(n + 1) * rise, half_inner, 1e9))  # floor / border
  return out


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--checkpoint", required=True)
  ap.add_argument("--task", default="Unitree-G1-23Dof-OpenStairs-Teacher")
  ap.add_argument("--rise", type=float, default=0.10)
  ap.add_argument("--direction", choices=["up", "down"], default="up")
  ap.add_argument("--vx", type=float, default=0.5)
  ap.add_argument("--num-envs", type=int, default=64)
  ap.add_argument("--seconds", type=float, default=14.0)
  ap.add_argument("--heading-hold", action="store_true")
  ap.add_argument("--clip-actions", type=float, default=None)
  ap.add_argument("--json", action="store_true", help="print one JSON line instead of the table")
  args = ap.parse_args()
  os.chdir(ROOT)

  import numpy as np
  import torch

  import mjlab.tasks  # noqa: F401
  import src.tasks  # noqa: F401
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
  from mjlab.tasks.registry import load_rl_cfg, load_runner_cls
  from mjlab.utils.lab_api.math import quat_apply

  import eval_open_stairs as E

  up = args.direction == "up"
  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  cfg = E.make_env_cfg(args.task, args.rise, up, args.vx, args.heading_hold, args.num_envs)
  agent_cfg = load_rl_cfg(args.task)
  clip = args.clip_actions if args.clip_actions is not None else agent_cfg.clip_actions
  env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg=cfg, device=device), clip_actions=clip)
  runner = (load_runner_cls(args.task) or MjlabOnPolicyRunner)(env, asdict(agent_cfg), device=device)
  runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location=device)
  policy = runner.get_inference_policy(device=device)

  base = env.unwrapped
  robot = base.scene["robot"]
  contact = base.scene.sensors["feet_ground_contact"]
  feet, _ = robot.find_bodies(["left_ankle_roll_link", "right_ankle_roll_link"], preserve_order=True)
  origins = base.scene.env_origins
  n = args.num_envs
  half_inner = 0.5 * E.PATCH - E.BORDER
  surf = surfaces(args.rise, up, E.TREAD, E.NOSING, half_inner, E.NUM_STEPS)
  surf_z = np.array([s[0] for s in surf])

  heel_l = torch.tensor([HEEL_X, 0.0, SOLE_Z], device=device)
  toe_l = torch.tensor([TOE_X, 0.0, SOLE_Z], device=device)

  touchdowns = []  # (env, level, heel_x, toe_x)
  stumble = torch.zeros(n, device=device)
  alive = torch.ones(n, dtype=torch.bool, device=device)
  fell = torch.zeros(n, dtype=torch.bool, device=device)
  last = (robot.data.root_link_pos_w - origins).clone()
  obs, _ = env.reset()
  prev = contact.data.found[:, :2] > 0
  steps = int(args.seconds / base.step_dt)
  with torch.inference_mode():
    for _ in range(steps):
      obs, _, dones, extras = env.step(policy(obs))
      timeouts = extras.get("time_outs", torch.zeros_like(dones)).bool()
      fell |= alive & dones.bool() & ~timeouts
      alive &= ~dones.bool()
      rel = robot.data.root_link_pos_w - origins
      last[alive] = rel[alive]
      alive &= ~(rel[:, 0].abs() > E.STAIRS_END)

      found = contact.data.found[:, :2] > 0
      force = contact.data.force[:, :2]  # world frame (netforce)
      horiz = force[..., :2].norm(dim=-1)
      stumble += (alive.unsqueeze(1) & found & (horiz > force[..., 2].abs()) & (horiz > 20.0)).any(dim=1).float()
      new = found & ~prev & alive.unsqueeze(1)
      prev = found
      if new.any():
        pos = robot.data.body_link_pos_w[:, feet] - origins.unsqueeze(1)
        quat = robot.data.body_link_quat_w[:, feet]
        heel = pos + quat_apply(quat, heel_l.expand_as(pos))
        toe = pos + quat_apply(quat, toe_l.expand_as(pos))
        e_idx, f_idx = new.nonzero(as_tuple=True)
        for e, f in zip(e_idx.tolist(), f_idx.tolist()):
          hx, tx = heel[e, f, 0].item(), toe[e, f, 0].item()
          z = min(heel[e, f, 2].item(), toe[e, f, 2].item())
          touchdowns.append((e, int(np.abs(surf_z - z).argmin()), hx, tx))
  env.close()

  total_steps = E.NUM_STEPS if up else E.NUM_STEPS + 1
  done_steps = torch.clamp(torch.floor((last[:, 0] - E.FIRST_EDGE) / E.TREAD) + 1, 0, total_steps)
  success = ((~fell) & (done_steps >= total_steps)).cpu().numpy()

  rows = []
  for e, lvl, hx, tx in touchdowns:
    if lvl == 0 or lvl == len(surf) - 1 and not up:
      continue  # start platform / floor: nothing to place against
    _, xb, xf = surf[lvl]
    back, front = min(hx, tx), max(hx, tx)
    length = front - back
    support = max(0.0, min(front, xf) - max(back, xb)) / length
    if up:  # next higher tread is ahead: its back edge
      under = max(0.0, front - surf[lvl + 1][1]) if lvl + 1 < len(surf) else 0.0
    else:  # next higher tread is behind: its front edge
      under = max(0.0, surf[lvl - 1][2] - back)
    centre = 0.5 * (back + front) - 0.5 * (xb + xf) if xf < 1e8 else float("nan")
    rows.append((e, support, max(0.0, front - xf), max(0.0, xb - back), under, centre))

  def summary(sel) -> dict:
    r = np.array([x[1:] for x in rows if sel[x[0]]]) if any(sel[x[0]] for x in rows) else np.zeros((0, 5))
    if len(r) == 0:
      return {"touchdowns": 0}
    return {
      "touchdowns": len(r),
      "support_median": round(float(np.median(r[:, 0])), 2),
      "support_lt_70_pct": round(100 * float((r[:, 0] < 0.7).mean()), 1),
      "off_front_gt_2cm_pct": round(100 * float((r[:, 1] > 0.02).mean()), 1),
      "off_back_gt_2cm_pct": round(100 * float((r[:, 2] > 0.02).mean()), 1),
      "under_gt_1cm_pct": round(100 * float((r[:, 3] > 0.01).mean()), 1),
      "under_median_cm": round(100 * float(np.median(r[:, 3])), 1),
      "centre_offset_median_cm": round(100 * float(np.nanmedian(r[:, 4])), 1),
    }

  st = stumble.cpu().numpy()
  out = {
    "direction": args.direction, "rise_m": args.rise, "envs": n,
    "success_pct": round(100 * float(success.mean()), 1),
    "fell_pct": round(100 * float(fell.float().mean().item()), 1),
    "ok": summary(success),
    "failed": summary(~success),
    "stumble_steps_ok": round(float(st[success].mean()), 1) if success.any() else None,
    "stumble_steps_failed": round(float(st[~success].mean()), 1) if (~success).any() else None,
    "checkpoint": Path(args.checkpoint).name,
  }
  if args.json:
    print("RESULT " + json.dumps(out), flush=True)
    return
  print(f"\n{args.direction} {args.rise * 100:.0f} cm  success {out['success_pct']}%  fell {out['fell_pct']}%")
  keys = ["touchdowns", "support_median", "support_lt_70_pct", "off_front_gt_2cm_pct",
          "off_back_gt_2cm_pct", "under_gt_1cm_pct", "under_median_cm", "centre_offset_median_cm"]
  print(f"{'':22s}{'made it':>10s}{'did not':>10s}")
  for k in keys:
    print(f"{k:22s}{str(out['ok'].get(k, '-')):>10s}{str(out['failed'].get(k, '-')):>10s}")
  print(f"{'stumble steps / robot':22s}{str(out['stumble_steps_ok']):>10s}{str(out['stumble_steps_failed']):>10s}")


if __name__ == "__main__":
  main()
