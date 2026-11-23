"""Explicit stair geometry token for the G1-23DoF stairs teacher8 (plan step 3h).

After Zhang et al., "Explicit Stair Geometry Conditioning for Robust Humanoid
Locomotion" (arXiv 2605.09944): give the policy a few interpretable numbers
about the next step instead of only a height map. teacher6/7 cleared step
edges but never changed their step length (plan step 3f); the paper argues an
explicit tread depth makes that easy to learn.

The token is MEASURED from terrain rays, not read from the terrain generator,
so it is the same on the training pyramids, the straight eval staircase
(eval_open_stairs.py) and any other terrain, and a student can learn to
estimate it from depth images later.

Sensor: three parallel ray lines along the pelvis heading (yaw-aligned), at
y = -0.15 / 0 / +0.15 m, x = -0.2 .. +1.2 m every 2 cm, cast down onto
terrain geoms only. Along each line the ground profile is a staircase; an
edge is a jump of more than EDGE_MIN_M between neighbouring rays.

Token (4 values, in this order):
  h     signed rise of the first edge ahead of the pelvis (x > 0), m;
        + up, - down, 0 when there is no edge in range
  d     tread depth = distance from the first to the second edge ahead,
        corrected for yaw, m; 0 when fewer than two edges are in range
  yaw   angle of the edge line against the robot's lateral axis, rad
        (0 = walking square to the stairs), from where the left and right
        lines cross the same edge; 0 when either line misses it
  dist  distance from the pelvis to the first edge ahead along the stair
        normal, m; X_MAX when there is no edge

The paper's 4th value is a terrain class (flat / up / down); here it is
sign(h), so the slot carries the edge distance instead, which is what foot
placement needs.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.sensor import GridPatternCfg, ObjRef, RayCastSensorCfg
from mjlab.sensor.raycast_sensor import RayCastSensor

STAIR_TOKEN_SENSOR = "stair_profile"
LINE_YS = (-0.15, 0.0, 0.15)
X_MIN, X_MAX, X_STEP = -0.2, 1.2, 0.02
EDGE_MIN_M = 0.01
# A side-line edge farther than this from the centre-line edge is another edge
# (|yaw| up to ~45 deg with lines 0.15 m apart).
MATCH_MAX_M = 0.15
# Rough unit scaling for the policy input (the checkpoint surgery starts the
# new normalizer entries at mean 0 / std 1).
TOKEN_SCALE = (10.0, 4.0, 1.0, 1.0)
TOKEN_DIM = 4


def _xs() -> list[float]:
  n = int(round((X_MAX - X_MIN) / X_STEP)) + 1
  return [X_MIN + i * X_STEP for i in range(n)]


@dataclass
class StairLinesPatternCfg(GridPatternCfg):
  """Parallel downward rays on lines along frame-local x (one line per y)."""

  def generate_rays(self, mj_model, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    pts = [(x, y, 0.0) for y in LINE_YS for x in _xs()]
    offsets = torch.tensor(pts, device=device, dtype=torch.float32)
    d = torch.tensor(self.direction, device=device, dtype=torch.float32)
    return offsets, (d / d.norm()).expand(len(pts), 3).clone()


def stair_profile_ray_cfg() -> RayCastSensorCfg:
  return RayCastSensorCfg(
    name=STAIR_TOKEN_SENSOR,
    frame=ObjRef(type="body", name="pelvis", entity="robot"),
    pattern=StairLinesPatternCfg(),
    ray_alignment="yaw",
    max_distance=5.0,
    exclude_parent_body=True,
    include_geom_groups=(0,),
  )


def _edges(z: torch.Tensor, xs: torch.Tensor):
  """z (B, L, N) ground height per ray. Returns x of every edge (inf where
  there is none) and the signed jump there, both (B, L, N-1). A riser that
  the 2 cm sampling splits over two neighbouring gaps counts once, at the
  first gap, with both jumps summed."""
  dz = z[..., 1:] - z[..., :-1]
  x_mid = (0.5 * (xs[1:] + xs[:-1])).expand_as(dz)
  flag = dz.abs() > EDGE_MIN_M
  prev = torch.cat([torch.zeros_like(flag[..., :1]), flag[..., :-1]], dim=-1)
  start = flag & ~prev
  nxt_dz = torch.cat([dz[..., 1:], torch.zeros_like(dz[..., :1])], dim=-1)
  nxt_flag = torch.cat([flag[..., 1:], torch.zeros_like(flag[..., :1])], dim=-1)
  jump = dz + torch.where(nxt_flag, nxt_dz, torch.zeros_like(dz))
  x_edge = torch.where(start, x_mid, torch.full_like(dz, float("inf")))
  return x_edge, jump


def stair_token_raw(env: ManagerBasedRlEnv, sensor_name: str = STAIR_TOKEN_SENSOR) -> torch.Tensor:
  """(B, 4) [h, d, yaw, dist] in meters / radians, unscaled."""
  sensor: RayCastSensor = env.scene[sensor_name]
  data = sensor.data
  n = len(_xs())
  z = torch.where(data.distances < 0, torch.full_like(data.distances, -1e3), data.hit_pos_w[..., 2])
  z = z.view(z.shape[0], len(LINE_YS), n)
  xs = torch.tensor(_xs(), device=z.device, dtype=z.dtype)
  x_edge, jump = _edges(z, xs)
  inf = float("inf")

  # Centre line: first and second edge ahead of the pelvis.
  xc = x_edge[:, 1]
  ahead = torch.where(xc > 0.0, xc, torch.full_like(xc, inf))
  first, i1 = ahead.min(dim=-1)
  second, _ = ahead.scatter(-1, i1.unsqueeze(-1), inf).min(dim=-1)
  rise = torch.gather(jump[:, 1], -1, i1.unsqueeze(-1)).squeeze(-1)
  has1 = torch.isfinite(first)
  has2 = has1 & torch.isfinite(second)

  # Side lines: the edge nearest to where the centre line crosses (behind
  # the pelvis too), so all three lines measure the SAME edge.
  def match(line: int) -> torch.Tensor:
    gap = (x_edge[:, line] - first.unsqueeze(-1)).abs()
    g, i = gap.min(dim=-1)
    x = torch.gather(x_edge[:, line], -1, i.unsqueeze(-1)).squeeze(-1)
    return torch.where(has1 & (g < MATCH_MAX_M), x, torch.full_like(x, inf))

  right, left = match(0), match(2)
  both = torch.isfinite(left) & torch.isfinite(right)
  span = LINE_YS[2] - LINE_YS[0]
  diff = torch.where(both, left - right, torch.zeros_like(first))
  yaw = torch.atan2(diff, torch.full_like(diff, span))
  cos = torch.cos(yaw)

  zero = torch.zeros_like(first)
  h = torch.where(has1, rise, zero)
  d = torch.where(has2, (second - first) * cos, zero)
  dist = torch.where(has1, first * cos, torch.full_like(first, X_MAX))
  return torch.stack([h, d, yaw, dist], dim=1)


def stair_token(env: ManagerBasedRlEnv, sensor_name: str = STAIR_TOKEN_SENSOR) -> torch.Tensor:
  """(B, 4) observation term: stair_token_raw scaled to roughly unit range."""
  raw = stair_token_raw(env, sensor_name)
  return raw * torch.tensor(TOKEN_SCALE, device=raw.device, dtype=raw.dtype)
