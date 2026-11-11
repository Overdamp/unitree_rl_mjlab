"""Open-riser stairs terrain for G1-23DoF (Phase 4).

Each tread is a thin floating plate with nothing between it and the tread
below, like a steel or wooden staircase with no risers. A blind policy cannot
feel a riser with its toe here: the toe slides under the next tread and the
shin hits its edge.

The layout follows mjlab's pyramid stairs (square rings around a centre
platform) so a robot walking out in any direction meets stairs. mjlab's own
BoxOpenStairsTerrainCfg has no floor under the plates (a depth camera would see
a void) and treads 0.4-0.8 m deep; this one adds a floor, real tread depths,
a nosing overhang and per-patch random tread depth / thickness / nosing.

  pyramid  (inverted=False): spawn on the top platform, walk out = go DOWN.
  bowl     (inverted=True):  spawn on the bottom platform, walk out = go UP.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from mjlab.terrains.terrain_generator import (
  SubTerrainCfg,
  TerrainGeometry,
  TerrainOutput,
)
from mjlab.terrains.utils import make_border
from mjlab.utils.color import brand_ramp, darken_rgba

_STAIR_RGB = (0.90, 0.55, 0.20)
_FLOOR_THICKNESS = 0.1


def _uniform(rng: np.random.Generator, lo_hi: tuple[float, float]) -> float:
  lo, hi = lo_hi
  return float(lo) if hi <= lo else float(rng.uniform(lo, hi))


@dataclass(kw_only=True)
class OpenRiserStairsTerrainCfg(SubTerrainCfg):
  """Pyramid (or bowl) of open-riser stairs with a floor underneath."""

  step_height_range: tuple[float, float] = (0.05, 0.18)
  """Rise per step, in meters. Interpolated by difficulty (row)."""
  tread_depth_range: tuple[float, float] = (0.25, 0.32)
  """Going (horizontal distance between nosings), in meters. Random per patch."""
  tread_thickness_range: tuple[float, float] = (0.03, 0.06)
  """Plate thickness, in meters. Random per patch; clipped below the rise."""
  nosing_range: tuple[float, float] = (0.0, 0.03)
  """How far each tread overhangs the one below, in meters. Random per patch."""
  platform_width: float = 2.0
  """Side of the flat square platform at the centre, in meters."""
  border_width: float = 1.0
  """Flat frame around the stairs at floor level, in meters."""
  inverted: bool = False
  """False: pyramid (walk out = down). True: bowl (walk out = up)."""

  def function(
    self, difficulty: float, spec: mujoco.MjSpec, rng: np.random.Generator
  ) -> TerrainOutput:
    body = spec.body("terrain")
    geometries: list[TerrainGeometry] = []

    rise = self.step_height_range[0] + difficulty * (
      self.step_height_range[1] - self.step_height_range[0]
    )
    run = _uniform(rng, self.tread_depth_range)
    # Keep an open gap under every tread (at least 40% of the rise).
    thick = min(_uniform(rng, self.tread_thickness_range), 0.6 * rise)
    nosing = min(_uniform(rng, self.nosing_range), 0.5 * run)

    inner = (
      self.size[0] - 2 * self.border_width,
      self.size[1] - 2 * self.border_width,
    )
    num_steps = max(0, int((min(inner) - self.platform_width) / (2 * run)))
    cx, cy = 0.5 * self.size[0], 0.5 * self.size[1]

    # Heights: the floor top is where the lowest walking surface sits.
    #   pyramid: border/floor at 0, ring k top at (k+1)*rise, platform on top.
    #   bowl:    border at 0, ring k top at -k*rise, floor/platform at -N*rise.
    if self.inverted:
      floor_top = -num_steps * rise
      ring_top = [-k * rise for k in range(num_steps)]
      platform_top = floor_top
    else:
      floor_top = 0.0
      ring_top = [(k + 1) * rise for k in range(num_steps)]
      platform_top = (num_steps + 1) * rise

    border_rgba = darken_rgba(brand_ramp(_STAIR_RGB, 0.0), 0.85)
    if self.border_width > 0.0:
      # Reach below the floor so a bowl has walls, not a gap into the void.
      border_h = max(0.5, -floor_top + _FLOOR_THICKNESS)
      for geom in make_border(
        body, self.size, inner, border_h, (cx, cy, -0.5 * border_h)
      ):
        geometries.append(TerrainGeometry(geom=geom, color=border_rgba))

    def add_box(center, half, rgba):
      geom = body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=tuple(float(max(1e-6, h)) for h in half),
        pos=tuple(float(c) for c in center),
      )
      geometries.append(TerrainGeometry(geom=geom, color=rgba))

    # Floor under the whole stair area: a depth camera looking through the
    # open risers sees this, as it would see the ground under real stairs.
    add_box(
      (cx, cy, floor_top - 0.5 * _FLOOR_THICKNESS),
      (0.5 * inner[0], 0.5 * inner[1], 0.5 * _FLOOR_THICKNESS),
      border_rgba,
    )

    for k in range(num_steps):
      rgba = brand_ramp(_STAIR_RGB, k / max(num_steps - 1, 1))
      # Ring k covers [k*run, (k+1)*run] in from the stair edge. The nosing
      # sticks out over the LOWER neighbour: outward for a pyramid, inward
      # for a bowl. Ring 0 of a pyramid has nothing lower outside it.
      if self.inverted:
        a, b = k * run, (k + 1) * run + nosing
      else:
        a, b = k * run - (nosing if k > 0 else 0.0), (k + 1) * run
      depth = b - a
      outer = (inner[0] - 2 * a, inner[1] - 2 * a)
      z = ring_top[k] - 0.5 * thick
      off_y = 0.5 * inner[1] - a - 0.5 * depth
      off_x = 0.5 * inner[0] - a - 0.5 * depth
      # Front/back strips span the full ring width; left/right fill between.
      for s in (1, -1):
        add_box((cx, cy + s * off_y, z), (0.5 * outer[0], 0.5 * depth, 0.5 * thick), rgba)
        add_box(
          (cx + s * off_x, cy, z),
          (0.5 * depth, 0.5 * (outer[1] - 2 * depth), 0.5 * thick),
          rgba,
        )

    if not self.inverted:
      # Top platform: also a plate, open underneath like the treads.
      side = (
        inner[0] - 2 * num_steps * run + 2 * nosing,
        inner[1] - 2 * num_steps * run + 2 * nosing,
      )
      add_box(
        (cx, cy, platform_top - 0.5 * thick),
        (0.5 * side[0], 0.5 * side[1], 0.5 * thick),
        brand_ramp(_STAIR_RGB, 1.0),
      )

    origin = np.array([cx, cy, platform_top])
    return TerrainOutput(origin=origin, geometries=geometries)
