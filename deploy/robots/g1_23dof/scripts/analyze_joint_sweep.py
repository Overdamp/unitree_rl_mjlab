#!/usr/bin/env python3
"""Summarize a hand joint sweep recorded with lowstate_monitor.py --log.

Finds when each joint was moved (from |dq|), in any order, and reports per
joint: move time, segments, other joints that moved at the same time
(index crosstalk), and the q range seen against the MJCF limits. It cannot
check the sign; that needs the visual check against RViz/MuJoCo.

  python deploy/robots/g1_23dof/scripts/analyze_joint_sweep.py ~/g1_joints_20261028_1848.csv
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowstate_monitor import DEFAULT_MJCF, G1_23DOF_JOINTS, NUM_SLOTS, UNUSED_SLOTS, load_joint_limits  # noqa: E402

MOVING_DQ = 0.3  # rad/s on the smoothed |dq|
SMOOTH_S = 0.2
MERGE_GAP_S = 1.0
MIN_SEGMENT_S = 0.3
CROSSTALK_FRAC = 0.3  # another joint moving for this share of a segment


def read_log(path: Path, decimate: int):
  """Return decimated t, q, dq and the full-rate q min/max per slot."""
  q_min = np.full(NUM_SLOTS, np.inf)
  q_max = np.full(NUM_SLOTS, -np.inf)
  t, q, dq = [], [], []
  with open(path, newline="") as f:
    reader = csv.reader(f)
    header = next(reader)
    col = {name: i for i, name in enumerate(header)}
    qi = [col[f"q{i}"] for i in range(NUM_SLOTS)]
    dqi = [col[f"dq{i}"] for i in range(NUM_SLOTS)]
    ti = col["t"]
    for n, row in enumerate(reader):
      qs = np.array([float(row[i]) for i in qi])
      np.minimum(q_min, qs, out=q_min)
      np.maximum(q_max, qs, out=q_max)
      if n % decimate == 0:
        t.append(float(row[ti]))
        q.append(qs)
        dq.append([float(row[i]) for i in dqi])
  t = np.array(t)
  return t - t[0], np.array(q), np.array(dq), q_min, q_max


def segments(mask: np.ndarray, t: np.ndarray) -> list[tuple[int, int]]:
  segs = []
  start = None
  for i, m in enumerate(mask):
    if m and start is None:
      start = i
    elif not m and start is not None:
      segs.append([start, i - 1])
      start = None
  if start is not None:
    segs.append([start, len(mask) - 1])
  merged = []
  for s in segs:
    if merged and t[s[0]] - t[merged[-1][1]] < MERGE_GAP_S:
      merged[-1][1] = s[1]
    else:
      merged.append(s)
  return [(a, b) for a, b in merged if t[b] - t[a] >= MIN_SEGMENT_S]


def mmss(sec: float) -> str:
  return f"{int(sec // 60):02d}:{sec % 60:04.1f}"


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("csv", type=Path)
  parser.add_argument("--decimate", type=int, default=10, help="keep every Nth row for segmentation")
  parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
  args = parser.parse_args()

  limits = load_joint_limits(args.mjcf)
  t, q, dq, q_min, q_max = read_log(args.csv, args.decimate)
  dt = np.median(np.diff(t))
  k = max(1, int(round(SMOOTH_S / dt)))
  kernel = np.ones(k) / k
  speed = np.stack([np.convolve(np.abs(dq[:, i]), kernel, mode="same") for i in range(NUM_SLOTS)], axis=1)
  moving = speed > MOVING_DQ

  print(f"{args.csv.name}: {t[-1]:.0f} s, {len(t)} samples after decimation x{args.decimate}")
  print()

  timeline = []
  rows = []
  for idx, name in G1_23DOF_JOINTS.items():
    segs = segments(moving[:, idx], t)
    move_time = sum(t[b] - t[a] for a, b in segs)
    crosstalk = set()
    for a, b in segs:
      share = moving[a : b + 1].mean(axis=0)
      dominant = int(np.argmax(speed[a : b + 1].mean(axis=0)))
      timeline.append((t[a], t[b], idx, dominant))
      for j in G1_23DOF_JOINTS:
        if j != idx and share[j] > CROSSTALK_FRAC:
          crosstalk.add(j)
    lo, hi = limits[name]
    span = q_max[idx] - q_min[idx]
    rows.append((idx, name, len(segs), move_time, q_min[idx], q_max[idx], lo, hi, span / (hi - lo), sorted(crosstalk)))

  print(f"{'idx':>3} {'joint':<27} {'segs':>4} {'move s':>6} {'q min':>7} {'q max':>7} {'MJCF lo':>8} {'MJCF hi':>8} {'cover':>6}  note")
  for idx, name, nseg, mt, qlo, qhi, lo, hi, cover, xt in rows:
    note = []
    if nseg == 0:
      note.append("NOT MOVED")
    if qlo < lo - 0.02 or qhi > hi + 0.02:
      note.append("BEYOND MJCF limit")
    if xt:
      note.append("moved together with " + ",".join(str(j) for j in xt))
    print(
      f"{idx:>3} {name:<27} {nseg:>4} {mt:6.1f} {qlo:7.3f} {qhi:7.3f} {lo:8.3f} {hi:8.3f} {cover * 100:5.0f}%  {'; '.join(note)}"
    )

  unused_moved = [i for i in UNUSED_SLOTS if q_max[i] - q_min[i] > 1e-3]
  print()
  print(f"unused slots {UNUSED_SLOTS}: " + ("moved: " + str(unused_moved) if unused_moved else "never changed"))

  print()
  print("Timeline (segment start-end, joint whose dq crossed the threshold, joint with the highest mean speed):")
  seen = set()
  for t0, t1, idx, dom in sorted(timeline):
    key = (round(t0, 1), idx)
    if key in seen:
      continue
    seen.add(key)
    flag = "" if dom == idx else f"  (fastest: {dom} {G1_23DOF_JOINTS.get(dom, '?')})"
    print(f"  {mmss(t0)}-{mmss(t1)}  {idx:>2} {G1_23DOF_JOINTS[idx]}{flag}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
