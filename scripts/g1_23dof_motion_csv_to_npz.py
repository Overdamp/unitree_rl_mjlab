"""Convert G1 motion CSV clips (29-DoF or 23-DoF) to G1-23DoF .npz for AMP.

    CSV G1 29-DoF (7 + 29 cols) --drop 6 joints--> CSV 23-DoF (7 + 23 cols)
    CSV 23-DoF ------------------------------------^
        --> scripts/csv_to_npz.py --robot g1_23dof (replay in MuJoCo, 50 fps)
        --> src/assets/motions/g1_23dof/<name>.npz  (format read by amp_ppo.py)

CSV row: base xyz (m), base quat xyzw, then joint angles (rad) in the order
below (the LAFAN1 G1 retargeting order, also what csv_to_npz.py expects).

29 -> 23 drops waist roll/pitch and wrist pitch/yaw. The pelvis pose and the
legs are kept as they are, so feet stay where the clip put them; what changes
is the torso (and so the arms) tilt where the clip used waist roll/pitch.
AMP features (amp_ppo.py) are pelvis-frame joints, base velocities, gravity
and feet, so a clip with little waist roll/pitch (walking, stairs) converts
well; the script prints how much the dropped joints moved and warns above
WAIST_WARN_RAD.

Usage (repo root, conda env active):
  python scripts/g1_23dof_motion_csv_to_npz.py --inputs walk1.csv stairs*.csv --input-fps 30
  python scripts/g1_23dof_motion_csv_to_npz.py --inputs clip.csv --start-s 2.0 --end-s 12.0 --name stairs_up_a
Large clip files are data: keep them out of git unless agreed.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]

G1_29_JOINTS = [
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
  "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
DROPPED = (
  "waist_roll_joint", "waist_pitch_joint",
  "left_wrist_pitch_joint", "left_wrist_yaw_joint",
  "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
G1_23_JOINTS = [j for j in G1_29_JOINTS if j not in DROPPED]
WAIST_WARN_RAD = 0.2
OUT_DIR = ROOT / "src" / "assets" / "motions" / "g1_23dof"


def to_23dof(motion: np.ndarray, path: str) -> np.ndarray:
  cols = motion.shape[1]
  if cols == 7 + 23:
    return motion
  if cols != 7 + 29:
    raise ValueError(f"{path}: {cols} columns; expected 36 (G1 29-DoF) or 30 (G1 23-DoF)")
  for name in DROPPED:
    i = G1_29_JOINTS.index(name)
    peak = float(np.abs(motion[:, 7 + i]).max())
    flag = "  <-- torso/arm pose will differ" if name.startswith("waist") and peak > WAIST_WARN_RAD else ""
    print(f"  dropped {name:24s} max |q| = {peak:.3f} rad{flag}")
  keep = list(range(7)) + [7 + G1_29_JOINTS.index(j) for j in G1_23_JOINTS]
  return motion[:, keep]


def load_csv_to_npz():
  """scripts/csv_to_npz.py is not a package module; load it by path."""
  spec = importlib.util.spec_from_file_location("csv_to_npz", ROOT / "scripts" / "csv_to_npz.py")
  assert spec is not None and spec.loader is not None
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def check_npz(path: Path, fps: float) -> None:
  d = np.load(path)
  t, j = d["joint_pos"].shape
  b = d["body_pos_w"].shape[1]
  ok = j == 23 and abs(float(d["fps"][0]) - fps) < 1e-6
  print(f"  -> {path.relative_to(ROOT)}: {t} frames ({t / fps:.1f} s), {j} joints, {b} bodies"
        f"{'' if ok else '  !! unexpected'}")
  z = d["body_pos_w"][:, 0, 2]
  print(f"     pelvis z {z.min():.2f}..{z.max():.2f} m")


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--inputs", nargs="+", required=True, help="CSV files (29- or 23-DoF G1)")
  ap.add_argument("--input-fps", type=float, default=30.0, help="CSV frame rate (LAFAN1 G1: 30)")
  ap.add_argument("--output-fps", type=float, default=50.0, help="must equal the policy rate (50 Hz)")
  ap.add_argument("--start-s", type=float, default=None, help="cut: start time in the clip (s)")
  ap.add_argument("--end-s", type=float, default=None, help="cut: end time in the clip (s)")
  ap.add_argument("--name", default=None, help="output name (single input only); default = CSV stem")
  ap.add_argument("--device", default="cuda:0")
  ap.add_argument("--no-npz", action="store_true", help="only write the 23-DoF CSV")
  args = ap.parse_args()
  if args.name and len(args.inputs) != 1:
    ap.error("--name needs exactly one input")

  OUT_DIR.mkdir(parents=True, exist_ok=True)
  # csv_to_npz imports src.* and writes to ./src/assets/motions/<robot>/.
  os.chdir(ROOT)
  sys.path.insert(0, str(ROOT))
  c2n = None if args.no_npz else load_csv_to_npz()

  for path in args.inputs:
    name = args.name or Path(path).stem
    print(f"[{name}] {path}")
    motion = np.loadtxt(path, delimiter=",", ndmin=2)
    first = 0 if args.start_s is None else int(round(args.start_s * args.input_fps))
    last = motion.shape[0] if args.end_s is None else int(round(args.end_s * args.input_fps)) + 1
    motion = motion[first:last]
    if motion.shape[0] < 3:
      raise ValueError(f"{path}: fewer than 3 frames after cutting")
    motion23 = to_23dof(motion, path)
    csv23 = OUT_DIR / f"{name}.csv"
    np.savetxt(csv23, motion23, delimiter=",", fmt="%.6f")
    print(f"  23-DoF CSV: {csv23.relative_to(ROOT)} ({motion23.shape[0]} frames at {args.input_fps:g} fps)")
    if c2n is None:
      continue
    c2n.main(
      robot="g1_23dof",
      input_file=str(csv23),
      output_name=name,
      input_fps=args.input_fps,
      output_fps=args.output_fps,
      device=args.device,
    )
    check_npz(OUT_DIR / f"{name}.npz", args.output_fps)


if __name__ == "__main__":
  main()
