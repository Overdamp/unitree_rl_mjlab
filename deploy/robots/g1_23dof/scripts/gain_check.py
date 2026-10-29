#!/usr/bin/env python3
"""Check a policy folder before deploy: deploy.yaml against the training values in the ONNX.

VelocityOnPolicyRunner writes the training joint order, kp, kd, default pose and
action scale into policy.onnx metadata. This compares them with deploy.yaml
(which g1_ctrl actually uses), and checks the joint map, the action offset, the
observation size against the ONNX input, and any action clip against the MJCF
joint range. Exit code 1 on any mismatch. No robot needed.

  python deploy/robots/g1_23dof/scripts/gain_check.py deploy/robots/g1_23dof/config/policy/velocity/v0
  python deploy/robots/g1_23dof/scripts/gain_check.py --onnx run/policy.onnx --deploy some/deploy.yaml
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import onnx
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lowstate_monitor import DEFAULT_MJCF, G1_23DOF_JOINTS, load_joint_limits  # noqa: E402

# deploy.yaml rounds to one or two decimals (kd 0.9 for 0.907, scale 0.55 for 0.548).
CHECKS = [
  ("joint_stiffness", "stiffness", 0.06),
  ("joint_damping", "damping", 0.06),
  ("default_joint_pos", "default_joint_pos", 1e-3),
  ("action_scale", "scale", 0.01),
]
OBS_SIZES = {
  "base_ang_vel": 3,
  "projected_gravity": 3,
  "velocity_commands": 3,
  "gait_phase": 2,
  "joint_pos_rel": 23,
  "joint_vel_rel": 23,
  "last_action": 23,
}


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("policy_dir", type=Path, nargs="?", help="folder with exported/policy.onnx and params/deploy.yaml")
  parser.add_argument("--onnx", type=Path)
  parser.add_argument("--deploy", type=Path)
  args = parser.parse_args()
  onnx_path = args.onnx or (args.policy_dir / "exported/policy.onnx" if args.policy_dir else None)
  deploy_path = args.deploy or (args.policy_dir / "params/deploy.yaml" if args.policy_dir else None)
  if onnx_path is None or deploy_path is None:
    parser.error("give a policy folder or both --onnx and --deploy")

  model = onnx.load(str(onnx_path))
  meta = {p.key: p.value for p in model.metadata_props}
  dep = yaml.safe_load(open(deploy_path))
  act = dep["actions"]["JointPositionAction"]
  values = {"stiffness": dep["stiffness"], "damping": dep["damping"],
            "default_joint_pos": dep["default_joint_pos"], "scale": act["scale"]}
  print(f"onnx   {onnx_path}\ndeploy {deploy_path}\n")
  problems = []

  names = meta["joint_names"].split(",")
  ids = dep["joint_ids_map"]
  bad_map = [(i, n, s, G1_23DOF_JOINTS.get(s)) for i, (n, s) in enumerate(zip(names, ids)) if G1_23DOF_JOINTS.get(s) != n]
  if len(names) != len(ids) or bad_map:
    problems.append(f"joint map: {bad_map or 'length differs'}")
  print(f"{'joint map (training order -> SDK slot)':40s} {'OK' if not problems else 'MISMATCH'}")

  for meta_key, dep_key, tol in CHECKS:
    a = np.array([float(x) for x in meta[meta_key].split(",")])
    b = np.array(values[dep_key], float)
    if a.shape != b.shape:
      problems.append(f"{dep_key}: {len(b)} values, training has {len(a)}")
      print(f"{dep_key:40s} MISMATCH (length)")
      continue
    rel = np.abs(a - b) / np.maximum(np.abs(a), 1e-6)
    bad = [(names[i], round(a[i], 3), b[i]) for i in range(len(a)) if abs(a[i] - b[i]) > 1e-3 and rel[i] > tol]
    if bad:
      problems.append(f"{dep_key}: {bad}")
    print(f"{dep_key:40s} {'OK' if not bad else 'MISMATCH'}  (max diff {rel.max() * 100:.2f}%, rounding)")

  offset_ok = np.allclose(act["offset"], dep["default_joint_pos"], atol=1e-6)
  if not offset_ok:
    problems.append("action offset != default_joint_pos")
  print(f"{'action offset == default_joint_pos':40s} {'OK' if offset_ok else 'MISMATCH'}")

  n_in = model.graph.input[0].type.tensor_type.shape.dim[-1].dim_value
  obs = dep["observations"]
  n_obs = sum(OBS_SIZES[k] * (v.get("history_length") or 1) for k, v in obs.items())
  if n_in != n_obs:
    problems.append(f"obs size {n_obs} != ONNX input {n_in}")
  print(f"{'obs size vs ONNX input':40s} {'OK' if n_in == n_obs else 'MISMATCH'}  ({n_obs} / {n_in})")

  if act.get("clip"):
    limits = load_joint_limits(DEFAULT_MJCF)
    clip = np.array(act["clip"], float)
    wide = [(n, c.tolist(), limits[n]) for n, c in zip(names, clip) if c[0] < limits[n][0] - 1e-6 or c[1] > limits[n][1] + 1e-6]
    narrow = [n for n, c in zip(names, clip) if c[0] >= c[1]]
    if wide or narrow:
      problems.append(f"clip outside MJCF range or empty: {wide or narrow}")
    print(f"{'action clip within MJCF joint range':40s} {'OK' if not (wide or narrow) else 'MISMATCH'}")
  else:
    print(f"{'action clip':40s} none (targets not limited)")

  print()
  if problems:
    print("FAIL:\n  " + "\n  ".join(problems))
    return 1
  print("PASS")
  return 0


if __name__ == "__main__":
  sys.exit(main())
