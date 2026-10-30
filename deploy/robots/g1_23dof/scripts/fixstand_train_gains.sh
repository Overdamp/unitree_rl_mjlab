#!/usr/bin/env bash
# Gantry kp/kd check at the training gains: a scratch copy of g1_ctrl whose
# FixStand uses the policy's deploy.yaml stiffness/damping (BeyondMimic) instead
# of Unitree's 100/150/40/200. The real config is not touched.
#
# The 30/10/2026 check verified kp 40-200 and kd 2.5-10; the policy uses arm
# kp 14.3 / kd 0.9 and leg kd 1.8-6.3, which this run measures directly.
#
# Usage: deploy/robots/g1_23dof/scripts/fixstand_train_gains.sh [policy folder, default v0]
#        WORK=/tmp/x deploy/robots/g1_23dof/scripts/fixstand_train_gains.sh hist5_yawstand
# Then:  G1_CTRL_DIR=/tmp/g1_fixstand_train/build deploy/robots/g1_23dof/scripts/deploy_byobu.sh 1 bm
#
# In the copy, FixStand has no RT + A transition (Velocity cannot be entered) and
# the unused slots get kp = kd = 0 like in Velocity. lowcmd_monitor shows
# "gains match: Velocity" in FixStand (same gains) and pd_check labels it Velocity.
# Robot hung on the gantry only: arms (kp 14.3) sag further from the pose than
# with Unitree's gains, that is expected.
set -euo pipefail

POLICY="${1:-v0}"
ROBOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORK="${WORK:-/tmp/g1_fixstand_train}"
PY="${PY:-$HOME/miniconda3/envs/g1_sdk/bin/python}"
DEPLOY="${ROBOT_DIR}/config/policy/velocity/${POLICY}/params/deploy.yaml"

if [ ! -f "${DEPLOY}" ]; then
  echo "No ${DEPLOY}" >&2
  exit 1
fi
if pgrep -x g1_ctrl >/dev/null; then
  echo "g1_ctrl is already running; stop it first." >&2
  exit 1
fi

rm -rf "${WORK}"
mkdir -p "${WORK}/build"
cp "${ROBOT_DIR}/build/g1_ctrl" "${WORK}/build/"
cp -r "${ROBOT_DIR}/config" "${WORK}/config"
sed -i "s#^    policy_dir: config/policy/velocity/.*#    policy_dir: config/policy/velocity/${POLICY}#" "${WORK}/config/config.yaml"

"${PY}" - "${WORK}/config/config.yaml" "${DEPLOY}" <<'EOF'
import re, sys
import yaml

cfg_path, deploy_path = sys.argv[1], sys.argv[2]
deploy = yaml.safe_load(open(deploy_path))
kp, kd = [0.0] * 29, [0.0] * 29
for j, s, d in zip(deploy["joint_ids_map"], deploy["stiffness"], deploy["damping"]):
  kp[j], kd[j] = float(s), float(d)

def block(name, vals):
  rows = [vals[0:6], vals[6:12], vals[12:15], vals[15:22], vals[22:29]]
  body = "\n".join("      " + ", ".join(f"{v:g}" for v in r) + "," for r in rows)
  return f"    {name}: [\n{body}\n    ]"

text = open(cfg_path).read()
start = text.index("\n  FixStand:\n")
end = text.index("\n  Velocity:\n", start)
fix = text[start:end]
fix = re.sub(r"    kp: \[.*?\]", lambda _: block("kp", kp), fix, count=1, flags=re.S)
fix = re.sub(r"    kd: \[.*?\]", lambda _: block("kd", kd), fix, count=1, flags=re.S)
fix = re.sub(r"\n      Velocity: RT \+ A\.on_pressed", "", fix)
text = text[:start] + fix + text[end:]
open(cfg_path, "w").write(text)

got = yaml.safe_load(text)["FSM"]["FixStand"]
assert [float(v) for v in got["kp"]] == kp and [float(v) for v in got["kd"]] == kd
assert "Velocity" not in got["transitions"]
names = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
print("FixStand gains in the copy (left leg / waist / left arm; right side is the same):")
for i, n in enumerate(names):
  print(f"  {n:<12} kp {kp[i]:6.1f}  kd {kd[i]:4.1f}")
print(f"  {'waist_yaw':<12} kp {kp[12]:6.1f}  kd {kd[12]:4.1f}")
print(f"  {'arm (all 5)':<12} kp {kp[15]:6.1f}  kd {kd[15]:4.1f}")
print("  RT + A removed from FixStand: Velocity cannot be entered")
EOF

echo
echo "Scratch g1_ctrl in ${WORK} (policy ${POLICY}). Start the deploy-day panes with it:"
echo "  G1_CTRL_DIR=${WORK}/build deploy/robots/g1_23dof/scripts/deploy_byobu.sh 1 bm"
