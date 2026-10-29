#!/usr/bin/env bash
# Sim2sim one policy folder with unitree_mujoco + g1_ctrl without touching the
# real config: g1_ctrl reads config.yaml next to its binary, so this copies the
# binary and config/ to a scratch directory and points Velocity.policy_dir there.
#
# Usage: deploy/robots/g1_23dof/scripts/sim2sim_policy.sh hist5_yawft
#        WORK=/tmp/x SESSION=name deploy/robots/g1_23dof/scripts/sim2sim_policy.sh v0
#
# Opens tmux session "sim2sim": window 0 unitree_mujoco, window 1 g1_ctrl (keys go
# here), window 2 sim_speed_monitor.py. In MuJoCo: Pause, load key 0 (stand_ready);
# in window 1 press u then v; Run MuJoCo; then w / q / e / a / d / x in window 1.
set -euo pipefail

POLICY="${1:?usage: $0 <folder under config/policy/velocity>}"
ROBOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "${ROBOT_DIR}/../../.." && pwd)"
WORK="${WORK:-/tmp/g1_sim2sim_${POLICY}}"
SESSION="${SESSION:-sim2sim}"
PY="${PY:-$HOME/miniconda3/envs/g1_sdk/bin/python}"

if [ ! -f "${ROBOT_DIR}/config/policy/velocity/${POLICY}/exported/policy.onnx" ]; then
  echo "No config/policy/velocity/${POLICY}/exported/policy.onnx" >&2
  exit 1
fi
if pgrep -x g1_ctrl >/dev/null || pgrep -x unitree_mujoco >/dev/null; then
  echo "g1_ctrl or unitree_mujoco is already running; stop it first." >&2
  exit 1
fi

rm -rf "${WORK}"
mkdir -p "${WORK}/build"
cp "${ROBOT_DIR}/build/g1_ctrl" "${WORK}/build/"
cp -r "${ROBOT_DIR}/config" "${WORK}/config"
sed -i "s#^    policy_dir: config/policy/velocity/.*#    policy_dir: config/policy/velocity/${POLICY}#" "${WORK}/config/config.yaml"
grep -q "policy_dir: config/policy/velocity/${POLICY}$" "${WORK}/config/config.yaml"

# The Unitree binaries crash on the ROS libddsc (dds_write.c iox_pub assertion).
CLEAN="env -u CYCLONEDDS_URI LD_LIBRARY_PATH=/opt/unitree_robotics/lib"

tmux kill-session -t "${SESSION}" 2>/dev/null || true
tmux new-session -d -s "${SESSION}" -n mujoco "cd ${REPO_ROOT}/simulate/build && ${CLEAN} ./unitree_mujoco; read"
sleep 3
tmux new-window -t "${SESSION}" -n g1_ctrl "cd ${WORK}/build && ${CLEAN} ./g1_ctrl --network lo; read"
tmux new-window -t "${SESSION}" -n speed "cd ${REPO_ROOT} && ${PY} deploy/robots/g1_23dof/scripts/sim_speed_monitor.py --iface lo --log ${WORK}/speed.csv; read"
tmux select-window -t "${SESSION}:g1_ctrl"
echo "Policy ${POLICY}, scratch copy in ${WORK}. Attach with: tmux attach -t ${SESSION}"
