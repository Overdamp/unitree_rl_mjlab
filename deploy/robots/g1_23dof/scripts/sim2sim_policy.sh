#!/usr/bin/env bash
# Sim2sim one policy folder with unitree_mujoco + g1_ctrl without touching the
# real config: g1_ctrl reads config.yaml next to its binary, so this copies the
# binary and config/ to a scratch directory and points Velocity.policy_dir there.
#
# Usage: deploy/robots/g1_23dof/scripts/sim2sim_policy.sh hist5_yawft
#        WORK=/tmp/x SESSION=name deploy/robots/g1_23dof/scripts/sim2sim_policy.sh v0
#
# Opens tmux session "sim2sim", one window with three panes (like deploy_byobu.sh):
#
#   +----------------------+----------------------+
#   | unitree_mujoco (log) | g1_ctrl              |
#   +----------------------+   (keys go here,     |
#   | sim_speed_monitor    |    selected at start)|
#   +----------------------+----------------------+
#
# The MuJoCo picture is its own desktop window; the left-top pane is only its log.
# In MuJoCo: Pause, load key 0 (stand_ready); in the g1_ctrl pane press u then v;
# Run MuJoCo; then w / q / e / a / d / x in the g1_ctrl pane.
# Mouse is on: click a pane to type in it. Ctrl+b then z zooms a pane.
# From a normal terminal it attaches by itself; NO_ATTACH=1 only starts it.
# Done: tmux kill-session -t sim2sim
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
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "  It is probably the last run: tmux kill-session -t ${SESSION}" >&2
  fi
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
P_SIM=$(tmux new-session -d -s "${SESSION}" -n sim2sim -P -F '#{pane_id}' \
  "cd ${REPO_ROOT}/simulate/build && ${CLEAN} ./unitree_mujoco; read")
sleep 3
P_CTRL=$(tmux split-window -h -t "${P_SIM}" -P -F '#{pane_id}' \
  "cd ${WORK}/build && ${CLEAN} ./g1_ctrl --network lo --domain 1; read")
P_SPEED=$(tmux split-window -v -t "${P_SIM}" -P -F '#{pane_id}' \
  "cd ${REPO_ROOT} && ${PY} deploy/robots/g1_23dof/scripts/sim_speed_monitor.py --network lo --domain 1 --log ${WORK}/speed.csv; read")

tmux set-option -t "${SESSION}" mouse on
tmux set-option -t "${SESSION}" pane-border-status top
tmux select-pane -t "${P_SIM}" -T "unitree_mujoco (log; the picture is its own window)"
tmux select-pane -t "${P_SPEED}" -T "sim_speed_monitor"
tmux select-pane -t "${P_CTRL}" -T "g1_ctrl ${POLICY} (keys here: u v w a s d q e x p)"
tmux select-pane -t "${P_CTRL}"   # -T only renames; this makes it the active pane
echo "Policy ${POLICY}, scratch copy in ${WORK}."
# Attach straight away from a normal terminal (like deploy_byobu.sh). Not from
# inside tmux (that would nest) or without a terminal (scripts, NO_ATTACH=1).
if [ -z "${TMUX:-}" ] && [ -t 0 ] && [ -t 1 ] && [ -z "${NO_ATTACH:-}" ]; then
  exec tmux attach -t "${SESSION}"
fi
echo "Attach with: tmux attach -t ${SESSION}   (or, inside tmux: tmux switch-client -t ${SESSION})"
