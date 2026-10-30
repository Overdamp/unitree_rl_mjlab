#!/usr/bin/env bash
# Real-robot deploy day in one byobu window, four panes = T1-T4 of
# doc/print/g1_23dof_deploy_day_checklist.pdf:
#
#   +-----------------------+-----------------------+
#   | T1 lowstate_monitor   | T2 lowcmd_monitor     |
#   +-----------------------+-----------------------+
#   | T3 g1_ctrl            | T4 checks / pd_check  |
#   +-----------------------+-----------------------+
#
# Every pane is set up (repo dir, conda g1_sdk, no CYCLONEDDS_URI, RUN, TAG) and
# has its command typed but NOT run: check the robot, then press Enter yourself.
# Ctrl-C a monitor to close its CSV, Up + Enter to start it again.
#
# Usage: deploy/robots/g1_23dof/scripts/deploy_byobu.sh [RUN] [TAG]
#        RUN = 1, 2, ... (default 1); TAG = run (hung, kp/kd check) or walk (default run)
#        logs: ~/g1_state_<date>_<TAG><RUN>.csv, ~/g1_cmd_<date>_<TAG><RUN>.csv
#        G1_CTRL_DIR=<dir> runs g1_ctrl from another build dir (e.g. the scratch copy
#        made by fixstand_train_gains.sh, TAG bm)
#
# Inside byobu: click a pane (mouse is on) or Shift + arrows to move between panes
# (Alt + arrows switch windows, not panes),
# F6 detaches (everything keeps running), `byobu attach -t g1deploy` comes back.
set -euo pipefail

RUN="${1:-1}"
TAG="${2:-run}"
SESSION="${SESSION:-g1deploy}"
IFACE="${IFACE:-enp43s0}"
ROBOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "${ROBOT_DIR}/../../.." && pwd)"
G1_CTRL_DIR="${G1_CTRL_DIR:-${ROBOT_DIR}/build}"
S=deploy/robots/g1_23dof/scripts

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  # Never kill it: g1_ctrl may be holding the robot.
  echo "Session ${SESSION} already exists, attaching (kill it yourself when g1_ctrl is stopped)."
  exec byobu attach -t "${SESSION}"
fi
if pgrep -x g1_ctrl >/dev/null; then
  echo "g1_ctrl is already running outside this session; stop it first." >&2
  exit 1
fi
if [ ! -x "${G1_CTRL_DIR}/g1_ctrl" ]; then
  echo "No ${G1_CTRL_DIR}/g1_ctrl" >&2
  exit 1
fi

SETUP="cd ${REPO_ROOT} && source ~/miniconda3/etc/profile.d/conda.sh && conda activate g1_sdk && unset CYCLONEDDS_URI && RUN=${RUN} TAG=${TAG} D=\$(date +%Y%m%d) && clear"

byobu new-session -d -s "${SESSION}" -n deploy
P1=$(tmux display-message -p -t "${SESSION}" '#{pane_id}')
P2=$(tmux split-window -h -t "${P1}" -P -F '#{pane_id}')
P3=$(tmux split-window -v -t "${P1}" -P -F '#{pane_id}')
P4=$(tmux split-window -v -t "${P2}" -P -F '#{pane_id}')

tmux set-option -t "${SESSION}" mouse on
tmux set-option -t "${SESSION}" pane-border-status top
tmux set-option -t "${SESSION}" pane-border-format ' #{pane_title} '
tmux select-pane -t "${P1}" -T "T1 lowstate_monitor (read-only)"
tmux select-pane -t "${P2}" -T "T2 lowcmd_monitor (read-only)"
tmux select-pane -t "${P3}" -T "T3 g1_ctrl (SENDS MOTOR COMMANDS) ${G1_CTRL_DIR}"
tmux select-pane -t "${P4}" -T "T4 checks / motion_mode / pd_check"

for p in "${P1}" "${P2}" "${P3}" "${P4}"; do
  tmux send-keys -t "${p}" "${SETUP}" Enter
done
sleep 1.5

# Typed, not run.
tmux send-keys -t "${P1}" "python ${S}/lowstate_monitor.py --iface ${IFACE} --log ~/g1_state_\${D}_\${TAG}\${RUN}.csv"
tmux send-keys -t "${P2}" "python ${S}/lowcmd_monitor.py --iface ${IFACE} --log ~/g1_cmd_\${D}_\${TAG}\${RUN}.csv"
tmux send-keys -t "${P3}" "cd ${G1_CTRL_DIR} && env -u CYCLONEDDS_URI LD_LIBRARY_PATH=/opt/unitree_robotics/lib ./g1_ctrl --network ${IFACE}"
tmux send-keys -t "${P4}" "echo '1) ping   2) gain_check   3) motion_mode check   4) motion_mode release (if L2+R2 fails)   5) pd_check (after T1/T2 Ctrl-C)'; \
history -s \"python ${S}/pd_check.py ~/g1_state_\${D}_\${TAG}\${RUN}.csv ~/g1_cmd_\${D}_\${TAG}\${RUN}.csv\"; \
history -s \"python ${S}/motion_mode.py release --iface ${IFACE}\"; \
history -s \"python ${S}/motion_mode.py check --iface ${IFACE}\"; \
history -s \"python ${S}/gain_check.py ${S%/scripts}/config/policy/velocity/v0\"; \
history -s \"ping -c 3 192.168.123.161\"" Enter

tmux select-pane -t "${P4}"
exec byobu attach -t "${SESSION}"
