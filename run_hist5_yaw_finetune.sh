#!/usr/bin/env bash
# Hist5 yaw fine-tune: resume the from-scratch Hist5 run (model_14999) with the
# angular tracking weight at 2.0 and ~70% of envs on constant wz commands
# (task Unitree-G1-23Dof-Rough-Blind-Hist5-Yaw). Same obs, same deploy.yaml.
# Logs go to logs/rsl_rl/g1_23dof_velocity_hist5/<date>_yaw_ft/.
# Usage: ./run_hist5_yaw_finetune.sh [extra train.py args]
#        LOAD_RUN=<run dir> ./run_hist5_yaw_finetune.sh
set -euo pipefail

cd "$(dirname "$0")"

# The conda env has this repo pip-installed in editable mode, which maps `src` to the
# original checkout. Put this checkout first so a git worktree runs its own code.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOAD_RUN="${LOAD_RUN:-2026-10-28_04-53-15}"
if [ ! -f "logs/rsl_rl/g1_23dof_velocity_hist5/$LOAD_RUN/model_14999.pt" ]; then
  echo "Missing logs/rsl_rl/g1_23dof_velocity_hist5/$LOAD_RUN/model_14999.pt" >&2
  exit 1
fi

python scripts/train.py Unitree-G1-23Dof-Rough-Blind-Hist5-Yaw \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run="$LOAD_RUN" \
  --agent.load-checkpoint=model_14999.pt \
  --agent.max-iterations=3000 \
  --agent.run-name=yaw_ft \
  --agent.logger=tensorboard \
  "$@"
