#!/usr/bin/env bash
# Hist5 yaw + stand fine-tune: resume the yaw fine-tune (model_17998) with 15% of
# envs told to stand (was 5%), so the policy stops swaying at cmd = 0
# (task Unitree-G1-23Dof-Rough-Blind-Hist5-YawStand). Same obs, same deploy.yaml.
# Logs go to logs/rsl_rl/g1_23dof_velocity_hist5/<date>_yaw_stand_ft/.
# Usage: ./run_hist5_yaw_stand_finetune.sh [extra train.py args]
#        LOAD_RUN=<run dir> ./run_hist5_yaw_stand_finetune.sh
set -euo pipefail

cd "$(dirname "$0")"

# The conda env has this repo pip-installed in editable mode, which maps `src` to the
# original checkout. Put this checkout first so a git worktree runs its own code.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOAD_RUN="${LOAD_RUN:-2026-10-29_14-08-55_yaw_ft}"
if [ ! -f "logs/rsl_rl/g1_23dof_velocity_hist5/$LOAD_RUN/model_17998.pt" ]; then
  echo "Missing logs/rsl_rl/g1_23dof_velocity_hist5/$LOAD_RUN/model_17998.pt" >&2
  exit 1
fi

python scripts/train.py Unitree-G1-23Dof-Rough-Blind-Hist5-YawStand \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run="$LOAD_RUN" \
  --agent.load-checkpoint=model_17998.pt \
  --agent.max-iterations=1500 \
  --agent.run-name=yaw_stand_ft \
  --agent.logger=tensorboard \
  "$@"
