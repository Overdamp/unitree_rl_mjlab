#!/usr/bin/env bash
# History-5 line: train Unitree-G1-23Dof-Rough-Blind-Hist5 from scratch (actor sees
# the last 5 frames = 100 ms). Rough terrain, action delay 0-20 ms and all DR are on
# from the start. Logs go to logs/rsl_rl/g1_23dof_velocity_hist5/.
# Usage: ./run_hist5_train.sh [extra train.py args, e.g. --env.scene.num-envs=4096]
set -euo pipefail

cd "$(dirname "$0")"

python scripts/train.py Unitree-G1-23Dof-Rough-Blind-Hist5 \
  --env.scene.num-envs=1024 \
  --agent.max-iterations=15000 \
  --agent.logger=tensorboard \
  "$@"
