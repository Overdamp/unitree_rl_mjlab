#!/usr/bin/env bash
# Phase 4: resume Phase 3b (model_15998) on blind rough terrain with stronger
# velocity tracking (task Unitree-G1-23Dof-Rough-Blind). Obs match Flat, so the
# result still deploys with the existing deploy.yaml.
# Usage: ./run_phase4_train.sh [extra train.py args, e.g. --env.scene.num-envs=4096]
set -euo pipefail

cd "$(dirname "$0")"

# The resume run must live under logs/. On machines without the original P3b
# run (e.g. the DGX), seed it from the checkpoint committed in the repo.
SEED_RUN=logs/rsl_rl/g1_23dof_velocity/phase3b_seed
if [ ! -f "$SEED_RUN/model_15998.pt" ]; then
  mkdir -p "$SEED_RUN"
  cp deploy/robots/g1_23dof/config/policy/velocity/v0/checkpoints/model_phase3b_15998.pt \
     "$SEED_RUN/model_15998.pt"
fi

python scripts/train.py Unitree-G1-23Dof-Rough-Blind \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run=phase3b_seed \
  --agent.load-checkpoint=model_15998.pt \
  --agent.max-iterations=4000 \
  --agent.logger=tensorboard \
  "$@"
