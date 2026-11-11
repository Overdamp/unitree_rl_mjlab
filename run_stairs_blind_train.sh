#!/usr/bin/env bash
# Stairs plan, blind fine-tune (doc/g1_23dof_stairs_plan.md, after step 2):
# resume Phase 3b (model_15998) on open-riser stairs with no vision, task
# Unitree-G1-23Dof-OpenStairs-Blind. Obs match Flat (actor 80), so the result
# still deploys with the existing deploy.yaml. It answers "how far does blind
# get" before the depth-camera work, and is the baseline the teacher must beat.
# Measure afterwards with: python eval_open_stairs.py --checkpoint <model.pt>
# Usage: ./run_stairs_blind_train.sh [extra train.py args, e.g. --env.scene.num-envs=4096]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

# The resume run must live under logs/. On machines without the original P3b
# run (e.g. the DGX), seed it from the checkpoint committed in the repo.
SEED_RUN=logs/rsl_rl/g1_23dof_velocity/phase3b_seed
if [ ! -f "$SEED_RUN/model_15998.pt" ]; then
  mkdir -p "$SEED_RUN"
  cp deploy/robots/g1_23dof/config/policy/velocity/v0/checkpoints/model_phase3b_15998.pt \
     "$SEED_RUN/model_15998.pt"
fi

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Blind \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run=phase3b_seed \
  --agent.load-checkpoint=model_15998.pt \
  --agent.max-iterations=4000 \
  --agent.run-name=stairs_blind \
  --agent.logger=tensorboard \
  "$@"
