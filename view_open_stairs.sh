#!/usr/bin/env bash
# Look at the open-riser stairs with a blind policy walking on them (Phase 4,
# doc/g1_23dof_stairs_plan.md). Default policy: Phase 3b flat checkpoint
# committed in the repo (the step-2 baseline).
# Usage: ./view_open_stairs.sh [checkpoint.pt] [extra play.py args, e.g. --viewer viser]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

CKPT=deploy/robots/g1_23dof/config/policy/velocity/v0/checkpoints/model_phase3b_15998.pt
if [ $# -gt 0 ] && [ "${1#--}" = "$1" ]; then CKPT=$1; shift; fi

python scripts/play.py Unitree-G1-23Dof-OpenStairs-Blind \
  --checkpoint-file "$CKPT" \
  --num-envs 16 \
  "$@"
