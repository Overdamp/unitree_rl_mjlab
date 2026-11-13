#!/usr/bin/env bash
# Resume the stairs TEACHER from an earlier teacher run with the current config
# (env_cfgs_stairs.py). Obs are unchanged across these fixes, so checkpoints
# load as-is and keep what was learned (standing, balance, low stairs).
#
#   12/11 fixes (f395750): easy start rows, linear tracking x2, vx <= 1.0
#   13/11 fix: foot clearance measured from the ground under each foot
#
# Defaults: resume the newest *_stairs_teacher2 run (13/11 result) at its newest
# model_*.pt for 5000 more iterations as run stairs_teacher3. Override with
#   FROM='*_stairs_teacher' CKPT=model_2400.pt ITERS=5600 RUN_NAME=stairs_teacher2 ./run_stairs_teacher_resume.sh
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher_resume.sh [extra train.py args, e.g. --env.scene.num-envs=2048]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher2}"
ITERS="${ITERS:-5000}"
RUN_NAME="${RUN_NAME:-stairs_teacher3}"

# The glob matches whole names only, so *_stairs_teacher2 never picks *_stairs_teacher3.
LOAD_RUN="${LOAD_RUN:-$(ls -d "$LOGS"/$FROM 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)}"
if [ -z "$LOAD_RUN" ] || [ ! -d "$LOGS/$LOAD_RUN" ]; then
  echo "No run matching '$FROM' under $LOGS (set FROM=... or LOAD_RUN=...)" >&2
  exit 1
fi
# Sort bare file names by number (the full path has underscores of its own).
CKPT="${CKPT:-$(ls "$LOGS/$LOAD_RUN" | grep -E '^model_[0-9]+\.pt$' | sort -V | tail -n 1)}"
echo "Resuming $LOAD_RUN/$CKPT for $ITERS iterations as $RUN_NAME"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${LOAD_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "$@"
