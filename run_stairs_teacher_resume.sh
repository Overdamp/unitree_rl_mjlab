#!/usr/bin/env bash
# Resume the stairs TEACHER from the first run's latest checkpoint, with the
# config fixes of 12/11 (env_cfgs_stairs.py: start on the easy rows, linear
# tracking x2, commands capped at vx 1.0). Obs are unchanged, so the checkpoint
# loads as-is and keeps what the first run learned (standing, balance).
#
# Picks the newest run named *_stairs_teacher and its newest model_*.pt.
# Override with:  LOAD_RUN=2026-11-12_xx-xx-xx_stairs_teacher CKPT=model_2400.pt ./run_stairs_teacher_resume.sh
# Stop the first run (Ctrl+C in its tmux pane) BEFORE starting this: two runs
# on one GPU halve the speed of both.
# Usage: ./run_stairs_teacher_resume.sh [extra train.py args, e.g. --env.scene.num-envs=2048]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
# Only first-run dirs (*_stairs_teacher), never an earlier resume (*_stairs_teacher2).
LOAD_RUN="${LOAD_RUN:-$(ls -d "$LOGS"/*_stairs_teacher 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)}"
if [ -z "$LOAD_RUN" ] || [ ! -d "$LOGS/$LOAD_RUN" ]; then
  echo "No teacher run found under $LOGS (set LOAD_RUN=...)" >&2
  exit 1
fi
CKPT="${CKPT:-$(ls "$LOGS/$LOAD_RUN"/model_*.pt | sort -t_ -k2 -n | tail -n 1 | xargs -n1 basename)}"
START="${CKPT//[!0-9]/}"
# Finish at iteration 8000 overall, like the first run planned.
ITERS=$(( 8000 - START ))
echo "Resuming $LOAD_RUN/$CKPT for $ITERS iterations"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${LOAD_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name=stairs_teacher2 \
  --agent.logger=tensorboard \
  "$@"
