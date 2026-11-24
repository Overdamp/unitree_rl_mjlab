#!/usr/bin/env bash
# teacher9 (plan step 3i): resume teacher8 into Unitree-G1-23Dof-OpenStairs-Teacher9
# = teacher8 + foothold reward at -3 (no stumble term) + actor token noise and
# 10% dropout (training only) + half the stair patches as straight flights
# between walls and a Chebyshev-distance curriculum (no corner shortcut). Obs are the same as teacher8 (actor 271,
# critic 286), so the checkpoint loads as-is; no surgery.
#
# Goal: foot placement going up 10 cm from -9.4 cm / heel over the gap 44%
# toward +-5 cm / < 15%, without losing teacher8's 97% (10 cm) / 92% (12 cm).
# Watch Episode_Reward/foothold (should shrink toward 0) and progress /
# terrain_levels (if they fall, the robot is stalling at edges again: stop and
# retry with TEACHER9_FOOTHOLD_W=-2).
# Then eval with wz = 0 (no heading hold): the straight-climb test
# (teacher8 up 10 cm: 23%).
#
# Defaults: newest *_stairs_teacher8 run at its newest model_*.pt, 3000 more
# iterations, run stairs_teacher9. Override with
#   FROM='*_stairs_teacher8' CKPT=model_30498.pt ITERS=3000 RUN_NAME=stairs_teacher9 TEACHER9_FOOTHOLD_W=-3 ./run_stairs_teacher9.sh
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher9.sh [extra train.py args]
set -euo pipefail

cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher8}"
ITERS="${ITERS:-3000}"
RUN_NAME="${RUN_NAME:-stairs_teacher9}"
export TEACHER9_FOOTHOLD_W="${TEACHER9_FOOTHOLD_W:--3.0}"

LOAD_RUN="$(ls -d "$LOGS"/$FROM 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)"
if [ -z "$LOAD_RUN" ] || [ ! -d "$LOGS/$LOAD_RUN" ]; then
  echo "No run matching '$FROM' under $LOGS" >&2
  exit 1
fi
CKPT="${CKPT:-$(ls "$LOGS/$LOAD_RUN" | grep -E '^model_[0-9]+\.pt$' | sort -V | tail -n 1)}"
if [ ! -f "$LOGS/$LOAD_RUN/$CKPT" ]; then
  echo "Checkpoint $LOGS/$LOAD_RUN/$CKPT not found" >&2
  exit 1
fi
echo "Resuming $LOAD_RUN/$CKPT for $ITERS iterations as $RUN_NAME (foothold weight $TEACHER9_FOOTHOLD_W)"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher9 \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${LOAD_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "$@"
