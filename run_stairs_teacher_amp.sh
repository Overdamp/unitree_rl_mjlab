#!/usr/bin/env bash
# Resume the clip-only stairs teacher7 into the Teacher-AMP task (plan step 3g):
# same env and rewards as teacher7 (no foothold / stumble) plus an AMP style
# reward from a discriminator trained on reference walking clips (amp_ppo.py).
# Actor and critic obs are unchanged, so the checkpoint loads as-is and the
# exported policy (ONNX / deploy) is the same shape as teacher7's.
#
# Defaults: newest *_stairs_teacher7 run (whole-name match, so *_teacher7_foothold
# is never picked) at model_27499.pt, 3000 more iterations, run stairs_teacher7_amp,
# reference clip walk1_straight.npz (LAFAN1 walk1 200.5-208.5 s, straight walk
# 0.72 m/s, 50 fps). The clip is not in git: copy it to the DGX first, e.g.
#   scp src/assets/motions/g1_23dof/walk1_straight.npz dgx:<repo>/src/assets/motions/g1_23dof/
# Override with
#   AMP_MOTION_FILES=a.npz,dir/ AMP_REWARD_WEIGHT=0.5 AMP_DISC_LR=1e-4 \
#   FROM='*_stairs_teacher7' CKPT=latest ITERS=3000 RUN_NAME=stairs_teacher7_amp ./run_stairs_teacher_amp.sh
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher_amp.sh [extra train.py args, e.g. --env.scene.num-envs=2048]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher7}"
ITERS="${ITERS:-3000}"
RUN_NAME="${RUN_NAME:-stairs_teacher7_amp}"
export AMP_MOTION_FILES="${AMP_MOTION_FILES:-src/assets/motions/g1_23dof/walk1_straight.npz}"
export AMP_REWARD_WEIGHT="${AMP_REWARD_WEIGHT:-0.5}"

IFS=',' read -ra _clips <<< "$AMP_MOTION_FILES"
for c in "${_clips[@]}"; do
  if [ ! -e "$c" ]; then
    echo "AMP clip '$c' not found (clips are not in git; copy it here first)" >&2
    exit 1
  fi
done

LOAD_RUN="${LOAD_RUN:-$(ls -d "$LOGS"/$FROM 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)}"
if [ -z "$LOAD_RUN" ] || [ ! -d "$LOGS/$LOAD_RUN" ]; then
  echo "No run matching '$FROM' under $LOGS (set FROM=... or LOAD_RUN=...)" >&2
  exit 1
fi
CKPT="${CKPT:-model_27499.pt}"
if [ "$CKPT" = latest ]; then
  CKPT="$(ls "$LOGS/$LOAD_RUN" | grep -E '^model_[0-9]+\.pt$' | sort -V | tail -n 1)"
fi
if [ ! -f "$LOGS/$LOAD_RUN/$CKPT" ]; then
  echo "Checkpoint $LOGS/$LOAD_RUN/$CKPT not found" >&2
  exit 1
fi
echo "Resuming $LOAD_RUN/$CKPT for $ITERS iterations as $RUN_NAME"
echo "AMP clips: $AMP_MOTION_FILES  (style weight $AMP_REWARD_WEIGHT)"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher-AMP \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${LOAD_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "$@"
