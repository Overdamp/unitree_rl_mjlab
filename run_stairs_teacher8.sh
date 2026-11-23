#!/usr/bin/env bash
# teacher8 (plan step 3h): clip-only teacher7 + an explicit stair geometry
# token (stair_token.py: rise, tread depth, yaw to the edge, edge distance)
# appended to actor and critic obs, after arXiv 2605.09944.
#
# 1. expand_checkpoint_obs.py widens the teacher7 checkpoint by 4 inputs with
#    zero weights (actor 267 -> 271, critic 282 -> 286) into <run>_t8init/;
#    checked: the widened policy's actions match teacher7 to 1.4e-6.
# 2. train.py resumes that into Unitree-G1-23Dof-OpenStairs-Teacher8 (teacher7
#    rewards, no foothold / stumble, clip 10).
#
# Defaults: newest *_stairs_teacher7 run (whole-name match) at model_27499.pt,
# 3000 more iterations, run stairs_teacher8. Override with
#   FROM='*_stairs_teacher7' CKPT=model_27499.pt ITERS=3000 RUN_NAME=stairs_teacher8 NOISE=0 ./run_stairs_teacher8.sh
# NOISE > 0 starts the new weights random instead of 0 (try 0.01 if the run
# ends up ignoring the token).
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher8.sh [extra train.py args, e.g. --env.scene.num-envs=2048]
set -euo pipefail

cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher7}"
CKPT="${CKPT:-model_27499.pt}"
ITERS="${ITERS:-3000}"
RUN_NAME="${RUN_NAME:-stairs_teacher8}"
NOISE="${NOISE:-0}"

SRC_RUN="$(ls -d "$LOGS"/$FROM 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)"
if [ -z "$SRC_RUN" ] || [ ! -f "$LOGS/$SRC_RUN/$CKPT" ]; then
  echo "No $CKPT in a run matching '$FROM' under $LOGS" >&2
  exit 1
fi
INIT_RUN="${SRC_RUN}_t8init"
if [ "$NOISE" != 0 ]; then INIT_RUN="${INIT_RUN}_noise${NOISE}"; fi

if [ ! -f "$LOGS/$INIT_RUN/$CKPT" ]; then
  python expand_checkpoint_obs.py --src "$LOGS/$SRC_RUN/$CKPT" --add 4 --out-run "$INIT_RUN" --noise "$NOISE"
fi
echo "Resuming $INIT_RUN/$CKPT for $ITERS iterations as $RUN_NAME"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher8 \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${INIT_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "$@"
