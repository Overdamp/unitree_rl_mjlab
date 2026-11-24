#!/usr/bin/env bash
# Clean control for teacher8 (plan step 3i): resume the clip-only teacher7 for
# the same 3000 iterations teacher8 got, in Unitree-G1-23Dof-OpenStairs-Teacher7
# (teacher7's env and rewards, plain PPO, no AMP, no token). teacher8 went from
# 81% (teacher7_amp) to 97% up 10 cm, but with the token replaced by the flat
# value it still made 94.5%; this run tells whether the gain came from the
# token (during training) or from the AMP run being a weak control.
#
# Defaults: newest *_stairs_teacher7 run (whole-name match) at model_27499.pt,
# 3000 more iterations, run stairs_teacher7_cont. Override with
#   FROM='*_stairs_teacher7' CKPT=model_27499.pt ITERS=3000 RUN_NAME=stairs_teacher7_cont ./run_stairs_teacher7_cont.sh
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher7_cont.sh [extra train.py args]
set -euo pipefail

cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher7}"
CKPT="${CKPT:-model_27499.pt}"
ITERS="${ITERS:-3000}"
RUN_NAME="${RUN_NAME:-stairs_teacher7_cont}"

LOAD_RUN="$(ls -d "$LOGS"/$FROM 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)"
if [ -z "$LOAD_RUN" ] || [ ! -f "$LOGS/$LOAD_RUN/$CKPT" ]; then
  echo "No $CKPT in a run matching '$FROM' under $LOGS" >&2
  exit 1
fi
echo "Resuming $LOAD_RUN/$CKPT for $ITERS iterations as $RUN_NAME"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher7 \
  --env.scene.num-envs=4096 \
  --agent.resume=True \
  --agent.load-run="^${LOAD_RUN}\$" \
  --agent.load-checkpoint="^${CKPT}\$" \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "$@"
