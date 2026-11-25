#!/usr/bin/env bash
# teacher-v2 (plan step 4): the stairs teacher trained FROM SCRATCH with the
# rule set learned from teacher1-9b (env_cfgs_stairs_v2.py), mainly: walking
# is scored in a heading-locked world frame (heading_lock.py), so turning away
# from a step no longer pays (teacher8: progress 0.91 turned away vs 0.82
# facing the stairs; heading-locked: 0.08 vs 0.81).
#
# Defaults: 30000 iterations, 4096 envs, run stairs_teacher_v2, checkpoint
# every 100 iterations. ~2 days on the DGX.
#   ITERS=30000 RUN_NAME=stairs_teacher_v2 ./run_stairs_teacher_v2.sh
# Resume after a crash (newest *_<RUN_NAME> run, newest checkpoint):
#   RESUME=1 ./run_stairs_teacher_v2.sh
# Check a checkpoint while it trains (copy it here first):
#   ./eval_stairs_quick.sh Unitree-G1-23Dof-OpenStairs-TeacherV2 <run>/model_N.pt
# Stop any other training on the GPU first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher_v2.sh [extra train.py args]
set -euo pipefail

cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
ITERS="${ITERS:-30000}"
RUN_NAME="${RUN_NAME:-stairs_teacher_v2}"
TASK=Unitree-G1-23Dof-OpenStairs-TeacherV2

RESUME_ARGS=()
if [ "${RESUME:-0}" != 0 ]; then
  LOAD_RUN="$(ls -d "$LOGS"/*_"$RUN_NAME" 2>/dev/null | sort | tail -n 1 | xargs -r -n1 basename)"
  CKPT="$(ls "$LOGS/$LOAD_RUN" 2>/dev/null | grep -E '^model_[0-9]+\.pt$' | sort -V | tail -n 1)"
  if [ -z "$LOAD_RUN" ] || [ -z "$CKPT" ]; then
    echo "RESUME=1 but no checkpoint in a *_$RUN_NAME run under $LOGS" >&2
    exit 1
  fi
  DONE="${CKPT//[^0-9]/}"
  LEFT=$(( ITERS - DONE ))
  if [ "$LEFT" -le 0 ]; then echo "$LOAD_RUN/$CKPT already reached $ITERS iterations" >&2; exit 1; fi
  echo "Resuming $LOAD_RUN/$CKPT for $LEFT more iterations"
  RESUME_ARGS=(--agent.resume=True --agent.load-run="^${LOAD_RUN}\$" --agent.load-checkpoint="^${CKPT}\$")
  ITERS="$LEFT"
else
  echo "Training $TASK from scratch for $ITERS iterations as $RUN_NAME"
fi

python scripts/train.py "$TASK" \
  --env.scene.num-envs=4096 \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "${RESUME_ARGS[@]}" \
  "$@"
