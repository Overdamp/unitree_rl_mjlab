#!/usr/bin/env bash
# teacher-v2b (plan step 4): fine-tune a teacher-v2 checkpoint with two
# posture penalties that do not saturate (posture.py, env_cfgs_stairs_v2.py):
#   N4 upper-body deviation  -0.5 x sum (q - q_default)^2 over waist + arms
#   N5 pelvis height floor   -20 x max(0, 0.70 m - (pelvis z - mean foot z))
# teacher-v2 model_15000 climbs straight at wz = 0 (up 10 cm 84%) but walks
# twisted ~100 deg at the waist, arms raised, crouched to 0.64 m.
#
# Defaults: newest *_stairs_teacher_v2 run, model_15000.pt, 5000 more
# iterations (-> model_20000), terrain starts at level <= 3, run
# stairs_teacher_v2b. Overrides:
#   FROM='*_stairs_teacher_v2' CKPT=model_15000.pt ITERS=5000 \
#   V2B_INIT_LEVEL=3 RUN_NAME=stairs_teacher_v2b ./run_stairs_teacher_v2b.sh
# Resume v2b itself after a crash: FROM='*_stairs_teacher_v2b' CKPT=model_N.pt
# ITERS=<what is left>.
# From scratch instead (if fine-tuning loses the stairs): FROM=none ITERS=30000
# V2B_INIT_LEVEL=1.
#
# Check while it trains: Episode_Reward/upper_body_deviation should go from
# about -6 toward -0.1, pelvis_height toward 0, terrain_levels back above ~4.
# Then: ./eval_stairs_quick.sh Unitree-G1-23Dof-OpenStairs-TeacherV2b <ckpt>
# Stop the teacher-v2 run first: two runs halve the speed of both.
# Usage: ./run_stairs_teacher_v2b.sh [extra train.py args]
set -euo pipefail

cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LOGS=logs/rsl_rl/g1_23dof_velocity
FROM="${FROM:-*_stairs_teacher_v2}"
CKPT="${CKPT:-model_15000.pt}"
ITERS="${ITERS:-5000}"
RUN_NAME="${RUN_NAME:-stairs_teacher_v2b}"
TASK=Unitree-G1-23Dof-OpenStairs-TeacherV2b
export V2B_INIT_LEVEL="${V2B_INIT_LEVEL:-3}"

RESUME_ARGS=()
if [ "$FROM" = none ]; then
  echo "Training $TASK from scratch for $ITERS iterations as $RUN_NAME"
else
  # Newest run matching FROM that has CKPT (the newest folder may be a resume
  # that started after it).
  LOAD_RUN=""
  for d in $(ls -d "$LOGS"/$FROM 2>/dev/null | sort -r); do
    if [ -f "$d/$CKPT" ]; then LOAD_RUN="$(basename "$d")"; break; fi
  done
  if [ -z "$LOAD_RUN" ]; then
    echo "No run matching '$FROM' under $LOGS has $CKPT" >&2
    exit 1
  fi
  echo "Fine-tuning $LOAD_RUN/$CKPT as $TASK for $ITERS iterations as $RUN_NAME (init level <= $V2B_INIT_LEVEL)"
  RESUME_ARGS=(--agent.resume=True --agent.load-run="^${LOAD_RUN}\$" --agent.load-checkpoint="^${CKPT}\$")
fi

python scripts/train.py "$TASK" \
  --env.scene.num-envs=4096 \
  --agent.max-iterations="$ITERS" \
  --agent.run-name="$RUN_NAME" \
  --agent.logger=tensorboard \
  "${RESUME_ARGS[@]}" \
  "$@"
