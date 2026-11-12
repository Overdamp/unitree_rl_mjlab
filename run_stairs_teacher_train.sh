#!/usr/bin/env bash
# Stairs camera plan part A (doc/g1_23dof_stairs_camera_plan.md): train the
# TEACHER on open-riser stairs with the privileged height scan. From scratch:
# its actor obs (80 + 187 height rays) differ from P3b. The student (depth
# camera + lidar) is distilled from this run later.
# Watch Curriculum/terrain_levels: if it is not past ~2 by iteration 2000,
# stop and fix the rewards (the blind run stalled at 1.5).
# Measure afterwards with:
#   python eval_open_stairs.py --task Unitree-G1-23Dof-OpenStairs-Teacher --checkpoint <model.pt>
# Usage: ./run_stairs_teacher_train.sh [extra train.py args, e.g. --env.scene.num-envs=2048]
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

python scripts/train.py Unitree-G1-23Dof-OpenStairs-Teacher \
  --env.scene.num-envs=4096 \
  --agent.max-iterations=8000 \
  --agent.run-name=stairs_teacher \
  --agent.logger=tensorboard \
  "$@"
