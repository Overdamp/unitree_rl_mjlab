#!/usr/bin/env bash
# teacher9b (plan step 3i): resume teacher8 into Unitree-G1-23Dof-OpenStairs-Teacher9b
# = teacher9 WITHOUT the foothold reward (teacher9 stalled in front of the
# first step: up 10 cm hold 97% -> 4%) and with demotion capped at 2 m (between
# the walls of a straight flight a robot commanded sideways was demoted;
# teacher9's curriculum sat at ~2.5). Kept: straight flights between walls,
# Chebyshev promotion, actor token noise + 10% dropout.
#
# Watch Curriculum/terrain_levels: it should climb past teacher9's 2.5 (the
# Chebyshev rule is stricter than teacher8's, so below 4.35 is expected).
# Then eval with wz = 0 (teacher8 up 10 cm: 23%) and with heading hold
# (teacher8: 97% / 92% at 10 / 12 cm).
#
# Defaults: newest *_stairs_teacher8 run at its newest model_*.pt, 3000 more
# iterations, run stairs_teacher9b. FROM / CKPT / ITERS / RUN_NAME override as
# in run_stairs_teacher9.sh.
# Usage: ./run_stairs_teacher9b.sh [extra train.py args]
set -euo pipefail
cd "$(dirname "$0")"
export TASK=Unitree-G1-23Dof-OpenStairs-Teacher9b
export RUN_NAME="${RUN_NAME:-stairs_teacher9b}"
exec ./run_stairs_teacher9.sh "$@"
