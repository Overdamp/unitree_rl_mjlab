#!/usr/bin/env bash
# Phase 3: resume Phase 2 (model_12999) with action-latency DR (0-20 ms per episode).
set -euo pipefail

cd "$(dirname "$0")"
# The conda env has the main checkout pip-installed (editable); without this,
# `import src` loads the MAIN folder's code even when run from a worktree.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
python scripts/train.py Unitree-G1-23Dof-Flat \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run=2026-10-21_11-41-28 \
  --agent.load-checkpoint=model_12999.pt \
  --agent.max-iterations=3000 \
  --agent.logger=tensorboard
