#!/usr/bin/env bash
# Phase 3: resume Phase 2 (model_12999) with action-latency DR (0-40 ms per episode).
set -euo pipefail

cd /home/luke/nvidia/unitree/unitree_rl_mjlab
python scripts/train.py Unitree-G1-23Dof-Flat \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run=2026-10-21_11-41-28 \
  --agent.load-checkpoint=model_12999.pt \
  --agent.max-iterations=3000 \
  --agent.logger=tensorboard
