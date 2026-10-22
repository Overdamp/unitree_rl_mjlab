#!/usr/bin/env bash
set -euo pipefail

cd /home/luke/nvidia/unitree/unitree_rl_mjlab
python scripts/train.py Unitree-G1-23Dof-Flat \
  --env.scene.num-envs=1024 \
  --agent.resume=True \
  --agent.load-run=2026-10-20_18-26-38 \
  --agent.load-checkpoint=model_10000.pt \
  --agent.max-iterations=3000 \
  --agent.logger=tensorboard
