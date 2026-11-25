#!/usr/bin/env bash
# Quick stairs check of one checkpoint (~5 min): up 8 / 10 / 12 cm, 64 envs,
# with wz = 0 (the straight-climb test, plan step 4 target >= 80% at 10 cm)
# and with heading hold. For the full sweep use eval_open_stairs.py --out.
# Usage: ./eval_stairs_quick.sh <task> <checkpoint.pt>
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
TASK="$1"; CK="$2"
for mode in wz0 hold; do
  extra=(); [ "$mode" = hold ] && extra=(--heading-hold)
  for r in 0.08 0.10 0.12; do
    out="$(python eval_open_stairs.py --task "$TASK" --checkpoint "$CK" --rise "$r" --direction up "${extra[@]}" 2>/dev/null | grep '^RESULT' | sed 's/^RESULT //')"
    python -c "import json,sys; d=json.loads(sys.argv[1]); print(f\"$mode up {d['rise_m']*100:.0f} cm  success {d['success_pct']:5.1f}%  fell {d['fell_pct']:4.1f}%  steps median {d['steps_median']:.0f}/{d['steps_total']}\")" "$out"
  done
done
