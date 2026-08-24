#!/usr/bin/env bash
set -euo pipefail
# P1: --auto-environment --sensor-map observe-only on flat / 10cm barrier / 4-step stairs.
repo_dir="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$repo_dir"
date_tag="$(date +%Y-%m-%d)"
common=(
  --headless --wall-clock-motion --kernel raibert-trot --wbc-full
  --auto-environment --sensor-map --task stand-walk-lie
  --controller-duration 30 --period .30 --duty .50
  --step-length .15 --foot-lift .08 --direction 1
)
scenes=(
  "flat|unitree_robots/go2/scene_leg_lift_demo.xml"
  "b15|unitree_robots/go2/scene_barrier_acceptance.xml"
  "stair|unitree_robots/go2/scene_stair_acceptance.xml"
)
for spec in "${scenes[@]}"; do
  name="${spec%%|*}"
  scene="${spec##*|}"
  for n in 1 2 3; do
    run="p1_${name}_n${n}_h2_${date_tag}"
    echo "===== RUN $run ====="
    bash example/cpp/scripts/run_trot.sh 90 "$run" "${common[@]}" --domain-id "$((212 + n))" --scene-file "$scene"
    python3 example/cpp/tools/analyze_terrain_observe.py \
      "example/cpp/experiments/_runs/$run/data.csv" \
      | tee "example/cpp/experiments/_runs/$run/observe_summary.txt"
  done
done
