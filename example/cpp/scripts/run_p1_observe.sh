#!/usr/bin/env bash
set -euo pipefail
# P1: --terrain-observe --sensor-map on flat / 10cm barrier / 4-step stairs.
repo_dir="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$repo_dir"
date_tag="$(date +%Y-%m-%d)"
common=(
  --headless --kernel crawl --wbc-full --terrain-observe --sensor-map
  --task stand-walk-lie --controller-duration 30 --period .80 --duty .75
  --step-length .13 --foot-lift .09 --kp 63 --kd 2.8 --direction 1
  --domain-id 212
)
scenes=(
  "flat|unitree_robots/go2/scene_leg_lift_demo.xml"
  "b10|unitree_robots/go2/scene_barrier_low10.xml"
  "stair|unitree_robots/go2/scene_stair_acceptance.xml"
)
for spec in "${scenes[@]}"; do
  name="${spec%%|*}"
  scene="${spec##*|}"
  for n in 1 2 3; do
    run="p1_${name}_n${n}_h2_${date_tag}"
    echo "===== RUN $run ====="
    bash example/cpp/scripts/run_trot.sh 90 "$run" "${common[@]}" --scene-file "$scene"
    python3 example/cpp/tools/analyze_terrain_observe.py \
      "example/cpp/experiments/_runs/$run/data.csv" \
      | tee "example/cpp/experiments/_runs/$run/observe_summary.txt"
  done
done
