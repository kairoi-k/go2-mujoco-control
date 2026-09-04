#!/usr/bin/env bash
set -euo pipefail

# B0's separate fixed 3 m/s regression pair. It reuses the accepted
# running-trot entry point and analyzer; terrain is sensor-only telemetry.
script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
set_name="${1:-development}"
repeat="${2:-0}"

read -r baseline_domain terrain_domain < <(
  python3 "$cpp_dir/tools/read_phase2_b0_domains.py" \
    fixed_3mps "$set_name" "$repeat"
)

export SUSTAINED_SPRINT_DURATION_S="${SUSTAINED_SPRINT_DURATION_S:-40}"
export SUSTAINED_SPRINT_WALL_TIMEOUT_S="${SUSTAINED_SPRINT_WALL_TIMEOUT_S:-75}"
export TROT_DYNAMICS_TOLERANCE_N="${TROT_DYNAMICS_TOLERANCE_N:-20}"
export GO2_PROFILE_PATH=""

stamp="$(date +%Y%m%d_%H%M%S)"
base_name="phase2_b0_${set_name}_fixed_3mps_r${repeat}_${stamp}"
baseline_name="_runs/${base_name}_baseline"
terrain_name="_runs/${base_name}_terrain"

set +e
SUSTAINED_SPRINT_NAME="$baseline_name" \
SUSTAINED_SPRINT_DOMAIN_ID="$baseline_domain" \
SUSTAINED_SPRINT_TERRAIN_SENSOR_ONLY=0 \
  bash "$script_dir/run_sustained_running.sh" --headless
baseline_status=$?
SUSTAINED_SPRINT_NAME="$terrain_name" \
SUSTAINED_SPRINT_DOMAIN_ID="$terrain_domain" \
SUSTAINED_SPRINT_TERRAIN_SENSOR_ONLY=1 \
  bash "$script_dir/run_sustained_running.sh" --headless
terrain_status=$?
set -e

baseline_path="$cpp_dir/experiments/$baseline_name"
terrain_path="$cpp_dir/experiments/$terrain_name"
baseline_analyzer=0
terrain_analyzer=0
b0_status=0
if [[ -s "$baseline_path/data.csv" && -s "$baseline_path/contact_ground_truth.csv" ]]; then
  python3 "$cpp_dir/tools/analysis/analyze_sustained_running.py" \
    "$baseline_path" >"$baseline_path/sustained_running_analysis.txt" \
    2>&1 || baseline_analyzer=$?
else
  baseline_analyzer=1
fi
if [[ -s "$terrain_path/data.csv" && -s "$terrain_path/contact_ground_truth.csv" ]]; then
  python3 "$cpp_dir/tools/analysis/analyze_sustained_running.py" \
    "$terrain_path" >"$terrain_path/sustained_running_analysis.txt" \
    2>&1 || terrain_analyzer=$?
else
  terrain_analyzer=1
fi
if [[ -s "$terrain_path/data.csv" && -s "$baseline_path/data.csv" ]]; then
  python3 "$cpp_dir/tools/analyze_phase2_b0.py" "$terrain_path" \
    --baseline "$baseline_path" --fixed-3mps \
    --fixed-analyzer-output "$terrain_path/sustained_running_analysis.txt" \
    --json-out "$terrain_path/b0_analyzer.json" || b0_status=$?
else
  b0_status=1
fi

if (( baseline_status != 0 || terrain_status != 0 ||
      baseline_analyzer != 0 || terrain_analyzer != 0 || b0_status != 0 )); then
  exit 1
fi
