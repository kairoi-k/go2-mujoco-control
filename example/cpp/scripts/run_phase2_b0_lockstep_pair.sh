#!/usr/bin/env bash
set -euo pipefail

# Order-103 lockstep fixed pair: identical frozen B0 config to the wall-clock
# fixed pair, but with SIM_LOCKSTEP=1 so the simulator advances exactly one
# frozen physics interval only after the ready barrier and the prior
# controller command/state exchange complete (timeout fail-closed). This is
# verification infrastructure only: controller, B0 contract, analyzers and
# thresholds are unchanged. WSL wall-clock robustness is not claimed by
# lockstep runs.
script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
set_name="${1:-development}"
repeat="${2:-0}"

read -r baseline_domain terrain_domain < <(
  python3 "$cpp_dir/tools/read_phase2_b0_domains.py" \
    fixed_3mps "$set_name" "$repeat"
)

export SIM_LOCKSTEP=1
# Exchange adds ~0.5 ms/interval; the wall-clock PASS pair used the same 75 s
# wall timeout, so the canary keeps the authoritative 75 s (run timing is
# 1:1 with the controller clock; see smoke evidence).
export SIM_LOCKSTEP_BARRIER_TIMEOUT_S="${SIM_LOCKSTEP_BARRIER_TIMEOUT_S:-120}"
export SIM_LOCKSTEP_EXCHANGE_TIMEOUT_S="${SIM_LOCKSTEP_EXCHANGE_TIMEOUT_S:-5}"
export SIM_LOCKSTEP_STEP_TIMEOUT_S="${SIM_LOCKSTEP_STEP_TIMEOUT_S:-5}"
export SUSTAINED_SPRINT_DURATION_S="${SUSTAINED_SPRINT_DURATION_S:-40}"
export SUSTAINED_SPRINT_WALL_TIMEOUT_S="${SUSTAINED_SPRINT_WALL_TIMEOUT_S:-75}"
export TROT_DYNAMICS_TOLERANCE_N="${TROT_DYNAMICS_TOLERANCE_N:-20}"
export TROT_CPU_AUTOPIN="${TROT_CPU_AUTOPIN:-1}"
export TROT_TERRAIN_SHADOW_DIAGNOSTICS="${TROT_TERRAIN_SHADOW_DIAGNOSTICS:-1}"
export GO2_PROFILE_PATH=""
# Optional DDS transport preload is host configuration, never a repository
# path. An unset or missing preload keeps the portable default.
dds_preload="${GO2_DDS_PRELOAD:-}"
if [[ -n "$dds_preload" && -f "$dds_preload" ]]; then
  export LD_PRELOAD="$dds_preload"
fi

stamp="$(date +%Y%m%d_%H%M%S)"
base_name="phase2_b0_lockstep_${set_name}_fixed_3mps_r${repeat}_${stamp}"
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

# Lockstep discipline check per run: trace present, no fail-closed marker,
# exact dt tick sequence, zero violations.
lockstep_check() {
  local dir="$1"
  [[ -s "$dir/lockstep_trace.csv" ]] || return 1
  if grep -q "SIM_LOCKSTEP_FAIL_CLOSED" "$dir/simulator.log"; then
    return 1
  fi
  python3 - "$dir/lockstep_trace.csv" <<'PY3' || return 1
import sys
path = sys.argv[1]
rows = []
for line in open(path):
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("sim_tick_ms"):
        continue
    cols = line.split(",")
    rows.append((int(cols[0]), int(cols[8])))
if not rows:
    sys.exit(1)
diffs = {rows[i + 1][0] - rows[i][0] for i in range(len(rows) - 1)}
if len(diffs) != 1 or next(iter(diffs)) <= 0:
    sys.exit(1)
if any(v != 0 for _, v in rows):
    sys.exit(1)
print("  lockstep_trace_ok rows=%d dt_ms=%d" % (len(rows), next(iter(diffs))))
PY3
}
lockstep_status=0
echo "lockstep_check baseline:"
lockstep_check "$baseline_path" || lockstep_status=1
echo "lockstep_check terrain:"
lockstep_check "$terrain_path" || lockstep_status=1

if (( baseline_status != 0 || terrain_status != 0 ||
      baseline_analyzer != 0 || terrain_analyzer != 0 || b0_status != 0 ||
      lockstep_status != 0 )); then
  exit 1
fi
