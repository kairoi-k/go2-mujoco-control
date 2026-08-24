#!/usr/bin/env bash
set -euo pipefail

# Reproducible low-duty high-speed running-trot profile.  It uses the same
# WBC/MPC plant as the natural trot; the lower duty factor creates an aerial
# phase while the straight-line MPC reference keeps the body aligned.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
export TROT_STRAIGHT_YAW_GAIN="${TROT_STRAIGHT_YAW_GAIN:-2.0}"
wall_timeout="${1:-100}"
experiment_name="${2:-running_trot_$(date +%Y%m%d_%H%M%S)}"
shift $(( $# >= 2 ? 2 : $# ))

view_flag="--headless"
extra=()
for arg in "$@"; do
  if [[ "$arg" == "--view" ]]; then
    view_flag=""
  else
    extra+=("$arg")
  fi
done

defaults=(
  --wbc-full
  --wbc-velocity-gain 8
  --tau-limit 35
  --period 0.26
  --duty 0.45
  --step-length 0.320
  --foot-lift 0.120
  --kernel raibert-trot
  --raibert-velocity-gain 0.015
  --raibert-max-adjustment 0.060
  --event-script "$cpp_dir/configs/running_trot_emergency_stop.txt"
  --controller-duration 20
  --domain-id 229
)

args=("${defaults[@]}")
[[ -n "$view_flag" ]] && args+=("$view_flag")
args+=("${extra[@]}")
exec bash "$script_dir/run_trot.sh" "$wall_timeout" "$experiment_name" "${args[@]}"
