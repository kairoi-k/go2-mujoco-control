#!/usr/bin/env bash
set -euo pipefail

# Reproducible straight natural-trot baseline.  The optional arguments are
# forwarded so --view, --camera-follow and --domain-id can vary without
# changing the checked-in gait defaults.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export TROT_STRAIGHT_YAW_GAIN="${TROT_STRAIGHT_YAW_GAIN:-1.0}"
wall_timeout="${1:-100}"
experiment_name="${2:-natural_trot_$(date +%Y%m%d_%H%M%S)}"
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
  --period 0.34
  --duty 0.50
  --step-length 0.340
  --foot-lift 0.120
  --kernel raibert-trot
  --raibert-velocity-gain 0.015
  --raibert-max-adjustment 0.060
  --max-cycles 30
  --controller-duration 30
  --domain-id 231
)

args=("${defaults[@]}")
[[ -n "$view_flag" ]] && args+=("$view_flag")
args+=("${extra[@]}")
exec bash "$script_dir/run_trot.sh" "$wall_timeout" "$experiment_name" "${args[@]}"
