#!/usr/bin/env bash
set -euo pipefail

# Release entry point for the independent low-duty running-trot gait.  It
# delegates lifecycle, WBC/MPC, braking, logging, and wall-clock phase to the
# common sustained-sprint runner; only the gait reference is specialized here.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SUSTAINED_SPRINT_NAME="${SUSTAINED_SPRINT_NAME:-sustained_running_3mps_$(date +%Y%m%d_%H%M%S)}"
export SUSTAINED_SPRINT_DOMAIN_ID="${SUSTAINED_SPRINT_DOMAIN_ID:-190}"
export SUSTAINED_SPRINT_DURATION_S="${SUSTAINED_SPRINT_DURATION_S:-40}"
export SUSTAINED_SPRINT_WALL_TIMEOUT_S="${SUSTAINED_SPRINT_WALL_TIMEOUT_S:-75}"
export SUSTAINED_SPRINT_WALL_CLOCK_MOTION="${SUSTAINED_SPRINT_WALL_CLOCK_MOTION:-1}"
export SUSTAINED_SPRINT_GAIT_PATTERN="${SUSTAINED_SPRINT_GAIT_PATTERN:-running-trot}"
export SUSTAINED_SPRINT_PERIOD="${SUSTAINED_SPRINT_PERIOD:-0.14}"
export SUSTAINED_SPRINT_DUTY="${SUSTAINED_SPRINT_DUTY:-0.44}"
export SUSTAINED_SPRINT_STEP_LENGTH="${SUSTAINED_SPRINT_STEP_LENGTH:-0.50}"
export SUSTAINED_SPRINT_FOOT_LIFT="${SUSTAINED_SPRINT_FOOT_LIFT:-0.20}"

exec bash "$script_dir/run_sustained_sprint.sh" "$@"
