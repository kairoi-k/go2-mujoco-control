#!/usr/bin/env bash
set -euo pipefail

# Run one frozen Phase1 runtime-command scenario and its strict analyzer.
script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "${script_dir}/../../.." && pwd)"
scenario="${1:-steps}"
run_root="${2:-_runs/phase1_velocity}"
domain_id="${3:-240}"
case "${run_root}" in
  example/cpp/experiments/*) run_root="${run_root#example/cpp/experiments/}" ;;
  experiments/*) run_root="${run_root#experiments/}" ;;
  /*) echo "run_root must be relative to example/cpp/experiments" >&2; exit 2 ;;
esac
mkdir -p "${repo_root}/example/cpp/experiments/${run_root}"

case "${scenario}" in
  steps) profile=example/cpp/configs/phase1_velocity_steps.csv; duration=96 ;;
  accel_1_to_3) profile=example/cpp/configs/phase1_velocity_accel_1_to_3.csv; duration=40 ;;
  brake_3_to_0) profile=example/cpp/configs/phase1_velocity_brake_3_to_0.csv; duration=44 ;;
  ramp) profile=example/cpp/configs/phase1_velocity_ramp.csv; duration=62 ;;
  varying) profile=example/cpp/configs/phase1_velocity_varying.csv; duration=86 ;;
  *) echo "unknown scenario: ${scenario}" >&2; exit 2 ;;
esac

run_dir="${run_root}/${scenario}_$(date +%Y%m%d_%H%M%S)"
export TROT_DYNAMICS_TOLERANCE_N="${TROT_DYNAMICS_TOLERANCE_N:-20}"
export TROT_HS_START_PERIOD="${TROT_HS_START_PERIOD:-0.20}"
export TROT_HS_START_DUTY="${TROT_HS_START_DUTY:-0.50}"
export TROT_HS_SPEED_LEAD="${TROT_HS_SPEED_LEAD:-0.25}"
export TROT_HS_ACC_GAIN="${TROT_HS_ACC_GAIN:-10}"
export TROT_HS_ACC_LIMIT="${TROT_HS_ACC_LIMIT:-4}"
export TROT_HS_STEP_CAP="${TROT_HS_STEP_CAP:-0.52}"
export TROT_HS_SWING_REACH="${TROT_HS_SWING_REACH:-0.90}"
export TROT_HS_HYBRID_CONTACT="${TROT_HS_HYBRID_CONTACT:-2}"
export TROT_HS_PITCH_GAIN="${TROT_HS_PITCH_GAIN:-24}"
export TROT_HS_PITCH_DAMP="${TROT_HS_PITCH_DAMP:-6}"
export TROT_HS_ROLL_GAIN="${TROT_HS_ROLL_GAIN:-20}"
export TROT_HS_ROLL_DAMP="${TROT_HS_ROLL_DAMP:-10}"
export TROT_HS_STABILITY_GOV="${TROT_HS_STABILITY_GOV:-1}"
unset TROT_EXPLORATORY_CONTINUE

(
  cd "${repo_root}"
  TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_trot.sh 140 "${run_dir}" \
    --headless --wall-clock-motion --controller-duration "${duration}" \
    --wbc-full --gait-pattern running-trot --kernel raibert-trot \
    --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 \
    --tau-limit 45 --raibert-velocity-gain 0.010 \
    --raibert-max-adjustment 0.06 --preview-horizon 4 \
    --support-anchor-feedback --support-anchor-gain 0.35 \
    --velocity-max-accel 0.80 --velocity-max-decel 1.20 \
    --velocity-max-jerk 4.0 --velocity-command-script "${profile}" \
    --velocity-max-tracking-lead "${TROT_VELOCITY_MAX_TRACKING_LEAD:-0.20}" \
    --domain-id "${domain_id}"
)
python3 "${repo_root}/example/cpp/scripts/analyze_phase1_velocity.py" \
  "${repo_root}/example/cpp/experiments/${run_dir}" \
  --profile "${repo_root}/${profile}"
