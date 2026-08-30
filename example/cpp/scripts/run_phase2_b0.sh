#!/usr/bin/env bash
set -euo pipefail

# A killed simulator can leave CycloneDDS shared-memory bookkeeping behind.
# B0 is serial, so it is safe to remove only known DDS/iceoryx objects at
# each harness boundary and on exit; never touch unrelated /dev/shm entries.
cleanup_cyclonedds_shm() {
  find /dev/shm -maxdepth 1 -mindepth 1     \( -name 'cdds*' -o -name 'cyclonedds*' -o -name 'iceoryx*' \)     -exec rm -rf -- {} + 2>/dev/null || true
}
cleanup_cyclonedds_shm
trap cleanup_cyclonedds_shm EXIT INT TERM

# B0 runner: development and holdout use the same frozen command contract;
# only the pre-declared domain/repeat membership differs.
script_dir="$(cd "$(dirname "$0")" && pwd)"
repo_root="$(cd "${script_dir}/../../.." && pwd)"
scenario="${1:-steps}"
set_name="${2:-development}"
repeat="${3:-0}"
case "${scenario}" in
  steps) profile=example/cpp/configs/phase1_velocity_steps.csv; duration=96 ;;
  accel_1_to_3) profile=example/cpp/configs/phase1_velocity_accel_1_to_3.csv; duration=40 ;;
  brake_3_to_0) profile=example/cpp/configs/phase1_velocity_brake_3_to_0.csv; duration=44 ;;
  ramp) profile=example/cpp/configs/phase1_velocity_ramp.csv; duration=62 ;;
  varying) profile=example/cpp/configs/phase1_velocity_varying.csv; duration=86 ;;
  *) echo "unknown scenario: ${scenario}" >&2; exit 2 ;;
esac
if [[ "${set_name}" == "holdout" ]]; then
  case "${repeat}" in 1) domain=200 ;; 2) domain=201 ;; 3) domain=202 ;; *) echo "holdout repeat must be 1..3" >&2; exit 2 ;; esac
else
  domain=190
fi
run_dir="_runs/phase2_b0_${set_name}_${scenario}_r${repeat}_$(date +%Y%m%d_%H%M%S)"
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
export GO2_PROFILE_PATH="${repo_root}/${profile}"
unset TROT_EXPLORATORY_CONTINUE

set +e
TROT_CPU_AUTOPIN=1 bash "${repo_root}/example/cpp/scripts/run_trot.sh" 140 "${run_dir}" \
  --headless --wall-clock-motion --controller-duration "${duration}" \
  --wbc-full --gait-pattern running-trot --kernel raibert-trot \
  --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 \
  --tau-limit 45 --raibert-velocity-gain 0.010 \
  --raibert-max-adjustment 0.06 --preview-horizon 4 \
  --support-anchor-feedback --support-anchor-gain 0.35 \
  --velocity-max-accel 0.80 --velocity-max-decel 1.20 \
  --velocity-max-jerk 4.0 --velocity-command-script "${repo_root}/${profile}" \
  --velocity-max-tracking-lead "${TROT_VELOCITY_MAX_TRACKING_LEAD:-0.20}" \
  --terrain-sensor-only --domain-id "${domain}"
runner_status=$?
set -e
run_path="${repo_root}/example/cpp/experiments/${run_dir}"
phase1_status=0
python3 "${repo_root}/example/cpp/scripts/analyze_phase1_velocity.py" \
  "${run_path}" --profile "${repo_root}/${profile}" \
  --json-out "${run_path}/phase1_quantitative.json" --require-quantitative || phase1_status=$?
baseline_arg=()
if [[ -n "${PHASE2_B0_BASELINE_DIR:-}" ]]; then
  baseline_arg=(--baseline "${PHASE2_B0_BASELINE_DIR}")
fi
b0_status=0
python3 "${repo_root}/example/cpp/tools/analyze_phase2_b0.py" \
  "${run_path}" "${baseline_arg[@]}" \
  --json-out "${run_path}/b0_analyzer.json" || b0_status=$?
if (( runner_status != 0 || phase1_status != 0 || b0_status != 0 )); then
  exit 1
fi
