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

# Reproducible B0 pair runner. The no-terrain member and sensor-only member
# use identical non-terrain arguments and a common flat scene. The baseline
# domain is separate only to avoid DDS collision; both runs are sequential.
script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
scenario="${1:-steps}"
set_name="${2:-development}"
repeat="${3:-0}"

case "$scenario" in
  steps) profile="example/cpp/configs/phase1_velocity_steps.csv"; duration=96 ;;
  accel_1_to_3) profile="example/cpp/configs/phase1_velocity_accel_1_to_3.csv"; duration=40 ;;
  brake_3_to_0) profile="example/cpp/configs/phase1_velocity_brake_3_to_0.csv"; duration=44 ;;
  ramp) profile="example/cpp/configs/phase1_velocity_ramp.csv"; duration=62 ;;
  varying) profile="example/cpp/configs/phase1_velocity_varying.csv"; duration=86 ;;
  *) echo "unknown scenario: $scenario" >&2; exit 2 ;;
esac

read -r baseline_domain terrain_domain < <(
  python3 "$cpp_dir/tools/read_phase2_b0_domains.py" \
    profiles "$set_name" "$repeat"
)

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
export GO2_PROFILE_PATH="$repo_dir/$profile"
unset TROT_EXPLORATORY_CONTINUE

stamp="$(date +%Y%m%d_%H%M%S)"
base_name="phase2_b0_${set_name}_${scenario}_r${repeat}_${stamp}"
baseline_name="_runs/${base_name}_baseline"
terrain_name="_runs/${base_name}_terrain"
common_args=(
  --headless --wall-clock-motion --controller-duration "$duration"
  --wbc-full --gait-pattern running-trot --kernel raibert-trot
  --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20
  --tau-limit 45 --raibert-velocity-gain 0.010
  --raibert-max-adjustment 0.06 --preview-horizon 4
  --support-anchor-feedback --support-anchor-gain 0.35
  --velocity-max-accel 0.80 --velocity-max-decel 1.20
  --velocity-max-jerk 4.0
  --velocity-command-script "$repo_dir/$profile"
  --velocity-max-tracking-lead 0.20
)

set +e
TROT_CPU_AUTOPIN=1 bash "$cpp_dir/scripts/run_trot.sh" 140 "$baseline_name" \
  "${common_args[@]}" --domain-id "$baseline_domain"
baseline_status=$?
cleanup_cyclonedds_shm
TROT_CPU_AUTOPIN=1 bash "$cpp_dir/scripts/run_trot.sh" 140 "$terrain_name" \
  "${common_args[@]}" --terrain-sensor-only --domain-id "$terrain_domain"
terrain_status=$?
set -e

baseline_path="$cpp_dir/experiments/$baseline_name"
terrain_path="$cpp_dir/experiments/$terrain_name"
baseline_phase1=0
terrain_phase1=0
b0_status=0
if [[ -s "$baseline_path/data.csv" ]]; then
  python3 "$cpp_dir/scripts/analyze_phase1_velocity.py" "$baseline_path" \
    --profile "$repo_dir/$profile" \
    --json-out "$baseline_path/phase1_quantitative.json" \
    --require-quantitative || baseline_phase1=$?
else
  baseline_phase1=1
fi
if [[ -s "$terrain_path/data.csv" ]]; then
  python3 "$cpp_dir/scripts/analyze_phase1_velocity.py" "$terrain_path" \
    --profile "$repo_dir/$profile" \
    --json-out "$terrain_path/phase1_quantitative.json" \
    --require-quantitative || terrain_phase1=$?
else
  terrain_phase1=1
fi
if [[ -s "$terrain_path/data.csv" && -s "$baseline_path/data.csv" ]]; then
  python3 "$cpp_dir/tools/analyze_phase2_b0.py" "$terrain_path" \
    --baseline "$baseline_path" \
    --json-out "$terrain_path/b0_analyzer.json" || b0_status=$?
else
  b0_status=1
fi

if (( baseline_status != 0 || terrain_status != 0 ||
      terrain_phase1 != 0 || b0_status != 0 )); then
  exit 1
fi
