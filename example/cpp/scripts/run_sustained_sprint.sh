#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
wall_timeout_s="${SUSTAINED_SPRINT_WALL_TIMEOUT_S:-65}"
controller_duration_s="${SUSTAINED_SPRINT_DURATION_S:-30}"
domain_id="${SUSTAINED_SPRINT_DOMAIN_ID:-190}"
experiment_name="${SUSTAINED_SPRINT_NAME:-sustained_sprint_3mps_$(date +%Y%m%d_%H%M%S)}"
period="${SUSTAINED_SPRINT_PERIOD:-0.14}"
duty="${SUSTAINED_SPRINT_DUTY:-0.44}"
step_length="${SUSTAINED_SPRINT_STEP_LENGTH:-0.50}"
foot_lift="${SUSTAINED_SPRINT_FOOT_LIFT:-0.20}"
tau_limit="${SUSTAINED_SPRINT_TAU_LIMIT:-45}"
raibert_gain="${SUSTAINED_SPRINT_RAIBERT_GAIN:-0.010}"
raibert_adjustment="${SUSTAINED_SPRINT_RAIBERT_ADJUSTMENT:-0.06}"
gait_pattern="${SUSTAINED_SPRINT_GAIT_PATTERN:-diagonal-trot}"
preview_horizon="${SUSTAINED_SPRINT_PREVIEW_HORIZON:-4}"
wall_clock_motion="${SUSTAINED_SPRINT_WALL_CLOCK_MOTION:-1}"
view=false

for arg in "$@"; do
  case "$arg" in
    --view) view=true ;;
    --headless) view=false ;;
    --help|-h)
      echo "usage: $0 [--view|--headless]"
      echo "env: SUSTAINED_SPRINT_{NAME,DOMAIN_ID,DURATION_S,WALL_TIMEOUT_S,WALL_CLOCK_MOTION}"
      exit 0
      ;;
    *)
      echo "unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

# Reproducible 3 m/s-class aerial diagonal sprint reference.  The health
# governor is allowed to lower the reference when attitude/contact quality
# degrades; it never masks a hard safety stop.
export SUSTAINED_SPRINT_PERIOD="$period"
export SUSTAINED_SPRINT_DUTY="$duty"
export SUSTAINED_SPRINT_STEP_LENGTH="$step_length"
export SUSTAINED_SPRINT_FOOT_LIFT="$foot_lift"
export SUSTAINED_SPRINT_TAU_LIMIT="$tau_limit"
export SUSTAINED_SPRINT_RAIBERT_GAIN="$raibert_gain"
export SUSTAINED_SPRINT_RAIBERT_ADJUSTMENT="$raibert_adjustment"
export SUSTAINED_SPRINT_GAIT_PATTERN="$gait_pattern"
export SUSTAINED_SPRINT_SUPPORT_ANCHOR_GAIN="${SUSTAINED_SPRINT_SUPPORT_ANCHOR_GAIN:-0.35}"
export TROT_DYNAMICS_TOLERANCE_N="${TROT_DYNAMICS_TOLERANCE_N:-20}"
export TROT_EXPLORATORY_CONTINUE=1
export TROT_HS_STABILITY_GOV="${TROT_HS_STABILITY_GOV:-0}"
export TROT_HS_GOV_CRITICAL="${TROT_HS_GOV_CRITICAL:-0}"
export TROT_HS_TARGET_SPEED="${TROT_HS_TARGET_SPEED:-3.0}"
export TROT_HS_EXTRA_SETTLE_S="${TROT_HS_EXTRA_SETTLE_S:-0.8}"
export TROT_HS_START_PERIOD="${TROT_HS_START_PERIOD:-0.20}"
export TROT_HS_START_DUTY="${TROT_HS_START_DUTY:-0.50}"
export TROT_HS_RAMP_STEP="${TROT_HS_RAMP_STEP:-0.04}"
export TROT_HS_SPEED_LEAD="${TROT_HS_SPEED_LEAD:-0.25}"
export TROT_HS_ACC_GAIN="${TROT_HS_ACC_GAIN:-10}"
export TROT_HS_ACC_LIMIT="${TROT_HS_ACC_LIMIT:-4}"
export TROT_HS_STEP_CAP="${TROT_HS_STEP_CAP:-0.52}"
export TROT_HS_SWING_REACH="${TROT_HS_SWING_REACH:-0.90}"
export TROT_HS_HYBRID_CONTACT="${TROT_HS_HYBRID_CONTACT:-2}"
export TROT_HS_PITCH_GAIN="${TROT_HS_PITCH_GAIN:-24}"
export TROT_HS_PITCH_DAMP="${TROT_HS_PITCH_DAMP:-6}"
export TROT_HS_PITCH_ACC_LIMIT="${TROT_HS_PITCH_ACC_LIMIT:-8}"
export TROT_HS_ROLL_GAIN="${TROT_HS_ROLL_GAIN:-20}"
export TROT_HS_ROLL_DAMP="${TROT_HS_ROLL_DAMP:-10}"
export TROT_HS_ROLL_ACC_LIMIT="${TROT_HS_ROLL_ACC_LIMIT:-8}"
export TROT_HS_PRIMARY_RAMP_S="${TROT_HS_PRIMARY_RAMP_S:-0.05}"
export TROT_HS_GOV_ANGLE_RAD="${TROT_HS_GOV_ANGLE_RAD:-0.14}"
export TROT_HS_GOV_CONTACT="${TROT_HS_GOV_CONTACT:-0.20}"
export TROT_HS_GOV_DEGRADE_STEP="${TROT_HS_GOV_DEGRADE_STEP:-0.20}"
export TROT_HS_GOV_DRIFT="${TROT_HS_GOV_DRIFT:-0.10}"
export TROT_HS_GOV_FOOT_ERROR="${TROT_HS_GOV_FOOT_ERROR:-0.20}"
export TROT_HS_GOV_HOLD_CYCLES="${TROT_HS_GOV_HOLD_CYCLES:-4}"
export TROT_HS_GOV_EMERGENCY_HOLD_CYCLES="${TROT_HS_GOV_EMERGENCY_HOLD_CYCLES:-25}"
export TROT_HS_GOV_JOINT_ERROR="${TROT_HS_GOV_JOINT_ERROR:-0.95}"
# Keep the release reference at 3 m/s; a degradation is handled by the
# emergency brake thresholds instead of silently becoming a slower demo.
export TROT_HS_GOV_MIN_CAP="${TROT_HS_GOV_MIN_CAP:-2.4}"
# Release-profile floor; callers may lower it for controlled diagnostic runs.
export TROT_HS_GOV_MIN_SPEED="${TROT_HS_GOV_MIN_SPEED:-2.90}"
export TROT_HS_GOV_RECOVER_STEP="${TROT_HS_GOV_RECOVER_STEP:-0.02}"
export TROT_HS_PREFLIGHT_STABLE_S="${TROT_HS_PREFLIGHT_STABLE_S:-0.30}"
export TROT_HS_PREFLIGHT_ANGLE_RAD="${TROT_HS_PREFLIGHT_ANGLE_RAD:-0.08}"
export TROT_HS_PREFLIGHT_MIN_CONTACTS="${TROT_HS_PREFLIGHT_MIN_CONTACTS:-3}"
export TROT_HS_GOV_AUTO_BRAKE="${TROT_HS_GOV_AUTO_BRAKE:-1}"
export TROT_HS_GOV_AUTO_BRAKE_MIN_SPEED="${TROT_HS_GOV_AUTO_BRAKE_MIN_SPEED:-2.90}"
export TROT_HS_GOV_STOP_ANGLE_RAD="${TROT_HS_GOV_STOP_ANGLE_RAD:-0.18}"
export TROT_HS_GOV_GUARD_ANGLE_RAD="${TROT_HS_GOV_GUARD_ANGLE_RAD:-0.18}"
export TROT_HS_GOV_GUARD_GYRO_RADPS="${TROT_HS_GOV_GUARD_GYRO_RADPS:-3.50}"
export TROT_HS_GOV_MIN_GAIT_S="${TROT_HS_GOV_MIN_GAIT_S:-10.0}"
export TROT_HS_GOV_GUARD_MIN_GAIT_S="${TROT_HS_GOV_GUARD_MIN_GAIT_S:-10.0}"
export TROT_HS_GOV_GUARD_HOLD_TICKS="${TROT_HS_GOV_GUARD_HOLD_TICKS:-3}"
export TROT_HS_GOV_SUPPORT_GAP_FRACTION="${TROT_HS_GOV_SUPPORT_GAP_FRACTION:-0.18}"
export TROT_HS_GOV_SUPPORT_GAP_SAMPLES="${TROT_HS_GOV_SUPPORT_GAP_SAMPLES:-45}"
export TROT_HS_GOV_SUPPORT_GAP_DRIFT="${TROT_HS_GOV_SUPPORT_GAP_DRIFT:-0.040}"
export TROT_HS_GOV_STOP_JOINT_ERROR="${TROT_HS_GOV_STOP_JOINT_ERROR:-1.40}"
export TROT_HS_GOV_STOP_FOOT_ERROR="${TROT_HS_GOV_STOP_FOOT_ERROR:-0.40}"
export TROT_HS_GOV_BRAKE_DURATION_S="${TROT_HS_GOV_BRAKE_DURATION_S:-2.0}"
export TROT_HS_GOV_HEALTH_BRAKE_DURATION_S="${TROT_HS_GOV_HEALTH_BRAKE_DURATION_S:-1.20}"
export TROT_HS_GOV_HEALTH_BRAKE_U="${TROT_HS_GOV_HEALTH_BRAKE_U:-0.0}"
export TROT_HS_BRAKE_VEL_KP="${TROT_HS_BRAKE_VEL_KP:-8.0}"
export TROT_HS_BRAKE_AX_LIMIT="${TROT_HS_BRAKE_AX_LIMIT:-5.0}"
export TROT_HS_BRAKE_AY_LIMIT="${TROT_HS_BRAKE_AY_LIMIT:-4.0}"
export TROT_HS_BRAKE_STEP_SLEW="${TROT_HS_BRAKE_STEP_SLEW:-0.20}"
export TROT_HS_BRAKE_PERIOD_SLEW="${TROT_HS_BRAKE_PERIOD_SLEW:-0.10}"
export TROT_HS_BRAKE_DUTY_SLEW="${TROT_HS_BRAKE_DUTY_SLEW:-0.20}"
export TROT_HS_GUARD_PERIOD="${TROT_HS_GUARD_PERIOD:-0.24}"
export TROT_HS_GUARD_DUTY="${TROT_HS_GUARD_DUTY:-0.52}"
export TROT_HS_GUARD_START_SPEED="${TROT_HS_GUARD_START_SPEED:-3.0}"

sim_args=(
  --forever --wbc-full --gait-pattern "$gait_pattern"
  --tau-limit "$tau_limit" --period "$period" --duty "$duty" --step-length "$step_length"
  --foot-lift "$foot_lift" --kernel raibert-trot
  --raibert-velocity-gain "$raibert_gain" --raibert-max-adjustment "$raibert_adjustment"
  --preview-horizon "$preview_horizon"
  --support-anchor-feedback --support-anchor-gain "$SUSTAINED_SPRINT_SUPPORT_ANCHOR_GAIN"
  --controller-duration "$controller_duration_s"
  --domain-id "$domain_id"
)
if [[ "$view" == true ]]; then
  sim_args+=(--camera-follow)
else
  sim_args+=(--headless)
fi
if [[ "$wall_clock_motion" == "1" ]]; then
  sim_args+=(--wall-clock-motion)
fi

cd "$repo_dir"
exec bash example/cpp/scripts/run_trot.sh "$wall_timeout_s" "$experiment_name" "${sim_args[@]}"
