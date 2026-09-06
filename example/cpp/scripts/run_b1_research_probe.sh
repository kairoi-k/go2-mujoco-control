#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
[[ $(git rev-parse HEAD) == ${1:?expected exact SHA} ]]
shift
[[ -z $(git status --porcelain) ]]
mode=${1:?terrain-b1-execution or terrain-sensor-only}
name=${2:?unique run name}
scene=${3:-phase2_step_5cm.xml}
duration=${4:-25}
[[ "$mode" == terrain-b1-execution || "$mode" == terrain-sensor-only ]]
[[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]]
[[ "$scene" == phase2_step_5cm.xml || "$scene" == phase2_flat.xml ]]
[[ ! -e example/cpp/experiments/_runs/$name ]]
export TROT_DYNAMICS_TOLERANCE_N=20
export TROT_HS_START_PERIOD=0.20 TROT_HS_START_DUTY=0.50 TROT_HS_SPEED_LEAD=0.25
export TROT_HS_ACC_GAIN=10 TROT_HS_ACC_LIMIT=4 TROT_HS_STEP_CAP=0.52 TROT_HS_SWING_REACH=0.90
export TROT_HS_HYBRID_CONTACT=2 TROT_HS_PITCH_GAIN=24 TROT_HS_PITCH_DAMP=6
export TROT_HS_ROLL_GAIN=20 TROT_HS_ROLL_DAMP=10 TROT_HS_STABILITY_GOV=1
export TROT_SEED=11 TROT_CPU_AUTOPIN=1
unset TROT_EXPLORATORY_CONTINUE
exec flock -n /tmp/go2_mujoco_experiment.lock bash example/cpp/scripts/run_trot.sh 65 _runs/$name \
 --headless --controller-duration "$duration" --phase2-milestone B1 \
 --scene-file "$PWD/unitree_robots/go2/$scene" \
 --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot \
 --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 --tau-limit 45 \
 --raibert-velocity-gain 0.010 --raibert-max-adjustment 0.06 --preview-horizon 4 \
 --support-anchor-feedback --support-anchor-gain 0.35 --velocity-max-accel 0.80 \
 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 \
 --velocity-command-script "$PWD/example/cpp/configs/phase1_velocity_steps.csv" \
 --velocity-max-tracking-lead 0.20 --domain-id 231 --gait-phase-offset 0 --initial-x 0 --initial-y 0 --$mode
