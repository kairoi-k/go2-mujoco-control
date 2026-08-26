#!/usr/bin/env bash
set -eo pipefail

# Lock-protected Phase 2 determinism benchmark.
# Functional runs are single-threaded simulation-tick experiments.
# Realtime runs retain the DDS/MuJoCo wall-clock architecture and telemetry.
script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
runner="$cpp_dir/build-determinism/phase2_deterministic_functional_runner"
run_trot="$cpp_dir/scripts/run_trot.sh"
simulator="$repo_dir/simulate/build-determinism/unitree_mujoco"
controller="$cpp_dir/build-determinism/real_trot_go2"
analyzer="$cpp_dir/tools/analyze_phase2_determinism.py"
realtime_analyzer="$cpp_dir/tools/analyze_phase2_realtime.py"
lock_path="/tmp/go2_mujoco_experiment.lock"
expected_head="d77a16b9ed2c914de96899e599f254032148c873"

kind=all
repeats=5
run_name="phase2_determinism_20260826"
if [[ "$#" -ge 1 ]]; then kind="$1"; fi
if [[ "$#" -ge 2 ]]; then repeats="$2"; fi
if [[ "$#" -ge 3 ]]; then run_name="$3"; fi

if [[ "$kind" != all && "$kind" != functional && "$kind" != realtime ]]; then
  echo "kind must be all, functional, or realtime" >&2
  exit 2
fi
if ! [[ "$repeats" =~ ^[1-9][0-9]*$ ]]; then
  echo "repeats must be a positive integer" >&2
  exit 2
fi
if [[ "$repeats" -ne 5 ]]; then
  echo "this frozen benchmark requires exactly 5 repeats" >&2
  exit 2
fi
actual_head="$(git -C "$repo_dir" rev-parse HEAD)"
if [[ "$actual_head" != "$expected_head" ]]; then
  echo "unexpected HEAD: $actual_head (expected $expected_head)" >&2
  exit 2
fi
if [[ ! -x "$runner" || ! -x "$simulator" || ! -x "$controller" ]]; then
  echo "build artifacts are missing" >&2
  exit 2
fi
if [[ -e "$cpp_dir/experiments/_determinism_runs/$run_name" ]]; then
  echo "output already exists: $cpp_dir/experiments/_determinism_runs/$run_name" >&2
  exit 2
fi

root_output="$cpp_dir/experiments/_determinism_runs/$run_name"
mkdir -p "$root_output"
{
  printf "format=phase2-determinism-benchmark-v1\n"
  printf "git_head=%s\n" "$actual_head"
  printf "origin_main=%s\n" "$(git -C "$repo_dir" rev-parse origin/main)"
  printf "git_dirty=%s\n" "$(git -C "$repo_dir" status --porcelain | wc -l)"
  printf "lock=%s\n" "$lock_path"
  printf "kind=%s\n" "$kind"
  printf "repeats=%s\n" "$repeats"
  printf "seed=1\n"
  printf "runner=%s\n" "$runner"
  printf "simulator=%s\n" "$simulator"
  printf "controller=%s\n" "$controller"
  printf "analyzer=%s\n" "$analyzer"
  printf "started_at=%s\n" "$(date --iso-8601=seconds)"
} >"$root_output/benchmark_metadata.txt"

if [[ -z "$TROT_DYNAMICS_TOLERANCE_N" ]]; then
  export TROT_DYNAMICS_TOLERANCE_N=10
fi

run_functional() {
  profile="$1"
  duration="$2"
  mode="$3"
  repeat="$4"
  output="$root_output/functional/$profile/$mode/r$repeat"
  mkdir -p "$output"
  if flock "$lock_path" "$runner" \
      --profile "$repo_dir/example/cpp/configs/phase1_velocity_$profile.csv" \
      --scene "$repo_dir/unitree_robots/go2/scene_leg_lift_demo.xml" \
      --model "$repo_dir/unitree_robots/go2/go2.xml" \
      --mode "$mode" --duration "$duration" --stand-duration 3.5 \
      --output "$output" --seed 1 >"$output/runner.log" 2>&1; then
    status=0
  else
    status="$?"
  fi
  printf "status=%s\n" "$status" >"$output/status.txt"
}

run_realtime() {
  profile="$1"
  duration="$2"
  mode="$3"
  repeat="$4"
  profile_index="$5"
  output="$root_output/realtime/$profile/$mode/r$repeat"
  experiment_name="_determinism_runs/$run_name/realtime/$profile/$mode/r$repeat"
  mkdir -p "$output"
  domain="$((180 + (profile_index * repeats + repeat - 1) * 2))"
  terrain_arg=""
  if [[ "$mode" == terrain_sensor_only ]]; then
    terrain_arg="--terrain-sensor-only"
  fi
  if flock "$lock_path" env \
      TROT_LOCK_HELD=1 \
      TROT_SIMULATOR_PATH="$simulator" \
      TROT_CONTROLLER_PATH="$controller" \
      TROT_TRACE_TELEMETRY=1 \
      TROT_CPU_AUTOPIN=1 \
      TROT_SEED=1 \
      GO2_PROFILE_PATH="$repo_dir/example/cpp/configs/phase1_velocity_$profile.csv" \
      bash "$run_trot" "$((duration + 20))" "$experiment_name" \
      --headless --wall-clock-motion --controller-duration "$duration" \
      --wbc-full --gait-pattern running-trot --kernel raibert-trot \
      --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 \
      --tau-limit 45 --raibert-velocity-gain 0.010 \
      --raibert-max-adjustment 0.06 --preview-horizon 4 \
      --support-anchor-feedback --support-anchor-gain 0.35 \
      --velocity-max-accel 0.80 --velocity-max-decel 1.20 \
      --velocity-max-jerk 4.0 \
      --velocity-command-script "$repo_dir/example/cpp/configs/phase1_velocity_$profile.csv" \
      --velocity-max-tracking-lead 0.20 $terrain_arg --domain-id "$domain" \
      >"$output/run_trot.log" 2>&1; then
    status=0
  else
    status="$?"
  fi
  printf "status=%s\ndomain=%s\n" "$status" "$domain" >"$output/status.txt"
}

analyze_group() {
  family="$1"
  profile="$2"
  mode="$3"
  base="$root_output/$family/$profile/$mode"
  if [[ ! -s "$base/r1/data.csv" ]]; then
    printf "missing data for %s/%s/%s\n" "$family" "$profile" "$mode" >"$base/analysis_status.txt"
    return 0
  fi
  if flock "$lock_path" python3 "$analyzer" \
      --run "$base/r1" --run "$base/r2" --run "$base/r3" \
      --run "$base/r4" --run "$base/r5" \
      --output "$base/run_to_run_analysis" >"$base/analyzer.log" 2>&1; then
    printf "analysis_status=0\n" >"$base/analysis_status.txt"
  else
    status="$?"
    printf "analysis_status=%s\n" "$status" >"$base/analysis_status.txt"
  fi
  if [[ "$family" == realtime ]]; then
    if flock "$lock_path" python3 "$realtime_analyzer" \
        --run "$base/r1" --run "$base/r2" --run "$base/r3" \
        --run "$base/r4" --run "$base/r5" \
        --output "$base/realtime_performance" >"$base/performance.log" 2>&1; then
      printf "performance_status=0\n" >>"$base/analysis_status.txt"
    else
      status="$?"
      printf "performance_status=%s\n" "$status" >>"$base/analysis_status.txt"
    fi
  fi
}

run_family() {
  family="$1"
  if [[ "$family" == functional ]]; then
    for profile in accel_1_to_3 brake_3_to_0 varying; do
      case "$profile" in
        accel_1_to_3) duration=40; profile_index=0 ;;
        brake_3_to_0) duration=44; profile_index=1 ;;
        varying) duration=86; profile_index=2 ;;
      esac
      for mode in phase1 terrain_sensor_only; do
        for repeat in $(seq 1 "$repeats"); do
          run_functional "$profile" "$duration" "$mode" "$repeat"
        done
        if [[ "$repeats" -eq 5 ]]; then
          analyze_group functional "$profile" "$mode"
        fi
      done
    done
  else
    for profile in accel_1_to_3 brake_3_to_0 varying; do
      case "$profile" in
        accel_1_to_3) duration=40; profile_index=0 ;;
        brake_3_to_0) duration=44; profile_index=1 ;;
        varying) duration=86; profile_index=2 ;;
      esac
      for mode in phase1 terrain_sensor_only; do
        for repeat in $(seq 1 "$repeats"); do
          run_realtime "$profile" "$duration" "$mode" "$repeat" "$profile_index"
        done
        if [[ "$repeats" -eq 5 ]]; then
          analyze_group realtime "$profile" "$mode"
        fi
      done
    done
  fi
}

if [[ "$kind" == all || "$kind" == functional ]]; then
  run_family functional
fi
if [[ "$kind" == all || "$kind" == realtime ]]; then
  run_family realtime
fi
printf "finished_at=%s\n" "$(date --iso-8601=seconds)" >>"$root_output/benchmark_metadata.txt"
