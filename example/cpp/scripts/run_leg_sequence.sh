#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
simulator="$repo_dir/simulate/build/unitree_mujoco"
controller="$cpp_dir/build/real_leg_lift_go2"
scene_file="$repo_dir/unitree_robots/go2/scene_leg_lift_demo.xml"
timeout_s="${1:-60}"
experiment_name="${2:-go2_leg_sequence_2026-08-02}"
sequence_file="${3:-$cpp_dir/configs/go2_four_step_fr_rl_fl_rr.txt}"
# Named go2_* directories stay under experiments/; other output goes to experiments/_runs/.
if [[ "$experiment_name" == go2_* || "$experiment_name" == _runs/* ]]; then
  experiment_dir="$cpp_dir/experiments/$experiment_name"
else
  experiment_dir="$cpp_dir/experiments/_runs/$experiment_name"
fi
controller_args=()
repeat_mode=false
max_steps_requested=""
if (( $# >= 4 )); then
  controller_args=("${@:4}")
  skip_next=false
  for option_index in "${!controller_args[@]}"; do
    if [[ "$skip_next" == true ]]; then
      skip_next=false
      continue
    fi
    option="${controller_args[$option_index]}"
    case "$option" in
      --world-feedback|--yaw-feedback|--adaptive-tempo) ;;
      --repeat-sequence|--infinite)
        repeat_mode=true ;;
      --tempo-scale)
        if (( option_index + 1 >= ${#controller_args[@]} )); then
          echo "--tempo-scale requires a value" >&2
          exit 2
        fi
        skip_next=true
        ;;
      --support-scale)
        if (( option_index + 1 >= ${#controller_args[@]} )); then
          echo "--support-scale requires a value" >&2
          exit 2
        fi
        skip_next=true
        ;;
      --max-steps)
        if (( option_index + 1 >= ${#controller_args[@]} )); then
          echo "--max-steps requires a positive integer" >&2
          exit 2
        fi
        if ! [[ "${controller_args[$((option_index + 1))]}" =~ ^[1-9][0-9]*$ ]]; then
          echo "--max-steps requires a positive integer" >&2
          exit 2
        fi
        max_steps_requested="${controller_args[$((option_index + 1))]}"
        skip_next=true
        ;;
      *)
        echo "Unknown controller option: $option" >&2
        exit 2
        ;;
    esac
  done
fi
display_value="${DISPLAY:-:0}"
runtime_dir="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
pulse_server="${PULSE_SERVER:-/mnt/wslg/PulseServer}"

mkdir -p "$experiment_dir"

if [[ ! -f "$sequence_file" ]]; then
  echo "Missing sequence file: $sequence_file" >&2
  exit 2
fi

if [[ ! -x "$simulator" || ! -x "$controller" || ! -f "$scene_file" ]]; then
  echo "Missing simulator, controller, or scene file." >&2
  exit 2
fi

lock_file="/tmp/unitree_mujoco_run_leg_sequence.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
  echo "Another leg-sequence experiment is already running; refusing to start." >&2
  exit 2
fi

existing_sim_pids="$(pgrep -f -x "$simulator -r go2 -s scene_leg_lift_demo.xml" || true)"
if [[ -n "$existing_sim_pids" ]]; then
  echo "Existing MuJoCo simulator detected (PID(s): $existing_sim_pids); refusing to start a second DDS participant." >&2
  exit 2
fi

metadata_file="$experiment_dir/run_metadata.txt"
{
  printf "started_at=%s\n" "$(date --iso-8601=seconds)"
  printf "git_head=%s\n" "$(git -C "$repo_dir" rev-parse HEAD)"
  printf "simulator_sha256=%s\n" "$(sha256sum "$simulator" | cut -d" " -f1)"
  printf "controller_sha256=%s\n" "$(sha256sum "$controller" | cut -d" " -f1)"
  printf "sequence_sha256=%s\n" "$(sha256sum "$sequence_file" | cut -d" " -f1)"
  printf "scene_sha256=%s\n" "$(sha256sum "$scene_file" | cut -d" " -f1)"
  printf "display=%s\n" "$display_value"
  printf "runtime_dir=%s\n" "$runtime_dir"
  printf "argv=%s\n" "$*"
  printf "repeat_mode=%s\n" "$repeat_mode"
  printf "max_steps_requested=%s\n" "$max_steps_requested"
} >"$metadata_file"

sim_pid=""
stop_simulator() {
  if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill "$sim_pid"
    wait "$sim_pid" 2>/dev/null || true
  fi
  sim_pid=""
}
trap stop_simulator EXIT

activate_simulator_window() {
  local xdotool_path
  local xdotool_lib
  local xdotool_ld
  local window_id=""
  local bundled_xdotool="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/bin/xdotool"

  if [[ -x "$bundled_xdotool" ]]; then
    xdotool_path="$bundled_xdotool"
    xdotool_lib="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/lib/x86_64-linux-gnu"
  else
    xdotool_path="$(command -v xdotool || true)"
    xdotool_lib=""
  fi
  if [[ -z "$xdotool_path" ]]; then
    return 0
  fi
  xdotool_ld="${xdotool_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

  for _ in $(seq 1 100); do
    window_id="$(LD_LIBRARY_PATH="$xdotool_ld" DISPLAY="$display_value" \
      "$xdotool_path" search --name "MuJoCo" 2>/dev/null | tail -n 1 || true)"
    if [[ -n "$window_id" ]]; then
      break
    fi
    sleep 0.05
  done
  if [[ -z "$window_id" ]]; then
    echo "MuJoCo window was not found; see $experiment_dir/simulator.log" >&2
    return 0
  fi

  LD_LIBRARY_PATH="$xdotool_ld" DISPLAY="$display_value" \
    "$xdotool_path" windowmap "$window_id" >/dev/null 2>&1 || true
  LD_LIBRARY_PATH="$xdotool_ld" DISPLAY="$display_value" \
    "$xdotool_path" windowmove "$window_id" 80 80 >/dev/null 2>&1 || true
  LD_LIBRARY_PATH="$xdotool_ld" DISPLAY="$display_value" \
    "$xdotool_path" windowraise "$window_id" >/dev/null 2>&1 || true
  LD_LIBRARY_PATH="$xdotool_ld" DISPLAY="$display_value" \
    "$xdotool_path" windowactivate --sync "$window_id" >/dev/null 2>&1 || true
  echo "MuJoCo window activated: $window_id"
}

env -u WAYLAND_DISPLAY \
DISPLAY="$display_value" \
XDG_RUNTIME_DIR="$runtime_dir" \
PULSE_SERVER="$pulse_server" \
  "$simulator" -r go2 -s scene_leg_lift_demo.xml \
  >"$experiment_dir/simulator.log" 2>&1 &
sim_pid=$!

sim_ready=false
for _ in $(seq 1 200); do
  if grep -q "Unitree DDS bridge ready" "$experiment_dir/simulator.log"; then
    sim_ready=true
    break
  fi
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    echo "MuJoCo simulator exited before DDS bridge became ready; see $experiment_dir/simulator.log" >&2
    exit 1
  fi
  sleep 0.05
done
if [[ "$sim_ready" != true ]]; then
  echo "MuJoCo simulator did not expose DDS ready marker within 10 seconds; see $experiment_dir/simulator.log" >&2
  exit 1
fi

activate_simulator_window

controller_status=0
printf '\n' | "$controller" \
  lo "$timeout_s" "$experiment_dir/data.csv" \
  "-0.070" "0.060" "0.040" \
  1 "FR" "0.000" "0.000" "0.000" "0.000" \
  --sequence-file "$sequence_file" \
  "${controller_args[@]}" \
  >"$experiment_dir/controller.log" 2>&1 || controller_status=$?

stop_simulator

safety_status=0
if grep -Eq "Safety guard rejected|IK failed|Requested motion exceeds|Joint limit at" "$experiment_dir/controller.log"; then
  echo "Controller reported a safety or motion-limit rejection; see $experiment_dir/controller.log" >&2
  safety_status=1
fi

analysis_status=0
if [[ -s "$experiment_dir/data.csv" ]]; then
  header_columns="$(awk -F, 'NR == 1 {print NF}' "$experiment_dir/data.csv")"
  sample_columns="$(awk -F, 'NR == 2 {print NF}' "$experiment_dir/data.csv")"
  if [[ -z "$header_columns" || "$header_columns" != "$sample_columns" ]]; then
    echo "CSV column mismatch: header=$header_columns sample=$sample_columns" >&2
    analysis_status=1
  else
    python3 "$cpp_dir/tools/analysis/analyze_leg_sequence.py" \
      "$experiment_dir/data.csv" \
      --output-dir "$experiment_dir" \
      --summary-name "sequence_summary.csv" \
      --plot-name "sequence_overview.png" \
      --title "Go2 leg sequence" || analysis_status=$?
  fi
else
  echo "Controller produced no data.csv; see $experiment_dir/controller.log" >&2
  analysis_status=1
fi

expected_steps="$(awk '/^Sequence steps:/{print $3; exit}' "$experiment_dir/controller.log" || true)"
completion_status=0
if [[ -n "$max_steps_requested" ]]; then
  if ! grep -q "Stopped after completing ${max_steps_requested} steps in repeat mode" "$experiment_dir/controller.log"; then
    echo "Repeat experiment did not stop after the requested ${max_steps_requested} steps; see $experiment_dir/controller.log" >&2
    completion_status=1
  fi
elif [[ "$repeat_mode" == true ]]; then
  if ! grep -Eq "Repeated sequence: completed [1-9][0-9]* steps" "$experiment_dir/controller.log"; then
    echo "Repeat experiment did not complete a full sequence; see $experiment_dir/controller.log" >&2
    completion_status=1
  fi
else
  if [[ -z "$expected_steps" ]] || ! grep -q "Completed ${expected_steps} steps at" "$experiment_dir/controller.log"; then
    echo "Experiment did not complete all ${expected_steps:-expected} steps; see $experiment_dir/controller.log" >&2
    completion_status=1
  fi
fi
analysis_partial_accepted=0
if [[ "$repeat_mode" == true ]] &&
   (( controller_status == 0 && safety_status == 0 && analysis_status != 0 )) &&
   grep -Eq "Timed out after completing|Stopped after completing" "$experiment_dir/controller.log"; then
  echo "Repeat run reached its requested duration or controlled stop; the final partial step is retained but not analyzed." >&2
  analysis_partial_accepted=1
fi


{
  printf "controller_status=%s\n" "$controller_status"
  printf "safety_status=%s\n" "$safety_status"
  printf "analysis_status=%s\n" "$analysis_status"
  printf "analysis_partial_accepted=%s\n" "$analysis_partial_accepted"
  printf "completion_status=%s\n" "$completion_status"
  printf "finished_at=%s\n" "$(date --iso-8601=seconds)"
} >>"$metadata_file"

if (( controller_status != 0 )); then
  echo "Controller exited with status $controller_status; experiment is not a success." >&2
  exit "$controller_status"
fi
if (( safety_status != 0 )); then
  exit 1
fi
if (( completion_status != 0 )); then
  exit 1
fi
if (( analysis_status != 0 && analysis_partial_accepted == 0 )); then
  echo "Experiment analysis failed; experiment is not a success." >&2
  exit "$analysis_status"
fi

echo "Leg-sequence experiment completed: $experiment_dir"
