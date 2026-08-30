#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
simulator="$repo_dir/simulate/build/unitree_mujoco"
controller="$cpp_dir/build/real_trot_go2"
scene_arg="scene_leg_lift_demo.xml"
scene_file="$repo_dir/unitree_robots/go2/scene_leg_lift_demo.xml"

timeout_s="$1"
experiment_name="$2"
shift 2
controller_args=("$@")

# The first script argument is the wall-clock timeout. Task experiments need
# a shorter locomotion duration than the full stand-walk-lie wall time, so
# expose an explicit controller-duration override and remove it before
# forwarding the remaining options to real_trot_go2.
controller_duration_s="$timeout_s"
sim_headless=false
sim_camera_follow=false
sim_terrain_lidar=false
sim_initial_args=()
sim_push_args=()
phase2_milestone=""
staged_start=false
sim_affinity="${TROT_CPU_AFFINITY_SIM:-}"
ctrl_affinity="${TROT_CPU_AFFINITY_CTRL:-}"
writer_affinity="${TROT_CPU_AFFINITY_WRITER:-}"
terrain_affinity="${TROT_CPU_AFFINITY_TERRAIN:-}"
sim_affinity_auto=false
sim_lidar_affinity="${TROT_SIM_LIDAR_CPU:-}"
sim_physics_affinity="${TROT_SIM_PHYSICS_CPU:-}"
sim_bridge_affinity="${TROT_SIM_BRIDGE_CPU:-}"
if [[ "${TROT_CPU_AUTOPIN:-1}" != "0" &&
      -z "$sim_affinity" && -z "$ctrl_affinity" ]]; then
  cpu_count="$(nproc 2>/dev/null || echo 0)"
  if [[ "$cpu_count" =~ ^[0-9]+$ ]] && (( cpu_count >= 5 )); then
    # MuJoCo and the DDS controller must not compete for the same WSL core.
    sim_affinity=2
    sim_affinity_auto=true
    ctrl_affinity=3,4
    writer_affinity="${writer_affinity:-3}"
    terrain_affinity="${terrain_affinity:-4}"
  elif [[ "$cpu_count" =~ ^[0-9]+$ ]] && (( cpu_count >= 4 )); then
    sim_affinity=2
    sim_affinity_auto=true
    ctrl_affinity=3
  fi
fi
if [[ -z "$writer_affinity" && "$ctrl_affinity" =~ ^[0-9]+ ]]; then
  writer_affinity="${ctrl_affinity%%,*}"
fi
if [[ -z "$terrain_affinity" && "$ctrl_affinity" == *,* ]]; then
  terrain_affinity="${ctrl_affinity##*,}"
fi
filtered_controller_args=()
profile_path="${GO2_PROFILE_PATH:-}"
for ((i = 0; i < ${#controller_args[@]}; ++i)); do
  arg="${controller_args[$i]}"
  if [[ "$arg" == "--terrain-sensor-only" || "$arg" == "--terrain-planner" ]]; then
    sim_terrain_lidar=true
  fi
  if [[ "$arg" == "--controller-duration" ]]; then
    if (( i + 1 >= ${#controller_args[@]} )); then
      echo "--controller-duration requires a value" >&2
      exit 2
    fi
    controller_duration_s="${controller_args[$((i + 1))]}"
    i=$((i + 1))
  elif [[ "$arg" == --controller-duration=* ]]; then
    controller_duration_s="${arg#*=}"
  elif [[ "$arg" == "--headless" ]]; then
    # simulator-only flag: strip from controller args, pass to simulator below
    sim_headless=true
  elif [[ "$arg" == "--camera-follow" ]]; then
    # simulator-only flag: track the robot body in the GUI camera
    sim_camera_follow=true
  elif [[ "$arg" == "--staged-start" ]]; then
    # Debug-only harness switch. Consume it here so the controller contract
    # and its argv remain unchanged.
    staged_start=true
  elif [[ "$arg" == "--initial-x" || "$arg" == "--initial-y" ]]; then
    if (( i + 1 >= ${#controller_args[@]} )); then
      echo "$arg requires a value" >&2
      exit 2
    fi
    # simulator-only harness variation: keep the controller unaware of the
    # initial pose used for development evidence.
    sim_initial_args+=("$arg" "${controller_args[$((i + 1))]}")
    i=$((i + 1))
  elif [[ "$arg" == "--phase2-milestone" ]]; then
    if (( i + 1 >= ${#controller_args[@]} )); then
      echo "--phase2-milestone requires B1, B2, or B3" >&2
      exit 2
    fi
    phase2_milestone="${controller_args[$((i + 1))]^^}"
    if [[ "$phase2_milestone" != "B1" &&
          "$phase2_milestone" != "B2" &&
          "$phase2_milestone" != "B3" ]]; then
      echo "--phase2-milestone requires B1, B2, or B3" >&2
      exit 2
    fi
    i=$((i + 1))
  elif [[ "$arg" == "--velocity-command-script" ]]; then
    if (( i + 1 >= ${#controller_args[@]} )); then
      echo "--velocity-command-script requires a path" >&2
      exit 2
    fi
    profile_path="${controller_args[$((i + 1))]}"
    if [[ "$profile_path" != /* ]]; then
      profile_path="$repo_dir/$profile_path"
    fi
    filtered_controller_args+=("$arg" "${controller_args[$((i + 1))]}")
    i=$((i + 1))
  elif [[ "$arg" == "--scene-file" ]]; then
    if (( i + 1 >= ${#controller_args[@]} )); then
      echo "--scene-file requires a value" >&2
      exit 2
    fi
    scene_arg="${controller_args[$((i + 1))]}"
    if [[ "$scene_arg" == /* ]]; then
      scene_file="$scene_arg"
    else
      scene_file="$repo_dir/$scene_arg"
      scene_arg="$scene_file"
    fi
    i=$((i + 1))
  elif [[ "$arg" == "--push-time" || "$arg" == "--push-force-x" || "$arg" == "--push-duration" || "$arg" == "--push-vel-x" || "$arg" == "--push-torque-pitch" || "$arg" == "--payload-kg" || "$arg" == "--friction-time" || "$arg" == "--friction-mu" || "$arg" == "--friction-duration" ]]; then
    # simulator-only disturbance flags: consume value
    sim_push_args+=("$arg" "${controller_args[$((i + 1))]}")
    i=$((i + 1))
  else
    filtered_controller_args+=("$arg")
  fi
done
if [[ "$sim_terrain_lidar" == true && "$sim_affinity_auto" == true ]]; then
  cpu_count="$(nproc 2>/dev/null || echo 0)"
  if [[ "$cpu_count" =~ ^[0-9]+$ ]] && (( cpu_count >= 6 )); then
    # Keep controller-side DDS callbacks on the terrain CPU. The accepted
    # 500 Hz writer is explicitly pinned to writer_affinity below; leaving
    # the process-wide mask at "3,4" lets an unpinned lidar callback preempt
    # that writer and perturb the inherited Phase 1 wall-clock contract.
    # Terrain DDS callbacks must not share the process-wide mask with the
    # accepted 500 Hz writer. Explicit controller affinity is handled by the
    # outer auto-pin guard; this path owns the terrain default.
    ctrl_affinity=4
    writer_affinity="${writer_affinity:-3}"
    terrain_affinity="${terrain_affinity:-4}"
    if [[ -z "${TROT_CPU_AFFINITY_TERRAIN:-}" ]] &&
       [[ "$cpu_count" =~ ^[0-9]+$ ]] && (( cpu_count >= 7 )); then
      # Keep the planner worker off CPU4, which carries controller-side DDS
      # callbacks. The accepted writer remains isolated on CPU3.
      terrain_affinity=6
    fi
    sim_affinity=2,5
    sim_lidar_affinity="${sim_lidar_affinity:-5}"
    sim_physics_affinity="${sim_physics_affinity:-2}"
    sim_bridge_affinity="${sim_bridge_affinity:-2}"
  fi
fi
export TROT_WRITER_CPU="$writer_affinity"
export TROT_TERRAIN_CPU="$terrain_affinity"
export TROT_SIM_LIDAR_CPU="$sim_lidar_affinity"
export TROT_SIM_PHYSICS_CPU="$sim_physics_affinity"
export TROT_SIM_BRIDGE_CPU="$sim_bridge_affinity"
controller_args=("${filtered_controller_args[@]}")

if [[ "$staged_start" == true ]]; then
  if [[ "$sim_terrain_lidar" != true ]]; then
    echo "--staged-start requires --terrain-sensor-only or --terrain-planner" >&2
    exit 2
  fi
  # The debug pose is derived from the measured 5 cm-step edge and the
  # observed edge-minus-base basin target. Allow an explicit override for
  # holdout scenes whose measured edge is intentionally different.
  staged_edge_minus_base="${TROT_STAGED_START_EDGE_MINUS_BASE_M:-0.324}"
  staged_start_x="${TROT_STAGED_START_BASE_X_M:-}"
  if [[ -z "$staged_start_x" ]]; then
    staged_start_x="$(python3 - "$scene_file" "$staged_edge_minus_base" <<'PY2'
import re
import sys

text = open(sys.argv[1], encoding="utf-8").read()
match = re.search(
    r"name=\"phase2_step_5cm\"\s+pos=\"([^\"]+)\"\s+type=\"box\"\s+size=\"([^\"]+)\"",
    text,
)
if match is None:
    raise SystemExit("unable to derive staged pose from phase2_step_5cm geometry")
pos = [float(value) for value in match.group(1).split()]
size = [float(value) for value in match.group(2).split()]
print(f"{pos[0] - size[0] - float(sys.argv[2]):.6f}")
PY2
)" || exit 2
  fi
  sim_initial_args+=(--initial-x "$staged_start_x" --staged-start)
  export TROT_TERRAIN_DEBUG_STAGED_START=1
  export TROT_STAGED_START_BASE_X_M="$staged_start_x"
  export TROT_STAGED_START_EDGE_MINUS_BASE_M="$staged_edge_minus_base"
fi

if (( ${#controller_args[@]} > 0 )) && [[ "${controller_args[0]}" != --* ]]; then
  echo "Unexpected positional controller argument '${controller_args[0]}'; use --controller-duration <s> for controller duration." >&2
  exit 2
fi
if ! awk -v value="$controller_duration_s" \
  'BEGIN { valid = (value ~ /^[0-9]+([.][0-9]*)?$/ || value ~ /^[.][0-9]+$/) && value + 0 > 0; exit !valid }'; then
  echo "Controller duration must be a positive number; got '$controller_duration_s'." >&2
  exit 2
fi

continuous_mode=false
for arg in "${controller_args[@]}"; do
  if [[ "$arg" == "--forever" ]]; then
    continuous_mode=true
  fi
done
task_mode=false
task_name=""
for ((i=0; i < ${#controller_args[@]}; ++i)); do
  if [[ "${controller_args[$i]}" == "--task" ]] &&
     (( i + 1 < ${#controller_args[@]} )); then
    task_mode=true
    task_name="${controller_args[$((i + 1))]}"
  fi
done
# Named go2_* directories stay under experiments/; other output goes to experiments/_runs/.
if [[ "$experiment_name" == go2_* || "$experiment_name" == _runs/* ]]; then
  experiment_dir="$cpp_dir/experiments/$experiment_name"
else
  experiment_dir="$cpp_dir/experiments/_runs/$experiment_name"
fi
ground_truth_file="$experiment_dir/contact_ground_truth.csv"
ground_truth_analysis_file="$experiment_dir/contact_ground_truth_analysis.txt"
ground_truth_dynamics_analysis_file="$experiment_dir/contact_ground_truth_dynamics_analysis.txt"
mkdir -p "$experiment_dir"
stop_file="$experiment_dir/stop.request"
rm -f "$ground_truth_file"
rm -f "$ground_truth_analysis_file"
rm -f "$ground_truth_dynamics_analysis_file"
rm -f "$stop_file"

max_cycles_requested=""
domain_id=1
for ((i=0; i < ${#controller_args[@]}; ++i)); do
  if [[ "${controller_args[$i]}" == "--max-cycles" ]] && (( i + 1 < ${#controller_args[@]} )); then
    max_cycles_requested="${controller_args[$((i + 1))]}"
  elif [[ "${controller_args[$i]}" == "--domain-id" ]] && (( i + 1 < ${#controller_args[@]} )); then
    domain_id="${controller_args[$((i + 1))]}"
  fi
done

if ! [[ "$domain_id" =~ ^[0-9]+$ ]] || (( domain_id < 0 || domain_id > 232 )); then
  echo "DDS domain must be an integer in [0, 232]; domain $domain_id is outside the CycloneDDS UDP port range." >&2
  exit 2
fi

display_value="${DISPLAY:-:0}"
runtime_dir="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
pulse_server="${PULSE_SERVER:-/mnt/wslg/PulseServer}"

if [[ ! -x "$simulator" || ! -x "$controller" || ! -f "$scene_file" ]]; then
  echo "Missing simulator, trot controller, or scene file." >&2
  exit 2
fi

task_torque_option=false
for arg in "${controller_args[@]}"; do
  if [[ "$arg" == "--wbc-task-torque-feedforward" ]]; then
    task_torque_option=true
    break
  fi
done
if [[ "$task_torque_option" == true ]]; then
  controller_help="$("$controller" 2>&1 || true)"
  if [[ "$controller_help" != *"--wbc-task-torque-feedforward"* ]]; then
    echo "Controller binary does not support --wbc-task-torque-feedforward; rebuild the binary used by run_trot.sh at $controller." >&2
    exit 2
  fi
fi

lock_file="/tmp/unitree_mujoco_run_trot_domain_${domain_id}.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
  echo "Another trot experiment is already running in DDS domain $domain_id; use a different --domain-id for parallel runs." >&2
  exit 2
fi
dynamics_tolerance_n="${TROT_DYNAMICS_TOLERANCE_N:-10}"

existing_sim_pids="$(pgrep -f -x "$simulator -i $domain_id -r go2 -s $scene_arg" || true)"
if [[ -n "$existing_sim_pids" ]]; then
  echo "Existing MuJoCo simulator detected in DDS domain $domain_id; use a different --domain-id for parallel runs." >&2
  exit 2
fi

metadata_file="$experiment_dir/run_metadata.txt"
environment_file="$experiment_dir/environment.txt"
env | LC_ALL=C sort | grep -E "^(TROT_|FULL2_|SUSTAINED_SPRINT_)" >"$environment_file" || true
{
  printf "started_at=%s\n" "$(date --iso-8601=seconds)"
  printf "git_head=%s\n" "$(git -C "$repo_dir" rev-parse HEAD)"
  printf "git_branch=%s\n" "$(git -C "$repo_dir" symbolic-ref --short -q HEAD || echo detached)"
  printf "git_dirty=%s\n" "$([[ -n "$(git -C "$repo_dir" status --porcelain)" ]] && echo true || echo false)"
  printf "simulator_sha256=%s\n" "$(sha256sum "$simulator" | cut -d" " -f1)"
  printf "controller_sha256=%s\n" "$(sha256sum "$controller" | cut -d" " -f1)"
  printf "scene_sha256=%s\n" "$(sha256sum "$scene_file" | cut -d" " -f1)"
  printf "scene_file=%s\n" "$scene_file"
  printf "phase2_milestone=%s\n" "$phase2_milestone"
  printf "display=%s\n" "$display_value"
  printf "runtime_dir=%s\n" "$runtime_dir"
  printf "headless=%s\n" "$([[ "$sim_headless" == true ]] && echo true || echo false)"
  printf "camera_follow=%s\n" "$([[ "$sim_camera_follow" == true ]] && echo true || echo false)"
  printf "terrain_lidar=%s\n" "$([[ "$sim_terrain_lidar" == true ]] && echo true || echo false)"
  printf "sim_cpu_affinity=%s\n" "${sim_affinity:-auto}"
  printf "controller_cpu_affinity=%s\n" "${ctrl_affinity:-auto}"
  printf "controller_writer_cpu_affinity=%s\n" "${writer_affinity:-auto}"
  printf "terrain_worker_cpu_affinity=%s\n" "${terrain_affinity:-auto}"
  printf "sim_lidar_cpu_affinity=%s\n" "${sim_lidar_affinity:-auto}"
  printf "sim_physics_cpu_affinity=%s\n" "${sim_physics_affinity:-auto}"
  printf "sim_bridge_cpu_affinity=%s\n" "${sim_bridge_affinity:-auto}"
  printf "argv=%s\n" "$*"
  printf "controller_argv_shell="
  printf "%q " "${controller_args[@]}"
  printf "\n"
  printf "profile_path=%s\n" "$profile_path"
  if [[ -n "$profile_path" && -f "$profile_path" ]]; then
    printf "profile_sha256=%s\n" "$(sha256sum "$profile_path" | cut -d" " -f1)"
  else
    printf "profile_sha256=\n"
  fi
  printf "seed=%s\n" "${TROT_SEED:-${RUN_SEED:-}}"
  printf "event_script_hash=%s\n" "${TROT_EVENT_SCRIPT_SHA256:-}"
  printf "controller_duration_s=%s\n" "$controller_duration_s"
  printf "max_cycles_requested=%s\n" "$max_cycles_requested"
  printf "domain_id=%s\n" "$domain_id"
  printf "run_mode=%s\n" "$([[ "$continuous_mode" == true ]] && echo continuous || echo bounded)"
  printf "task=%s\n" "$task_name"
  printf "wall_timeout_s=%s\n" "$timeout_s"
  printf "stop_file=%s\n" "$stop_file"
  printf "contact_ground_truth_file=%s\n" "$ground_truth_file"
  printf "contact_ground_truth_dynamics_analysis_file=%s\n" "$ground_truth_dynamics_analysis_file"
  printf "environment_file=%s\n" "$environment_file"
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
  local xdotool_lib=""
  local window_id=""
  local bundled_xdotool="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/bin/xdotool"

  if [[ -x "$bundled_xdotool" ]]; then
    xdotool_path="$bundled_xdotool"
    xdotool_lib="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/lib/x86_64-linux-gnu"
  else
    xdotool_path="$(command -v xdotool || true)"
  fi
  if [[ -z "$xdotool_path" ]]; then
    return 0
  fi

  for _ in $(seq 1 400); do
    window_id="$(DISPLAY="$display_value" LD_LIBRARY_PATH="$xdotool_lib" \
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

  DISPLAY="$display_value" LD_LIBRARY_PATH="$xdotool_lib" \
    "$xdotool_path" windowmap "$window_id" >/dev/null 2>&1 || true
  DISPLAY="$display_value" LD_LIBRARY_PATH="$xdotool_lib" \
    "$xdotool_path" windowmove "$window_id" 80 80 >/dev/null 2>&1 || true
  DISPLAY="$display_value" LD_LIBRARY_PATH="$xdotool_lib" \
    "$xdotool_path" windowraise "$window_id" >/dev/null 2>&1 || true
  DISPLAY="$display_value" LD_LIBRARY_PATH="$xdotool_lib" \
    "$xdotool_path" windowactivate --sync "$window_id" >/dev/null 2>&1 || true
  echo "MuJoCo window activated: $window_id"
}

env -u WAYLAND_DISPLAY \
DISPLAY="$display_value" \
XDG_RUNTIME_DIR="$runtime_dir" \
PULSE_SERVER="$pulse_server" \
  ${sim_affinity:+taskset -c "$sim_affinity"} "$simulator" -i "$domain_id" -r go2 -s "$scene_arg" \
  --ground-truth-log "$ground_truth_file" \
  $([[ "$sim_headless" == true ]] && printf %s --headless) \
  $([[ "$sim_camera_follow" == true ]] && printf %s --camera-follow) \
  $([[ "$sim_terrain_lidar" == true ]] && printf %s --terrain-lidar) \
  "${sim_initial_args[@]}" \
  "${sim_push_args[@]}" \
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

# Headless has no GLFW window. Waiting on xdotool here lets the robot sag
# under gravity for up to 20s before the controller starts, which trips the
# cycle-quality gate. Only focus a window when the viewer is actually up.
if [[ "$sim_headless" != true ]]; then
  activate_simulator_window
fi

controller_status=0
if [[ "$continuous_mode" == true ]]; then
  printf '\n' | ${ctrl_affinity:+taskset -c "$ctrl_affinity"} "$controller" \
    lo "$controller_duration_s" "$experiment_dir/data.csv" \
    "${controller_args[@]}" \
    --stop-file "$stop_file" \
    >"$experiment_dir/controller.log" 2>&1 &
  controller_pid=$!
  (
    sleep "$timeout_s"
    if kill -0 "$controller_pid" 2>/dev/null; then
      : >"$stop_file"
      sleep 5
      if kill -0 "$controller_pid" 2>/dev/null; then
        kill -TERM "$controller_pid" 2>/dev/null || true
        sleep 5
        kill -KILL "$controller_pid" 2>/dev/null || true
      fi
    fi
  ) &
  wall_timer_pid=$!
  wait "$controller_pid" || controller_status=$?
  kill "$wall_timer_pid" 2>/dev/null || true
  wait "$wall_timer_pid" 2>/dev/null || true
else
  printf '\n' | ${ctrl_affinity:+taskset -c "$ctrl_affinity"} "$controller" \
    lo "$controller_duration_s" "$experiment_dir/data.csv" \
    "${controller_args[@]}" \
    >"$experiment_dir/controller.log" 2>&1 || controller_status=$?
fi

if [[ "${TROT_RECORDING_GRACE_S:-0}" != "0" ]]; then
  sleep "${TROT_RECORDING_GRACE_S}"
fi

stop_simulator

safety_status=0
if grep -Eq "Trot hard safety limit reached|Trot hard posture limit|Trot hard joint limit|Trot IK failed|Requested motion" \
    "$experiment_dir/controller.log"; then
  echo "Trot controller reported a safety or motion rejection; see $experiment_dir/controller.log" >&2
  safety_status=1
fi
quality_status=0
if grep -Eq "Trot cycle quality guard rejected|Trot safety rejected" \
    "$experiment_dir/controller.log"; then
  echo "Trot controller reported a cycle-quality rejection; see $experiment_dir/controller.log" >&2
  quality_status=1
fi

analysis_status=0
if [[ ! -s "$experiment_dir/data.csv" ]]; then
  echo "Trot controller produced no data.csv; see $experiment_dir/controller.log" >&2
  analysis_status=1
else
  header_columns="$(awk -F, 'NR == 1 {print NF}' "$experiment_dir/data.csv")"
  sample_columns="$(awk -F, 'NR == 2 {print NF}' "$experiment_dir/data.csv")"
  if [[ -z "$header_columns" || "$header_columns" != "$sample_columns" ]]; then
    echo "CSV column mismatch: header=$header_columns sample=$sample_columns" >&2
    analysis_status=1
  fi
fi

ground_truth_status=0
if [[ ! -s "$ground_truth_file" ]]; then
  echo "MuJoCo simulator produced no contact_ground_truth.csv; see $experiment_dir/simulator.log" >&2
  ground_truth_status=1
else
  ground_truth_header_columns="$(awk -F, 'NR == 1 {print NF}' "$ground_truth_file")"
  ground_truth_sample_columns="$(awk -F, 'NR == 2 {print NF}' "$ground_truth_file")"
  if [[ -z "$ground_truth_header_columns" ||
        "$ground_truth_header_columns" != "$ground_truth_sample_columns" ]]; then
    echo "Ground-truth CSV column mismatch: header=$ground_truth_header_columns sample=$ground_truth_sample_columns" >&2
    ground_truth_status=1
  fi
fi
if (( ground_truth_status == 0 )); then
  if ! python3 "$cpp_dir/tools/analysis/analyze_contact_ground_truth.py" "$ground_truth_file" >"$ground_truth_analysis_file" 2>&1; then
    echo "Ground-truth contact-force analysis failed; see $ground_truth_analysis_file" >&2
    ground_truth_status=1
  fi
fi
dynamics_status=0
if (( ground_truth_status == 0 )); then
  if ! python3 "$cpp_dir/tools/analysis/analyze_contact_dynamics.py" "$ground_truth_file" --balance-tolerance-n "$dynamics_tolerance_n" >"$ground_truth_dynamics_analysis_file" 2>&1; then
    echo "Ground-truth dynamics analysis failed; see $ground_truth_dynamics_analysis_file" >&2
    dynamics_status=1
  fi
else
  dynamics_status=1
fi
completion_status=0
if [[ "$task_mode" == true ]]; then
  if ! grep -q "Task completed: stand-walk-lie" "$experiment_dir/controller.log"; then
    echo "Task did not complete stand-walk-lie; see $experiment_dir/controller.log" >&2
    completion_status=1
  fi
elif [[ -n "$max_cycles_requested" ]]; then
  cycle_health_count="$(grep -c "Trot cycle .* health:" "$experiment_dir/controller.log" || true)"
  if (( cycle_health_count < max_cycles_requested )) ||
     ! grep -q "Trot stopping; returning to stand" "$experiment_dir/controller.log"; then
    echo "Trot experiment did not complete the requested cycles; see $experiment_dir/controller.log" >&2
    completion_status=1
  fi
elif grep -q "Emergency stop hold complete; ending in WBC stance" \
    "$experiment_dir/controller.log"; then
  :
elif grep -q "High-speed stop: WBC four-contact hold complete; finished in WBC stance" \
    "$experiment_dir/controller.log"; then
  :
elif ! grep -q "Trot stopping; returning to stand" "$experiment_dir/controller.log"; then
  echo "Trot experiment did not reach a controlled stop; see $experiment_dir/controller.log" >&2
  completion_status=1
fi

{
  printf "controller_status=%s\n" "$controller_status"
  printf "safety_status=%s\n" "$safety_status"
  printf "quality_status=%s\n" "$quality_status"
  printf "analysis_status=%s\n" "$analysis_status"
  printf "ground_truth_status=%s\n" "$ground_truth_status"
  printf "contact_ground_truth_analysis_file=%s\n" "$ground_truth_analysis_file"
  printf "dynamics_status=%s\n" "$dynamics_status"
  printf "contact_ground_truth_dynamics_analysis_file=%s\n" "$ground_truth_dynamics_analysis_file"
  printf "dynamics_tolerance_n=%s\n" "$dynamics_tolerance_n"
  printf "completion_status=%s\n" "$completion_status"
} >>"$metadata_file"

phase1_quantitative_status=0
terrain_analysis_status=0
if [[ -n "$phase2_milestone" ]]; then
  if [[ -z "$profile_path" || ! -f "$profile_path" ]]; then
    echo "A Phase 2 milestone run requires --velocity-command-script." >&2
    phase1_quantitative_status=1
    terrain_analysis_status=1
  else
    if ! python3 "$cpp_dir/scripts/analyze_phase1_velocity.py" \
        "$experiment_dir" --profile "$profile_path" \
        --json-out "$experiment_dir/phase1_quantitative.json" \
        --require-quantitative >"$experiment_dir/phase1_quantitative.log" 2>&1; then
      echo "Phase 1 quantitative analysis failed; see $experiment_dir/phase1_quantitative.log" >&2
      phase1_quantitative_status=1
    fi
    if ! python3 "$cpp_dir/tools/analyze_phase2_terrain.py" \
        "$experiment_dir" --milestone "$phase2_milestone" \
        --scene "$scene_file" \
        --json-out "$experiment_dir/phase2_terrain_analysis.json" \
        >"$experiment_dir/phase2_terrain_analysis.log" 2>&1; then
      echo "Phase 2 terrain analysis failed; see $experiment_dir/phase2_terrain_analysis.log" >&2
      terrain_analysis_status=1
    fi
  fi
fi

{
  printf "phase1_quantitative_status=%s\n" "$phase1_quantitative_status"
  printf "terrain_analysis_status=%s\n" "$terrain_analysis_status"
  printf "finished_at=%s\n" "$(date --iso-8601=seconds)"
} >>"$metadata_file"

manifest_status=0
if ! python3 "$cpp_dir/tools/write_run_manifest.py" "$experiment_dir" \
    --repo "$repo_dir" --cpp-dir "$cpp_dir"; then
  echo "Run manifest generation failed; see $experiment_dir/run_metadata.txt" >&2
  manifest_status=1
fi

if (( controller_status != 0 || safety_status != 0 || quality_status != 0 ||
      analysis_status != 0 || ground_truth_status != 0 || dynamics_status != 0 ||
      completion_status != 0 || phase1_quantitative_status != 0 ||
      terrain_analysis_status != 0 || manifest_status != 0 )); then
  exit 1
fi
