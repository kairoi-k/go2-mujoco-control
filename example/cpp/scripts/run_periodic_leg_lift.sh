#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "${script_dir}/.." && pwd)"
repo_dir="$(cd "${cpp_dir}/../.." && pwd)"
simulator="${repo_dir}/simulate/build/unitree_mujoco"
controller="${cpp_dir}/build/real_leg_lift_go2"
cycle_count="${1:-3}"
timeout_s="${2:-45}"
lift_leg="${4:-FR}"
experiment_name="${3:-go2_periodic_leg_lift_2026-08-02_${lift_leg}}"
body_shift_x="${5:-}"
body_shift_y="${6:-}"
foot_lift_height="${7:-0.040}"
# Named go2_* directories stay under experiments/; other output goes to experiments/_runs/.
if [[ "$experiment_name" == go2_* || "$experiment_name" == _runs/* ]]; then
  experiment_dir="$cpp_dir/experiments/$experiment_name"
else
  experiment_dir="$cpp_dir/experiments/_runs/$experiment_name"
fi

mkdir -p "${experiment_dir}"

case "${lift_leg}" in
  FR|FL|RR|RL) ;;
  *)
    echo "Unknown lift leg: ${lift_leg}. Use FR, FL, RR, or RL." >&2
    exit 2
    ;;
esac

if [[ -z "${body_shift_y}" ]]; then
  if [[ "${lift_leg}" == "FL" || "${lift_leg}" == "RL" ]]; then
    body_shift_y="-0.060"
  else
    body_shift_y="0.060"
  fi
fi

if [[ -z "${body_shift_x}" ]]; then
  if [[ "${lift_leg}" == "RR" || "${lift_leg}" == "RL" ]]; then
    body_shift_x="0.070"
  else
    body_shift_x="-0.070"
  fi
fi

sim_pid=""
stop_simulator() {
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    kill "${sim_pid}"
    wait "${sim_pid}" 2>/dev/null || true
  fi
  sim_pid=""
}
trap stop_simulator EXIT

env -u WAYLAND_DISPLAY \
DISPLAY="${DISPLAY:-:0}" \
XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}" \
PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}" \
  "${simulator}" -r go2 -s scene_leg_lift_demo.xml \
  >"${experiment_dir}/simulator.log" 2>&1 &
sim_pid=$!
sleep 1

printf '\n' | "${controller}" \
  lo "${timeout_s}" "${experiment_dir}/data.csv" \
  "${body_shift_x}" "${body_shift_y}" "${foot_lift_height}" \
  "${cycle_count}" "${lift_leg}" \
  >"${experiment_dir}/controller.log" 2>&1

stop_simulator
python3 "${cpp_dir}/tools/analysis/analyze_periodic_leg_lift.py" \
  "${experiment_dir}/data.csv" \
  --leg "${lift_leg}"

echo "Periodic lift experiment completed: ${experiment_dir}"
