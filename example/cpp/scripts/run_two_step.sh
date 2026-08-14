#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "${script_dir}/.." && pwd)"
repo_dir="$(cd "${cpp_dir}/../.." && pwd)"
simulator="${repo_dir}/simulate/build/unitree_mujoco"
controller="${cpp_dir}/build/real_leg_lift_go2"
timeout_s="${1:-45}"
experiment_name="${2:-go2_two_step_fr_fl_2026-08-02}"
sequence_file="${3:-${cpp_dir}/configs/go2_two_step_fr_fl.txt}"
# Named go2_* directories stay under experiments/; other output goes to experiments/_runs/.
if [[ "$experiment_name" == go2_* || "$experiment_name" == _runs/* ]]; then
  experiment_dir="$cpp_dir/experiments/$experiment_name"
else
  experiment_dir="$cpp_dir/experiments/_runs/$experiment_name"
fi

mkdir -p "${experiment_dir}"

if [[ ! -f "${sequence_file}" ]]; then
  echo "Missing sequence file: ${sequence_file}" >&2
  exit 2
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
  "-0.070" "0.060" "0.040" \
  1 "FR" "0.000" "0.000" "0.000" "0.000" \
  --sequence-file "${sequence_file}" \
  >"${experiment_dir}/controller.log" 2>&1

stop_simulator
python3 "${cpp_dir}/tools/analysis/analyze_leg_sequence.py" \
  "${experiment_dir}/data.csv" \
  --output-dir "${experiment_dir}"

echo "Two-step experiment completed: ${experiment_dir}"
