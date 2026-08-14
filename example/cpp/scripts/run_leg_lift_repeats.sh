#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "${script_dir}/.." && pwd)"
repo_dir="$(cd "${cpp_dir}/../.." && pwd)"
simulator="${repo_dir}/simulate/build/unitree_mujoco"
controller="${cpp_dir}/build/real_leg_lift_go2"
experiment_dir="${cpp_dir}/experiments/go2_contact_triggered_landing_repeatability_2026-07-16"
repeat_count="${1:-3}"

mkdir -p "${experiment_dir}"

sim_pid=""
stop_simulator() {
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    kill "${sim_pid}"
    wait "${sim_pid}" 2>/dev/null || true
  fi
  sim_pid=""
}
trap stop_simulator EXIT

for ((repeat = 1; repeat <= repeat_count; ++repeat)); do
  run_dir="${experiment_dir}/repeat_$(printf '%02d' "${repeat}")"
  mkdir -p "${run_dir}"
  if [[ -s "${run_dir}/data.csv" ]]; then
    echo "Skipping existing repeat ${repeat}"
    continue
  fi

  echo "Running lift repeat ${repeat}/${repeat_count}"
  DISPLAY="${DISPLAY:-:0}" \
  WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
  PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}" \
    "${simulator}" -r go2 -s scene_leg_lift_demo.xml >"${run_dir}/simulator.log" 2>&1 &
  sim_pid=$!
  sleep 1

  printf '\n' | "${controller}" \
    lo 16 "${run_dir}/data.csv" -0.070 0.060 0.040 \
    >"${run_dir}/controller.log" 2>&1

  stop_simulator
done

echo "Lift repetitions completed: ${experiment_dir}"
