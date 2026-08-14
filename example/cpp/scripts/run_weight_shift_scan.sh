#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "${script_dir}/.." && pwd)"
repo_dir="$(cd "${cpp_dir}/../.." && pwd)"
simulator="${repo_dir}/simulate/build/unitree_mujoco"
controller="${cpp_dir}/build/real_leg_lift_go2"
experiment_dir="${cpp_dir}/experiments/go2_weight_shift_scan_2026-07-16"

conditions=(
  "xm03_yp02 -0.03 0.02"
  "xm03_yp03 -0.03 0.03"
  "xm03_yp04 -0.03 0.04"
  "xm04_yp02 -0.04 0.02"
  "xm04_yp03 -0.04 0.03"
  "xm04_yp04 -0.04 0.04"
  "xm05_yp02 -0.05 0.02"
  "xm05_yp03 -0.05 0.03"
  "xm05_yp04 -0.05 0.04"
  "xm05_yp05 -0.05 0.05"
  "xm05_yp06 -0.05 0.06"
  "xm06_yp04 -0.06 0.04"
  "xm06_yp05 -0.06 0.05"
  "xm06_yp06 -0.06 0.06"
  "xm07_yp04 -0.07 0.04"
  "xm07_yp05 -0.07 0.05"
  "xm07_yp06 -0.07 0.06"
  "xm08_yp06 -0.08 0.06"
  "xm08_yp07 -0.08 0.07"
  "xm08_yp08 -0.08 0.08"
  "xm09_yp06 -0.09 0.06"
  "xm09_yp07 -0.09 0.07"
  "xm075_yp055 -0.075 0.055"
  "xm075_yp060 -0.075 0.060"
  "xm075_yp065 -0.075 0.065"
  "xm078_yp055 -0.078 0.055"
  "xm078_yp060 -0.078 0.060"
)

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

for condition in "${conditions[@]}"; do
  read -r name shift_x shift_y <<<"${condition}"
  run_dir="${experiment_dir}/${name}"
  mkdir -p "${run_dir}"
  if [[ -s "${run_dir}/data.csv" ]]; then
    echo "Skipping existing ${name}"
    continue
  fi

  echo "Running ${name}: shift_x=${shift_x}, shift_y=${shift_y}"
  DISPLAY="${DISPLAY:-:0}" \
  WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
  PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}" \
    "${simulator}" -r go2 -s scene_fast.xml >/dev/null 2>&1 &
  sim_pid=$!
  sleep 1

  set +e
  printf '\n' | "${controller}" \
    lo 6 "${run_dir}/data.csv" "${shift_x}" "${shift_y}" 0
  controller_status="${PIPESTATUS[1]}"
  set -e

  stop_simulator
  if [[ "${controller_status}" -ne 0 ]]; then
    if [[ -s "${run_dir}/data.csv" ]]; then
      echo "Controller exited with ${controller_status}, data was saved"
    else
      echo "Controller failed before saving data: ${name}" >&2
      exit "${controller_status}"
    fi
  fi
done

echo "Weight-shift scan completed: ${experiment_dir}"
