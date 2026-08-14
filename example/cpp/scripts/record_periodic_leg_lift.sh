#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "${script_dir}/.." && pwd)"
repo_dir="$(cd "${cpp_dir}/../.." && pwd)"
tools_dir="${GO2_CAPTURE_TOOLS:-}"
if [[ -z "${tools_dir}" ]]; then
  echo "Set GO2_CAPTURE_TOOLS to the capture-tools root containing ffmpeg-static/ and root/usr/bin/xdotool." >&2
  exit 2
fi
ffmpeg="${tools_dir}/ffmpeg-static/bin/ffmpeg"
xdotool="${tools_dir}/root/usr/bin/xdotool"
if [[ ! -x "${ffmpeg}" || ! -x "${xdotool}" ]]; then
  echo "GO2_CAPTURE_TOOLS does not contain the expected ffmpeg/xdotool executables: ${tools_dir}" >&2
  exit 2
fi
simulator="${repo_dir}/simulate/build/unitree_mujoco"
controller="${cpp_dir}/build/real_leg_lift_go2"
recorder="${cpp_dir}/tools/recording/capture_x11_video.py"
experiment_dir="${cpp_dir}/experiments/go2_periodic_leg_lift_final_2026-07-16"
output="${1:-${experiment_dir}/media/fr_periodic_lift_final.mp4}"

ready_file="/tmp/go2_recording_ready"
start_file="/tmp/go2_recording_start"
stop_file="/tmp/go2_recording_stop"

find /tmp -maxdepth 1 \
  \( -name go2_recording_ready -o -name go2_recording_start \
     -o -name go2_recording_stop \) -delete

if pgrep -f '[u]nitree_mujoco -r go2' >/dev/null; then
  echo "A Go2 MuJoCo simulator is already running; stop it first." >&2
  exit 1
fi

mkdir -p "$(dirname "${output}")" "${experiment_dir}/recording_validation"

sim_pid=""
controller_pid=""
recorder_pid=""
cleanup() {
  for pid in "${recorder_pid}" "${controller_pid}" "${sim_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}"
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

DISPLAY="${DISPLAY:-:0}" \
WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}" \
  "${simulator}" -r go2 -s scene_leg_lift_demo.xml \
  >"${experiment_dir}/recording_validation/simulator_reframed.log" 2>&1 &
sim_pid=$!

export PATH="${tools_dir}/root/usr/bin:${PATH}"
export LD_LIBRARY_PATH="${tools_dir}/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"

window=""
for _ in $(seq 1 100); do
  window="$("${xdotool}" search --name MuJoCo 2>/dev/null | tail -1 || true)"
  [[ -n "${window}" ]] && break
  sleep 0.05
done
[[ -n "${window}" ]]
"${xdotool}" windowactivate "${window}"
sleep 0.2
"${xdotool}" key Tab
"${xdotool}" key shift+Tab
sleep 0.5

(
  printf '\n' |
    GO2_READY_FILE="${ready_file}" \
    GO2_START_FILE="${start_file}" \
    stdbuf -oL -eL "${controller}" \
      lo 45 \
      "${experiment_dir}/recording_validation/data_reframed.csv" \
      -0.070 0.060 0.040 3
) >"${experiment_dir}/recording_validation/controller_reframed.log" 2>&1 &
controller_pid=$!

for _ in $(seq 1 1000); do
  [[ -e "${ready_file}" ]] && break
  kill -0 "${controller_pid}" 2>/dev/null || {
    cat "${experiment_dir}/recording_validation/controller_reframed.log" >&2
    exit 1
  }
  sleep 0.01
done
[[ -e "${ready_file}" ]]

python3 "${recorder}" "${output}" \
  --ffmpeg "${ffmpeg}" \
  --stop-file "${stop_file}" \
  --post-roll 2 \
  --fps 30 \
  --crop-x 280 --crop-y 0 \
  --crop-width 720 --crop-height 720 \
  --output-width 1080 --output-height 1080 \
  --preset medium --crf 18 &
recorder_pid=$!

sleep 0.5
touch "${start_file}"
wait "${controller_pid}"
controller_pid=""
touch "${stop_file}"
wait "${recorder_pid}"
recorder_pid=""

echo "Saved synchronized recording: ${output}"
