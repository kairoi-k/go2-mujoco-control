#!/usr/bin/env bash
set -euo pipefail

if (( $# < 3 )); then
  echo "usage: $0 <wall-timeout-s> <experiment-name> <output.mp4> [run_trot args...]" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cpp_dir="$(cd "$script_dir/.." && pwd)"
repo_dir="$(cd "$cpp_dir/../.." && pwd)"
tools_dir="$HOME/.local/share/unitree_mujoco_capture_tools/tools"
ffmpeg="$tools_dir/ffmpeg-static/bin/ffmpeg"
xdotool="$tools_dir/root/usr/bin/xdotool"
xdotool_lib="$tools_dir/root/usr/lib/x86_64-linux-gnu"
recorder="$cpp_dir/tools/recording/capture_x11_video.py"
wall_timeout="$1"
experiment_name="$2"
output="$3"
shift 3

if [[ ! -x "$ffmpeg" || ! -x "$xdotool" ]]; then
  echo "capture tools not found under $tools_dir" >&2
  exit 2
fi

mkdir -p "$(dirname "$output")"
run_log="/tmp/${experiment_name//\//_}.recording.log"
rm -f "$output" "$run_log"

TROT_RECORDING_GRACE_S="${TROT_RECORDING_GRACE_S:-5}" bash "$cpp_dir/scripts/run_trot.sh" "$wall_timeout" "$experiment_name" "$@" >"$run_log" 2>&1 &
run_pid=$!

window=""
for _ in $(seq 1 400); do
  activation="$(grep -m 1 "MuJoCo window activated:" "$run_log" || true)"
  if [[ -n "$activation" ]]; then
    window="${activation##*: }"
    break
  fi
  sleep 0.05
done
if [[ -z "$window" ]]; then
  echo "MuJoCo window was not found; see $run_log" >&2
  kill "$run_pid" 2>/dev/null || true
  wait "$run_pid" 2>/dev/null || true
  exit 1
fi

DISPLAY=:0 LD_LIBRARY_PATH="$xdotool_lib" "$xdotool" windowactivate "$window" >/dev/null 2>&1 || true
python3 "$recorder" "$output" \
  --ffmpeg "$ffmpeg" \
  --title MuJoCo \
  --duration "${TROT_RECORD_DURATION_S:-24}" \
  --fps "${TROT_RECORD_FPS:-20}" \
  --crop-x 0 --crop-y 0 --crop-width 1280 --crop-height 720 \
  --output-width 1280 --output-height 720 \
  --preset medium --crf 18
capture_status=$?
wait "$run_pid" || run_status=$?
run_status="${run_status:-0}"
if (( capture_status != 0 || run_status != 0 )); then
  echo "capture_status=$capture_status run_status=$run_status" >&2
  tail -40 "$run_log" >&2 || true
  exit 1
fi
printf 'Saved %s\n' "$output"
