#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cpp_dir="$repo_dir/example/cpp"
duration_s="${SUSTAINED_SPRINT_DURATION_S:-30}"
domain_id="${SUSTAINED_SPRINT_DOMAIN_ID:-231}"
name="${SUSTAINED_SPRINT_NAME:-sustained_sprint_3mps_gui_$(date +%Y%m%d_%H%M%S)}"
video="${SUSTAINED_SPRINT_VIDEO:-$cpp_dir/experiments/_videos/${name}.mp4}"
capture_tool="$cpp_dir/tools/recording/capture_x11_video.py"
ffmpeg_bin="${SUSTAINED_SPRINT_FFMPEG:-$HOME/.local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg}"
run_pid=""
capture_pid=""

if [[ ! -x "$ffmpeg_bin" ]]; then
  echo "Missing ffmpeg capture binary: $ffmpeg_bin" >&2
  exit 2
fi

export SUSTAINED_SPRINT_DURATION_S="$duration_s"
export SUSTAINED_SPRINT_DOMAIN_ID="$domain_id"
export SUSTAINED_SPRINT_NAME="$name"
export UNITREE_MUJOCO_REFRESH_RATE="${UNITREE_MUJOCO_REFRESH_RATE:-30}"
run_dir="$cpp_dir/experiments/_runs/$name"
mkdir -p "$run_dir" "$(dirname "$video")"

cleanup() {
  if [[ -n "$capture_pid" ]] && kill -0 "$capture_pid" 2>/dev/null; then
    kill "$capture_pid" 2>/dev/null || true
    wait "$capture_pid" 2>/dev/null || true
  fi
  if [[ -n "$run_pid" ]] && kill -0 "$run_pid" 2>/dev/null; then
    kill "$run_pid" 2>/dev/null || true
    wait "$run_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cd "$repo_dir"
bash example/cpp/scripts/run_sustained_sprint.sh --view >"$run_dir/launcher.log" 2>&1 &
run_pid=$!

# run_trot activates the MuJoCo window before starting the controller. Wait
# for that exact point so the capture helper never races a nonexistent window.
display_value="${DISPLAY:-:0}"
runtime_dir="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
xdotool_path="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/bin/xdotool"
window_id=""
for _ in $(seq 1 300); do
  if ! kill -0 "$run_pid" 2>/dev/null; then
    break
  fi
  if [[ -x "$xdotool_path" ]]; then
    window_id="$(DISPLAY="$display_value" XDG_RUNTIME_DIR="$runtime_dir" \
      LD_LIBRARY_PATH="$HOME/.local/share/unitree_mujoco_capture_tools/tools/root/usr/lib/x86_64-linux-gnu" \
      "$xdotool_path" search --name "MuJoCo" 2>/dev/null | tail -n 1 || true)"
  fi
  if [[ -n "$window_id" ]]; then
    break
  fi
  sleep 0.05
done

if [[ -z "$window_id" ]]; then
  wait "$run_pid" || true
  echo "MuJoCo window was not available; inspect $run_dir/launcher.log" >&2
  exit 1
fi

python3 "$capture_tool" "$video" \
  --ffmpeg "$ffmpeg_bin" \
  --title MuJoCo \
  --duration "$((duration_s + 10))" \
  --stop-file "$run_dir/stop.request" \
  --post-roll 3 \
  --fps 30 \
  --crop-width 1280 --crop-height 720 \
  --output-width 1920 --output-height 1080 \
  --preset medium --crf 18 &
capture_pid=$!

wait "$run_pid" || run_status=$?
run_status="${run_status:-0}"
wait "$capture_pid" || capture_status=$?
capture_status="${capture_status:-0}"

if (( run_status != 0 || capture_status != 0 )); then
  echo "run_status=$run_status capture_status=$capture_status" >&2
  exit 1
fi
[[ -s "$video" ]] || { echo "No video produced: $video" >&2; exit 1; }
echo "Saved $video"
