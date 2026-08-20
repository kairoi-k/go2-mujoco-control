#!/usr/bin/env python3
"""Render representative videos with a synchronized data panel and collection."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image, ImageDraw

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COLORS = {"red": "#ff4d5a", "orange": "#ffae42", "blue": "#4da6ff", "cyan": "#21d4fd", "green": "#34d058"}


def read_series(path: Path, key: str, clip_start: float, duration: float) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                t = float(row["cmd_time_s"]) - clip_start
                value = float(row[key])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(t) and math.isfinite(value) and -0.5 <= t <= duration + 0.5:
                times.append(t)
                values.append(value)
    if not times:
        return np.array([0.0, duration]), np.zeros(2)
    order = np.argsort(times)
    return np.asarray(times)[order], np.asarray(values)[order]


def interpolate(path: Path, key: str, clip_start: float, duration: float, frame_times: np.ndarray) -> np.ndarray:
    times, values = read_series(path, key, clip_start, duration)
    return np.interp(frame_times, times, values)


def bounds(*series: np.ndarray, minimum_span: float) -> tuple[float, float]:
    values = np.concatenate([value[np.isfinite(value)] for value in series if value.size])
    if values.size == 0:
        return -minimum_span, minimum_span
    low, high = float(values.min()), float(values.max())
    center = (low + high) / 2.0
    span = max((high - low) * 0.20, minimum_span)
    return center - span, center + span


def render_chart_base(
    data_path: Path,
    title: str,
    subtitle: str,
    spans: list[dict[str, object]],
    clip_start: float,
    duration: float,
    output: Path,
) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
    times = np.linspace(0.0, duration, max(2, int(round(duration * 20))))
    vx = interpolate(data_path, "world_velocity_x_mps", clip_start, duration, times)
    vx_ref = interpolate(data_path, "event_ref_vx_mps", clip_start, duration, times)
    vy = interpolate(data_path, "world_velocity_y_mps", clip_start, duration, times)
    vy_ref = interpolate(data_path, "event_ref_vy_mps", clip_start, duration, times)
    yaw = interpolate(data_path, "imu_gyro_z_radps", clip_start, duration, times)
    yaw_ref = interpolate(data_path, "event_ref_yaw_rate_radps", clip_start, duration, times)

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9})
    fig, (ax_v, ax_yaw) = plt.subplots(2, 1, figsize=(6.4, 7.2), dpi=100, sharex=True, gridspec_kw={"height_ratios": [1.25, 1.0]})
    fig.patch.set_facecolor("#0b1117")
    for ax in (ax_v, ax_yaw):
        ax.set_facecolor("#101923")
        ax.grid(True, color="#29404f", alpha=0.55, linewidth=0.6)
        for spine in ax.spines.values():
            spine.set_color("#5a7180")
        ax.tick_params(colors="#d9e2e8", labelsize=8)
        ax.set_xlim(0.0, duration)
        for span in spans:
            color = COLORS.get(str(span.get("color", "orange")), COLORS["orange"])
            ax.axvspan(float(span["start_s"]), float(span["end_s"]), color=color, alpha=0.12, linewidth=0)

    ax_v.plot(times, vx, color="#f4f7f9", linewidth=1.4, label="world vx")
    ax_v.plot(times, vx_ref, color="#65e572", linewidth=1.2, linestyle="--", label="ref vx")
    ax_v.plot(times, vy, color="#46d7ff", linewidth=1.2, label="world vy")
    ax_v.plot(times, vy_ref, color="#ffbd63", linewidth=1.2, linestyle="--", label="ref vy")
    ax_yaw.plot(times, yaw, color="#ff74c8", linewidth=1.4, label="gyro z")
    ax_yaw.plot(times, yaw_ref, color="#f7ef64", linewidth=1.2, linestyle="--", label="ref yaw")
    ax_v.set_ylim(*bounds(vx, vx_ref, vy, vy_ref, minimum_span=0.12))
    ax_yaw.set_ylim(*bounds(yaw, yaw_ref, minimum_span=0.18))
    ax_v.set_ylabel("velocity (m/s)", color="#d9e2e8")
    ax_yaw.set_ylabel("yaw rate (rad/s)", color="#d9e2e8")
    ax_yaw.set_xlabel("video time (s)", color="#d9e2e8")
    ax_v.legend(loc="upper right", ncol=2, fontsize=8, facecolor="#182532", edgecolor="#5a7180", labelcolor="#f1f5f7")
    ax_yaw.legend(loc="upper right", ncol=2, fontsize=8, facecolor="#182532", edgecolor="#5a7180", labelcolor="#f1f5f7")
    fig.suptitle(title, x=0.06, ha="left", color="white", fontsize=12, fontweight="bold")
    fig.text(0.06, 0.955, subtitle, color="#a9bdc8", fontsize=8, ha="left")
    fig.text(0.06, 0.015, "solid = measured feedback   dashed = controller reference   cursor = current frame", color="#9eb0ba", fontsize=7, ha="left")
    fig.subplots_adjust(left=0.13, right=0.97, top=0.90, bottom=0.08, hspace=0.32)
    for span in spans:
        start, end = float(span["start_s"]), float(span["end_s"])
        label = str(span.get("label", "EVENT"))
        ax_v.text((start + end) / 2.0, 0.97, label, transform=ax_v.get_xaxis_transform(), ha="center", va="top", color=COLORS.get(str(span.get("color", "orange")), "#ffae42"), fontsize=7, fontweight="bold", clip_on=True)
    fig.canvas.draw()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, facecolor=fig.get_facecolor())
    bbox_v = ax_v.get_position()
    bbox_y = ax_yaw.get_position()
    width, height = 640, 720
    x_bounds = (int(bbox_v.x0 * width), int(bbox_v.x1 * width))
    y_v = (int((1.0 - bbox_v.y1) * height), int((1.0 - bbox_v.y0) * height))
    y_yaw = (int((1.0 - bbox_y.y1) * height), int((1.0 - bbox_y.y0) * height))
    plt.close(fig)
    return x_bounds, y_v, y_yaw


def make_chart_frames(base_path: Path, output_dir: Path, duration: float, geometry: tuple[tuple[int, int], tuple[int, int], tuple[int, int]]) -> int:
    x_bounds, y_v, y_yaw = geometry
    base = Image.open(base_path).convert("RGB")
    frame_count = int(round(duration * 20))
    output_dir.mkdir(parents=True, exist_ok=True)
    width, height = base.size
    for index in range(frame_count):
        t = index / 20.0
        frame = base.copy()
        draw = ImageDraw.Draw(frame)
        x = int(round(x_bounds[0] + (x_bounds[1] - x_bounds[0]) * min(1.0, max(0.0, t / duration))))
        draw.line((x, y_v[0], x, y_yaw[1]), fill="#ff4358", width=3)
        draw.line((x - 1, y_v[0], x - 1, y_yaw[1]), fill="#ffd1d6", width=1)
        draw.rectangle((width - 135, 34, width - 8, 56), fill="#182532", outline="#718896")
        draw.text((width - 128, 39), f"t = {t:5.2f} s", fill="#ffffff")
        frame.save(output_dir / f"{index + 1:06d}.png", format="PNG", optimize=False)
    return frame_count


def render_panel(ffmpeg: Path, source: Path, frames: Path, output: Path, duration: float) -> None:
    command = [
        str(ffmpeg), "-y", "-hide_banner", "-loglevel", "error",
        "-i", str(source), "-framerate", "20", "-start_number", "1", "-i", str(frames / "%06d.png"),
        "-filter_complex", "[0:v]fps=20[sim];[1:v]format=yuv420p[data];[sim][data]hstack=inputs=2:shortest=1[v]",
        "-map", "[v]", "-an", "-t", f"{duration:.3f}", "-c:v", "libx264", "-preset", "medium", "-crf", "19", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(command)
    if result.returncode != 0 or not output.exists() or output.stat().st_size < 100_000:
        raise RuntimeError(f"panel render failed: {output}")


def make_collection(ffmpeg: Path, paths: list[Path], output: Path, scratch: Path) -> None:
    list_path = scratch / "panel_concat.txt"
    list_path.write_text("".join(f"file '{path.resolve()}'\n" for path in paths), encoding="utf-8")
    result = subprocess.run([str(ffmpeg), "-y", "-hide_banner", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(list_path), "-c", "copy", "-movflags", "+faststart", str(output)])
    if result.returncode != 0 or not output.exists() or output.stat().st_size < 1_000_000:
        raise RuntimeError("panel collection failed")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--only", nargs="*")
    args = parser.parse_args()
    ffmpeg = Path.home() / ".local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg"
    records = json.loads(args.manifest.read_text(encoding="utf-8"))
    wanted = set(args.only or [str(item["id"]) for item in records])
    args.output_root.mkdir(parents=True, exist_ok=True)
    panel_records: list[dict[str, object]] = []
    with tempfile.TemporaryDirectory(prefix="reactive_data_panels_") as temp:
        temp_root = Path(temp)
        for item in records:
            item_id = str(item["id"])
            if item_id not in wanted:
                continue
            source = Path(str(item["video"]))
            data_path = Path(str(item["run_directory"])) / "data.csv"
            duration = float(item["clip_duration_s"])
            panel = args.output_root / "videos_data_panel" / f"{item_id}_data_panel.mp4"
            base = temp_root / f"{item_id}_base.png"
            frames = temp_root / f"{item_id}_frames"
            geometry = render_chart_base(data_path, str(item["title"]), str(item["subtitle"]), list(item.get("event_spans", [])), float(item["clip_start_s"]), duration, base)
            frame_count = make_chart_frames(base, frames, duration, geometry)
            render_panel(ffmpeg, source, frames, panel, duration)
            panel_records.append({"id": item_id, "source_video": str(source), "video": str(panel), "duration_s": duration, "width": 1920, "height": 720, "fps": 20, "frame_count": frame_count, "status": "pass"})
    panel_records.sort(key=lambda item: [str(x["id"]) for x in records].index(str(item["id"])))
    (args.output_root / "data_panel_manifest.json").write_text(json.dumps(panel_records, ensure_ascii=False, indent=2), encoding="utf-8")
    readme = args.output_root / "DATA_PANEL_README.md"
    readme.write_text("# Representative data-panel videos\n\n左侧是原始 MuJoCo 仿真，右侧是逐帧同步面板：实线为测量反馈，虚线为控制器参考，红色游标为当前视频时刻。上图为 vx/vy，下图为 yaw rate；底色和标签对应事件窗口。原始视频仍保留在 `videos/`。\n", encoding="utf-8")
    collection = args.output_root / "reactive_representatives_data_panel_collection.mp4"
    with tempfile.TemporaryDirectory(prefix="reactive_panel_collection_") as temp:
        make_collection(ffmpeg, [Path(str(item["video"])) for item in panel_records], collection, Path(temp))
    print(f"rendered data panels: {len(panel_records)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
