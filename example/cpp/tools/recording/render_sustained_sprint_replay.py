#!/usr/bin/env python3
"""Render a measured Go2 sprint trajectory without perturbing the experiment.

The controller/simulator run remains the source of truth.  This tool only
replays the recorded base pose and joint state into MuJoCo's renderer, so a
WSLg window cannot change the dynamics being demonstrated.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


LEG_ORDER = (("FL", 7), ("FR", 10), ("RL", 13), ("RR", 16))
JOINTS = ("hip", "thigh", "calf")


def load_rows(path: Path, time_key: str) -> tuple[list[float], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    times = [float(row[time_key]) for row in rows]
    return times, rows


def nearest_index(times: list[float], value: float) -> int:
    index = max(0, min(len(times) - 1, bisect.bisect_left(times, value)))
    if index > 0 and abs(times[index - 1] - value) <= abs(times[index] - value):
        index -= 1
    return index


def font() -> ImageFont.ImageFont:
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, 22)
    return ImageFont.load_default()


def set_state(model: mujoco.MjModel, data: mujoco.MjData,
              row: dict[str, str], ground_row: dict[str, str]) -> None:
    data.qpos[:3] = [
        float(row["world_base_x_m"]),
        float(row["world_base_y_m"]),
        float(row["world_base_z_m"]),
    ]
    data.qpos[3:7] = [
        float(ground_row["base_quat_w"]),
        float(ground_row["base_quat_x"]),
        float(ground_row["base_quat_y"]),
        float(ground_row["base_quat_z"]),
    ]
    for leg, qpos_start in LEG_ORDER:
        for offset, joint in enumerate(JOINTS):
            data.qpos[qpos_start + offset] = float(row[f"{leg}_{joint}_q_state"])
    mujoco.mj_forward(model, data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera-distance", type=float, default=1.8)
    parser.add_argument("--camera-azimuth", type=float, default=90.0)
    parser.add_argument("--camera-elevation", type=float, default=-18.0)
    parser.add_argument("--ffmpeg", type=Path,
                        default=Path("/home/che/.local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg"))
    args = parser.parse_args()

    data_times, data_rows = load_rows(args.run_dir / "data.csv", "cmd_time_s")
    truth_times, truth_rows = load_rows(args.run_dir / "contact_ground_truth.csv", "time_s")
    start = max(args.start, data_times[0])
    end = data_times[-1] if args.duration is None else min(data_times[-1], start + args.duration)
    if end <= start:
        raise ValueError("replay interval is empty")

    repo_dir = Path(__file__).resolve().parents[4]
    scene = repo_dir / "unitree_robots/go2/scene_leg_lift_demo.xml"
    os.environ.setdefault("MUJOCO_GL", "egl")
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = args.camera_distance
    camera.azimuth = args.camera_azimuth
    camera.elevation = args.camera_elevation
    draw_font = font()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(args.ffmpeg), "-y", "-f", "rawvideo", "-pixel_format", "rgb24",
        "-video_size", f"{args.width}x{args.height}", "-framerate", str(args.fps),
        "-i", "-", "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output),
    ]
    encoder = subprocess.Popen(command, stdin=subprocess.PIPE,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    frame_count = 0
    frame_period = 1.0 / args.fps
    try:
        t = start
        while t <= end + 1e-9:
            row = data_rows[nearest_index(data_times, t)]
            truth = truth_rows[nearest_index(truth_times, t)]
            set_state(model, data, row, truth)
            camera.lookat[:] = [float(row["world_base_x_m"]),
                                float(row["world_base_y_m"]),
                                float(row["world_base_z_m"])]
            renderer.update_scene(data, camera)
            image = Image.fromarray(renderer.render(), mode="RGB")
            overlay = ImageDraw.Draw(image, "RGBA")
            speed = abs(float(row["world_velocity_x_mps"]))
            roll = math.degrees(float(row["imu_roll_rad"]))
            pitch = math.degrees(float(row["imu_pitch_rad"]))
            stage = row.get("motion_stage", "?")
            text = (f"Go2 sustained sprint replay  t={t:5.2f}s  "
                    f"v={speed:4.2f} m/s  roll={roll:+4.1f}°  pitch={pitch:+4.1f}°  stage={stage}")
            overlay.rounded_rectangle((18, 16, min(args.width - 18, 930), 54),
                                      radius=8, fill=(8, 20, 34, 205))
            overlay.text((32, 25), text, font=draw_font, fill=(240, 248, 255, 255))
            encoder.stdin.write(np.asarray(image, dtype=np.uint8).tobytes())
            frame_count += 1
            t += frame_period
    finally:
        renderer.close()
        if encoder.stdin:
            encoder.stdin.close()
        stderr = encoder.stderr.read().decode(errors="replace")
        code = encoder.wait()
    if code:
        raise RuntimeError(stderr)
    print(f"Saved {frame_count} frames to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
