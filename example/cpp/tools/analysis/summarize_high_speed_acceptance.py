#!/usr/bin/env python3
"""Summarize bounded >3 m/s sprint acceptance runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    metadata = path / "run_metadata.txt"
    if metadata.exists():
        for line in metadata.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                result[key] = value
    return result


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))
    return ordered[index]


def summarize(path: Path, speed_threshold: float, angle_limit_deg: float) -> dict[str, object]:
    with (path / "data.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    speeds = [float(row["world_velocity_x_mps"]) for row in rows]
    angles = [
        max(
            abs(float(row["imu_roll_rad"])),
            abs(float(row["imu_pitch_rad"])),
        )
        * 180.0
        / math.pi
        for row in rows
    ]
    good = [speed >= speed_threshold and angle <= angle_limit_deg for speed, angle in zip(speeds, angles)]
    best_duration = 0.0
    best_start = math.nan
    best_end = math.nan
    start = None
    for row, is_good in zip(rows, good):
        time_s = float(row["cmd_time_s"])
        if is_good and start is None:
            start = time_s
        if not is_good and start is not None:
            duration = time_s - start
            if duration > best_duration:
                best_duration, best_start, best_end = duration, start, time_s
            start = None
    if start is not None and rows:
        end = float(rows[-1]["cmd_time_s"])
        if end - start > best_duration:
            best_duration, best_start, best_end = end - start, start, end
    window_angles = [angle for angle, is_good in zip(angles, good) if is_good]
    stage3 = [row for row in rows if row.get("motion_stage") == "3"]
    final = stage3[-1] if stage3 else rows[-1]
    metadata = read_metadata(path)
    log = (path / "controller.log").read_text(errors="replace")
    controlled_stop = "High-speed stop: WBC four-contact hold complete; finished in WBC stance" in log
    safe = "Trot hard safety limit reached" not in log and "Trot hard posture limit" not in log
    final_angle = max(
        abs(float(final["imu_roll_rad"])),
        abs(float(final["imu_pitch_rad"])),
    ) * 180.0 / math.pi
    return {
        "run": path.name,
        "max_speed": max(speeds),
        "window_s": best_duration,
        "window_start_s": best_start,
        "window_end_s": best_end,
        "window_angle_p95_deg": percentile(window_angles, 0.95),
        "whole_angle_p95_deg": percentile(angles, 0.95),
        "stage3_rows": len(stage3),
        "final_speed": float(final["world_velocity_x_mps"]),
        "final_angle_deg": final_angle,
        "controlled_stop": controlled_stop,
        "safe": safe,
        "dynamics_status": metadata.get("dynamics_status", "missing"),
        "pass": (
            max(speeds) >= speed_threshold
            and best_duration >= 0.60
            and percentile(window_angles, 0.95) <= angle_limit_deg
            and len(stage3) >= 500
            and abs(float(final["world_velocity_x_mps"])) <= 0.15
            and final_angle <= angle_limit_deg
            and controlled_stop
            and safe
            and metadata.get("safety_status") == "0"
            and metadata.get("dynamics_status") == "0"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("--speed-threshold", type=float, default=3.0)
    parser.add_argument("--angle-limit-deg", type=float, default=10.0)
    args = parser.parse_args()
    results = [
        summarize(Path(run), args.speed_threshold, args.angle_limit_deg)
        for run in args.run_dirs
    ]
    print("run,max_speed_mps,good_window_s,window_angle_p95_deg,stage3_rows,final_speed_mps,final_angle_deg,dynamics_status,controlled_stop,safe,pass")
    for result in results:
        print(
            f"{result['run']},{result['max_speed']:.3f},{result['window_s']:.3f},"
            f"{result['window_angle_p95_deg']:.3f},{result['stage3_rows']},"
            f"{result['final_speed']:.3f},{result['final_angle_deg']:.3f},"
            f"{result['dynamics_status']},{int(result['controlled_stop'])},"
            f"{int(result['safe'])},{int(result['pass'])}"
        )
    passed = sum(bool(result["pass"]) for result in results)
    print(f"passed={passed}/{len(results)}")
    print("acceptance=PASS" if passed == len(results) else "acceptance=FAIL")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
