#!/usr/bin/env python3
"""Strict, reproducible acceptance check for sustained 3 m/s sprint runs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def metadata(run: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    path = run / "run_metadata.txt"
    if path.exists():
        for line in path.read_text(errors="replace").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key] = value
    return values


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]


def intervals(rows: list[dict[str, str]], speed: float, angle: float) -> tuple[float, float, float, float]:
    best = (0.0, math.nan, math.nan)
    start: float | None = None
    good_angles: list[float] = []
    for row in rows:
        t = float(row["cmd_time_s"])
        v = abs(float(row["world_velocity_x_mps"]))
        a = max(abs(float(row["imu_roll_rad"])), abs(float(row["imu_pitch_rad"]))) * 180.0 / math.pi
        ok = v >= speed and a <= angle
        if ok:
            if start is None:
                start = t
            good_angles.append(a)
        elif start is not None:
            candidate = (t - start, start, t)
            if candidate[0] > best[0]:
                best = candidate
            start = None
    if start is not None and rows:
        end = float(rows[-1]["cmd_time_s"])
        candidate = (end - start, start, end)
        if candidate[0] > best[0]:
            best = candidate
    return best[0], best[1], best[2], percentile(good_angles, 0.95)


def inspect(run: Path, speed: float, angle: float, min_window: float) -> dict[str, object]:
    rows = list(csv.DictReader((run / "data.csv").open(newline="")))
    meta = metadata(run)
    stage2 = [r for r in rows if r.get("motion_stage") == "2"]
    stage3 = [r for r in rows if r.get("motion_stage") == "3"]
    source = stage2 or rows
    speeds = [abs(float(r["world_velocity_x_mps"])) for r in source]
    angles = [
        max(abs(float(r["imu_roll_rad"])), abs(float(r["imu_pitch_rad"]))) * 180.0 / math.pi
        for r in source
    ]
    window_s, window_start, window_end, window_p95 = intervals(source, speed, angle)
    final = stage3[-1] if stage3 else rows[-1]
    final_speed = abs(float(final["world_velocity_x_mps"]))
    final_angle = max(abs(float(final["imu_roll_rad"])), abs(float(final["imu_pitch_rad"]))) * 180.0 / math.pi
    log = (run / "controller.log").read_text(errors="replace")
    statuses = [
        meta.get(name, "missing")
        for name in ("controller_status", "safety_status", "quality_status", "analysis_status", "ground_truth_status", "dynamics_status", "completion_status")
    ]
    controlled_stop = "High-speed stop: WBC four-contact hold complete; finished in WBC stance" in log
    safe = "Trot hard safety limit reached" not in log and "Trot hard posture limit" not in log
    passed = bool(rows) and bool(stage3) and max(speeds, default=0.0) >= speed and window_s >= min_window and window_p95 <= angle and len(stage3) >= 500 and final_speed <= 0.15 and final_angle <= angle and controlled_stop and safe and all(s == "0" for s in statuses)
    return {
        "run": run.name,
        "max_speed": max(speeds, default=0.0),
        "median_speed": percentile(speeds, 0.50),
        "p05_speed": percentile(speeds, 0.05),
        "p95_speed": percentile(speeds, 0.95),
        "window_s": window_s,
        "window_start": window_start,
        "window_end": window_end,
        "window_angle_p95": window_p95,
        "whole_angle_p95": percentile(angles, 0.95),
        "stage2_rows": len(stage2),
        "stage3_rows": len(stage3),
        "final_speed": final_speed,
        "final_angle": final_angle,
        "controlled_stop": controlled_stop,
        "safe": safe,
        "statuses": "/".join(statuses),
        "pass": passed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("--speed-threshold", type=float, default=3.0)
    parser.add_argument("--angle-limit-deg", type=float, default=10.0)
    parser.add_argument("--min-window-s", type=float, default=8.0)
    args = parser.parse_args()
    results = [inspect(Path(path), args.speed_threshold, args.angle_limit_deg, args.min_window_s) for path in args.run_dirs]
    print("run,max_speed_mps,median_speed_mps,p05_speed_mps,p95_speed_mps,good_window_s,window_start_s,window_end_s,window_angle_p95_deg,whole_angle_p95_deg,stage2_rows,stage3_rows,final_speed_mps,final_angle_deg,controlled_stop,safe,statuses,pass")
    for r in results:
        print(f"{r['run']},{r['max_speed']:.3f},{r['median_speed']:.3f},{r['p05_speed']:.3f},{r['p95_speed']:.3f},{r['window_s']:.3f},{r['window_start']:.3f},{r['window_end']:.3f},{r['window_angle_p95']:.3f},{r['whole_angle_p95']:.3f},{r['stage2_rows']},{r['stage3_rows']},{r['final_speed']:.3f},{r['final_angle']:.3f},{int(r['controlled_stop'])},{int(r['safe'])},{r['statuses']},{int(r['pass'])}")
    passed = sum(bool(r["pass"]) for r in results)
    print(f"passed={passed}/{len(results)}")
    print("acceptance=PASS" if passed == len(results) else "acceptance=FAIL")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
