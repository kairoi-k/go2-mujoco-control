#!/usr/bin/env python3
"""Measure straight-line quality from MuJoCo ground truth."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * q / 100.0
    lo, hi = math.floor(index), math.ceil(index)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def yaw(row: dict[str, str]) -> float:
    w, x, y, z = (f(row, key) for key in ("base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unwrap_delta(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--trim-start-s", type=float, default=1.0)
    parser.add_argument("--trim-end-s", type=float, default=1.5)
    parser.add_argument(
        "--reference-heading-deg",
        type=float,
        default=0.0,
        help="world-frame heading of the commanded straight path (default: +X)",
    )
    parser.add_argument("--max-cross-track-p95-m", type=float, default=0.10)
    parser.add_argument("--max-end-cross-track-m", type=float, default=0.12)
    parser.add_argument("--max-heading-drift-deg", type=float, default=8.0)
    parser.add_argument("--max-lateral-speed-p95-mps", type=float, default=0.25)
    args = parser.parse_args()
    run_dir = args.run_dir
    data_path, truth_path = run_dir / "data.csv", run_dir / "contact_ground_truth.csv"
    if not data_path.exists() or not truth_path.exists():
        print("validation=FAIL\nfailure=missing data.csv or contact_ground_truth.csv")
        return 2
    with data_path.open(newline="") as handle:
        data = list(csv.DictReader(handle))
    with truth_path.open(newline="") as handle:
        truth = list(csv.DictReader(handle))
    stage2 = [row for row in data if int(float(row["motion_stage"])) == 2]
    if not stage2:
        print("validation=FAIL\nfailure=no motion_stage=2")
        return 1
    start = min(f(row, "state_tick_s") for row in stage2) + args.trim_start_s
    end = max(f(row, "state_tick_s") for row in stage2) - args.trim_end_s
    cruise = [row for row in truth if start <= f(row, "time_s") < end]
    if len(cruise) < 100:
        print("validation=FAIL")
        print(f"failure=cruise_samples={len(cruise)}<100")
        return 1
    y0, x0 = f(cruise[0], "base_pos_world_y_m"), f(cruise[0], "base_pos_world_x_m")
    reference_heading = math.radians(args.reference_heading_deg)
    cross_track = []
    heading = []
    lateral_speed = []
    for row in cruise:
        dx = f(row, "base_pos_world_x_m") - x0
        dy = f(row, "base_pos_world_y_m") - y0
        cross_track.append(-math.sin(reference_heading) * dx + math.cos(reference_heading) * dy)
        heading.append(math.degrees(unwrap_delta(yaw(row) - reference_heading)))
        lateral_speed.append(abs(f(row, "base_qvel_world_y_mps")))
    cross_p95 = percentile([abs(v) for v in cross_track], 95.0)
    cross_end = abs(cross_track[-1])
    heading_p95 = percentile([abs(v) for v in heading], 95.0)
    lateral_p95 = percentile(lateral_speed, 95.0)
    failures = []
    if cross_p95 > args.max_cross_track_p95_m:
        failures.append(f"cross_track_p95_m={cross_p95:.4f}>{args.max_cross_track_p95_m:.4f}")
    if cross_end > args.max_end_cross_track_m:
        failures.append(f"cross_track_end_m={cross_end:.4f}>{args.max_end_cross_track_m:.4f}")
    if heading_p95 > args.max_heading_drift_deg:
        failures.append(f"heading_drift_p95_deg={heading_p95:.4f}>{args.max_heading_drift_deg:.4f}")
    if lateral_p95 > args.max_lateral_speed_p95_mps:
        failures.append(f"lateral_speed_p95_mps={lateral_p95:.4f}>{args.max_lateral_speed_p95_mps:.4f}")
    print(f"cruise_window_s={start:.3f}..{end:.3f}")
    print(f"reference_heading_deg={args.reference_heading_deg:.3f}")
    print(f"path_start_xy_m={x0:.6f},{y0:.6f}")
    print(f"path_end_xy_m={f(cruise[-1], 'base_pos_world_x_m'):.6f},{f(cruise[-1], 'base_pos_world_y_m'):.6f}")
    print(f"cross_track_p95_m={cross_p95:.6f}")
    print(f"cross_track_end_m={cross_end:.6f}")
    print(f"heading_drift_p95_deg={heading_p95:.6f}")
    print(f"lateral_speed_p95_mps={lateral_p95:.6f}")
    if failures:
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1
    print("validation=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
