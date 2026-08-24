#!/usr/bin/env python3
"""Report and strictly validate a runtime velocity-command experiment."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

REQUIRED = {
    "cmd_time_s", "velocity_command_requested_mps",
    "velocity_command_shaped_mps", "velocity_command_applied_mps",
    "velocity_command_measured_mps", "velocity_command_tracking_error_mps",
    "velocity_command_accel_mps2", "velocity_command_jerk_mps3",
    "velocity_command_gait_period_s", "velocity_command_gait_duty",
    "velocity_command_gait_step_length_m", "velocity_command_gait_foot_lift_m",
    "velocity_command_gait_regime", "body_velocity_x_mps",
    "velocity_command_active",
    "imu_roll_rad", "imu_pitch_rad",
}

def floats(rows, key):
    return [float(row[key]) for row in rows if row.get(key, "") not in ("", "nan")]

def profile_sample(points, time_s):
    if time_s <= points[0][0]:
        return points[0][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if time_s <= t1:
            u = (time_s - t0) / max(1e-9, t1 - t0)
            return v0 + u * (v1 - v0)
    return points[-1][1]

def read_profile(path):
    points = []
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            t, v = (float(x.strip()) for x in line.replace(",", " ").split()[:2])
            points.append((t, v))
    return points

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    with (args.run_dir / "data.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    missing = sorted(REQUIRED - set(rows[0])) if rows else sorted(REQUIRED)
    if missing:
        raise SystemExit("missing diagnostics columns: " + ", ".join(missing))
    if not rows:
        raise SystemExit("empty diagnostics CSV")
    numeric = {
        key: floats(rows, key) for key in REQUIRED
        if key not in {"velocity_command_gait_regime", "velocity_command_active"}
    }
    evaluation_rows = [
        row for row in rows
        if int(float(row["velocity_command_active"])) != 0
        and row["velocity_command_gait_regime"] == "continuous-trot"
    ]
    if not evaluation_rows:
        raise SystemExit("no continuous-trot evaluation rows")
    active_start = float(evaluation_rows[0]["cmd_time_s"])
    evaluation_numeric = {
        key: floats(evaluation_rows, key) for key in REQUIRED
        if key not in {"velocity_command_gait_regime", "velocity_command_active"}
    }
    profile = read_profile(args.profile)
    logged_profile_error = max(
        abs(float(row["velocity_command_requested_mps"]) -
            profile_sample(profile, float(row["cmd_time_s"]) - active_start))
        for row in evaluation_rows
    )
    tracking = evaluation_numeric["velocity_command_tracking_error_mps"]
    applied_gap = [
        abs(float(row["velocity_command_shaped_mps"]) -
            float(row["velocity_command_applied_mps"]))
        for row in evaluation_rows
    ]
    result = {
        "rows": len(rows),
        "active_start_cmd_time_s": active_start,
        "duration_s": float(rows[-1]["cmd_time_s"]) - float(rows[0]["cmd_time_s"]),
        "requested_profile_max_error_mps": logged_profile_error,
        "tracking_error_abs_max_mps": max(map(abs, tracking)),
        "tracking_error_abs_mean_mps": sum(map(abs, tracking)) / len(tracking),
        "applied_shaped_gap_max_mps": max(applied_gap),
        "shaped_speed_max_mps": max(evaluation_numeric["velocity_command_shaped_mps"]),
        "measured_speed_max_mps": max(evaluation_numeric["velocity_command_measured_mps"]),
        "accel_abs_max_mps2": max(map(abs, evaluation_numeric["velocity_command_accel_mps2"])),
        "jerk_abs_max_mps3": max(map(abs, evaluation_numeric["velocity_command_jerk_mps3"])),
        "period_range_s": [min(evaluation_numeric["velocity_command_gait_period_s"]),
                           max(evaluation_numeric["velocity_command_gait_period_s"])],
        "duty_range": [min(evaluation_numeric["velocity_command_gait_duty"]),
                       max(evaluation_numeric["velocity_command_gait_duty"])],
        "step_length_max_m": max(evaluation_numeric["velocity_command_gait_step_length_m"]),
        "foot_lift_max_m": max(evaluation_numeric["velocity_command_gait_foot_lift_m"]),
        "roll_abs_max_deg": math.degrees(max(map(abs, numeric["imu_roll_rad"]))),
        "pitch_abs_max_deg": math.degrees(max(map(abs, numeric["imu_pitch_rad"]))),
        "regimes": sorted({row["velocity_command_gait_regime"] for row in rows}),
    }
    metadata = {}
    metadata_path = args.run_dir / "run_metadata.txt"
    if metadata_path.exists():
        for line in metadata_path.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                metadata[key] = value
    for key in ("controller_status", "safety_status", "quality_status",
                "completion_status", "analysis_status", "git_head"):
        result[key] = metadata.get(key, "")
    result["strict_pass"] = all(
        result[key] == "0"
        for key in ("controller_status", "safety_status", "quality_status",
                    "completion_status", "analysis_status")
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.json_out:
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    raise SystemExit(0 if result["strict_pass"] else 1)

if __name__ == "__main__":
    main()
