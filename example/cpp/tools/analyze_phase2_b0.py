#!/usr/bin/env python3
"""Frozen B0 sensor-only contract analyzer."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def rows(path: Path):
    with (path / "data.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(row, key, default=float("nan")):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def fraction(values, predicate):
    finite = [x for x in values if math.isfinite(x)]
    return sum(predicate(x) for x in finite) / len(finite) if finite else float("nan")


def metadata(path: Path):
    result = {}
    file = path / "run_metadata.txt"
    if file.exists():
        for line in file.read_text(encoding="utf-8", errors="replace").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                result[key] = value
    return result


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def paired_differences(active, baseline):
    keys = {
        "velocity_command_requested_mps": 1.0e-6,
        "velocity_command_shaped_mps": 0.010,
        "velocity_command_applied_mps": 0.010,
        "velocity_command_gait_period_s": 1.0e-5,
        "velocity_command_gait_duty": 1.0e-5,
        "velocity_command_gait_step_length_m": 1.0e-5,
        "velocity_command_gait_foot_lift_m": 1.0e-5,
        "event_active": 0.5,
        "event_type": 0.5,
        "event_target_vx_mps": 0.020,
        "wbc_full_velocity_target_x_mps": 0.020,
        "wbc_full_requested_acc_x_mps2": 0.20,
    }
    result = {}
    count = min(len(active), len(baseline))
    for key, tolerance in keys.items():
        diffs = [abs(number(active[i], key) - number(baseline[i], key))
                 for i in range(count)
                 if math.isfinite(number(active[i], key)) and
                 math.isfinite(number(baseline[i], key))]
        result[key] = {
            "max_abs_difference": max(diffs, default=float("nan")),
            "tolerance": tolerance,
            "pass": bool(diffs) and max(diffs) <= tolerance,
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    active = rows(args.run_dir)
    if not active:
        raise SystemExit("empty data.csv")
    required = {
        "terrain_enabled", "terrain_sensor_only", "terrain_actuation",
        "terrain_map_valid", "terrain_map_source", "terrain_map_epoch",
        "terrain_map_age_s", "terrain_plan_status", "terrain_plan_valid",
        "terrain_planner_updates", "terrain_planner_rejections",
        "terrain_planner_deadline_misses", "terrain_solver_elapsed_us",
        "terrain_safe_stop_requested", "event_active", "event_type",
    }
    missing = sorted(required - set(active[0]))
    if missing:
        raise SystemExit("missing B0 columns: " + ", ".join(missing))
    md = metadata(args.run_dir)
    checks = {}
    checks["sensor_only_flag"] = all(number(row, "terrain_sensor_only") > 0.5 for row in active)
    checks["no_terrain_actuation"] = all(number(row, "terrain_actuation") < 0.5 for row in active)
    checks["no_hidden_event"] = all(number(row, "event_active") < 0.5 and
                                     number(row, "event_type") < 0.5 for row in active)
    checks["lidar_provenance"] = all(row.get("terrain_map_source") in ("lidar", "none")
                                     for row in active)
    checks["finite_map_age"] = all(
        not math.isfinite(number(row, "terrain_map_age_s")) or
        number(row, "terrain_map_age_s") >= 0.0 for row in active)
    checks["planner_does_not_safe_stop"] = all(
        number(row, "terrain_safe_stop_requested") < 0.5 for row in active)
    checks["planner_deadline"] = all(
        number(row, "terrain_solver_elapsed_us") <= 5000.0 or
        number(row, "terrain_planner_deadline_misses") > 0.5 for row in active)
    comparison = None
    if args.baseline:
        baseline = rows(args.baseline)
        comparison = paired_differences(active, baseline)
        checks["paired_control_interface"] = all(item["pass"] for item in comparison.values())
    else:
        checks["paired_control_interface"] = False
    result = {
        "contract": "b0-contract-v1.1",
        "run_dir": str(args.run_dir),
        "git_head": md.get("git_head", ""),
        "controller_status": md.get("controller_status", ""),
        "safety_status": md.get("safety_status", ""),
        "quality_status": md.get("quality_status", ""),
        "analysis_status": md.get("analysis_status", ""),
        "completion_status": md.get("completion_status", ""),
        "terrain_rows": len(active),
        "terrain_map_valid_fraction": fraction(
            [number(row, "terrain_map_valid") for row in active], lambda x: x > 0.5),
        "terrain_planner_updates": max(
            (number(row, "terrain_planner_updates") for row in active), default=float("nan")),
        "terrain_planner_deadline_misses": max(
            (number(row, "terrain_planner_deadline_misses") for row in active), default=float("nan")),
        "checks": checks,
        "paired_comparison": comparison,
    }
    result["legacy_status_pass"] = all(result[key] == "0" for key in (
        "controller_status", "safety_status", "quality_status",
        "analysis_status", "completion_status"))
    result["acceptance_status"] = "PASS" if result["legacy_status_pass"] and all(checks.values()) else "FAIL"
    output = json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n"
    print(output, end="")
    if args.json_out:
        args.json_out.write_text(output, encoding="utf-8")
    raise SystemExit(0 if result["acceptance_status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
