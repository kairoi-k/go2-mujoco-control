#!/usr/bin/env python3
"""Strict acceptance checks for automatic height-map obstacle sensing."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import xml.etree.ElementTree as ET
from pathlib import Path

EVENT_NAMES = {
    0: "none", 1: "emergency_stop", 2: "obstacle_left",
    3: "obstacle_right", 4: "turn_left", 5: "turn_right",
    6: "slip", 7: "low_friction", 8: "impact",
}


def number(row: dict[str, str], key: str, default=math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def median(rows, key: str) -> float:
    values = [number(row, key) for row in rows]
    values = [value for value in values if math.isfinite(value)]
    return statistics.median(values) if values else math.nan


def metadata(path: Path) -> dict[str, str]:
    result = {}
    file = path / "run_metadata.txt"
    if file.exists():
        for line in file.read_text(errors="replace").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                result[key] = value
    return result


def rows(path: Path):
    with (path / "data.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def transitions(data):
    result, previous = [], None
    for row in data:
        event = int(number(row, "event_type", 0.0))
        if event != previous:
            result.append({
                "time_s": round(number(row, "cmd_time_s"), 4),
                "type": EVENT_NAMES.get(event, f"unknown_{event}"),
                "priority": int(number(row, "event_priority", 0.0)),
            })
            previous = event
    return result


def contact_max(path: Path):
    file = path / "contact_ground_truth.csv"
    if not file.exists():
        return {"rows": 0, "max_count": math.nan, "max_force_N": math.nan}
    with file.open(newline="", encoding="utf-8", errors="replace") as stream:
        data = list(csv.DictReader(stream))
    def values(key):
        return [number(row, key) for row in data if math.isfinite(number(row, key))]
    counts, forces = values("reactive_obstacle_contact_count"), values(
        "reactive_obstacle_contact_normal_force_N")
    return {"rows": len(data), "max_count": max(counts, default=math.nan),
            "max_force_N": max(forces, default=math.nan)}


def has_obstacle_scene(path: Path) -> bool:
    argv = metadata(path).get("argv", "")
    marker = "--scene-file "
    if marker not in argv:
        return False
    scene = Path(argv.split(marker, 1)[1].split(" --", 1)[0])
    if not scene.exists():
        return False
    try:
        return ET.parse(scene).getroot().find(
            ".//geom[@name='reactive_obstacle']") is not None
    except (OSError, ET.ParseError):
        return False


def analyze(path: Path, expect_obstacle: bool):
    result = {"experiment": str(path), "expect_obstacle": expect_obstacle,
              "strict_pass": False}
    if not (path / "data.csv").exists() or not (path / "run_metadata.txt").exists():
        result["failure"] = "missing data.csv or run_metadata.txt"
        return result
    data = rows(path)
    if not data:
        result["failure"] = "empty data.csv"
        return result
    meta = metadata(path)
    status_keys = ("controller_status", "safety_status", "quality_status",
                   "analysis_status", "ground_truth_status",
                   "dynamics_status", "completion_status")
    statuses = {key: int(meta.get(key, "-1")) for key in status_keys}
    stage = [row for row in data if int(number(row, "motion_stage", -1)) == 2]
    gait_start = min((number(row, "cmd_time_s") for row in stage), default=math.nan)
    valid_map = [row for row in data if number(row, "environment_map_valid", 0) >= .5]
    first_map = min((number(row, "cmd_time_s") for row in valid_map), default=math.nan)
    tail = [row for row in data if math.isfinite(first_map) and
            number(row, "cmd_time_s") >= first_map]
    map_rate = len(valid_map) / len(tail) if tail else 0.0
    max_age = max((number(row, "environment_map_age_s") for row in valid_map),
                  default=math.nan)
    observed = transitions(data)
    non_none = [item for item in observed if item["type"] != "none"]
    obstacle = next((item for item in non_none if str(item["type"]).startswith(
        "obstacle_")), None)
    obstacle_events = [item for item in non_none if str(item["type"]).startswith("obstacle_")]
    contact = contact_max(path)
    required = ("world_velocity_x_mps", "world_velocity_y_mps", "imu_roll_rad",
                "imu_pitch_rad", "wbc_full_eq_residual")
    finite = all(math.isfinite(number(row, key)) for row in data for key in required)
    max_roll = max((abs(number(row, "imu_roll_rad")) for row in data), default=math.nan)
    max_pitch = max((abs(number(row, "imu_pitch_rad")) for row in data), default=math.nan)
    max_residual = max((number(row, "wbc_full_eq_residual") for row in data),
                       default=math.nan)
    target_ok = response_ok = False
    latency = lateral_shift = math.nan
    if obstacle is not None and math.isfinite(gait_start):
        event_time = float(obstacle["time_s"])
        latency = event_time - gait_start - 1.50
        next_transition = next((item for item in observed
                                if item["time_s"] > event_time), None)
        event_end = float(next_transition["time_s"]) if next_transition else event_time + 8.0
        active = [row for row in data
                  if event_time <= number(row, "cmd_time_s") <= event_end]
        before = [row for row in data
                  if event_time - .8 <= number(row, "cmd_time_s") < event_time]
        target_window = active[:500]
        vy, yaw, kind = median(target_window, "event_target_vy_mps"), median(
            target_window, "event_target_yaw_rate_radps"), str(obstacle["type"])
        target_ok = ((kind == "obstacle_left" and vy >= .20 and yaw >= .10) or
                     (kind == "obstacle_right" and vy <= -.20 and yaw <= -.10))
        if active and before:
            lateral_shift = median(active[-10:], "world_base_y_m") - median(
                before[-10:], "world_base_y_m")
        response_ok = target_ok and (
            (kind == "obstacle_left" and lateral_shift >= .05)
            or (kind == "obstacle_right" and lateral_shift <= -.05))
    map_ok = bool(valid_map) and map_rate >= .95 and max_age <= .15
    status_ok = all(value == 0 for value in statuses.values())
    posture_ok = (math.isfinite(max_roll) and math.isfinite(max_pitch) and
                  max_roll <= .25 and max_pitch <= .25)
    solver_ok = math.isfinite(max_residual) and max_residual <= 1e-3
    contact_ok = (not expect_obstacle or (contact["rows"] > 100 and
                  contact["max_count"] == 0 and contact["max_force_N"] == 0))
    if expect_obstacle:
        unexpected = [item for item in non_none
                      if not str(item["type"]).startswith("obstacle_")]
        gaps_ok = all(b["time_s"] - a["time_s"] >= 0.25
                      for a, b in zip(obstacle_events, obstacle_events[1:]))
        event_ok = bool(obstacle_events) and not unexpected and gaps_ok
        detection_ok = (event_ok and 0 <= latency <= .50 and
                        has_obstacle_scene(path) and response_ok)
    else:
        event_ok = not non_none
        detection_ok = event_ok
    result.update({
        "rows": len(data), "statuses": statuses, "observed_transitions": observed,
        "gait_start_s": gait_start, "first_map_s": first_map,
        "map_valid_rows": len(valid_map), "map_valid_rate_after_first": map_rate,
        "max_map_age_s": max_age, "obstacle_event": obstacle,
        "obstacle_events": obstacle_events,
        "detection_latency_after_warmup_s": latency, "lateral_shift_m": lateral_shift,
        "obstacle_contact": contact, "map_ok": map_ok, "event_ok": event_ok,
        "detection_ok": detection_ok, "response_ok": response_ok,
        "contact_ok": contact_ok, "finite_required_columns": finite,
        "max_abs_roll_rad": max_roll, "max_abs_pitch_rad": max_pitch,
        "posture_ok": posture_ok, "max_wbc_full_eq_residual": max_residual,
        "solver_ok": solver_ok, "status_ok": status_ok,
    })
    result["strict_pass"] = all((status_ok, map_ok, finite, posture_ok, solver_ok,
                                  contact_ok, event_ok, detection_ok))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", nargs="+", type=Path)
    parser.add_argument("--obstacle", action="append", default=[], type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    obstacle_paths = {path.resolve() for path in args.obstacle}
    reports = [analyze(path.resolve(), path.resolve() in obstacle_paths)
               for path in args.experiment]
    summary = {"protocol": "automatic environment sensing acceptance v1.0",
               "reports": reports,
               "strict_pass": all(item["strict_pass"] for item in reports)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    md = args.output.with_suffix(".md")
    lines = ["# Automatic environment sensing acceptance", "",
             f"Overall strict pass: {summary['strict_pass']}", ""]
    for item in reports:
        lines += [f"## {Path(item['experiment']).name}", "",
                  f"strict_pass: {item['strict_pass']}",
                  f"statuses: {item['statuses']}",
                  f"map valid rate: {item.get('map_valid_rate_after_first', math.nan):.3f}",
                  f"obstacle event: {item.get('obstacle_event')}",
                  f"obstacle events: {item.get('obstacle_events')}",
                  f"detection latency after warmup: {item.get('detection_latency_after_warmup_s', math.nan):.3f} s",
                  f"lateral shift: {item.get('lateral_shift_m', math.nan):.3f} m",
                  f"obstacle contact: {item.get('obstacle_contact')}", ""]
    md.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"strict_pass": summary["strict_pass"],
                      "experiments": len(reports)}))
    return 0 if summary["strict_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
