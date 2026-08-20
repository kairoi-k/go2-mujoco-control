#!/usr/bin/env python3
"""Strict acceptance checks for an automatic physical-impact response run."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

EVENT_NAMES = {
    0: "none",
    1: "emergency_stop",
    2: "obstacle_left",
    3: "obstacle_right",
    4: "turn_left",
    5: "turn_right",
    6: "slip",
    7: "low_friction",
    8: "impact",
}

SOURCE_NAMES = {
    0: "none",
    1: "scheduled",
    2: "sensor",
    3: "safety_latch",
}


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def rows(path: Path) -> list[dict[str, str]]:
    with (path / "data.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    with (path / "run_metadata.txt").open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if "=" in line:
                key, value = line.rstrip("\n").split("=", 1)
                result[key] = value
    return result


def transitions(data: list[dict[str, str]]) -> list[dict[str, float | str]]:
    result: list[dict[str, float | str]] = []
    previous = None
    for row in data:
        code = int(number(row, "event_type", -1))
        event = EVENT_NAMES.get(code, f"unknown_{code}")
        if event != previous:
            result.append({
                "cmd_time_s": number(row, "cmd_time_s"),
                "state_tick_s": number(row, "state_tick_s"),
                "type": event,
                "priority": int(number(row, "event_priority", 0)),
                "source": int(number(row, "event_source", 0)),
                "source_name": SOURCE_NAMES.get(
                    int(number(row, "event_source", 0)), "unknown"),
            })
            previous = event
    return result


def push_time(path: Path) -> float:
    text = (path / "simulator.log").read_text(encoding="utf-8", errors="replace")
    values = [float(match.group(1)) for match in re.finditer(
        r"PUSH (?:vel applied|active) t=([0-9]+(?:\.[0-9]+)?)", text)]
    return values[0] if values else math.nan


def statuses(meta: dict[str, str]) -> dict[str, int]:
    keys = ("controller_status", "safety_status", "quality_status",
            "analysis_status", "ground_truth_status", "dynamics_status",
            "completion_status")
    return {key: int(meta.get(key, "-1")) for key in keys}


def max_adjacent_velocity_jump(data: list[dict[str, str]], center: float) -> float:
    values = []
    previous = None
    for row in data:
        state_time = number(row, "state_tick_s")
        if not math.isfinite(state_time) or abs(state_time - center) > 0.40:
            continue
        velocity = (number(row, "raw_body_velocity_x_mps"),
                    number(row, "raw_body_velocity_y_mps"))
        if not all(math.isfinite(item) for item in velocity):
            continue
        if previous is not None:
            values.append(math.hypot(velocity[0] - previous[0], velocity[1] - previous[1]))
        previous = velocity
    return max(values, default=math.nan)


def analyze(path: Path, push_tolerance_s: float) -> dict:
    result: dict = {"experiment": str(path), "strict_pass": False}
    if not (path / "data.csv").exists() or not (path / "run_metadata.txt").exists():
        result["failure"] = "missing data.csv or run_metadata.txt"
        return result
    data = rows(path)
    meta = metadata(path)
    if not data:
        result["failure"] = "empty data.csv"
        return result
    observed = transitions(data)
    push = push_time(path)
    impact = next((item for item in observed if item["type"] == "impact"), None)
    stop = next((item for item in observed if item["type"] == "emergency_stop"), None)
    state_impact = float(impact["state_tick_s"]) if impact else math.nan
    state_stop = float(stop["state_tick_s"]) if stop else math.nan
    valid_map = [row for row in data if number(row, "environment_map_valid", 0) >= 0.5]
    first_map = min((number(row, "cmd_time_s") for row in valid_map), default=math.nan)
    map_tail = [row for row in data if math.isfinite(first_map) and
                number(row, "cmd_time_s") >= first_map]
    map_rate = len(valid_map) / len(map_tail) if map_tail else 0.0
    max_age = max((number(row, "environment_map_age_s") for row in valid_map), default=math.nan)
    required = ("raw_body_velocity_x_mps", "raw_body_velocity_y_mps",
                "imu_roll_rad", "imu_pitch_rad", "wbc_full_eq_residual")
    finite = all(math.isfinite(number(row, key)) for row in data for key in required)
    max_roll = max((abs(number(row, "imu_roll_rad")) for row in data), default=math.nan)
    max_pitch = max((abs(number(row, "imu_pitch_rad")) for row in data), default=math.nan)
    max_residual = max((number(row, "wbc_full_eq_residual") for row in data), default=math.nan)
    sequence_ok = [item["type"] for item in observed] == ["none", "impact", "emergency_stop"]
    source_ok = (
        impact is not None and stop is not None and
        impact.get("source") == 2 and stop.get("source") == 1)
    latency = state_impact - push if math.isfinite(state_impact) and math.isfinite(push) else math.nan
    stop_delay = state_stop - state_impact if math.isfinite(state_stop) and math.isfinite(state_impact) else math.nan
    jump = max_adjacent_velocity_jump(data, push) if math.isfinite(push) else math.nan
    log = (path / "controller.log").read_text(encoding="utf-8", errors="replace")
    hold_complete = "Emergency stop hold complete; ending in WBC stance" in log
    status = statuses(meta)
    status_ok = all(value == 0 for value in status.values())
    map_ok = bool(valid_map) and map_rate >= 0.95 and max_age <= 0.15
    timing_ok = (math.isfinite(latency) and -0.10 <= latency <= push_tolerance_s and
                 math.isfinite(stop_delay) and 0.35 <= stop_delay <= 0.80)
    dynamics_ok = math.isfinite(jump) and jump >= 0.35
    posture_ok = (math.isfinite(max_roll) and math.isfinite(max_pitch) and
                  max_roll <= 0.25 and max_pitch <= 0.25)
    solver_ok = math.isfinite(max_residual) and max_residual <= 1e-3
    result.update({
        "rows": len(data), "statuses": status, "observed_transitions": observed,
        "push_state_time_s": push, "impact_state_time_s": state_impact,
        "emergency_stop_state_time_s": state_stop,
        "impact_detection_latency_s": latency, "emergency_stop_delay_s": stop_delay,
        "max_velocity_jump_near_push_mps": jump,
        "map_valid_rate_after_first": map_rate, "max_map_age_s": max_age,
        "max_abs_roll_rad": max_roll, "max_abs_pitch_rad": max_pitch,
        "max_wbc_full_eq_residual": max_residual,
        "sequence_ok": sequence_ok, "hold_complete": hold_complete,
        "source_ok": source_ok,
        "status_ok": status_ok, "map_ok": map_ok, "timing_ok": timing_ok,
        "dynamics_ok": dynamics_ok, "finite_required_columns": finite,
        "posture_ok": posture_ok, "solver_ok": solver_ok,
    })
    result["strict_pass"] = all((status_ok, map_ok, timing_ok, dynamics_ok,
                                  sequence_ok, source_ok, hold_complete, finite,
                                  posture_ok, solver_ok))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--push-state-tolerance-s", type=float, default=0.20)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.experiment.resolve(), args.push_state_tolerance_s)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md = args.output.with_suffix(".md")
    lines = ["# Automatic physical-impact acceptance", "",
             f"Strict pass: {report['strict_pass']}", "",
             f"Observed transitions: {report.get('observed_transitions')}",
             f"Event sources: {[(x.get('type'), x.get('source_name')) for x in report.get('observed_transitions', [])]}",
             f"Push / impact / emergency state time: {report.get('push_state_time_s')} / {report.get('impact_state_time_s')} / {report.get('emergency_stop_state_time_s')} s",
             f"Impact latency: {report.get('impact_detection_latency_s')} s",
             f"Emergency-stop delay: {report.get('emergency_stop_delay_s')} s",
             f"Velocity jump near push: {report.get('max_velocity_jump_near_push_mps')} m/s",
             f"Map valid rate / max age: {report.get('map_valid_rate_after_first')} / {report.get('max_map_age_s')} s",
             f"Max |roll| / |pitch|: {report.get('max_abs_roll_rad')} / {report.get('max_abs_pitch_rad')} rad",
             f"Max WBC residual: {report.get('max_wbc_full_eq_residual')}", "",
             "The push time is taken from simulator.log and compared with state_tick_s, not controller wall time."]
    md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"strict_pass": report["strict_pass"], "output": str(args.output)}))
    return 0 if report["strict_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
