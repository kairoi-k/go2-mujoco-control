#!/usr/bin/env python3
"""Quantify whether a reactive acceptance run actually shows the requested response."""

from __future__ import annotations

import argparse
import csv
import re
import json
import math
import statistics
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

SCRIPT_EVENTS = {
    "emergency_stop": "emergency_stop",
    "obstacle_left": "obstacle_left",
    "obstacle_right": "obstacle_right",
    "turn_left": "turn_left",
    "turn_right": "turn_right",
    "slip": "slip",
    "low_friction": "low_friction",
    "impact": "impact",
}


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def finite(rows: list[dict[str, str]], key: str) -> list[float]:
    return [value for row in rows if math.isfinite(value := number(row, key))]


def median(rows: list[dict[str, str]], key: str) -> float:
    values = finite(rows, key)
    return statistics.median(values) if values else math.nan


def extrema(rows: list[dict[str, str]], key: str) -> tuple[float, float]:
    values = finite(rows, key)
    return (min(values), max(values)) if values else (math.nan, math.nan)


def slice_time(
    rows: list[dict[str, str]], start: float, end: float
) -> list[dict[str, str]]:
    return [
        row for row in rows
        if start <= number(row, "cmd_time_s") <= end
    ]


def read_metadata(path: Path) -> dict[str, str]:
    metadata: dict[str, str] = {}
    file = path / "run_metadata.txt"
    if file.exists():
        for line in file.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                metadata[key] = value
    return metadata


def read_rows(path: Path) -> list[dict[str, str]]:
    with (path / "data.csv").open(newline="") as stream:
        return list(csv.DictReader(stream))


def transitions(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    previous = None
    for row in rows:
        event = int(number(row, "event_type", 0.0))
        if event != previous:
            result.append({
                "time_s": round(number(row, "cmd_time_s", 0.0), 4),
                "type": EVENT_NAMES.get(event, f"unknown_{event}"),
                "priority": int(number(row, "event_priority", 0.0)),
            })
            previous = event
    return result


def scripted_transition_ok(event_transitions, expected):
    return [item["type"] for item in event_transitions if item["type"] != "none"] == [expected]


def script_event(metadata: dict[str, str]) -> tuple[str, float, float] | None:
    argv = metadata.get("argv", "")
    marker = "--event-script "
    if marker not in argv:
        return None
    path = Path(argv.split(marker, 1)[1].split(" --", 1)[0])
    if not path.exists():
        return None
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) < 3:
            continue
        try:
            start, duration = float(fields[0]), float(fields[1])
        except ValueError:
            continue
        return SCRIPT_EVENTS.get(fields[2], fields[2]), start, start + duration
    return None

def external_event(
    path: Path, rows: list[dict[str, str]]
) -> tuple[str, float, float, str] | None:
    log = path / "simulator.log"
    text = log.read_text(errors="replace") if log.exists() else ""
    if "PUSH vel applied" in text:
        jumps = []
        for previous, current in zip(rows, rows[1:]):
            try:
                dt = number(current, "cmd_time_s") - number(previous, "cmd_time_s")
                dv = math.hypot(
                    number(current, "world_velocity_x_mps")
                    - number(previous, "world_velocity_x_mps"),
                    number(current, "world_velocity_y_mps")
                    - number(previous, "world_velocity_y_mps"),
                )
            except (TypeError, ValueError):
                continue
            if dt > 0.0 and math.isfinite(dv):
                jumps.append((dv, number(current, "cmd_time_s")))
        if jumps:
            dv, start = max(jumps)
            return "impact", start, start + 0.8, f"push_dv={dv:.3f}mps"
    match = re.search(
        r"FRICTION event active t=([0-9.]+).*?FRICTION restored t=([0-9.]+)",
        text,
        re.S,
    )
    if match:
        return "low_friction", float(match.group(1)), float(match.group(2)), "simulator_friction"
    return None



def unwrap_delta(values: list[float]) -> float:
    if len(values) < 2:
        return math.nan
    total = 0.0
    previous = values[0]
    for current in values[1:]:
        delta = current - previous
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        total += delta
        previous = current
    return total


def response_metrics(
    rows: list[dict[str, str]], event: str, start: float, end: float
) -> dict[str, float]:
    pre = [row for row in rows if start - 1.0 <= number(row, "cmd_time_s") < start]
    active = slice_time(rows, start, end)
    post = [row for row in rows if end < number(row, "cmd_time_s") <= end + 3.0]
    yaw_values = [
        value for row in active
        if math.isfinite(value := number(row, "imu_yaw_rad"))
    ]
    pre_vx = median(pre, "world_velocity_x_mps")
    active_vx_min, active_vx_max = extrema(active, "world_velocity_x_mps")
    post_vx = median(post, "world_velocity_x_mps")
    velocity_jumps = [
        math.hypot(
            number(current, "world_velocity_x_mps")
            - number(previous, "world_velocity_x_mps"),
            number(current, "world_velocity_y_mps")
            - number(previous, "world_velocity_y_mps"),
        )
        for previous, current in zip(pre[-1:] + active, active)
    ]
    max_velocity_jump_mps = max(velocity_jumps, default=math.nan)
    pre_y = median(pre, "world_base_y_m")
    end_y = median(active[-10:], "world_base_y_m") if active else math.nan
    pre_yaw = median(pre, "imu_yaw_rad")
    end_yaw = median(active[-10:], "imu_yaw_rad") if active else math.nan
    ref_vx = median(active, "event_ref_vx_mps")
    ref_yaw = median(active, "event_ref_yaw_rate_radps")
    target_vx = median(active, "event_target_vx_mps")
    target_yaw = median(active, "event_target_yaw_rate_radps")
    metrics = {
        "pre_vx_mps": pre_vx,
        "active_vx_min_mps": active_vx_min,
        "active_vx_max_mps": active_vx_max,
        "post_vx_mps": post_vx,
        "max_velocity_jump_mps": max_velocity_jump_mps,
        "braking_drop_mps": pre_vx - active_vx_min,
        "pre_y_m": pre_y,
        "active_end_y_m": end_y,
        "lateral_shift_m": end_y - pre_y,
        "pre_yaw_rad": pre_yaw,
        "active_end_yaw_rad": end_yaw,
        "yaw_change_rad": end_yaw - pre_yaw,
        "yaw_integral_delta_rad": unwrap_delta(yaw_values),
        "reference_vx_mps": ref_vx,
        "reference_yaw_rate_radps": ref_yaw,
        "target_vx_mps": target_vx,
        "target_yaw_rate_radps": target_yaw,
        "event_rows": float(len(active)),
        "post_rows": float(len(post)),
    }
    if event in {"turn_left", "obstacle_left"}:
        metrics["directional_yaw_ok"] = float(metrics["yaw_change_rad"] > 0.12)
    elif event in {"turn_right", "obstacle_right"}:
        metrics["directional_yaw_ok"] = float(metrics["yaw_change_rad"] < -0.12)
    else:
        metrics["directional_yaw_ok"] = math.nan
    if event == "emergency_stop":
        metrics["brake_ok"] = float(
            math.isfinite(pre_vx) and math.isfinite(active_vx_min)
            and pre_vx - active_vx_min >= 0.05
            and abs(target_vx) <= 0.02
        )
    elif event in {"turn_left", "turn_right", "obstacle_left", "obstacle_right"}:
        metrics["brake_ok"] = float(
            math.isfinite(ref_yaw) and abs(ref_yaw) >= 0.20
            and math.isfinite(metrics["yaw_change_rad"])
            and abs(metrics["yaw_change_rad"]) >= 0.12
        )
    elif event in {"slip", "low_friction", "impact"}:
        metrics["brake_ok"] = float(
            math.isfinite(pre_vx) and math.isfinite(active_vx_min)
            and pre_vx - active_vx_min >= 0.03
        )
    else:
        metrics["brake_ok"] = math.nan
    return metrics


def gait_start_time(rows: list[dict[str, str]]) -> float:
    values = [
        number(row, "cmd_time_s")
        for row in rows
        if int(number(row, "motion_stage", -1.0)) == 2
    ]
    return min(values) if values else math.nan

def analyze(path_string: str) -> dict[str, object]:
    path = Path(path_string)
    metadata = read_metadata(path)
    rows = read_rows(path)
    event_transitions = transitions(rows)
    scheduled = script_event(metadata)
    external = external_event(path, rows) if scheduled is None else None
    external_detail = None
    metrics: dict[str, float] = {}
    expected = None
    relative_start = math.nan
    relative_end = math.nan
    gait_start_s = gait_start_time(rows)
    if scheduled:
        expected, relative_start, relative_end = scheduled
        start = gait_start_s + relative_start
        end = gait_start_s + relative_end
        metrics = response_metrics(rows, expected, start, end)
    elif external:
        expected, start, end, external_detail = external
        metrics = response_metrics(rows, expected, start, end)
    status_keys = (
        "controller_status", "safety_status", "quality_status",
        "analysis_status", "ground_truth_status", "dynamics_status",
        "completion_status",
    )
    statuses = {key: int(metadata.get(key, -1)) for key in status_keys}
    max_time = max((number(row, "cmd_time_s") for row in rows), default=math.nan)
    result: dict[str, object] = {
        "experiment": path.name,
        "rows": len(rows),
        "max_time_s": max_time,
        "statuses": statuses,
        "event_transitions": event_transitions,
        "scheduled_event": (
            {"type": expected, "start_s": start, "end_s": end,
             "relative_start_s": relative_start,
             "relative_end_s": relative_end}
            if scheduled else (
                {"type": expected, "start_s": start, "end_s": end,
                 "source": external_detail}
                if external else None
            )
        ),
        "gait_start_s": gait_start_s,
        "metrics": metrics,
    }
    status_ok = bool(rows) and all(value == 0 for value in statuses.values())
    script_ok = bool(
        scheduled
        and scripted_transition_ok(event_transitions, expected)
        and metrics.get("event_rows", 0.0) >= 50.0
        and metrics.get("post_rows", 0.0) >= 50.0
        and metrics.get("brake_ok", 0.0) == 1.0
        and (
            expected not in {"turn_left", "turn_right",
                             "obstacle_left", "obstacle_right"}
            or metrics.get("directional_yaw_ok", 0.0) == 1.0
        )
    )
    external_ok = bool(
        external and (
            (expected == "impact" and
             float(re.search(r"push_dv=([0-9.]+)", external_detail or "0").group(1)) >= 0.35 and
             sum(item["type"] == "impact" for item in event_transitions) == 1)
            or (expected == "low_friction" and
                external_detail == "simulator_friction")
        )
    )
    result["strict_pass"] = status_ok and (script_ok or external_ok)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", nargs="+")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    results = [analyze(item) for item in args.experiment]
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
        return
    for result in results:
        event = result["scheduled_event"]
        metrics = result["metrics"]
        print(
            f"{result['experiment']}: "
            f"strict={'PASS' if result['strict_pass'] else 'FAIL'} "
            f"rows={result['rows']} t={result['max_time_s']:.2f}s "
            f"event={event} "
            f"yaw={metrics.get('yaw_change_rad', math.nan):.3f}rad "
            f"dv={metrics.get('braking_drop_mps', math.nan):.3f}m/s "
            f"ref_yaw={metrics.get('reference_yaw_rate_radps', math.nan):.3f}"
        )


if __name__ == "__main__":
    main()
