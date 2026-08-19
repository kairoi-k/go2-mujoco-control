#!/usr/bin/env python3
"""Quantify whether a reactive acceptance run actually shows the requested response."""

from __future__ import annotations

import argparse
import csv
import re
import json
import math
import statistics
import xml.etree.ElementTree as ET
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


def scene_obstacle_status(metadata: dict[str, str]) -> tuple[bool, bool, str]:
    argv = metadata.get("argv", "")
    marker = "--scene-file "
    if marker not in argv:
        return False, False, ""
    token = argv.split(marker, 1)[1].split(" --", 1)[0]
    scene = Path(token)
    if not scene.is_absolute() and not scene.exists():
        scene = Path(__file__).resolve().parents[3] / scene
    if not scene.exists():
        return False, False, str(scene)
    try:
        root = ET.parse(scene).getroot()
        geom = root.find(".//geom[@name='reactive_obstacle']")
        if geom is None:
            return True, False, str(scene)
        contype = int(geom.attrib.get("contype", "1"))
        conaffinity = int(geom.attrib.get("conaffinity", "1"))
        return True, bool(contype and conaffinity), str(scene)
    except (OSError, ET.ParseError, ValueError):
        return True, False, str(scene)


def obstacle_contact_metrics(path: Path) -> dict[str, float]:
    file = path / "contact_ground_truth.csv"
    if not file.exists():
        return {
            "obstacle_contact_data_ok": 0.0,
            "obstacle_contact_max_force_N": math.nan,
            "obstacle_contact_max_normal_force_N": math.nan,
            "obstacle_contact_max_count": math.nan,
        }
    try:
        with file.open(newline="") as stream:
            reader = csv.DictReader(stream)
            required = {
                "reactive_obstacle_contact_count",
                "reactive_obstacle_contact_force_N",
                "reactive_obstacle_contact_normal_force_N",
            }
            if not required.issubset(reader.fieldnames or set()):
                return {
                    "obstacle_contact_data_ok": 0.0,
                    "obstacle_contact_max_force_N": math.nan,
                    "obstacle_contact_max_normal_force_N": math.nan,
                    "obstacle_contact_max_count": math.nan,
                }
            rows = list(reader)
        force = finite(rows, "reactive_obstacle_contact_force_N")
        normal = finite(rows, "reactive_obstacle_contact_normal_force_N")
        counts = finite(rows, "reactive_obstacle_contact_count")
        if not force or not normal or not counts:
            raise ValueError("empty obstacle contact data")
        return {
            "obstacle_contact_data_ok": 1.0,
            "obstacle_contact_max_force_N": max(force),
            "obstacle_contact_max_normal_force_N": max(normal),
            "obstacle_contact_max_count": max(counts),
        }
    except (OSError, csv.Error, ValueError):
        return {
            "obstacle_contact_data_ok": 0.0,
            "obstacle_contact_max_force_N": math.nan,
            "obstacle_contact_max_normal_force_N": math.nan,
            "obstacle_contact_max_count": math.nan,
        }


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
    stop_tail = [
        row for row in active
        if number(row, "cmd_time_s") >= max(start, end - 0.75)
    ]
    stop_tail_vx = [
        abs(value) for row in stop_tail
        if math.isfinite(value := number(row, "world_velocity_x_mps"))
    ]
    target_yaw = median(active, "event_target_yaw_rate_radps")
    metrics = {
        "pre_vx_mps": pre_vx,
        "active_vx_min_mps": active_vx_min,
        "active_vx_max_mps": active_vx_max,
        "post_vx_mps": post_vx,
        "stop_tail_abs_vx_max_mps": max(stop_tail_vx, default=math.nan),
        "stop_tail_abs_vx_median_mps": statistics.median(stop_tail_vx) if stop_tail_vx else math.nan,
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
    if event == "obstacle_left":
        metrics["lateral_shift_ok"] = float(metrics["lateral_shift_m"] >= 0.05)
    elif event == "obstacle_right":
        metrics["lateral_shift_ok"] = float(metrics["lateral_shift_m"] <= -0.05)
    else:
        metrics["lateral_shift_ok"] = math.nan
    if event == "emergency_stop":
        hold_values = [
            int(number(row, "event_hold_stance", 0.0)) for row in stop_tail
        ]
        metrics["brake_ok"] = float(
            math.isfinite(pre_vx)
            and math.isfinite(metrics["stop_tail_abs_vx_max_mps"])
            and metrics["stop_tail_abs_vx_max_mps"] <= 0.035
            and abs(target_vx) <= 0.02
            and bool(hold_values)
            and min(hold_values) == 1
        )
    elif event in {"turn_left", "turn_right"}:
        metrics["brake_ok"] = float(
            math.isfinite(ref_yaw) and abs(ref_yaw) >= 0.20
            and math.isfinite(metrics["yaw_change_rad"])
            and abs(metrics["yaw_change_rad"]) >= 0.12
        )
    elif event in {"obstacle_left", "obstacle_right"}:
        metrics["brake_ok"] = float(
            math.isfinite(ref_yaw) and abs(ref_yaw) >= 0.10
            and math.isfinite(metrics["yaw_change_rad"])
            and abs(metrics["yaw_change_rad"]) >= 0.08
            and metrics["lateral_shift_ok"] == 1.0
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
    if expected in {"obstacle_left", "obstacle_right"}:
        scene_exists, scene_physical, _ = scene_obstacle_status(metadata)
        contact = obstacle_contact_metrics(path)
        metrics.update(contact)
        metrics["obstacle_scene_physical_ok"] = float(
            scene_exists and scene_physical
        )
        metrics["obstacle_contact_ok"] = float(
            contact["obstacle_contact_data_ok"] == 1.0
            and contact["obstacle_contact_max_count"] <= 0.0
            and contact["obstacle_contact_max_force_N"] <= 1.0e-6
        )
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
    obstacle_ok = (
        expected not in {"obstacle_left", "obstacle_right"}
        or (metrics.get("obstacle_scene_physical_ok", 0.0) == 1.0
            and metrics.get("obstacle_contact_ok", 0.0) == 1.0)
    )
    script_ok = bool(
        scheduled
        and scripted_transition_ok(event_transitions, expected)
        and metrics.get("event_rows", 0.0) >= 50.0
        and metrics.get("post_rows", 0.0) >= 50.0
        and metrics.get("brake_ok", 0.0) == 1.0
        and obstacle_ok
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
    baseline_ok = (
        scheduled is None and external is None
        and len(rows) >= 10000 and math.isfinite(max_time)
        and max_time >= 20.0
    )
    result["strict_pass"] = status_ok and (script_ok or external_ok or baseline_ok)
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
