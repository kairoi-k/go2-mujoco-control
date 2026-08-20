#!/usr/bin/env python3
"""Strict acceptance for an automatically sensed obstacle followed by an impact."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from analyze_auto_environment import (
    contact_max,
    has_obstacle_scene,
    metadata,
    number,
    rows,
)
from analyze_auto_impact import max_adjacent_velocity_jump, push_time, statuses


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


def finite_column(data: list[dict[str, str]], key: str) -> bool:
    return all(math.isfinite(number(row, key)) for row in data)


def transitions_with_state(data: list[dict[str, str]]) -> list[dict[str, object]]:
    """Return controller and MuJoCo clocks for each event transition."""
    result: list[dict[str, object]] = []
    previous = None
    for row in data:
        event = int(number(row, "event_type", 0.0))
        if event != previous:
            result.append(
                {
                    "time_s": round(number(row, "cmd_time_s"), 4),
                    "state_tick_s": round(number(row, "state_tick_s"), 4),
                    "type": EVENT_NAMES.get(event, f"unknown_{event}"),
                    "priority": int(number(row, "event_priority", 0.0)),
                }
            )
            previous = event
    return result


def analyze(path: Path, push_tolerance_s: float) -> dict:
    result: dict = {"experiment": str(path), "strict_pass": False}
    if not ((path / "data.csv").exists() and (path / "run_metadata.txt").exists()):
        result["failure"] = "missing data.csv or run_metadata.txt"
        return result
    data = rows(path)
    if not data:
        result["failure"] = "empty data.csv"
        return result

    meta = metadata(path)
    observed = transitions_with_state(data)
    non_none = [item for item in observed if item["type"] != "none"]
    obstacle = next(
        (item for item in non_none if str(item["type"]).startswith("obstacle_")),
        None,
    )
    impact = next((item for item in non_none if item["type"] == "impact"), None)
    emergency = next(
        (item for item in non_none if item["type"] == "emergency_stop"),
        None,
    )
    push = push_time(path)
    obstacle_time = float(obstacle["time_s"]) if obstacle else math.nan
    impact_time = float(impact["time_s"]) if impact else math.nan
    obstacle_state_time = number(obstacle or {}, "state_tick_s")
    impact_state_time = number(impact or {}, "state_tick_s")
    emergency_state_time = number(emergency or {}, "state_tick_s")
    obstacle_type = str(obstacle["type"]) if obstacle else ""
    expected = ["none", obstacle_type, "impact", "emergency_stop"]
    observed_types = [str(item["type"]) for item in observed]

    obstacle_rows = [
        row
        for row in data
        if math.isfinite(obstacle_time)
        and math.isfinite(impact_time)
        and obstacle_time <= number(row, "cmd_time_s") < impact_time
    ]
    target_vy = (
        sum(number(row, "event_target_vy_mps") for row in obstacle_rows)
        / len(obstacle_rows)
        if obstacle_rows
        else math.nan
    )
    target_yaw = (
        sum(number(row, "event_target_yaw_rate_radps") for row in obstacle_rows)
        / len(obstacle_rows)
        if obstacle_rows
        else math.nan
    )
    target_ok = (
        (obstacle_type == "obstacle_left" and target_vy >= 0.20 and target_yaw >= 0.10)
        or (
            obstacle_type == "obstacle_right"
            and target_vy <= -0.20
            and target_yaw <= -0.10
        )
    )

    contact = contact_max(path)
    valid_map = [
        row for row in data if number(row, "environment_map_valid", 0.0) >= 0.5
    ]
    first_map = min(
        (number(row, "cmd_time_s") for row in valid_map), default=math.nan
    )
    map_tail = [
        row
        for row in data
        if math.isfinite(first_map) and number(row, "cmd_time_s") >= first_map
    ]
    map_rate = len(valid_map) / len(map_tail) if map_tail else 0.0
    max_map_age = max(
        (number(row, "environment_map_age_s") for row in valid_map),
        default=math.nan,
    )
    max_roll = max(
        (abs(number(row, "imu_roll_rad")) for row in data), default=math.nan
    )
    max_pitch = max(
        (abs(number(row, "imu_pitch_rad")) for row in data), default=math.nan
    )
    max_residual = max(
        (number(row, "wbc_full_eq_residual") for row in data), default=math.nan
    )
    push_latency = (
        impact_state_time - push
        if math.isfinite(impact_state_time) and math.isfinite(push)
        else math.nan
    )
    emergency_delay = (
        emergency_state_time - impact_state_time
        if math.isfinite(emergency_state_time)
        and math.isfinite(impact_state_time)
        else math.nan
    )
    velocity_jump = max_adjacent_velocity_jump(data, push)
    statuses_value = statuses(meta)
    sequence_ok = observed_types == expected
    preemption_ok = (
        obstacle is not None
        and impact is not None
        and math.isfinite(obstacle_state_time)
        and math.isfinite(impact_state_time)
        and impact_state_time >= obstacle_state_time
        and impact_state_time < obstacle_state_time + 8.0
        and int(obstacle.get("priority", 0)) == 80
        and int(impact.get("priority", 0)) == 100
        and emergency is not None
        and int(emergency.get("priority", 0)) == 100
    )
    map_ok = bool(valid_map) and map_rate >= 0.95 and max_map_age <= 0.15
    timing_ok = (
        math.isfinite(push_latency)
        and -0.10 <= push_latency <= push_tolerance_s
        and math.isfinite(emergency_delay)
        and 0.35 <= emergency_delay <= 0.80
    )
    finite = all(
        finite_column(data, key)
        for key in (
            "raw_body_velocity_x_mps",
            "raw_body_velocity_y_mps",
            "imu_roll_rad",
            "imu_pitch_rad",
            "wbc_full_eq_residual",
        )
    )
    dynamics_ok = math.isfinite(velocity_jump) and velocity_jump >= 0.35
    posture_ok = (
        math.isfinite(max_roll)
        and math.isfinite(max_pitch)
        and max_roll <= 0.25
        and max_pitch <= 0.25
    )
    solver_ok = math.isfinite(max_residual) and max_residual <= 1e-3
    contact_ok = (
        contact["rows"] > 100
        and contact["max_count"] == 0
        and contact["max_force_N"] == 0
    )
    status_ok = all(value == 0 for value in statuses_value.values())
    hold_complete = (
        "Emergency stop hold complete; ending in WBC stance"
        in (path / "controller.log").read_text(errors="replace")
    )
    result.update(
        {
            "rows": len(data),
            "statuses": statuses_value,
            "observed_transitions": observed,
            "push_state_time_s": push,
            "obstacle_state_time_s": obstacle_state_time,
            "impact_state_time_s": impact_state_time,
            "impact_detection_latency_s": push_latency,
            "emergency_stop_delay_s": emergency_delay,
            "obstacle_type": obstacle_type,
            "obstacle_target_vy_mps": target_vy,
            "obstacle_target_yaw_rate_radps": target_yaw,
            "max_velocity_jump_near_push_mps": velocity_jump,
            "map_valid_rate_after_first": map_rate,
            "max_map_age_s": max_map_age,
            "max_abs_roll_rad": max_roll,
            "max_abs_pitch_rad": max_pitch,
            "max_wbc_full_eq_residual": max_residual,
            "sequence_ok": sequence_ok,
            "preemption_ok": preemption_ok,
            "target_ok": target_ok,
            "map_ok": map_ok,
            "timing_ok": timing_ok,
            "dynamics_ok": dynamics_ok,
            "finite_required_columns": finite,
            "posture_ok": posture_ok,
            "solver_ok": solver_ok,
            "contact_ok": contact_ok,
            "status_ok": status_ok,
            "hold_complete": hold_complete,
            "physical_obstacle_scene_ok": has_obstacle_scene(path),
        }
    )
    result["strict_pass"] = all(
        (
            sequence_ok,
            preemption_ok,
            target_ok,
            result["physical_obstacle_scene_ok"],
            map_ok,
            timing_ok,
            dynamics_ok,
            finite,
            posture_ok,
            solver_ok,
            contact_ok,
            status_ok,
            hold_complete,
        )
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--push-state-tolerance-s", type=float, default=0.20)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.experiment.resolve(), args.push_state_tolerance_s)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    markdown = args.output.with_suffix(".md")
    markdown.write_text(
        "\n".join(
            [
                "# Automatic obstacle-impact priority acceptance",
                "",
                f"Strict pass: {report['strict_pass']}",
                f"Observed transitions: {report.get('observed_transitions')}",
                f"Impact latency: {report.get('impact_detection_latency_s')} s",
                f"Emergency-stop delay: {report.get('emergency_stop_delay_s')} s",
                f"Obstacle-to-impact state interval: {report.get('impact_state_time_s', math.nan) - report.get('obstacle_state_time_s', math.nan)} s",
                f"Obstacle target vy/yaw: {report.get('obstacle_target_vy_mps')} / {report.get('obstacle_target_yaw_rate_radps')}",
                f"Velocity jump near push: {report.get('max_velocity_jump_near_push_mps')} m/s",
                f"Obstacle contact: {report.get('contact_ok')}",
                "",
                "The push occurs while the physical obstacle response is active; impact must preempt obstacle before its eight-second response window expires.",
                "Event transition times use controller cmd_time_s; physical latency uses MuJoCo state_tick_s.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"strict_pass": report["strict_pass"], "output": str(args.output)}))
    return 0 if report["strict_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
