#!/usr/bin/env python3
"""Strict acceptance for automatic sensing of a physical low-friction patch."""

from __future__ import annotations

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

EVENT_NAMES = {0: "none", 1: "emergency_stop", 2: "obstacle_left",
               3: "obstacle_right", 4: "turn_left", 5: "turn_right",
               6: "slip", 7: "low_friction", 8: "impact"}
SOURCE_NAMES = {0: "none", 1: "scheduled", 2: "sensor", 3: "safety_latch"}
STATUS_KEYS = ("controller_status", "safety_status", "quality_status",
               "analysis_status", "ground_truth_status", "dynamics_status",
               "completion_status")


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def read_rows(path: Path) -> list[dict[str, str]]:
    with (path / "data.csv").open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_metadata(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    with (path / "run_metadata.txt").open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if "=" in line:
                key, value = line.rstrip("\n").split("=", 1)
                result[key] = value
    return result


def transitions(data: list[dict[str, str]]) -> list[dict[str, float | int | str]]:
    result: list[dict[str, float | int | str]] = []
    previous = None
    for row in data:
        code = int(number(row, "event_type", -1))
        event = EVENT_NAMES.get(code, f"unknown_{code}")
        if event == previous:
            continue
        source = int(number(row, "event_source", 0))
        result.append({
            "cmd_time_s": number(row, "cmd_time_s"),
            "state_tick_s": number(row, "state_tick_s"),
            "type": event,
            "priority": int(number(row, "event_priority", 0)),
            "source": source,
            "source_name": SOURCE_NAMES.get(source, "unknown"),
            "world_base_x_m": number(row, "world_base_x_m"),
            "support_low_friction_evidence": number(
                row, "support_low_friction_evidence"),
        })
        previous = event
    return result


def patch_bounds(path: Path) -> tuple[float, float, float, float] | None:
    argv = read_metadata(path).get("argv", "")
    marker = "--scene-file "
    if marker not in argv:
        return None
    scene_text = argv.split(marker, 1)[1].split(" --", 1)[0]
    scene = Path(scene_text)
    if not scene.is_absolute():
        repo_root = Path(__file__).resolve().parents[3]
        scene = next((candidate for candidate in
                      (Path.cwd() / scene, repo_root / scene)
                      if candidate.exists()), scene)
    try:
        root = ET.parse(scene).getroot()
        geom = root.find(".//geom[@name='low_friction_patch']")
        if geom is None:
            return None
        pos = [float(value) for value in geom.attrib["pos"].split()]
        size = [float(value) for value in geom.attrib["size"].split()]
        friction = float(geom.attrib["friction"].split()[0])
        if friction > 0.02:
            return None
        return pos[0] - size[0], pos[0] + size[0], pos[1] - size[1], pos[1] + size[1]
    except (KeyError, OSError, ValueError, ET.ParseError):
        return None


def median(values: list[float]) -> float:
    values = sorted(value for value in values if math.isfinite(value))
    return values[len(values) // 2] if values else math.nan


def analyze(path: Path) -> dict:
    result: dict = {"experiment": str(path), "strict_pass": False}
    if not (path / "data.csv").exists() or not (path / "run_metadata.txt").exists():
        result["failure"] = "missing data.csv or run_metadata.txt"
        return result
    data = read_rows(path)
    meta = read_metadata(path)
    if not data:
        result["failure"] = "empty data.csv"
        return result
    observed = transitions(data)
    low = next((item for item in observed if item["type"] == "low_friction"), None)
    non_none = [item for item in observed if item["type"] != "none"]
    statuses = {key: int(meta.get(key, "-1")) for key in STATUS_KEYS}
    status_ok = all(value == 0 for value in statuses.values())
    bounds = patch_bounds(path)
    simulator_log = (path / "simulator.log").read_text(
        encoding="utf-8", errors="replace")
    no_scripted_friction = ("--friction-time" not in meta.get("argv", "") and
                            "FRICTION config time=-1" in simulator_log)
    scene_ok = bounds is not None and no_scripted_friction
    finite_keys = ("world_base_x_m", "world_base_y_m", "imu_roll_rad",
                   "imu_pitch_rad", "wbc_full_eq_residual",
                   "support_low_friction_evidence")
    finite = all(math.isfinite(number(row, key)) for row in data for key in finite_keys)
    max_roll = max((abs(number(row, "imu_roll_rad")) for row in data), default=math.nan)
    max_pitch = max((abs(number(row, "imu_pitch_rad")) for row in data), default=math.nan)
    max_residual = max((number(row, "wbc_full_eq_residual") for row in data), default=math.nan)
    posture_ok = (math.isfinite(max_roll) and math.isfinite(max_pitch) and
                  max_roll <= 0.25 and max_pitch <= 0.25)
    solver_ok = math.isfinite(max_residual) and max_residual <= 1e-3
    event_source_ok = low is not None and low["source"] == 2
    event_sequence_ok = ([item["type"] for item in observed] ==
                         ["none", "low_friction", "none"])
    event_x_ok = False
    entry_time = math.nan
    event_time = float(low["cmd_time_s"]) if low else math.nan
    if bounds is not None and low is not None:
        x0, x1, _, _ = bounds
        gait_rows = [row for row in data if int(number(row, "motion_stage", -1)) == 2]
        entry = next((row for row in gait_rows
                      if number(row, "world_base_x_m") >= x0), None)
        entry_time = number(entry, "cmd_time_s") if entry else math.nan
        event_x_ok = x0 - 0.10 <= float(low["world_base_x_m"]) <= x1 + 0.10
    pre = [row for row in data if math.isfinite(event_time) and
           event_time - 0.50 <= number(row, "cmd_time_s") < event_time]
    active = [row for row in data if math.isfinite(event_time) and
              event_time <= number(row, "cmd_time_s") < event_time + 0.80]
    pre_vx = median([number(row, "event_ref_vx_mps") for row in pre])
    active_target_vx = median([number(row, "event_target_vx_mps") for row in active])
    response_ok = (math.isfinite(pre_vx) and math.isfinite(active_target_vx) and
                   abs(active_target_vx) <= max(0.06, 0.70 * abs(pre_vx)))
    evidence = max((number(row, "support_low_friction_evidence")
                    for row in data), default=math.nan)
    evidence_ok = math.isfinite(evidence) and evidence >= 0.08
    result.update({
        "rows": len(data), "statuses": statuses,
        "observed_transitions": observed, "patch_bounds_m": bounds,
        "patch_entry_cmd_time_s": entry_time,
        "event_cmd_time_s": event_time,
        "event_detection_after_entry_s": event_time - entry_time
        if math.isfinite(event_time) and math.isfinite(entry_time) else math.nan,
        "max_support_low_friction_evidence": evidence,
        "pre_event_ref_vx_mps": pre_vx,
        "active_target_vx_mps": active_target_vx,
        "max_abs_roll_rad": max_roll, "max_abs_pitch_rad": max_pitch,
        "max_wbc_full_eq_residual": max_residual,
        "scene_ok": scene_ok, "no_scripted_friction": no_scripted_friction,
        "event_source_ok": event_source_ok,
        "event_sequence_ok": event_sequence_ok, "event_x_ok": event_x_ok,
        "evidence_ok": evidence_ok, "response_ok": response_ok,
        "finite_required_columns": finite, "posture_ok": posture_ok,
        "solver_ok": solver_ok, "status_ok": status_ok,
    })
    result["strict_pass"] = all((scene_ok, status_ok, finite, posture_ok,
                                  solver_ok, event_source_ok, event_sequence_ok,
                                  event_x_ok, evidence_ok, response_ok))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.experiment.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    args.output.with_suffix(".md").write_text(
        "# Automatic low-friction patch acceptance\n\n"
        f"Strict pass: {report['strict_pass']}\n\n"
        f"Observed transitions: {report.get('observed_transitions')}\n"
        f"Patch bounds: {report.get('patch_bounds_m')}\n"
        f"Entry / detection: {report.get('patch_entry_cmd_time_s')} / "
        f"{report.get('event_cmd_time_s')} s\n"
        f"Detection delay after entry: {report.get('event_detection_after_entry_s')} s\n"
        f"Max support-foot evidence: {report.get('max_support_low_friction_evidence')}\n"
        f"Pre-event vx / active target vx: {report.get('pre_event_ref_vx_mps')} / "
        f"{report.get('active_target_vx_mps')} m/s\n"
        f"Max |roll| / |pitch|: {report.get('max_abs_roll_rad')} / "
        f"{report.get('max_abs_pitch_rad')} rad\n"
        f"Max WBC residual: {report.get('max_wbc_full_eq_residual')}\n",
        encoding="utf-8")
    print(json.dumps({"strict_pass": report["strict_pass"],
                      "output": str(args.output)}))
    return 0 if report["strict_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
