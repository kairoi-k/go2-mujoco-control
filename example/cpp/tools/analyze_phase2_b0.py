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


def manifest(path: Path):
    file = path / "run_manifest.json"
    if not file.exists():
        return {}
    try:
        return json.loads(file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def normalized_argv(value):
    result = []
    skip_next = False
    for item in value or []:
        if skip_next:
            skip_next = False
            continue
        if item in ("--terrain-sensor-only", "--terrain-planner"):
            continue
        if item == "--domain-id":
            skip_next = True
            continue
        result.append(item)
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
            "diagnostic_pass": bool(diffs) and max(diffs) <= tolerance,
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--fixed-3mps", action="store_true")
    parser.add_argument("--fixed-analyzer-output", type=Path)
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
        "terrain_plan_published", "terrain_plan_consumed",
        "terrain_gait_target_overrides", "terrain_mpc_plan_consumed",
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
    checks["terrain_enabled"] = all(number(row, "terrain_enabled") > 0.5
                                     for row in active)
    checks["lidar_provenance"] = all(row.get("terrain_map_source") in ("lidar", "none")
                                     for row in active)
    lidar_rows = sum(row.get("terrain_map_source") == "lidar" for row in active)
    checks["lidar_observation"] = lidar_rows / len(active) >= 0.80
    map_valid_rows = sum(number(row, "terrain_map_valid") > 0.5
                         for row in active)
    checks["map_telemetry"] = map_valid_rows / len(active) >= 0.80
    checks["finite_map_age"] = all(
        not math.isfinite(number(row, "terrain_map_age_s")) or
        number(row, "terrain_map_age_s") >= 0.0 for row in active)
    checks["planner_does_not_safe_stop"] = all(
        number(row, "terrain_safe_stop_requested") < 0.5 for row in active)
    checks["terrain_cap_not_requested"] = all(
        math.isinf(number(row, "terrain_velocity_cap_mps")) and
        number(row, "terrain_velocity_cap_mps") > 0.0 for row in active)
    checks["no_plan_publish"] = all(
        number(row, "terrain_plan_published") < 0.5 for row in active)
    checks["no_plan_consumer"] = all(
        number(row, "terrain_plan_consumed") < 0.5 and
        number(row, "terrain_gait_target_overrides") < 0.5 and
        number(row, "terrain_mpc_plan_consumed") < 0.5 for row in active)
    checks["planner_deadline"] = all(
        math.isfinite(number(row, "terrain_solver_elapsed_us")) and
        number(row, "terrain_solver_elapsed_us") <= 5000.0 and
        number(row, "terrain_planner_deadline_misses") < 0.5 for row in active)
    checks["planner_updated"] = max(
        (number(row, "terrain_planner_updates") for row in active),
        default=0.0) > 0.0
    phase1 = {}
    phase1_path = args.run_dir / "phase1_quantitative.json"
    if phase1_path.exists():
        try:
            phase1 = json.loads(phase1_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            phase1 = {}
    checks["phase1_quantitative"] = True if args.fixed_3mps else (
        phase1.get("acceptance_status") == "PASS" and
        phase1.get("strict_pass") is True and
        phase1.get("quantitative_pass") is True)
    run_manifest = manifest(args.run_dir)
    artifact_manifest = run_manifest.get("artifacts", {})
    contract_path = args.run_dir.parents[4] / "docs" / "research" / \
        "PHASE2_B0_ACCEPTANCE_CONTRACT.md"
    analyzer_path = args.run_dir.parents[4] / "example" / "cpp" / \
        "tools" / "analyze_phase2_b0.py"
    checks["contract_hash"] = (
        artifact_manifest.get("phase2_b0_contract_sha256") ==
        sha256(contract_path) if contract_path.is_file() else False)
    checks["analyzer_hash"] = (
        artifact_manifest.get("phase2_b0_analyzer_sha256") ==
        sha256(analyzer_path) if analyzer_path.is_file() else False)
    fixed_output = args.fixed_analyzer_output or (
        args.run_dir / "sustained_running_analysis.txt")
    fixed_analyzer_path = args.run_dir.parents[4] / "example" / "cpp" / \
        "tools" / "analysis" / "analyze_sustained_running.py"
    if args.fixed_3mps:
        fixed_text = fixed_output.read_text(encoding="utf-8", errors="replace") \
            if fixed_output.is_file() else ""
        checks["fixed_3mps_analyzer"] = "validation=PASS" in fixed_text
        checks["fixed_analyzer_hash"] = (
            artifact_manifest.get("phase2_fixed_3mps_analyzer_sha256") ==
            sha256(fixed_analyzer_path)
            if fixed_analyzer_path.is_file() else False)
    else:
        checks["fixed_3mps_analyzer"] = True
        checks["fixed_analyzer_hash"] = True
    comparison = None
    paired_contract = {}
    baseline_phase1 = {}
    if args.baseline:
        baseline = rows(args.baseline)
        comparison = paired_differences(active, baseline)
        active_manifest = manifest(args.run_dir)
        baseline_manifest = manifest(args.baseline)
        baseline_metadata = metadata(args.baseline)
        baseline_phase1_path = args.baseline / "phase1_quantitative.json"
        if baseline_phase1_path.exists():
            try:
                baseline_phase1 = json.loads(
                    baseline_phase1_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                baseline_phase1 = {}
        active_argv = normalized_argv(active_manifest.get("effective_argv"))
        baseline_argv = normalized_argv(baseline_manifest.get("effective_argv"))
        paired_contract = {
            "same_nonterrain_argv": active_argv == baseline_argv,
            "same_controller_sha256": active_manifest.get("artifacts", {}).get(
                "controller_sha256") == baseline_manifest.get("artifacts", {}).get(
                "controller_sha256"),
            "same_simulator_sha256": active_manifest.get("artifacts", {}).get(
                "simulator_sha256") == baseline_manifest.get("artifacts", {}).get(
                "simulator_sha256"),
            "same_scene_sha256": active_manifest.get("artifacts", {}).get(
                "scenario_sha256") == baseline_manifest.get("artifacts", {}).get(
                "scenario_sha256"),
            "same_profile_sha256": active_manifest.get("profile", {}).get(
                "sha256") == baseline_manifest.get("profile", {}).get("sha256"),
        }
        checks["paired_control_interface"] = bool(baseline_manifest) and all(
            paired_contract.values())
        checks["paired_baseline_lifecycle"] = bool(baseline_metadata) and all(
            baseline_metadata.get(key) == "0" for key in (
                "controller_status", "safety_status", "quality_status",
                "analysis_status", "completion_status", "dynamics_status"))
    else:
        checks["paired_control_interface"] = False
    result = {
        "contract": "b0-contract-v1.2",
        "run_dir": str(args.run_dir),
        "git_head": md.get("git_head", ""),
        "controller_status": md.get("controller_status", ""),
        "safety_status": md.get("safety_status", ""),
        "quality_status": md.get("quality_status", ""),
        "analysis_status": md.get("analysis_status", ""),
        "completion_status": md.get("completion_status", ""),
        "dynamics_status": md.get("dynamics_status", ""),
        "terrain_rows": len(active),
        "terrain_map_valid_fraction": fraction(
            [number(row, "terrain_map_valid") for row in active], lambda x: x > 0.5),
        "terrain_planner_updates": max(
            (number(row, "terrain_planner_updates") for row in active), default=float("nan")),
        "terrain_planner_deadline_misses": max(
            (number(row, "terrain_planner_deadline_misses") for row in active), default=float("nan")),
        "checks": checks,
        "paired_contract": paired_contract,
        "paired_comparison": comparison,
        "paired_baseline_phase1": baseline_phase1,
        "phase1_quantitative": phase1,
    }
    result["legacy_status_pass"] = all(result[key] == "0" for key in (
        "controller_status", "safety_status", "quality_status",
        "analysis_status", "completion_status", "dynamics_status"))
    result["acceptance_status"] = "PASS" if result["legacy_status_pass"] and all(checks.values()) else "FAIL"
    output = json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n"
    print(output, end="")
    if args.json_out:
        args.json_out.write_text(output, encoding="utf-8")
    raise SystemExit(0 if result["acceptance_status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
