#!/usr/bin/env python3
"""Analyze B1/B2/B3 terrain runs from controller and harness evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
import shlex
import xml.etree.ElementTree as ET
from pathlib import Path


LEGS = ("FR", "FL", "RR", "RL")
FOOT_RADIUS_M = 0.025


def number(row, key, default=float("nan")):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def percentile(values, q):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return float("nan")
    position = (len(values) - 1) * q
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_metadata(run_dir):
    result = {}
    path = run_dir / "run_metadata.txt"
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            key, separator, value = line.partition("=")
            if separator:
                result[key] = value
    return result


def repo_root(run_dir):
    for parent in (run_dir, *run_dir.parents):
        if (parent / ".git").exists() and (parent / "example" / "cpp").is_dir():
            return parent
    return None


def scene_path_from_metadata(run_dir, metadata, explicit):
    if explicit:
        return explicit.resolve()
    if metadata.get("scene_file"):
        return Path(metadata["scene_file"]).resolve()
    argv = shlex.split(metadata.get("argv", ""))
    if "--scene-file" in argv:
        value = Path(argv[argv.index("--scene-file") + 1])
        if value.is_absolute():
            return value
        root = repo_root(run_dir)
        if root:
            return (root / value).resolve()
    raise SystemExit("scene path is absent; pass --scene")


def vector(text, count):
    values = [float(item) for item in (text or "").split()]
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {text!r}")
    return values


def terrain_boxes(scene):
    root = ET.parse(scene).getroot()
    boxes = []
    for geom in root.findall(".//geom"):
        name = geom.get("name", "")
        if geom.get("type") != "box" or not name.startswith("phase2_step"):
            continue
        position = vector(geom.get("pos"), 3)
        size = vector(geom.get("size"), 3)
        boxes.append({
            "name": name,
            "x_min_m": position[0] - size[0],
            "x_max_m": position[0] + size[0],
            "y_min_m": position[1] - size[1],
            "y_max_m": position[1] + size[1],
            "top_z_m": position[2] + size[2],
        })
    boxes.sort(key=lambda item: item["x_min_m"])
    if not boxes:
        raise SystemExit(f"no phase2_step box found in {scene}")
    return boxes


def mask_population(value):
    return int(value).bit_count() if math.isfinite(value) and value >= 0 else 0


def sustained_surface_contact(rows, leg, box, deadline_s):
    expected_z = box["top_z_m"] + FOOT_RADIUS_M
    intervals = []
    start = previous = None
    peak_force = 0.0
    for row in rows:
        time_s = number(row, "time_s")
        x = number(row, f"{leg}_pos_world_x_m")
        y = number(row, f"{leg}_pos_world_y_m")
        z = number(row, f"{leg}_pos_world_z_m")
        force = number(row, f"{leg}_foot_contact_grf_world_z_N", 0.0)
        inside = (
            time_s <= deadline_s and
            box["x_min_m"] + 0.01 <= x <= box["x_max_m"] - 0.01 and
            box["y_min_m"] + 0.03 <= y <= box["y_max_m"] - 0.03 and
            abs(z - expected_z) <= 0.030 and force >= 5.0
        )
        if inside:
            if start is None or (previous is not None and time_s - previous > 0.006):
                if start is not None:
                    intervals.append((start, previous, peak_force))
                start = time_s
                peak_force = force
            peak_force = max(peak_force, force)
            previous = time_s
        elif start is not None:
            intervals.append((start, previous, peak_force))
            start = previous = None
            peak_force = 0.0
    if start is not None:
        intervals.append((start, previous, peak_force))
    best = max(intervals, key=lambda item: item[1] - item[0], default=None)
    return {
        "pass": bool(best and best[1] - best[0] >= 0.040),
        "first_s": best[0] if best else None,
        "duration_s": best[1] - best[0] if best else 0.0,
        "peak_force_n": best[2] if best else 0.0,
    }


def first_clear_time(rows, final_box, deadline_s):
    clear_x = final_box["x_max_m"] + 0.20
    foot_clear_x = final_box["x_max_m"] + 0.02
    for row in rows:
        time_s = number(row, "time_s")
        if time_s > deadline_s:
            break
        if (number(row, "base_pos_world_x_m") >= clear_x and
                number(row, "base_pos_world_z_m") >= 0.28 and
                all(number(row, f"{leg}_pos_world_x_m") >= foot_clear_x
                    for leg in LEGS)):
            return time_s
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--milestone", choices=("B1", "B2", "B3"), required=True)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    data_path = args.run_dir / "data.csv"
    truth_path = args.run_dir / "contact_ground_truth.csv"
    if not data_path.is_file() or not truth_path.is_file():
        raise SystemExit("data.csv and contact_ground_truth.csv are required")
    data = read_csv(data_path)
    truth = read_csv(truth_path)
    if not data or not truth:
        raise SystemExit("empty controller or ground-truth CSV")
    truth_required = {
        "time_s", "base_pos_world_x_m", "base_pos_world_z_m",
        "phase2_terrain_foot_contact_mask",
        "phase2_terrain_nonfoot_contact_count",
    }
    for leg in LEGS:
        truth_required |= {
            f"{leg}_pos_world_x_m", f"{leg}_pos_world_y_m",
            f"{leg}_pos_world_z_m", f"{leg}_foot_contact_grf_world_z_N",
        }
    truth_missing = sorted(truth_required - set(truth[0]))
    if truth_missing:
        raise SystemExit("missing terrain ground-truth columns: " +
                         ", ".join(truth_missing))
    metadata = read_metadata(args.run_dir)
    scene = scene_path_from_metadata(args.run_dir, metadata, args.scene)
    boxes = terrain_boxes(scene)
    phase1_path = args.run_dir / "phase1_quantitative.json"
    if not phase1_path.is_file():
        raise SystemExit("phase1_quantitative.json is required")
    phase1 = json.loads(phase1_path.read_text(encoding="utf-8"))

    required = {
        "state_tick_s", "motion_stage", "velocity_command_active",
        "velocity_command_shaped_mps", "velocity_command_measured_mps",
        "velocity_command_accel_mps2", "velocity_command_jerk_mps3",
        "imu_roll_rad", "imu_pitch_rad", "world_base_z_m",
        "terrain_map_source", "terrain_plan_status", "terrain_plan_valid",
        "terrain_planner_updates", "terrain_planner_deadline_misses",
        "terrain_solver_elapsed_us", "terrain_safe_stop_requested",
        "terrain_plan_published", "terrain_plan_consumed",
        "terrain_gait_target_overrides", "terrain_mpc_plan_consumed",
        "terrain_min_edge_margin_m", "terrain_min_uncertainty_edge_margin_m",
        "terrain_min_slope_rad", "terrain_max_roughness_m",
        "terrain_min_reachability_margin_m", "terrain_min_swing_clearance_m",
        "terrain_min_support_margin_m",
        "terrain_min_uncertainty_support_margin_m",
        "terrain_support_failure_knot",
        "terrain_support_failure_contact_mask",
        "terrain_support_failure_margin_m",
        "terrain_execution_plan_id", "terrain_execution_map_epoch",
        "terrain_execution_plan_usable",
        "terrain_execution_planned_contact_mask", "terrain_transfer_hold_active",
        "terrain_surface_transition_active",
        "terrain_surface_transition_required_mask",
        "terrain_surface_transition_committed_mask",
        "terrain_surface_transition_completions",
        "terrain_surface_transition_last_required_mask",
        "terrain_surface_transition_last_committed_mask",
        "terrain_target_prepare_attempts", "terrain_target_prepared",
        "terrain_target_prepare_rejections", "wbc_measured_contact_mask",
        "wbc_scheduled_contact_mask", "wbc_terrain_planned_contact_mask",
        "wbc_mpc_update_count", "wbc_mpc_contact_mask_k0",
        "wbc_mpc_min_contact_count", "wbc_mpc_reference_x_first_m",
        "wbc_mpc_reference_x_last_m", "wbc_mpc_reference_vx_first_mps",
        "wbc_mpc_reference_vx_last_mps",
        "wbc_terrain_contact_coherent", "wbc_terrain_plan_id",
        "kernel_footstep_plan_valid", "wbc_full_srbd_ok", "wbc_full_id_ok",
        "wbc_shadow_within_budget", "support_low_friction_evidence",
        "support_foot_speed_mps", "contact_count", "touchdown_x_error_m",
        "touchdown_y_error_m", "wbc_shadow_max_abs_tau",
    }
    for leg in LEGS:
        required |= {
            f"terrain_exec_{leg}_valid", f"terrain_exec_{leg}_in_flight",
            f"terrain_exec_{leg}_measured_touchdown",
            f"terrain_exec_{leg}_wbc_endpoint_error_m",
            f"terrain_exec_{leg}_wbc_at_endpoint",
            f"terrain_exec_{leg}_wbc_measured_contact",
            f"terrain_exec_{leg}_target_required",
            f"terrain_exec_{leg}_target_world_x_m",
            f"terrain_exec_{leg}_target_world_y_m",
            f"terrain_exec_{leg}_target_world_z_m",
            f"terrain_exec_{leg}_foot_world_x_m",
            f"terrain_exec_{leg}_foot_world_y_m",
            f"terrain_exec_{leg}_foot_world_z_m",
            f"terrain_{leg}_candidate_count",
            f"terrain_{leg}_swing_candidate_count",
            f"terrain_{leg}_candidate_required",
            f"terrain_{leg}_touchdown_knot",
        }
    missing = sorted(required - set(data[0]))
    if missing:
        raise SystemExit("missing terrain evidence columns: " + ", ".join(missing))

    active = [row for row in data if number(row, "motion_stage") == 2 and
              number(row, "velocity_command_active") > 0.5]
    if not active:
        raise SystemExit("no active locomotion rows")
    first_active_tick = number(active[0], "state_tick_s")
    last_tick = number(active[-1], "state_tick_s")
    failure_ticks = [number(row, "state_tick_s") for row in active
                     if number(row, "terrain_safe_stop_requested") > 0.5 or
                     abs(math.degrees(number(row, "imu_roll_rad"))) > 15.0 or
                     abs(math.degrees(number(row, "imu_pitch_rad"))) > 15.0 or
                     number(row, "world_base_z_m") < 0.28]
    evidence_deadline_s = min(failure_ticks, default=last_tick)
    safe_active = [row for row in active if number(row, "state_tick_s") <= evidence_deadline_s]

    planner_samples = []
    previous_update = -1
    for row in data:
        update = int(number(row, "terrain_planner_updates", -1))
        if update > previous_update:
            if update > 0:
                planner_samples.append(number(row, "terrain_solver_elapsed_us"))
            previous_update = update
    valid_plans = [row for row in active if number(row, "terrain_plan_status") == 1 and
                   number(row, "terrain_plan_valid") > 0.5]
    execution_rows = [row for row in active if any(
        number(row, f"terrain_exec_{leg}_valid") > 0.5 and
        number(row, f"terrain_exec_{leg}_target_required") > 0.5
        for leg in LEGS)]
    coherent_rows = [row for row in execution_rows if number(row, "wbc_terrain_plan_id") > 0]
    transfer_mpc_rows = []
    previous_mpc_update = -1
    for row in execution_rows:
        update = int(number(row, "wbc_mpc_update_count", -1))
        if ((number(row, "terrain_transfer_hold_active") > 0.5 or
             number(row, "terrain_surface_transition_active") > 0.5) and
                update > previous_mpc_update):
            transfer_mpc_rows.append(row)
        previous_mpc_update = max(previous_mpc_update, update)
    required_rejection_rows = [row for row in active if
        number(row, "terrain_plan_status") == 4 and any(
            number(row, f"terrain_{leg}_candidate_required") > 0.5
            for leg in LEGS)]

    tracking_error = [abs(number(row, "velocity_command_shaped_mps") -
                          number(row, "velocity_command_measured_mps"))
                      for row in active]
    rolls = [abs(math.degrees(number(row, "imu_roll_rad"))) for row in active]
    pitches = [abs(math.degrees(number(row, "imu_pitch_rad"))) for row in active]
    torque_keys = [key for key in data[0] if key.endswith("_tau_ff") or
                   key.endswith("_tau_est")]
    torque_values = [abs(number(row, key)) for row in active for key in torque_keys]
    touchdown_rows = [row for row in active if number(row, "touchdown_event_count", 0) > 0]
    target_touchdown_errors = []
    for row in execution_rows:
        for leg in LEGS:
            if number(row, f"terrain_exec_{leg}_measured_touchdown") <= 0.5:
                continue
            error = math.sqrt(sum((number(row, f"terrain_exec_{leg}_foot_world_{axis}_m") -
                                   number(row, f"terrain_exec_{leg}_target_world_{axis}_m")) ** 2
                                  for axis in ("x", "y", "z")))
            if math.isfinite(error):
                target_touchdown_errors.append(error)

    surface_results = []
    for box in boxes:
        leg_results = {leg: sustained_surface_contact(
            truth, leg, box, evidence_deadline_s) for leg in LEGS}
        surface_results.append({
            **box,
            "legs": leg_results,
            "all_legs_supported": all(item["pass"] for item in leg_results.values()),
        })
    clear_time_s = first_clear_time(truth, boxes[-1], evidence_deadline_s)
    exit_truth = [row for row in truth if clear_time_s is not None and
                  clear_time_s <= number(row, "time_s") <= clear_time_s + 0.50]

    metrics = {
        "active_rows": len(active),
        "active_time_start_s": first_active_tick,
        "active_time_end_s": last_tick,
        "evidence_deadline_s": evidence_deadline_s,
        "roll_abs_p95_deg": percentile(rolls, 0.95),
        "roll_abs_max_deg": max(rolls),
        "pitch_abs_p95_deg": percentile(pitches, 0.95),
        "pitch_abs_max_deg": max(pitches),
        "base_height_min_m": min(number(row, "world_base_z_m") for row in active),
        "shaped_to_measured_abs_p95_mps": percentile(tracking_error, 0.95),
        "shaped_to_measured_abs_max_mps": max(tracking_error),
        "acceleration_abs_max_mps2": max(abs(number(row, "velocity_command_accel_mps2")) for row in active),
        "jerk_abs_max_mps3": max(abs(number(row, "velocity_command_jerk_mps3")) for row in active),
        "contact_loss_fraction": sum(number(row, "contact_count") <= 0 for row in active) / len(active),
        "single_contact_fraction": sum(number(row, "contact_count") <= 1 for row in active) / len(active),
        "slip_evidence_rows": sum(number(row, "support_low_friction_evidence") > 0 for row in active),
        "torque_saturation_fraction": sum(value >= 45.0 for value in torque_values) / len(torque_values),
        "touchdown_x_abs_max_m": max((abs(number(row, "touchdown_x_error_m")) for row in touchdown_rows), default=0.0),
        "touchdown_y_abs_max_m": max((abs(number(row, "touchdown_y_error_m")) for row in touchdown_rows), default=0.0),
        "terrain_touchdown_3d_error_max_m": max(target_touchdown_errors, default=float("nan")),
        "planner_latency_p95_us": percentile(planner_samples, 0.95),
        "planner_latency_max_us": max(planner_samples, default=float("nan")),
        "planner_deadline_misses": max(number(row, "terrain_planner_deadline_misses") for row in data),
        "plan_published": max(number(row, "terrain_plan_published") for row in data),
        "plan_consumed": max(number(row, "terrain_plan_consumed") for row in data),
        "gait_target_overrides": max(number(row, "terrain_gait_target_overrides") for row in data),
        "mpc_plan_consumed": max(number(row, "terrain_mpc_plan_consumed") for row in data),
        "target_prepare_attempts": max(number(row, "terrain_target_prepare_attempts") for row in data),
        "target_prepared": max(number(row, "terrain_target_prepared") for row in data),
        "target_prepare_rejections": max(number(row, "terrain_target_prepare_rejections") for row in data),
        "surface_transition_completions": max(number(
            row, "terrain_surface_transition_completions") for row in data),
        "surface_transition_last_required_mask": int(max(number(
            row, "terrain_surface_transition_last_required_mask") for row in data)),
        "surface_transition_last_committed_mask": int(max(number(
            row, "terrain_surface_transition_last_committed_mask") for row in data)),
        "surface_transition_max_committed_contacts": max(mask_population(number(
            row, "terrain_surface_transition_committed_mask")) for row in data),
        "required_plan_rejection_rows": len(required_rejection_rows),
        "valid_plan_rows": len(valid_plans),
        "execution_rows": len(execution_rows),
        "execution_rows_without_wbc_plan": len(execution_rows) - len(coherent_rows),
        "execution_plan_ids": len({
            int(number(row, "terrain_execution_plan_id"))
            for row in execution_rows
            if number(row, "terrain_execution_plan_id") > 0}),
        "execution_map_epochs": len({
            int(number(row, "terrain_execution_map_epoch"))
            for row in execution_rows
            if number(row, "terrain_execution_map_epoch") > 0}),
        "wbc_plan_coherence_fraction": (
            sum(number(row, "wbc_terrain_contact_coherent") > 0.5 for row in coherent_rows) /
            len(coherent_rows) if coherent_rows else float("nan")),
        "transfer_mpc_samples": len(transfer_mpc_rows),
        "transfer_mpc_min_contacts": min((number(
            row, "wbc_mpc_min_contact_count") for row in transfer_mpc_rows),
            default=float("nan")),
        "transfer_mpc_k0_contact_mismatches": sum(
            int(number(row, "wbc_mpc_contact_mask_k0")) !=
            int(number(row, "wbc_shadow_contact_mask"))
            for row in transfer_mpc_rows),
        "mpc_vcmd_authority_abs_max_mps": max((max(
            abs(number(row, "wbc_mpc_reference_vx_first_mps") -
                number(row, "velocity_command_applied_mps")),
            abs(number(row, "wbc_mpc_reference_vx_last_mps") -
                number(row, "velocity_command_applied_mps")))
            for row in transfer_mpc_rows), default=float("nan")),
        "mpc_horizontal_reference_span_abs_max_m": max((abs(
            number(row, "wbc_mpc_reference_x_last_m") -
            number(row, "wbc_mpc_reference_x_first_m"))
            for row in transfer_mpc_rows), default=float("nan")),
        "clear_time_s": clear_time_s,
        "exit_window_s": (number(exit_truth[-1], "time_s") - number(exit_truth[0], "time_s"))
        if len(exit_truth) >= 2 else 0.0,
        "ground_truth_collision_rows": sum(
            number(row, "phase2_terrain_nonfoot_contact_count") > 0 and
            number(row, "time_s") <= evidence_deadline_s for row in truth),
    }

    plan_edge = [number(row, "terrain_min_edge_margin_m") for row in valid_plans]
    plan_uncertainty_edge = [number(row, "terrain_min_uncertainty_edge_margin_m") for row in valid_plans]
    plan_slope = [number(row, "terrain_min_slope_rad") for row in valid_plans]
    plan_roughness = [number(row, "terrain_max_roughness_m") for row in valid_plans]
    plan_reach = [number(row, "terrain_min_reachability_margin_m") for row in valid_plans]
    plan_clearance = [number(row, "terrain_min_swing_clearance_m") for row in valid_plans]
    plan_support = [number(row, "terrain_min_support_margin_m") for row in valid_plans]
    plan_uncertainty_support = [number(row, "terrain_min_uncertainty_support_margin_m") for row in valid_plans]

    statuses = ("controller_status", "safety_status", "quality_status",
                "analysis_status", "ground_truth_status", "dynamics_status",
                "completion_status")
    checks = {
        "lifecycle_status": all(metadata.get(key) == "0" for key in statuses),
        "clean_source": metadata.get("git_dirty") == "false",
        "phase1_steps_quantitative":
            phase1.get("quantitative_scenario") == "steps" and
            phase1.get("quantitative_pass") is True and
            phase1.get("acceptance_status") == "PASS",
        "lidar_only": all(row.get("terrain_map_source") in ("lidar", "none") for row in active) and
                      any(row.get("terrain_map_source") == "lidar" for row in active),
        "no_safe_stop": all(number(row, "terrain_safe_stop_requested") < 0.5 for row in active),
        "posture_p95": metrics["roll_abs_p95_deg"] <= 4.0 and metrics["pitch_abs_p95_deg"] <= 4.0,
        "posture_hard": metrics["roll_abs_max_deg"] <= 15.0 and metrics["pitch_abs_max_deg"] <= 15.0,
        "base_height": metrics["base_height_min_m"] >= 0.28,
        "velocity_tracking": metrics["shaped_to_measured_abs_p95_mps"] <= 0.45,
        "command_acceleration": metrics["acceleration_abs_max_mps2"] <= 1.25 + 1e-6,
        "command_jerk": metrics["jerk_abs_max_mps3"] <= 4.20 + 1e-6,
        "contact_loss": metrics["contact_loss_fraction"] <= 0.25,
        "single_contact": metrics["single_contact_fraction"] <= 0.45,
        "touchdown_xy": metrics["touchdown_x_abs_max_m"] <= 0.18 and
                        metrics["touchdown_y_abs_max_m"] <= 0.07,
        "terrain_touchdown_error": bool(target_touchdown_errors) and
                                   metrics["terrain_touchdown_3d_error_max_m"] <= 0.075,
        "torque_saturation": metrics["torque_saturation_fraction"] <= 0.003,
        "no_slip_evidence": metrics["slip_evidence_rows"] == 0,
        "model_validity": all(number(row, "kernel_footstep_plan_valid") > 0.5 and
                              number(row, "wbc_full_srbd_ok") > 0.5 and
                              number(row, "wbc_full_id_ok") > 0.5 for row in safe_active),
        "solver_budget": sum(number(row, "wbc_shadow_within_budget") > 0.5 for row in safe_active) /
                         len(safe_active) >= 0.80,
        "planner_latency": metrics["planner_latency_p95_us"] <= 2000.0 and
                           metrics["planner_latency_max_us"] <= 5000.0 and
                           metrics["planner_deadline_misses"] == 0,
        "required_plan_rejections":
            metrics["required_plan_rejection_rows"] == 0,
        "plan_pipeline": metrics["plan_published"] > 0 and metrics["plan_consumed"] > 0 and
                         metrics["gait_target_overrides"] > 0 and
                         metrics["mpc_plan_consumed"] > 0 and metrics["target_prepared"] > 0,
        "surface_transition_transaction":
            metrics["surface_transition_completions"] > 0 and
            metrics["surface_transition_last_required_mask"] > 0 and
            metrics["surface_transition_last_required_mask"] ==
                metrics["surface_transition_last_committed_mask"],
        "plan_geometry": bool(valid_plans) and min(plan_edge) >= 0.040 and
                         min(plan_uncertainty_edge) >= 0.0 and
                         max(plan_slope) <= math.radians(20.0) and
                         max(plan_roughness) <= 0.025 and min(plan_reach) >= 0.010 and
                         min(plan_clearance) >= 0.0,
        "plan_support": bool(valid_plans) and min(plan_support) >= 0.015 and
                        min(plan_uncertainty_support) >= 0.0 and
                        all(mask_population(number(row, "terrain_execution_planned_contact_mask")) != 1
                            for row in execution_rows),
        "wbc_plan_coherence": bool(coherent_rows) and
                              len(coherent_rows) == len(execution_rows) and
                              metrics["wbc_plan_coherence_fraction"] == 1.0,
        "transfer_mpc_support": metrics["transfer_mpc_samples"] > 0 and
                                metrics["transfer_mpc_min_contacts"] >= 2 and
                                metrics["transfer_mpc_k0_contact_mismatches"] == 0,
        "single_vcmd_authority": metrics["transfer_mpc_samples"] > 0 and
                                 metrics["mpc_vcmd_authority_abs_max_mps"] <= 0.020 and
                                 metrics["mpc_horizontal_reference_span_abs_max_m"] <= 0.001,
        "ground_truth_surface_support": all(item["all_legs_supported"] for item in surface_results),
        "ground_truth_collision": metrics["ground_truth_collision_rows"] == 0,
        "body_and_all_legs_clear": clear_time_s is not None,
        "stable_exit": metrics["exit_window_s"] >= 0.45,
        "b3_plan_replacement": args.milestone != "B3" or (
            metrics["execution_plan_ids"] >= 2 and
            metrics["execution_map_epochs"] >= 2),
    }

    result = {
        "contract": "phase2-b123-v1",
        "milestone": args.milestone,
        "run_dir": str(args.run_dir),
        "scene": str(scene),
        "git_head": metadata.get("git_head", ""),
        "metrics": metrics,
        "surface_support": surface_results,
        "phase1_quantitative": phase1,
        "checks": checks,
    }
    result["acceptance_status"] = "PASS" if all(checks.values()) else "FAIL"
    output = json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n"
    print(output, end="")
    if args.json_out:
        args.json_out.write_text(output, encoding="utf-8")
    return 0 if result["acceptance_status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
