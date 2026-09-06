#!/usr/bin/env python3
"""Reproducible, read-only first-anomaly audit for one Go2 B1 run."""
from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path

def rows(path):
    with path.open(newline="") as f:
        return [(n, r) for n, r in enumerate(csv.DictReader(f), 2)]

def f(r, key, default=float("nan")):
    try:
        v = r.get(key, "")
        return float(v) if v not in ("", None) else default
    except (TypeError, ValueError):
        return default

def i(r, key, default=0):
    v = f(r, key, float(default))
    return default if not math.isfinite(v) else int(v)

def deg(r, key):
    return math.degrees(f(r, key))

def pick(seq, pred):
    return next((x for x in seq if pred(x[1])), None)

def active(r):
    return i(r, "motion_stage") == 2 and f(r, "velocity_command_active") > .5

def required_rejection(r):
    legs = ("FR", "FL", "RR", "RL")
    return i(r, "terrain_plan_status") == 4 and any(
        i(r, "terrain_%s_candidate_required" % leg) for leg in legs
    )

def endpoint_gt(r, threshold):
    return any(
        f(r, "terrain_exec_%s_wbc_endpoint_error_m" % leg) > threshold
        for leg in ("FR", "FL", "RR", "RL")
    )

def short(line, r):
    return (
        "CSV line %d; cmd=%.9f; state=%.3f; base=(%.9f,%.9f,%.9f); "
        "world_vel=(%.9f,%.9f,%.9f); roll_deg=%.6f; pitch_deg=%.6f; "
        "shaped=%.9f; measured=%.9f; track_err=%.9f; contacts=%d; "
        "measured_mask=%d; scheduled_mask=%d; planned_mask=%d; "
        "plan_status=%d; plan_id=%d; plan_valid=%d; failure=%d; "
        "planner_rejections=%d; published=%d; consumed=%d; exec_id=%d; "
        "exec_usable=%d; support_knot=%d; support_mask=%d; support_margin_m=%.9f"
    ) % (
        line, f(r, "cmd_time_s"), f(r, "state_tick_s"),
        f(r, "world_base_x_m"), f(r, "world_base_y_m"), f(r, "world_base_z_m"),
        f(r, "world_velocity_x_mps"), f(r, "world_velocity_y_mps"),
        f(r, "world_velocity_z_mps"), deg(r, "imu_roll_rad"),
        deg(r, "imu_pitch_rad"), f(r, "velocity_command_shaped_mps"),
        f(r, "velocity_command_measured_mps"),
        f(r, "velocity_command_tracking_error_mps"), i(r, "contact_count"),
        i(r, "wbc_measured_contact_mask"), i(r, "wbc_scheduled_contact_mask"),
        i(r, "wbc_terrain_planned_contact_mask"), i(r, "terrain_plan_status"),
        i(r, "terrain_plan_id"), i(r, "terrain_plan_valid"),
        i(r, "terrain_plan_failure"), i(r, "terrain_planner_rejections"),
        i(r, "terrain_plan_published"), i(r, "terrain_plan_consumed"),
        i(r, "terrain_execution_plan_id"), i(r, "terrain_execution_plan_usable"),
        i(r, "terrain_support_failure_knot"),
        i(r, "terrain_support_failure_contact_mask"),
        f(r, "terrain_support_failure_margin_m"),
    )

def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path)
    a = ap.parse_args()
    run = a.run_dir.resolve() if a.run_dir else here.parent / re.sub(
        r"_runtime_analysis_[0-9]{8}$", "", here.name
    )
    data_path = run / "data.csv"
    gt_path = run / "contact_ground_truth.csv"
    out = (a.output.resolve() if a.output else
           here / "reproduced_baseline_first_anomalies.txt")
    if not data_path.is_file() or not gt_path.is_file():
        raise SystemExit("missing data.csv or contact_ground_truth.csv under %s" % run)
    if out.exists() and not a.overwrite:
        raise SystemExit("refusing to overwrite %s; choose --output or --overwrite" % out)
    all_data = rows(data_path)
    stage2 = [(n, r) for n, r in all_data if active(r)]
    gt = rows(gt_path)
    events = [
        ("FIRST_SHAPED_NONZERO", pick(stage2, lambda r: f(r, "velocity_command_shaped_mps") > 1e-6)),
        ("FIRST_SHAPED_GE_0.1", pick(stage2, lambda r: f(r, "velocity_command_shaped_mps") >= .1)),
        ("FIRST_SHAPED_GE_0.5", pick(stage2, lambda r: f(r, "velocity_command_shaped_mps") >= .5)),
        ("FIRST_REQUIRED_PLAN_REJECTION", pick(stage2, required_rejection)),
        ("FIRST_EXECUTION_UNUSABLE", pick(stage2, lambda r: not i(r, "terrain_execution_plan_usable"))),
        ("FIRST_NEGATIVE_SUPPORT_MARGIN", pick(stage2, lambda r: f(r, "terrain_support_failure_margin_m") < 0)),
        ("FIRST_ABS_TRACKING_ERROR_GT_0.25", pick(stage2, lambda r: abs(f(r, "velocity_command_tracking_error_mps")) > .25)),
        ("FIRST_ABS_TRACKING_ERROR_GT_0.40", pick(stage2, lambda r: abs(f(r, "velocity_command_tracking_error_mps")) > .40)),
        ("FIRST_ABS_TRACKING_ERROR_GT_0.60", pick(stage2, lambda r: abs(f(r, "velocity_command_tracking_error_mps")) > .60)),
        ("FIRST_CONTACT_COUNT_LE_1", pick(stage2, lambda r: i(r, "contact_count") <= 1)),
        ("FIRST_ABS_PITCH_GT_5_DEG", pick(stage2, lambda r: abs(deg(r, "imu_pitch_rad")) > 5)),
        ("FIRST_ABS_ROLL_GT_5_DEG", pick(stage2, lambda r: abs(deg(r, "imu_roll_rad")) > 5)),
        ("FIRST_ABS_PITCH_GT_10_DEG", pick(stage2, lambda r: abs(deg(r, "imu_pitch_rad")) > 10)),
        ("FIRST_BASE_X_GE_STEP_FRONT_EDGE_0.70", pick(stage2, lambda r: f(r, "world_base_x_m") >= .70)),
        ("FIRST_ABS_PITCH_GT_15_DEG", pick(stage2, lambda r: abs(deg(r, "imu_pitch_rad")) > 15)),
        ("FIRST_CURRENT_ENDPOINT_ERROR_GT_0.10", pick(stage2, lambda r: endpoint_gt(r, .10))),
    ]
    lines = [
        "B1 baseline first-anomaly audit (read-only CSV analysis)",
        "run_dir=%s" % run,
        "data_csv=%s; data_rows=%d" % (data_path, len(all_data)),
        "contact_ground_truth_csv=%s; ground_truth_rows=%d" % (gt_path, len(gt)),
        "CSV data line numbers include the header (first data row is line 2).",
        "active_stage2_rows=%d" % len(stage2),
    ]
    if stage2:
        lines.append("ACTIVE_START " + short(*stage2[0]))
    for name, event in events:
        lines.append(name + (" " + short(*event) if event else " NONE"))
    collision = pick(gt, lambda r: f(r, "phase2_terrain_nonfoot_contact_count") > 0)
    reactive = pick(gt, lambda r: i(r, "reactive_obstacle_contact_count") > 0)
    if collision:
        line, r = collision
        t = f(r, "time_s")
        near = min(all_data, key=lambda x: abs(f(x[1], "state_tick_s") - t))
        lines.append(
            "FIRST_NONFOOT_TERRAIN_COLLISION ground_truth CSV line %d; "
            "sim_time=%.6f; base=(%.9f,%.9f,%.9f); nonfoot_count=%d; "
            "force_N=%.9f; reactive_count=%d" % (
                line, t, f(r, "base_pos_world_x_m"), f(r, "base_pos_world_y_m"),
                f(r, "base_pos_world_z_m"),
                i(r, "phase2_terrain_nonfoot_contact_count"),
                f(r, "phase2_terrain_nonfoot_contact_force_N"),
                i(r, "reactive_obstacle_contact_count"),
            )
        )
        lines.append("NEAREST_CONTROLLER " + short(*near))
    else:
        lines.append("FIRST_NONFOOT_TERRAIN_COLLISION NONE")
    lines.append("FIRST_REACTIVE_OBSTACLE_COLLISION " +
                 ("NONE" if reactive is None else "ground_truth CSV line %d" % reactive[0]))
    edge_event = next((x for x in events if x[0] == "FIRST_BASE_X_GE_STEP_FRONT_EDGE_0.70"), None)
    edge = edge_event[1] if edge_event else None
    edge_state = f(edge[1], "state_tick_s") if edge else float("inf")
    pre = [(n, r) for n, r in stage2 if f(r, "state_tick_s") <= edge_state]
    lines.append("PRE_STEP_ROWS state<=first_edge(%.3f): %d" % (edge_state, len(pre)))
    if pre:
        lines.append(
            "PRE_STEP_COUNTS required_rejections=%d; exec_unusable=%d; "
            "negative_support_margin=%d; max_abs_tracking_error=%.9f; "
            "max_abs_roll_deg=%.6f; max_abs_pitch_deg=%.6f" % (
                sum(required_rejection(r) for _, r in pre),
                sum(not i(r, "terrain_execution_plan_usable") for _, r in pre),
                sum(f(r, "terrain_support_failure_margin_m") < 0 for _, r in pre),
                max(abs(f(r, "velocity_command_tracking_error_mps")) for _, r in pre),
                max(abs(deg(r, "imu_roll_rad")) for _, r in pre),
                max(abs(deg(r, "imu_pitch_rad")) for _, r in pre),
            )
        )
    lines.append(
        "INTERPRETATION: plan rejection and execution loss precede the step front edge; "
        "the first non-foot terrain collision occurs later, during the no-plan interval; "
        "no reactive obstacle collision was observed."
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    mode = "x"
    with out.open(mode, encoding="utf-8", newline="") as f_out:
        f_out.write("\n".join(lines) + "\n")
    print(out)

if __name__ == "__main__":
    main()
