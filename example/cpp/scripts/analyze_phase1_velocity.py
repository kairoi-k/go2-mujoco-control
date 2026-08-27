#!/usr/bin/env python3
"""Report runtime velocity-command evidence without changing legacy gates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

REQUIRED = {
    "cmd_time_s", "velocity_command_requested_mps",
    "velocity_command_shaped_mps", "velocity_command_applied_mps",
    "velocity_command_measured_mps", "velocity_command_tracking_error_mps",
    "velocity_command_accel_mps2", "velocity_command_jerk_mps3",
    "velocity_command_gait_period_s", "velocity_command_gait_duty",
    "velocity_command_gait_step_length_m", "velocity_command_gait_foot_lift_m",
    "velocity_command_gait_regime", "body_velocity_x_mps",
    "velocity_command_active", "imu_roll_rad", "imu_pitch_rad",
    "world_base_z_m", "support_foot_count", "support_foot_speed_mps",
    "support_low_friction_evidence", "contact_count", "touchdown_event_count",
    "touchdown_x_error_m", "touchdown_y_error_m", "kernel_footstep_plan_valid",
    "wbc_shadow_solver_ok", "wbc_shadow_within_budget", "wbc_full_srbd_ok",
    "wbc_full_id_ok", "wbc_shadow_max_abs_tau", "wbc_shadow_feedforward_max_abs_tau",
}


def value(row, key, default=float("nan")):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def finite(rows, key):
    return [x for x in (value(row, key) for row in rows) if math.isfinite(x)]


def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * q
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def profile_sample(points, time_s):
    if time_s <= points[0][0]:
        return points[0][1]
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        if time_s <= t1:
            u = (time_s - t0) / max(1e-9, t1 - t0)
            return v0 + u * (v1 - v0)
    return points[-1][1]


def read_profile(path):
    points = []
    for raw in Path(path).read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            t, v = (float(x.strip()) for x in line.replace(",", " ").split()[:2])
            points.append((t, v))
    if not points:
        raise SystemExit("empty velocity profile")
    return points


def metadata_for(run_dir):
    metadata = {}
    path = run_dir / "run_metadata.txt"
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line:
                key, item = line.split("=", 1)
                metadata[key] = item
    return metadata


def transition_metrics(rows, profile, active_start):
    """Measure step transitions only on constant-command profile segments."""
    records = []
    end_time = value(rows[-1], "cmd_time_s") - active_start
    for index in range(1, len(profile)):
        t0, target = profile[index]
        previous = profile[index - 1][1]
        t1 = profile[index + 1][0] if index + 1 < len(profile) else end_time
        if abs(target - previous) < 1e-9 or t1 - t0 < 1.0:
            continue
        window = [row for row in rows if t0 <= value(row, "cmd_time_s") - active_start <= t1]
        if not window:
            continue
        errors = [value(row, "velocity_command_measured_mps") - target for row in window]
        excursion = max(errors) if target > previous else min(errors)
        tolerance = max(0.15, 0.05 * max(abs(target), 1.0))
        settling = float("nan")
        for candidate in window:
            candidate_time = value(candidate, "cmd_time_s") - active_start
            tail = [row for row in window if candidate_time <= value(row, "cmd_time_s") - active_start <= candidate_time + 1.0]
            if tail and max(abs(value(row, "velocity_command_measured_mps") - target) for row in tail) <= tolerance:
                settling = candidate_time - t0
                break
        records.append({"time_s": t0, "from_mps": previous, "to_mps": target,
                        "excursion_mps": excursion, "settling_time_s": settling,
                        "settling_tolerance_mps": tolerance})
    return records


def command_quality(rows, profile, active_start):
    samples = []
    for row in rows:
        t = value(row, "cmd_time_s") - active_start
        target = profile_sample(profile, t)
        measured = value(row, "velocity_command_measured_mps")
        if math.isfinite(target) and math.isfinite(measured):
            samples.append((t, target, measured))
    error = [measured - target for _, target, measured in samples]
    plateaus = []
    for index, (start, target) in enumerate(profile):
        stop = profile[index + 1][0] if index + 1 < len(profile) else None
        if stop is None or abs(target - profile[index + 1][1]) > 1e-9:
            continue
        section = [abs(measured - target) for t, _, measured in samples if start <= t <= stop]
        steady = [abs(measured - target) for t, _, measured in samples if max(start, stop - 2.0) <= t <= stop]
        if section:
            plateaus.append({"start_s": start, "end_s": stop, "target_mps": target,
                             "mean_abs_error_mps": sum(section) / len(section),
                             "steady_state_mean_abs_error_mps": sum(steady) / len(steady) if steady else float("nan"),
                             "steady_state_max_abs_error_mps": max(steady) if steady else float("nan")})
    transitions = transition_metrics(rows, profile, active_start)
    settling = [item["settling_time_s"] for item in transitions if math.isfinite(item["settling_time_s"])]
    return {
        "tracking_to_profile_error_abs_max_mps": max(map(abs, error)),
        "tracking_to_profile_error_abs_mean_mps": sum(map(abs, error)) / len(error),
        "tracking_to_profile_error_abs_p95_mps": percentile(list(map(abs, error)), 0.95),
        "tracking_to_profile_signed_bias_mps": sum(error) / len(error),
        "steady_state_plateaus": plateaus,
        "steady_state_error_abs_max_mps": max((item["steady_state_max_abs_error_mps"] for item in plateaus), default=float("nan")),
        "overshoot_excursion_max_mps": max((item["excursion_mps"] for item in transitions), default=float("nan")),
        "overshoot_excursion_min_mps": min((item["excursion_mps"] for item in transitions), default=float("nan")),
        "transitions": transitions,
        "settling_time_max_s": max(settling, default=float("nan")),
        "settling_time_mean_s": sum(settling) / len(settling) if settling else float("nan"),
        "settling_evaluated_transitions": len(transitions),
    }


def diagnostic_metrics(rows, evaluation_rows, metadata):
    def fraction(key, predicate):
        values = finite(evaluation_rows, key)
        return sum(predicate(item) for item in values) / len(values) if values else float("nan")

    time = finite(evaluation_rows, "cmd_time_s")
    torque_keys = [key for key in rows[0] if key.endswith("_tau_ff") or key.endswith("_tau_est")]
    torque_keys += ["wbc_shadow_max_abs_tau", "wbc_shadow_feedforward_max_abs_tau"]
    torque_values = [abs(x) for key in torque_keys for x in finite(evaluation_rows, key)]
    tau_limit = 45.0
    match = re.search(r"--tau-limit\s+([0-9.]+)", metadata.get("argv", ""))
    if match:
        tau_limit = float(match.group(1))
    touchdowns = [row for row in evaluation_rows if value(row, "touchdown_event_count") > 0]
    touchdown_x = [abs(value(row, "touchdown_x_error_m")) for row in touchdowns]
    touchdown_y = [abs(value(row, "touchdown_y_error_m")) for row in touchdowns]
    result = {
        "roll_abs_p95_deg": math.degrees(percentile([abs(x) for x in finite(evaluation_rows, "imu_roll_rad")], 0.95)),
        "pitch_abs_p95_deg": math.degrees(percentile([abs(x) for x in finite(evaluation_rows, "imu_pitch_rad")], 0.95)),
        "base_height_min_m": min(finite(evaluation_rows, "world_base_z_m"), default=float("nan")),
        "contact_count_min": min(finite(evaluation_rows, "contact_count"), default=float("nan")),
        "contact_loss_fraction": fraction("contact_count", lambda x: x <= 0),
        "single_contact_fraction": fraction("contact_count", lambda x: x <= 1),
        "support_count_min": min(finite(evaluation_rows, "support_foot_count"), default=float("nan")),
        "slip_indicator_max_mps": max(finite(evaluation_rows, "support_foot_speed_mps"), default=float("nan")),
        "slip_evidence_fraction": fraction("support_low_friction_evidence", lambda x: x > 0),
        "touchdown_events": sum(max(0.0, value(row, "touchdown_event_count")) for row in touchdowns),
        "touchdown_x_error_abs_max_m": max(touchdown_x, default=float("nan")),
        "touchdown_y_error_abs_max_m": max(touchdown_y, default=float("nan")),
        "torque_limit_magnitude_nm": tau_limit,
        "torque_abs_max_nm": max(torque_values, default=float("nan")),
        "torque_saturation_fraction": sum(x >= 0.98 * tau_limit for x in torque_values) / len(torque_values) if torque_values else float("nan"),
        "solver_ok_fraction": fraction("wbc_shadow_solver_ok", lambda x: x > 0.5),
        "solver_budget_ok_fraction": fraction("wbc_shadow_within_budget", lambda x: x > 0.5),
        "srbd_ok_fraction": fraction("wbc_full_srbd_ok", lambda x: x > 0.5),
        "id_wbc_ok_fraction": fraction("wbc_full_id_ok", lambda x: x > 0.5),
        "footstep_plan_valid_fraction": fraction("kernel_footstep_plan_valid", lambda x: x > 0.5),
        "duration_s": max(time) - min(time) if time else float("nan"),
    }
    stop = [abs(value(row, "velocity_command_measured_mps")) for row in evaluation_rows if time and value(row, "cmd_time_s") >= max(time) - 2.0]
    result["stop_tail_speed_abs_p95_mps"] = percentile(stop, 0.95)
    result["stop_tail_speed_abs_final_mps"] = stop[-1] if stop else float("nan")
    return result


QUANTITATIVE_LIMITS = {
    "steps": {"tracking_p95": 0.40, "steady_state": 0.40,
              "overshoot": 0.50, "undershoot": -0.25, "settling": 8.2},
    "accel_1_to_3": {"tracking_p95": 0.42, "steady_state": 0.40,
                     "overshoot": 0.50, "undershoot": -0.25, "settling": 10.0},
    "brake_3_to_0": {"tracking_p95": 1.50, "steady_state": 0.55,
                     "overshoot": 0.05, "undershoot": -0.20, "settling": 1.5},
    "ramp": {"tracking_p95": 0.42, "steady_state": 0.18,
             "overshoot": 0.60, "undershoot": -0.20, "settling": 2.0},
    "varying": {"tracking_p95": 0.42, "steady_state": 0.45,
                "overshoot": 0.50, "undershoot": -0.40, "settling": 8.2},
}


def quantitative_acceptance(result, scenario):
    limits = dict(QUANTITATIVE_LIMITS[scenario])
    checks = {}

    def upper(name, key, limit):
        actual = result.get(key, float("nan"))
        checks[name] = math.isfinite(actual) and actual <= limit

    def lower(name, key, limit):
        actual = result.get(key, float("nan"))
        checks[name] = math.isfinite(actual) and actual >= limit

    upper("requested_profile_reproduction", "requested_profile_max_error_mps", 1.0e-6)
    upper("shaped_to_measured_p95", "shaped_measured_error_abs_p95_mps", 0.45)
    upper("shaper_accel", "accel_abs_max_mps2", 1.25)
    upper("shaper_jerk", "jerk_abs_max_mps3", 4.20)
    upper("shaper_accel_continuity", "accel_step_abs_max_mps3", 0.02)
    upper("tracking_p95", "tracking_to_profile_error_abs_p95_mps", limits["tracking_p95"])
    upper("steady_state_error", "steady_state_error_abs_max_mps", limits["steady_state"])
    upper("overshoot", "overshoot_excursion_max_mps", limits["overshoot"])
    lower("undershoot", "overshoot_excursion_min_mps", limits["undershoot"])
    upper("settling", "settling_time_max_s", limits["settling"])
    upper("roll_p95", "roll_abs_p95_deg", 4.0)
    upper("pitch_p95", "pitch_abs_p95_deg", 4.0)
    upper("contact_loss", "contact_loss_fraction", 0.25)
    upper("single_contact", "single_contact_fraction", 0.45)
    upper("touchdown_x", "touchdown_x_error_abs_max_m", 0.18)
    upper("touchdown_y", "touchdown_y_error_abs_max_m", 0.07)
    upper("torque_saturation", "torque_saturation_fraction", 0.003)
    upper("slip_evidence", "slip_evidence_fraction", 0.0)
    lower("solver_ok", "solver_ok_fraction", 1.0)
    lower("srbd_ok", "srbd_ok_fraction", 1.0)
    lower("id_wbc_ok", "id_wbc_ok_fraction", 1.0)
    lower("footstep_plan_valid", "footstep_plan_valid_fraction", 1.0)
    lower("solver_budget", "solver_budget_ok_fraction", 0.80)
    lower("base_height", "base_height_min_m", 0.28)
    if scenario != "accel_1_to_3":
        upper("stop_tail", "stop_tail_speed_abs_p95_mps", 0.05)
    return limits, checks

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--require-quantitative", action="store_true")
    args = parser.parse_args()
    with (args.run_dir / "data.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("empty diagnostics CSV")
    missing = sorted(REQUIRED - set(rows[0]))
    if missing:
        raise SystemExit("missing diagnostics columns: " + ", ".join(missing))
    evaluation_rows = [row for row in rows if value(row, "velocity_command_active") > 0.5 and row["velocity_command_gait_regime"] == "continuous-trot"]
    if not evaluation_rows:
        raise SystemExit("no continuous-trot evaluation rows")
    scenario = args.profile.stem.removeprefix("phase1_velocity_")
    phase2_profile_scenarios = {
        "phase2_b1_velocity_0p3": "steps",
        "phase2_b2_velocity_0p15": "steps",
    }
    scenario = phase2_profile_scenarios.get(scenario, scenario)
    profile = read_profile(args.profile)
    active_start = value(evaluation_rows[0], "cmd_time_s")
    profile_error = max(abs(value(row, "velocity_command_requested_mps") - profile_sample(profile, value(row, "cmd_time_s") - active_start)) for row in evaluation_rows)
    requested_shaped = [abs(value(row, "velocity_command_requested_mps") - value(row, "velocity_command_shaped_mps")) for row in evaluation_rows]
    applied_gap = [abs(value(row, "velocity_command_shaped_mps") - value(row, "velocity_command_applied_mps")) for row in evaluation_rows]
    shaped_measured = [abs(value(row, "velocity_command_shaped_mps") - value(row, "velocity_command_measured_mps")) for row in evaluation_rows]
    metadata = metadata_for(args.run_dir)
    result = {
        "rows": len(rows), "active_rows": len(evaluation_rows), "active_start_cmd_time_s": active_start,
        "duration_s": value(rows[-1], "cmd_time_s") - value(rows[0], "cmd_time_s"),
        "requested_profile_max_error_mps": profile_error,
        "requested_shaped_error_abs_max_mps": max(requested_shaped),
        "requested_shaped_error_abs_p95_mps": percentile(requested_shaped, 0.95),
        "shaped_measured_error_abs_max_mps": max(shaped_measured),
        "shaped_measured_error_abs_p95_mps": percentile(shaped_measured, 0.95),
        "applied_shaped_gap_max_mps": max(applied_gap),
        "shaped_speed_max_mps": max(finite(evaluation_rows, "velocity_command_shaped_mps")),
        "measured_speed_max_mps": max(finite(evaluation_rows, "velocity_command_measured_mps")),
        "accel_abs_max_mps2": max(map(abs, finite(evaluation_rows, "velocity_command_accel_mps2"))),
        "jerk_abs_max_mps3": max(map(abs, finite(evaluation_rows, "velocity_command_jerk_mps3"))),
        "accel_step_abs_max_mps3": max(abs(a - b) for a, b in zip(finite(evaluation_rows, "velocity_command_accel_mps2"), finite(evaluation_rows, "velocity_command_accel_mps2")[1:])),
        "period_range_s": [min(finite(evaluation_rows, "velocity_command_gait_period_s")), max(finite(evaluation_rows, "velocity_command_gait_period_s"))],
        "duty_range": [min(finite(evaluation_rows, "velocity_command_gait_duty")), max(finite(evaluation_rows, "velocity_command_gait_duty"))],
        "step_length_max_m": max(finite(evaluation_rows, "velocity_command_gait_step_length_m")),
        "foot_lift_max_m": max(finite(evaluation_rows, "velocity_command_gait_foot_lift_m")),
        "roll_abs_max_deg": math.degrees(max(map(abs, finite(evaluation_rows, "imu_roll_rad")))),
        "pitch_abs_max_deg": math.degrees(max(map(abs, finite(evaluation_rows, "imu_pitch_rad")))),
        "regimes": sorted({row["velocity_command_gait_regime"] for row in rows}),
        "acceptance_semantics": "legacy status gate unchanged; quantitative gate is explicit and threshold-frozen",
    }
    result.update(command_quality(evaluation_rows, profile, active_start))
    result.update(diagnostic_metrics(rows, evaluation_rows, metadata))
    for key in ("controller_status", "safety_status", "quality_status", "completion_status", "analysis_status", "git_head"):
        result[key] = metadata.get(key, "")
    result["strict_pass"] = all(result[key] == "0" for key in ("controller_status", "safety_status", "quality_status", "completion_status", "analysis_status"))
    limits, checks = quantitative_acceptance(result, scenario)
    result["quantitative_scenario"] = scenario
    result["quantitative_thresholds"] = limits
    result["quantitative_checks"] = checks
    result["quantitative_pass"] = all(checks.values())
    result["acceptance_status"] = "PASS" if result["strict_pass"] and result["quantitative_pass"] else "FAIL"
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=True))
    if args.json_out:
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n")
    raise SystemExit(0 if result["strict_pass"] and (not args.require_quantitative or result["quantitative_pass"]) else 1)


if __name__ == "__main__":
    main()
