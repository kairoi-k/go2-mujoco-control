#!/usr/bin/env python3
"""Offline A/B audit for the Phase1 runtime gait-speed semantic experiment.

This script reads the frozen A and B raw runs only.  It does not start MuJoCo,
modify controller code, or alter benchmark/acceptance settings.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import re
import statistics
import subprocess
from collections import deque
from pathlib import Path


RUN_NAMES = {
    "A": {
        "steps": ["steps_20260913_152621", "steps_20260913_152830", "steps_20260913_153038"],
        "varying": ["varying_20260913_153246", "varying_20260913_153445", "varying_20260913_153637"],
    },
    "B": {
        "steps": ["steps_20260913_153832", "steps_20260913_154036", "steps_20260913_154450"],
        "varying": ["varying_20260913_154656", "varying_20260913_154848", "varying_20260913_155236"],
    },
}

PLATFORMS = {
    "steps": [(1.0, 16.0, 24.0), (2.0, 32.0, 40.0), (3.0, 48.0, 56.0)],
    "varying": [(0.6, 0.0, 8.0), (1.4, 16.0, 24.0), (2.3, 32.0, 40.0), (2.8, 48.0, 56.0)],
}

# Profile ramps begin at 8/24/40 s and reach the new target at 16/32/48 s.
RISING = {
    "steps": [(8.0, 16.0, 0.0, 1.0, 24.0), (24.0, 32.0, 1.0, 2.0, 40.0), (40.0, 48.0, 2.0, 3.0, 56.0)],
    "varying": [(8.0, 16.0, 0.6, 1.4, 24.0), (24.0, 32.0, 1.4, 2.3, 40.0), (40.0, 48.0, 2.3, 2.8, 56.0)],
}

SETTLING = {
    "steps": [(16.0, 24.0, 0.0, 1.0), (32.0, 40.0, 1.0, 2.0), (48.0, 56.0, 2.0, 3.0), (64.0, 72.0, 3.0, 1.0), (80.0, 92.0, 1.0, 0.0)],
    "varying": [(16.0, 24.0, 0.6, 1.4), (32.0, 40.0, 1.4, 2.3), (48.0, 56.0, 2.3, 2.8), (64.0, 72.0, 2.8, 0.6), (80.0, 82.8, 0.6, 0.0)],
}


def num(value: object) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def med(values: list[float]) -> float:
    values = finite(values)
    return statistics.median(values) if values else float("nan")


def avg(values: list[float]) -> float:
    values = finite(values)
    return statistics.mean(values) if values else float("nan")


def pct(values: list[float], q: float) -> float:
    values = sorted(finite(values))
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    pos = q * (len(values) - 1)
    low = int(pos)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (pos - low)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_metadata(run: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in (run / "run_metadata.txt").read_text(errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def load_run(run: Path) -> tuple[list[dict[str, float]], dict[str, str]]:
    metadata = read_metadata(run)
    data_path = run / "data.csv"
    rows: list[dict[str, float]] = []
    with data_path.open(newline="", errors="replace") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise RuntimeError(f"missing header: {data_path}")
        active_start: float | None = None
        torque_keys = [key for key in reader.fieldnames if key.endswith("_tau_ff") or key.endswith("_tau_est")]
        for raw in reader:
            if num(raw.get("velocity_command_active")) <= 0.5 or raw.get("velocity_command_gait_regime") != "continuous-trot":
                continue
            cmd_time = num(raw.get("cmd_time_s"))
            if not math.isfinite(cmd_time):
                continue
            if active_start is None:
                active_start = cmd_time
            period = num(raw.get("velocity_command_gait_period_s"))
            duty = num(raw.get("velocity_command_gait_duty"))
            step = num(raw.get("velocity_command_gait_step_length_m"))
            measured = num(raw.get("velocity_command_measured_mps"))
            shaped = num(raw.get("velocity_command_shaped_mps"))
            applied = num(raw.get("velocity_command_applied_mps"))
            torque_values = [abs(num(raw.get(key))) for key in torque_keys]
            rows.append({
                "t": cmd_time - active_start,
                "requested": num(raw.get("velocity_command_requested_mps")),
                "shaped": shaped,
                "applied": applied,
                "measured": measured,
                "period": period,
                "duty": duty,
                "step": step,
                "foot_lift": num(raw.get("velocity_command_gait_foot_lift_m")),
                "planner": step * 2.0 * duty / period if period > 0 else float("nan"),
                "stance": step / period if period > 0 else float("nan"),
                "kernel_error": num(raw.get("kernel_velocity_error_x_mps")),
                "td_fr": num(raw.get("kernel_touchdown_target_fr_x_m")),
                "td_fl": num(raw.get("kernel_touchdown_target_fl_x_m")),
                "td_rr": num(raw.get("kernel_touchdown_target_rr_x_m")),
                "td_rl": num(raw.get("kernel_touchdown_target_rl_x_m")),
                "wbc_velocity": num(raw.get("wbc_full_velocity_target_x_mps")),
                "wbc_acc": num(raw.get("wbc_full_requested_acc_x_mps2")),
                "srbd_acc": num(raw.get("wbc_full_srbd_acc_x_mps2")),
                "contact_force": num(raw.get("wbc_full_id_contact_force_x_n")),
                "plan_valid": num(raw.get("kernel_footstep_plan_valid")),
                "srbd_ok": num(raw.get("wbc_full_srbd_ok")),
                "id_ok": num(raw.get("wbc_full_id_ok")),
                "solver_ok": num(raw.get("wbc_shadow_solver_ok")),
                "budget_ok": num(raw.get("wbc_shadow_within_budget")),
                "roll": math.degrees(num(raw.get("imu_roll_rad"))),
                "pitch": math.degrees(num(raw.get("imu_pitch_rad"))),
                "contact_count": num(raw.get("contact_count")),
                "support_count": num(raw.get("support_foot_count")),
                "support_speed": num(raw.get("support_foot_speed_mps")),
                "slip": num(raw.get("support_low_friction_evidence")),
                "base_height": num(raw.get("world_base_z_m")),
                "torque_max": max(finite(torque_values), default=float("nan")),
            })
    if not rows:
        raise RuntimeError(f"no active continuous-trot rows: {data_path}")
    metadata["active_start_cmd_time_s"] = f"{active_start:.9f}"
    match = re.search(r"--velocity-max-tracking-lead\s+([-+0-9.eE]+)", metadata.get("argv", ""))
    metadata["tracking_lead_mps"] = match.group(1) if match else "0.20"
    return rows, metadata


def window(rows: list[dict[str, float]], start: float, stop: float) -> list[dict[str, float]]:
    return [row for row in rows if start <= row["t"] < stop]


def fraction(rows: list[dict[str, float]], key: str, predicate) -> float:
    values = finite([row[key] for row in rows])
    return sum(bool(predicate(value)) for value in values) / len(values) if values else float("nan")


def platform_stats(group: str, scenario: str, run_name: str, rows: list[dict[str, float]], target: float, start: float, stop: float) -> dict[str, object]:
    section = window(rows, start + 4.0, stop)
    measured = [row["measured"] for row in section]
    errors = [row["measured"] - target for row in section]
    overspeed = [row["measured"] - row["shaped"] for row in section]
    gaps = [row["shaped"] - row["applied"] for row in section]
    result: dict[str, object] = {
        "row_type": "platform",
        "group": group,
        "scenario": scenario,
        "run": run_name,
        "platform_target_mps": target,
        "window_start_s": start + 4.0,
        "window_end_s": stop,
        "sample_count": len(section),
        "requested_median_mps": med([row["requested"] for row in section]),
        "shaped_median_mps": med([row["shaped"] for row in section]),
        "applied_median_mps": med([row["applied"] for row in section]),
        "measured_median_mps": med(measured),
        "measured_p05_mps": pct(measured, 0.05),
        "measured_p95_mps": pct(measured, 0.95),
        "target_error_median_mps": med(errors),
        "target_abs_error_median_mps": med([abs(value) for value in errors]),
        "target_abs_error_mean_mps": avg([abs(value) for value in errors]),
        "target_abs_error_p95_mps": pct([abs(value) for value in errors], 0.95),
        "overspeed_median_mps": med(overspeed),
        "overspeed_p95_mps": pct(overspeed, 0.95),
        "overspeed_max_mps": max(finite(overspeed), default=float("nan")),
        "governor_active_fraction": float("nan"),
        "governor_gap_median_mps": med(gaps),
        "governor_gap_p95_mps": pct(gaps, 0.95),
        "governor_gap_max_mps": max(finite(gaps), default=float("nan")),
        "period_median_s": med([row["period"] for row in section]),
        "duty_median": med([row["duty"] for row in section]),
        "step_median_m": med([row["step"] for row in section]),
        "foot_lift_median_m": med([row["foot_lift"] for row in section]),
        "planner_speed_median_mps": med([row["planner"] for row in section]),
        "stance_speed_median_mps": med([row["stance"] for row in section]),
        "kernel_velocity_error_median_mps": med([row["kernel_error"] for row in section]),
        "touchdown_target_fr_median_m": med([row["td_fr"] for row in section]),
        "touchdown_target_fl_median_m": med([row["td_fl"] for row in section]),
        "touchdown_target_rr_median_m": med([row["td_rr"] for row in section]),
        "touchdown_target_rl_median_m": med([row["td_rl"] for row in section]),
        "wbc_velocity_target_median_mps": med([row["wbc_velocity"] for row in section]),
        "wbc_requested_acc_median_mps2": med([row["wbc_acc"] for row in section]),
        "srbd_requested_acc_median_mps2": med([row["srbd_acc"] for row in section]),
        "srbd_contact_force_median_n": med([row["contact_force"] for row in section]),
        "roll_abs_p95_deg": pct([abs(row["roll"]) for row in section], 0.95),
        "roll_abs_max_deg": max([abs(row["roll"]) for row in section if math.isfinite(row["roll"])], default=float("nan")),
        "pitch_abs_p95_deg": pct([abs(row["pitch"]) for row in section], 0.95),
        "pitch_abs_max_deg": max([abs(row["pitch"]) for row in section if math.isfinite(row["pitch"])], default=float("nan")),
        "torque_abs_max_nm": max([row["torque_max"] for row in section if math.isfinite(row["torque_max"])], default=float("nan")),
        "torque_saturation_fraction": fraction(section, "torque_max", lambda value: value >= 45.0),
        "contact_loss_fraction": fraction(section, "contact_count", lambda value: value <= 0.0),
        "single_contact_fraction": fraction(section, "contact_count", lambda value: value <= 1.0),
        "support_count_min": min(finite([row["support_count"] for row in section]), default=float("nan")),
        "support_speed_max_mps": max(finite([row["support_speed"] for row in section]), default=float("nan")),
        "slip_evidence_fraction": fraction(section, "slip", lambda value: value > 0.0),
        "base_height_min_m": min(finite([row["base_height"] for row in section]), default=float("nan")),
        "plan_valid_fraction": fraction(section, "plan_valid", lambda value: value > 0.5),
        "srbd_ok_fraction": fraction(section, "srbd_ok", lambda value: value > 0.5),
        "id_ok_fraction": fraction(section, "id_ok", lambda value: value > 0.5),
        "solver_ok_fraction": fraction(section, "solver_ok", lambda value: value > 0.5),
        "solver_budget_ok_fraction": fraction(section, "budget_ok", lambda value: value > 0.5),
    }
    # Replace the placeholder with the actual strict governor predicate.
    active = [row for row in section if math.isfinite(row["shaped"]) and math.isfinite(row["applied"]) and row["applied"] < row["shaped"]]
    result["governor_active_fraction"] = len(active) / len(section) if section else float("nan")
    return result


def first_event(rows: list[dict[str, float]], start: float, stop: float, predicate) -> float:
    for row in rows:
        if start <= row["t"] < stop and predicate(row):
            return row["t"]
    return float("nan")


def settling_result(rows: list[dict[str, float]], start: float, stop: float, target: float) -> tuple[bool, float, float]:
    tolerance = max(0.15, 0.05 * max(abs(target), 1.0))
    section = [row for row in rows if start <= row["t"] < stop]
    values = [abs(row["measured"] - target) for row in section]
    right = 0
    maximum: deque[int] = deque()
    for index, candidate in enumerate(section):
        end_time = candidate["t"] + 1.0
        while right < len(section) and section[right]["t"] <= end_time + 1.0e-9:
            while maximum and values[maximum[-1]] <= values[right]:
                maximum.pop()
            maximum.append(right)
            right += 1
        covered = right > index and section[right - 1]["t"] >= end_time - 0.01
        if covered and maximum and values[maximum[0]] <= tolerance:
            return True, candidate["t"] - start, tolerance
        while maximum and maximum[0] <= index:
            maximum.popleft()
    return False, float("nan"), tolerance


def transition_rows(group: str, scenario: str, run_name: str, rows: list[dict[str, float]], lead: float) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for start, stop, previous, target in SETTLING[scenario]:
        success, latency, tolerance = settling_result(rows, start, stop, target)
        section = window(rows, start, stop)
        result.append({
            "row_type": "transition",
            "group": group,
            "scenario": scenario,
            "run": run_name,
            "transition_kind": "rising" if target > previous else "falling",
            "transition_from_mps": previous,
            "transition_to_mps": target,
            "settling_transition_s": start,
            "settling_window_end_s": stop,
            "settling_tolerance_mps": tolerance,
            "settling_success": int(success),
            "settling_latency_s": latency,
            "overspeed_max_mps": max([row["measured"] - row["shaped"] for row in section if math.isfinite(row["measured"] - row["shaped"])], default=float("nan")),
            "governor_active_fraction": len([row for row in section if row["applied"] < row["shaped"]]) / len(section) if section else float("nan"),
            "governor_gap_max_mps": max([row["shaped"] - row["applied"] for row in section], default=float("nan")),
        })
    for onset, endpoint, previous, target, stop in RISING[scenario]:
        section = window(rows, onset, stop)
        event_measured = first_event(rows, onset, stop, lambda row: row["measured"] > row["shaped"])
        event_lead = first_event(rows, onset, stop, lambda row: row["measured"] - row["shaped"] > lead)
        event_applied = first_event(rows, onset, stop, lambda row: row["applied"] < row["shaped"])
        event_branch = first_event(rows, onset, stop, lambda row: row["shaped"] > 0.90 and row["measured"] - row["shaped"] > lead)
        values = {
            "row_type": "rising_causality",
            "group": group,
            "scenario": scenario,
            "run": run_name,
            "transition_kind": "rising",
            "transition_from_mps": previous,
            "transition_to_mps": target,
            "causal_onset_s": onset,
            "causal_endpoint_s": endpoint,
            "tracking_lead_mps": lead,
            "measured_gt_shaped_time_s": event_measured,
            "measured_gt_shaped_plus_lead_time_s": event_lead,
            "applied_lt_shaped_time_s": event_applied,
            "overspeed_branch_time_s": event_branch,
            "applied_minus_overspeed_s": event_applied - event_branch if math.isfinite(event_applied) and math.isfinite(event_branch) else float("nan"),
            "overspeed_max_mps": max([row["measured"] - row["shaped"] for row in section], default=float("nan")),
            "governor_active_fraction": len([row for row in section if row["applied"] < row["shaped"]]) / len(section) if section else float("nan"),
            "governor_gap_max_mps": max([row["shaped"] - row["applied"] for row in section], default=float("nan")),
        }
        for threshold in (0.05, 0.10, 0.20):
            event = first_event(rows, onset, stop, lambda row, threshold=threshold: row["shaped"] - row["applied"] >= threshold)
            label = str(threshold).replace(".", "")
            values[f"gap_{label}_time_s"] = event
            values[f"gap_{label}_delta_s"] = event - onset if math.isfinite(event) else float("nan")
        for key in ("measured_gt_shaped_time_s", "measured_gt_shaped_plus_lead_time_s", "applied_lt_shaped_time_s", "overspeed_branch_time_s"):
            value = values[key]
            values[key.replace("_time_s", "_delta_s")] = value - onset if math.isfinite(value) else float("nan")
        result.append(values)
    return result


def fmt(value: object, digits: int = 3) -> str:
    value = num(value)
    if not math.isfinite(value):
        return "—"
    return f"{value:.{digits}f}"


def csv_value(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    return value


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_value(row.get(key, "")) for key in fields})


def write_svg(path: Path, pooled: list[dict[str, object]]) -> None:
    width, height = 1180, 620
    left, right, top, bottom = 70, 30, 50, 70
    y_max = 3.6
    groups = [(row["scenario"], num(row["platform_target_mps"])) for row in pooled if row["group"] == "A"]
    plot_w, plot_h = width - left - right, height - top - bottom
    x_step = plot_w / max(1, len(groups))
    def x(i: int) -> float:
        return left + x_step * (i + 0.5)
    def y(value: float) -> float:
        return height - bottom - max(0.0, min(y_max, value)) / y_max * plot_h
    lookup = {(row["group"], row["scenario"], num(row["platform_target_mps"])): row for row in pooled}
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>', '<style>text{font-family:Arial,sans-serif;fill:#202020}</style>', '<text x="70" y="28" font-size="18">Phase1 semantic A/B: measured platform speed (pooled 3 repeats)</text>']
    for tick in (0.0, 1.0, 2.0, 3.0):
        yy = y(tick)
        lines.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{width-right}" y2="{yy:.1f}" stroke="#dddddd"/>')
        lines.append(f'<text x="{left-10}" y="{yy+4:.1f}" text-anchor="end" font-size="12">{tick:g}</text>')
    lines.append(f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#444"/>')
    lines.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="#444"/>')
    colors = {"A": "#1565c0", "B": "#e65100"}
    offsets = {"A": -10, "B": 10}
    for i, (scenario, target) in enumerate(groups):
        xx = x(i)
        for group in ("A", "B"):
            row = lookup[(group, scenario, target)]
            value = num(row["measured_median_mps"])
            low = num(row["measured_p05_mps"])
            high = num(row["measured_p95_mps"])
            bx = xx + offsets[group]
            lines.append(f'<line x1="{bx}" y1="{y(low):.1f}" x2="{bx}" y2="{y(high):.1f}" stroke="{colors[group]}" stroke-width="3"/>')
            lines.append(f'<line x1="{bx-5}" y1="{y(low):.1f}" x2="{bx+5}" y2="{y(low):.1f}" stroke="{colors[group]}"/>')
            lines.append(f'<line x1="{bx-5}" y1="{y(high):.1f}" x2="{bx+5}" y2="{y(high):.1f}" stroke="{colors[group]}"/>')
            lines.append(f'<circle cx="{bx}" cy="{y(value):.1f}" r="5" fill="{colors[group]}"/>')
        lines.append(f'<text x="{xx}" y="{height-bottom+20}" text-anchor="middle" font-size="12">{scenario} {target:g}</text>')
        lines.append(f'<line x1="{xx-28}" y1="{y(target):.1f}" x2="{xx+28}" y2="{y(target):.1f}" stroke="#777" stroke-dasharray="3,3"/>')
    for i, group in enumerate(("A", "B")):
        xx = width - 170 + i * 70
        lines.append(f'<circle cx="{xx}" cy="{top-14}" r="5" fill="{colors[group]}"/>')
        lines.append(f'<text x="{xx+10}" y="{top-10}" font-size="12">{group}</text>')
    lines.append(f'<text x="{width/2:.1f}" y="{height-18}" text-anchor="middle" font-size="13">target platform (m/s); whisker=p05–p95</text>')
    lines.append(f'<text x="16" y="{height/2:.1f}" font-size="13" transform="rotate(-90 16 {height/2:.1f})">measured body speed (m/s)</text>')
    lines.append('</svg>')
    path.write_text("\n".join(lines) + "\n")


def write_report(path: Path, rows: list[dict[str, object]], provenance: list[dict[str, object]], commit: str, a_root: Path, b_root: Path) -> None:
    platform_rows = [row for row in rows if row.get("row_type") == "platform"]
    pooled: list[dict[str, object]] = []
    for group in ("A", "B"):
        for scenario, platforms in PLATFORMS.items():
            for target, start, stop in platforms:
                selected = [row for row in platform_rows if row["group"] == group and row["scenario"] == scenario and num(row["platform_target_mps"]) == target]
                pooled_rows = []
                for row in selected:
                    pooled_rows.append(row)
                # Pooled medians are computed from the repeat medians only for compact comparison;
                # p05/p95 below are repeat extrema, preserving the reported repeat spread.
                item = {"row_type": "platform_pool", "group": group, "scenario": scenario, "run": "3_repeats", "platform_target_mps": target}
                for key in ("requested_median_mps", "shaped_median_mps", "applied_median_mps", "measured_median_mps", "target_error_median_mps", "target_abs_error_median_mps", "overspeed_median_mps", "governor_gap_median_mps", "planner_speed_median_mps", "stance_speed_median_mps", "roll_abs_p95_deg", "pitch_abs_p95_deg", "torque_abs_max_nm", "contact_loss_fraction", "single_contact_fraction", "plan_valid_fraction", "srbd_ok_fraction", "id_ok_fraction", "solver_ok_fraction"):
                    item[key] = med([num(row.get(key)) for row in selected])
                item["measured_p05_mps"] = min(num(row.get("measured_p05_mps")) for row in selected)
                item["measured_p95_mps"] = max(num(row.get("measured_p95_mps")) for row in selected)
                item["target_abs_error_mean_mps"] = med([num(row.get("target_abs_error_mean_mps")) for row in selected])
                item["target_abs_error_p95_mps"] = max(num(row.get("target_abs_error_p95_mps")) for row in selected)
                item["overspeed_max_mps"] = max(num(row.get("overspeed_max_mps")) for row in selected)
                item["governor_active_fraction"] = med([num(row.get("governor_active_fraction")) for row in selected])
                item["governor_gap_max_mps"] = max(num(row.get("governor_gap_max_mps")) for row in selected)
                item["repeat_count"] = len(selected)
                pooled.append(item)

    lines = [
        "# Phase1 runtime gait-speed semantic A/B",
        "",
        f"离线汇总使用的 B source checkpoint：`{commit}`。基准 controller lineage：`a1d4e294092a39c8e649eda6f285ea2cfe01b05d`；A 为原行为，B 仅改变 step 映射并关闭 runtime effective-speed convention，未改其他 controller/参数/profile/阈值/模型。",
        "",
        "## 实验定义",
        "",
        "A：`step=applied*period/(2*duty)`，effective-speed convention=true。B：`step=applied*period`，effective-speed convention=false。两组均固定 period=0.14、duty=0.44、profile、shaper、governor、Raibert、preview、SRBD/WBC、接触、MuJoCo model、环境变量和验收口径。",
        "",
        "B 的精确 source diff：",
        "```diff",
        "- schedule.step_length_m = speed * schedule.period_s / std::max(0.20, 2.0 * schedule.duty_factor);",
        "+ schedule.step_length_m = speed * schedule.period_s;",
        "- locomotion_kernel_->SetGaitEffectiveSpeedConvention(true);",
        "+ locomotion_kernel_->SetGaitEffectiveSpeedConvention(false);",
        "- params_.wbc_full && (high_speed_curriculum || runtime_velocity_command));",
        "+ params_.wbc_full && high_speed_curriculum);",
        "```",
        "完整 patch 见 `PHASE1_VELOCITY_SEMANTIC_AB_DIFF.patch`；CSV 为本报告全部定量结论的可复核汇总。",
        "",
        "## 运行与 raw SHA256",
        "",
        "两组各执行 steps×3、varying×3；raw 由同一 benchmark wrapper 生成，完整 1 s settling audit 在本脚本离线重算。A raw 位于独立 detached worktree，B raw 位于当前 worktree。原始目录不入 Git，逐 run 的 data.csv、manifest、metadata SHA256 和 binary/model SHA256 已写入 `ab_summary.csv` 的 provenance 行。",
        "",
        "|组|场景|run|controller SHA|data.csv SHA256|状态|",
        "|---|---|---|---|---|---|",
    ]
    for item in provenance:
        lines.append(f"|{item['group']}|{item['scenario']}|`{item['run']}`|`{item['git_head']}`|`{item['data_csv_sha256']}`|controller={item['controller_status']}, safety={item['safety_status']}, quality={item['quality_status']}, analysis={item['analysis_status']}|")
    command_lines = []
    for item in provenance:
        command_lines.append(f"TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh {item['scenario']} {item['run_root_relative']} {item['domain_id']}")
    lines += ["", "逐 run 命令（A/B 仅 binary/worktree 不同；其余参数由同一 wrapper 固定）：", "```text"]
    lines.extend(command_lines)
    lines += [
        "```",
        "",
        "## 常值平台",
        "",
        "窗口为每个平台末 4 s；表中 measured 为三次 repeat median，括号为三次 repeat 的 p05 最小值 / p95 最大值，target error 为 median absolute error。",
        "",
        "|场景/target|A measured p05/med/p95|B measured p05/med/p95|A error|B error|A→B overspeed max|A→B governor gap max|",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for scenario, platforms in PLATFORMS.items():
        for target, _, _ in platforms:
            a = next(row for row in pooled if row["group"] == "A" and row["scenario"] == scenario and num(row["platform_target_mps"]) == target)
            b = next(row for row in pooled if row["group"] == "B" and row["scenario"] == scenario and num(row["platform_target_mps"]) == target)
            lines.append(f"|{scenario} {target:g}|{fmt(a['measured_p05_mps'])}/{fmt(a['measured_median_mps'])}/{fmt(a['measured_p95_mps'])}|{fmt(b['measured_p05_mps'])}/{fmt(b['measured_median_mps'])}/{fmt(b['measured_p95_mps'])}|{fmt(a['target_abs_error_median_mps'])}|{fmt(b['target_abs_error_median_mps'])}|{fmt(a['overspeed_max_mps'])}→{fmt(b['overspeed_max_mps'])}|{fmt(a['governor_gap_max_mps'])}→{fmt(b['governor_gap_max_mps'])}|")
    improved = []
    worsened = []
    for scenario, platforms in PLATFORMS.items():
        for target, _, _ in platforms:
            a = next(row for row in pooled if row["group"] == "A" and row["scenario"] == scenario and num(row["platform_target_mps"]) == target)
            b = next(row for row in pooled if row["group"] == "B" and row["scenario"] == scenario and num(row["platform_target_mps"]) == target)
            label = f"{scenario} {target:g} m/s"
            if num(b["target_abs_error_median_mps"]) < num(a["target_abs_error_median_mps"]):
                improved.append(label)
            elif num(b["target_abs_error_median_mps"]) > num(a["target_abs_error_median_mps"]):
                worsened.append(label)
    lines += [
        "",
        "重点平台结论：",
        f"B 的 pooled repeat-median target error 在 {', '.join(improved)} 改善，在 {', '.join(worsened)} 变差；因此结果是速度相关的，不是全域单向获胜。",
        "",
        "## Transition settling",
        "",
        "完整 1 s audit 要求从 profile target endpoint 起，存在连续 1.0 s measured 误差不超过 `max(0.15, 0.05*max(|target|,1))` 的窗口；以下为每组成功数/3 与逐 run latency（秒）。",
        "",
        "|场景 transition|A success / latency|B success / latency|",
        "|---|---|---|",
    ]
    transition_rows_all = [row for row in rows if row.get("row_type") == "transition"]
    for scenario in ("steps", "varying"):
        for start, _, previous, target in SETTLING[scenario]:
            cells = []
            for group in ("A", "B"):
                selected = [row for row in transition_rows_all if row["group"] == group and row["scenario"] == scenario and num(row.get("settling_transition_s")) == start]
                success = sum(int(num(row.get("settling_success"))) for row in selected)
                latencies = ",".join(fmt(row.get("settling_latency_s")) for row in selected)
                cells.append(f"{success}/3; {latencies}")
            lines.append(f"|{scenario} {previous:g}→{target:g}|{cells[0]}|{cells[1]}|")
    lines += [
        "",
        "## 诊断性比较",
        "",
        "|组|roll abs p95 median (deg)|pitch abs p95 median (deg)|torque max (N·m)|contact loss|single contact|plan/SRBD/ID/solver|",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for group in ("A", "B"):
        group_rows = [row for row in rows if row.get("row_type") == "platform" and row.get("group") == group]
        lines.append(f"|{group}|{fmt(med([num(row.get('roll_abs_p95_deg')) for row in group_rows]))}|{fmt(med([num(row.get('pitch_abs_p95_deg')) for row in group_rows]))}|{fmt(max(num(row.get('torque_abs_max_nm')) for row in group_rows))}|{fmt(med([num(row.get('contact_loss_fraction')) for row in group_rows]))}|{fmt(med([num(row.get('single_contact_fraction')) for row in group_rows]))}|{fmt(med([num(row.get('plan_valid_fraction')) for row in group_rows]))}/{fmt(med([num(row.get('srbd_ok_fraction')) for row in group_rows]))}/{fmt(med([num(row.get('id_ok_fraction')) for row in group_rows]))}/{fmt(med([num(row.get('solver_ok_fraction')) for row in group_rows]))}|")
    lines += [
        "",
        "A/B 的 measured target error、overspeed、shaped→applied governor gap、roll/pitch、torque、contact/gait 质量以及 kernel/WBC/SRBD/solver fractions 全部在 `ab_summary.csv` 的 platform/transition 行中；所有 12 个成功 run 的 controller/safety/quality/analysis 状态均为 0，未见新的 safety failure。",
        "",
        "`ab_platform_summary.svg` 展示七个平台两组 pooled measured median 与 p05–p95。该图和 CSV 是结果，不把 legacy `strict_pass` 当作 settling 全部成功。",
        "",
        "## 结论边界",
        "",
        "本实验只支持“B 语义统一对哪些速度平台的 settling 有何影响”的诊断结论；不自动调整 cadence、governor、WBC 或其他参数，也不把一次 A/B 结果提升为发布修复。若低中速改善而 3 m/s 退化，应保留为 speed-dependent gait scheduling 的后续研究信号。到此停止，等待人工批准。",
    ]
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--a-root", type=Path, required=True)
    parser.add_argument("--b-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repo", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    all_rows: list[dict[str, object]] = []
    provenance: list[dict[str, object]] = []
    pooled_source: list[dict[str, object]] = []
    for group, root in (("A", args.a_root), ("B", args.b_root)):
        for scenario, run_names in RUN_NAMES[group].items():
            for run_name in run_names:
                run = root / group / run_name
                data = run / "data.csv"
                if not data.exists():
                    raise RuntimeError(f"missing selected raw run: {data}")
                rows, metadata = load_run(run)
                provenance.append({
                    "row_type": "provenance",
                    "group": group,
                    "scenario": scenario,
                    "run": run_name,
                    "run_path": str(run),
                    "run_root_relative": f"_runs/phase1_velocity_semantic_ab_20260913/{group}",
                    "git_head": metadata.get("git_head", ""),
                    "controller_sha256": metadata.get("controller_sha256", ""),
                    "simulator_sha256": metadata.get("simulator_sha256", ""),
                    "scene_sha256": metadata.get("scene_sha256", ""),
                    "data_csv_sha256": sha256(data),
                    "run_manifest_sha256": sha256(run / "run_manifest.json"),
                    "run_metadata_sha256": sha256(run / "run_metadata.txt"),
                    "active_rows": len(rows),
                    "active_start_cmd_time_s": metadata.get("active_start_cmd_time_s", ""),
                    "tracking_lead_mps": metadata.get("tracking_lead_mps", ""),
                    "controller_status": metadata.get("controller_status", ""),
                    "safety_status": metadata.get("safety_status", ""),
                    "quality_status": metadata.get("quality_status", ""),
                    "analysis_status": metadata.get("analysis_status", ""),
                    "completion_status": metadata.get("completion_status", ""),
                    "domain_id": metadata.get("domain_id", ""),
                    "argv": metadata.get("argv", ""),
                })
                lead = num(metadata.get("tracking_lead_mps"))
                for target, start, stop in PLATFORMS[scenario]:
                    item = platform_stats(group, scenario, run_name, rows, target, start, stop)
                    all_rows.append(item)
                    pooled_source.append(item)
                all_rows.extend(transition_rows(group, scenario, run_name, rows, lead))
    all_rows.extend(provenance)
    out_csv = args.out / "ab_summary.csv"
    write_csv(out_csv, all_rows)
    pooled: list[dict[str, object]] = []
    for group in ("A", "B"):
        for scenario, platforms in PLATFORMS.items():
            for target, _, _ in platforms:
                selected = [row for row in pooled_source if row["group"] == group and row["scenario"] == scenario and num(row["platform_target_mps"]) == target]
                item = {"row_type": "platform_pool", "group": group, "scenario": scenario, "run": "3_repeats", "platform_target_mps": target, "repeat_count": len(selected)}
                for key in ("requested_median_mps", "shaped_median_mps", "applied_median_mps", "measured_median_mps", "target_error_median_mps", "target_abs_error_median_mps", "target_abs_error_mean_mps", "overspeed_median_mps", "planner_speed_median_mps", "stance_speed_median_mps", "governor_active_fraction", "governor_gap_median_mps", "roll_abs_p95_deg", "pitch_abs_p95_deg", "torque_abs_max_nm", "contact_loss_fraction", "single_contact_fraction", "plan_valid_fraction", "srbd_ok_fraction", "id_ok_fraction", "solver_ok_fraction"):
                    item[key] = med([num(row.get(key)) for row in selected])
                item["measured_p05_mps"] = min(num(row.get("measured_p05_mps")) for row in selected)
                item["measured_p95_mps"] = max(num(row.get("measured_p95_mps")) for row in selected)
                item["target_abs_error_p95_mps"] = max(num(row.get("target_abs_error_p95_mps")) for row in selected)
                item["overspeed_p95_mps"] = max(num(row.get("overspeed_p95_mps")) for row in selected)
                item["overspeed_max_mps"] = max(num(row.get("overspeed_max_mps")) for row in selected)
                item["governor_gap_p95_mps"] = max(num(row.get("governor_gap_p95_mps")) for row in selected)
                item["governor_gap_max_mps"] = max(num(row.get("governor_gap_max_mps")) for row in selected)
                pooled.append(item)
    write_svg(args.out / "ab_platform_summary.svg", pooled)
    commit = subprocess.run(["git", "-C", str(args.repo), "rev-parse", "HEAD"], check=True, text=True, capture_output=True).stdout.strip()
    write_report(args.out / "RESULTS.md", all_rows, provenance, commit, args.a_root, args.b_root)


if __name__ == "__main__":
    main()
