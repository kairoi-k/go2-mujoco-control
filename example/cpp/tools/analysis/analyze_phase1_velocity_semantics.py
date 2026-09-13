#!/usr/bin/env python3
"""Offline Phase1 velocity-semantic and governor-causality audit.

This script reads only the six frozen Phase1 runs and Git history.  It never
starts a simulator and never mutates controller, profile, or acceptance code.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import statistics
import subprocess
from pathlib import Path


RUNS = {
    "steps": [
        "steps_20260913_130149",
        "steps_20260913_130415",
        "steps_20260913_130640",
    ],
    "varying": [
        "varying_20260913_130859",
        "varying_20260913_131059",
        "varying_20260913_131301",
    ],
}

TRANSITIONS = {
    # The profile sampler linearly interpolates between points.  These are
    # (rise onset, target endpoint, from, to), not instantaneous jumps.
    "steps": [(8.0, 16.0, 0.0, 1.0), (24.0, 32.0, 1.0, 2.0), (40.0, 48.0, 2.0, 3.0)],
    "varying": [(8.0, 16.0, 0.6, 1.4), (24.0, 32.0, 1.4, 2.3), (40.0, 48.0, 2.3, 2.8)],
}

PLATFORMS = {
    "steps": [(1.0, 16.0, 24.0), (2.0, 32.0, 40.0), (3.0, 48.0, 56.0)],
    "varying": [
        (0.6, 0.0, 8.0),
        (1.4, 16.0, 24.0),
        (2.3, 32.0, 40.0),
        (2.8, 48.0, 56.0),
    ],
}

REQUIRED = {
    "cmd_time_s",
    "cycle_index",
    "velocity_command_active",
    "velocity_command_gait_regime",
    "velocity_command_requested_mps",
    "velocity_command_shaped_mps",
    "velocity_command_applied_mps",
    "velocity_command_measured_mps",
    "velocity_command_gait_period_s",
    "velocity_command_gait_duty",
    "velocity_command_gait_step_length_m",
    "velocity_command_gait_foot_lift_m",
    "kernel_footstep_plan_valid",
    "kernel_velocity_error_x_mps",
    "kernel_touchdown_target_fr_x_m",
    "kernel_touchdown_target_fl_x_m",
    "kernel_touchdown_target_rr_x_m",
    "kernel_touchdown_target_rl_x_m",
    "wbc_full_velocity_target_x_mps",
    "wbc_full_requested_acc_x_mps2",
    "wbc_full_srbd_acc_x_mps2",
    "wbc_full_id_contact_force_x_n",
    "wbc_full_srbd_ok",
    "wbc_full_id_ok",
    "wbc_shadow_solver_ok",
}

SAMPLE_FIELDS = [
    "scenario",
    "run",
    "t_rel_s",
    "cmd_time_s",
    "cycle_index",
    "requested_mps",
    "shaped_mps",
    "applied_mps",
    "measured_mps",
    "period_s",
    "duty",
    "schedule_step_m",
    "foot_lift_m",
    "planner_semantic_speed_mps",
    "stance_kinematic_speed_mps",
    "measured_minus_applied_mps",
    "measured_minus_planner_semantic_mps",
    "measured_minus_stance_kinematic_mps",
    "shaped_minus_applied_mps",
    "measured_minus_shaped_mps",
    "tracking_lead_mps",
    "overspeed_mps",
    "kernel_velocity_error_mps",
    "touchdown_target_fr_m",
    "touchdown_target_fl_m",
    "touchdown_target_rr_m",
    "touchdown_target_rl_m",
    "kernel_footstep_plan_valid",
    "wbc_full_velocity_target_mps",
    "wbc_full_requested_acc_mps2",
    "wbc_full_srbd_acc_mps2",
    "wbc_full_id_contact_force_n",
    "wbc_full_srbd_ok",
    "wbc_full_id_ok",
    "wbc_shadow_solver_ok",
]

ALIGNED_FIELDS = [
    "scenario", "run", "transition_s", "transition_endpoint_s",
    "from_mps", "to_mps", "offset_s", "row_t_rel_s",
    "alignment_abs_error_s",
] + [field for field in SAMPLE_FIELDS if field not in ("scenario", "run")]

HISTORY_COMMITS = [
    "037c7c06d6cde519b9972327244bf60bbe379463",
    "77f0e8e46b585847ac6fc6f079c69c59ea00ab19",
    "d41143faba7e7064a7064adc6470adec9b30a529",
    "388310e8853733dedb27ae2f215e7a82a0161c18",
    "048eb1b5aae019fc6fad26f956c0aeeef8d9b816",
    "a1d4e294092a39c8e649eda6f285ea2cfe01b05d",
]

KEYWORDS = (
    "nominal_velocity",
    "step_length",
    "duty_factor",
    "stance_travel",
    "commanded_travel",
    "SetGaitEffectiveSpeedConvention",
    "max_tracking_lead",
    "velocity_command_shaper",
    "runtime_velocity_command",
)


def number(value: object) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def median(values: list[float]) -> float:
    values = finite(values)
    return statistics.median(values) if values else float("nan")


def mean(values: list[float]) -> float:
    values = finite(values)
    return statistics.mean(values) if values else float("nan")


def percentile(values: list[float], quantile: float) -> float:
    values = sorted(finite(values))
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    position = quantile * (len(values) - 1)
    low = int(position)
    high = min(low + 1, len(values) - 1)
    return values[low] + (values[high] - values[low]) * (position - low)


def json_value(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, list):
        return [json_value(item) for item in value]
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def metadata(run: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    path = run / "run_metadata.txt"
    if not path.exists():
        return result
    for line in path.read_text(errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def tracking_lead(run: Path) -> float:
    text = metadata(run).get("argv", "")
    match = re.search(r"--velocity-max-tracking-lead\s+([-+0-9.eE]+)", text)
    return number(match.group(1)) if match else 0.20


def load_active(run: Path, scenario: str) -> tuple[list[dict[str, float]], dict[str, str]]:
    path = run / "data.csv"
    lead = tracking_lead(run)
    active: list[dict[str, float]] = []
    with path.open(newline="", errors="replace") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        missing = sorted(REQUIRED - set(header))
        if missing:
            raise RuntimeError(f"{path}: missing fields {missing}")
        index = {name: header.index(name) for name in REQUIRED}
        active_start: float | None = None
        for values in reader:
            if len(values) < len(header):
                continue
            if number(values[index["velocity_command_active"]]) <= 0.5:
                continue
            if values[index["velocity_command_gait_regime"]] != "continuous-trot":
                continue
            cmd_time = number(values[index["cmd_time_s"]])
            if not math.isfinite(cmd_time):
                continue
            if active_start is None:
                active_start = cmd_time
            t_rel = cmd_time - active_start
            period = number(values[index["velocity_command_gait_period_s"]])
            duty = number(values[index["velocity_command_gait_duty"]])
            step = number(values[index["velocity_command_gait_step_length_m"]])
            requested = number(values[index["velocity_command_requested_mps"]])
            shaped = number(values[index["velocity_command_shaped_mps"]])
            applied = number(values[index["velocity_command_applied_mps"]])
            measured = number(values[index["velocity_command_measured_mps"]])
            planner = step * (2.0 * duty) / period if period > 0 else float("nan")
            stance = step / period if period > 0 else float("nan")
            active.append(
                {
                    "t_rel_s": t_rel,
                    "cmd_time_s": cmd_time,
                    "cycle_index": number(values[index["cycle_index"]]),
                    "requested_mps": requested,
                    "shaped_mps": shaped,
                    "applied_mps": applied,
                    "measured_mps": measured,
                    "period_s": period,
                    "duty": duty,
                    "schedule_step_m": step,
                    "foot_lift_m": number(values[index["velocity_command_gait_foot_lift_m"]]),
                    "planner_semantic_speed_mps": planner,
                    "stance_kinematic_speed_mps": stance,
                    "measured_minus_applied_mps": measured - applied,
                    "measured_minus_planner_semantic_mps": measured - planner,
                    "measured_minus_stance_kinematic_mps": measured - stance,
                    "shaped_minus_applied_mps": shaped - applied,
                    "measured_minus_shaped_mps": measured - shaped,
                    "tracking_lead_mps": lead,
                    "overspeed_mps": measured - shaped,
                    "kernel_velocity_error_mps": number(values[index["kernel_velocity_error_x_mps"]]),
                    "touchdown_target_fr_m": number(values[index["kernel_touchdown_target_fr_x_m"]]),
                    "touchdown_target_fl_m": number(values[index["kernel_touchdown_target_fl_x_m"]]),
                    "touchdown_target_rr_m": number(values[index["kernel_touchdown_target_rr_x_m"]]),
                    "touchdown_target_rl_m": number(values[index["kernel_touchdown_target_rl_x_m"]]),
                    "kernel_footstep_plan_valid": number(values[index["kernel_footstep_plan_valid"]]),
                    "wbc_full_velocity_target_mps": number(values[index["wbc_full_velocity_target_x_mps"]]),
                    "wbc_full_requested_acc_mps2": number(values[index["wbc_full_requested_acc_x_mps2"]]),
                    "wbc_full_srbd_acc_mps2": number(values[index["wbc_full_srbd_acc_x_mps2"]]),
                    "wbc_full_id_contact_force_n": number(values[index["wbc_full_id_contact_force_x_n"]]),
                    "wbc_full_srbd_ok": number(values[index["wbc_full_srbd_ok"]]),
                    "wbc_full_id_ok": number(values[index["wbc_full_id_ok"]]),
                    "wbc_shadow_solver_ok": number(values[index["wbc_shadow_solver_ok"]]),
                }
            )
    if not active:
        raise RuntimeError(f"{path}: no active continuous-trot rows")
    return active, {"active_start_cmd_time_s": str(active[0]["cmd_time_s"]), "tracking_lead_mps": str(lead)}


def stats_for(rows: list[dict[str, float]], scope: str, scenario: str, run: str, target: float, start: float, stop: float) -> dict[str, object]:
    window = [row for row in rows if start + 4.0 <= row["t_rel_s"] < stop]
    fields = {
        "requested_median_mps": [row["requested_mps"] for row in window],
        "shaped_median_mps": [row["shaped_mps"] for row in window],
        "applied_median_mps": [row["applied_mps"] for row in window],
        "measured_median_mps": [row["measured_mps"] for row in window],
        "measured_p05_mps": [row["measured_mps"] for row in window],
        "measured_p95_mps": [row["measured_mps"] for row in window],
        "planner_semantic_speed_median_mps": [row["planner_semantic_speed_mps"] for row in window],
        "stance_kinematic_speed_median_mps": [row["stance_kinematic_speed_mps"] for row in window],
        "measured_minus_applied_median_mps": [row["measured_minus_applied_mps"] for row in window],
        "measured_minus_planner_semantic_median_mps": [row["measured_minus_planner_semantic_mps"] for row in window],
        "measured_minus_stance_kinematic_median_mps": [row["measured_minus_stance_kinematic_mps"] for row in window],
        "abs_error_planner_median_mps": [abs(row["measured_minus_planner_semantic_mps"]) for row in window],
        "abs_error_stance_median_mps": [abs(row["measured_minus_stance_kinematic_mps"]) for row in window],
    }
    result: dict[str, object] = {
        "scope": scope,
        "scenario": scenario,
        "run": run,
        "platform_target_mps": target,
        "steady_window_start_s": start + 4.0,
        "steady_window_end_s": stop,
        "sample_count": len(window),
    }
    for key, values in fields.items():
        if key.endswith("p05_mps"):
            result[key] = percentile(values, 0.05)
        elif key.endswith("p95_mps"):
            result[key] = percentile(values, 0.95)
        elif key.endswith("_median_mps"):
            result[key] = median(values)
        else:
            result[key] = median(values)
    planner_abs = [abs(row["measured_minus_planner_semantic_mps"]) for row in window]
    stance_abs = [abs(row["measured_minus_stance_kinematic_mps"]) for row in window]
    result["planner_closer_fraction"] = (
        sum(planner < stance for planner, stance in zip(planner_abs, stance_abs)) / len(window)
        if window else float("nan")
    )
    result["stance_closer_fraction"] = (
        sum(stance < planner for planner, stance in zip(planner_abs, stance_abs)) / len(window)
        if window else float("nan")
    )
    result["mean_abs_error_planner_mps"] = mean(planner_abs)
    result["mean_abs_error_stance_mps"] = mean(stance_abs)
    return result


def first_event(rows: list[dict[str, float]], start: float, stop: float, predicate) -> dict[str, float] | None:
    for row in rows:
        if start <= row["t_rel_s"] < stop and predicate(row):
            return {"time_s": row["t_rel_s"], "delta_s": row["t_rel_s"] - start, "value_mps": row["measured_mps"]}
    return None


def transition_timing(rows: list[dict[str, float]], scenario: str, run: str) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    lead = rows[0]["tracking_lead_mps"]
    for index, (start, endpoint, from_mps, to_mps) in enumerate(TRANSITIONS[scenario]):
        stop = TRANSITIONS[scenario][index + 1][0] if index + 1 < len(TRANSITIONS[scenario]) else 56.0
        event_map = {
            "measured_gt_shaped": lambda row: row["measured_mps"] > row["shaped_mps"],
            "measured_gt_shaped_plus_lead": lambda row: row["measured_mps"] - row["shaped_mps"] > lead,
            "applied_lt_shaped": lambda row: row["applied_mps"] < row["shaped_mps"],
            "overspeed_braking_branch": lambda row: row["shaped_mps"] > 0.90 and row["measured_mps"] - row["shaped_mps"] > lead,
        }
        record: dict[str, object] = {
            "scenario": scenario,
            "run": run,
            "transition_s": start,
            "transition_endpoint_s": endpoint,
            "from_mps": from_mps,
            "to_mps": to_mps,
            "window_end_s": stop,
            "tracking_lead_mps": lead,
            "event_definition": "first active continuous-trot row at or after transition; strict inequalities; overspeed branch reconstructed from exact source condition",
        }
        for name, predicate in event_map.items():
            event = first_event(rows, start, stop, predicate)
            record[f"{name}_time_s"] = event["time_s"] if event else float("nan")
            record[f"{name}_delta_s"] = event["delta_s"] if event else float("nan")
            record[f"{name}_value_mps"] = event["value_mps"] if event else float("nan")
        for threshold in (0.05, 0.10, 0.20):
            event = first_event(rows, start, stop, lambda row, threshold=threshold: row["shaped_minus_applied_mps"] >= threshold)
            label = str(threshold).replace(".", "")
            record[f"gap_{label}_time_s"] = event["time_s"] if event else float("nan")
            record[f"gap_{label}_delta_s"] = event["delta_s"] if event else float("nan")
        a = record["applied_lt_shaped_time_s"]
        o = record["overspeed_braking_branch_time_s"]
        record["applied_minus_overspeed_s"] = a - o if math.isfinite(a) and math.isfinite(o) else float("nan")
        record["overspeed_precedes_applied_reduction"] = bool(math.isfinite(a) and math.isfinite(o) and o <= a + 1.0e-12)
        result.append(record)
    return result


def run_git(repo: Path, args: list[str]) -> str:
    completed = subprocess.run(["git", "-C", str(repo), *args], check=True, text=True, capture_output=True)
    return completed.stdout.strip()


def history_evidence(repo: Path) -> list[dict[str, object]]:
    result = []
    for commit in HISTORY_COMMITS:
        parents = run_git(repo, ["rev-list", "--parents", "-n", "1", commit]).split()
        parent = parents[1] if len(parents) > 1 else ""
        info = run_git(repo, ["show", "-s", "--format=%H|%P|%ad|%s", "--date=iso-strict", commit])
        changed = run_git(repo, ["diff-tree", "--no-commit-id", "--name-status", "-r", "--find-renames", commit]).splitlines()
        diff = run_git(repo, ["diff", parent, commit, "--unified=0"]) if parent else ""
        keyword_lines = [line for line in diff.splitlines() if any(keyword in line for keyword in KEYWORDS)]
        result.append({
            "commit": commit,
            "parent": parent,
            "info": info,
            "changed_files": changed,
            "keyword_diff_lines": keyword_lines[:240],
        })
    return result


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fields = fields or list(rows[0])
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json_value(row.get(key, "")) for key in fields})


def xml(value: object) -> str:
    text = str(value)
    return (text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;"))


def write_steady_svg(path: Path, aggregate: list[dict[str, object]]) -> None:
    width, height = 1120, 560
    left, right, top, bottom = 72, 26, 42, 62
    y_max = max([number(row["platform_target_mps"]) for row in aggregate] + [3.0]) * 1.25
    plot_w = width - left - right
    plot_h = height - top - bottom
    groups = [(str(row["scenario"]), number(row["platform_target_mps"]), row) for row in aggregate]
    x_step = plot_w / max(1, len(groups))
    def x(index: int) -> float:
        return left + x_step * (index + 0.5)
    def y(value: float) -> float:
        return height - bottom - value / y_max * plot_h
    colors = {"measured": "#202020", "planner": "#1565c0", "stance": "#ef6c00"}
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>', '<style>text{font-family:Arial,sans-serif;fill:#202020}</style>']
    lines.append('<text x="72" y="24" font-size="17">Steady platform speed semantics (pooled across 3 repeats)</text>')
    lines.append(f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#444"/>')
    lines.append(f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="#444"/>')
    for tick in range(0, 4):
        value = tick
        yy = y(value)
        lines.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{width-right}" y2="{yy:.1f}" stroke="#dddddd"/>')
        lines.append(f'<text x="{left-10}" y="{yy+4:.1f}" text-anchor="end" font-size="12">{value}</text>')
    for i, (scenario, target, row) in enumerate(groups):
        xx = x(i)
        vals = [(number(row["measured_median_mps"]), "measured"), (number(row["planner_semantic_speed_median_mps"]), "planner"), (number(row["stance_kinematic_speed_median_mps"]), "stance")]
        for offset, (value, label) in enumerate(vals):
            bx = xx + (offset - 1) * 14
            yy = y(value)
            lines.append(f'<line x1="{bx:.1f}" y1="{height-bottom}" x2="{bx:.1f}" y2="{yy:.1f}" stroke="{colors[label]}" stroke-width="9"/>')
        lines.append(f'<text x="{xx:.1f}" y="{height-bottom+18}" text-anchor="middle" font-size="12">{xml(scenario)} {target:g}</text>')
    for index, label in enumerate(("measured median", "planner semantic", "stance kinematic")):
        color = colors[("measured", "planner", "stance")[index]]
        xx = width - 290 + index * 95
        lines.append(f'<line x1="{xx}" y1="{top-10}" x2="{xx+18}" y2="{top-10}" stroke="{color}" stroke-width="5"/>')
        lines.append(f'<text x="{xx+23}" y="{top-6}" font-size="11">{label}</text>')
    lines.append(f'<text x="{width/2:.1f}" y="{height-12}" text-anchor="middle" font-size="13">platform (m/s)</text>')
    lines.append(f'<text x="16" y="{height/2:.1f}" font-size="13" transform="rotate(-90 16 {height/2:.1f})">speed (m/s)</text>')
    lines.append('</svg>')
    path.write_text("\n".join(lines))


def write_timing_svg(path: Path, timing: list[dict[str, object]]) -> None:
    width, height = 1200, 700
    left, right, top, bottom = 270, 24, 36, 48
    rows = timing
    y_step = (height - top - bottom) / max(1, len(rows))
    x_max = 10.0
    def x(value: float) -> float:
        return left + max(0.0, min(x_max, value)) / x_max * (width - left - right)
    colors = {"measured_gt_shaped_delta_s": "#202020", "overspeed_braking_branch_delta_s": "#1565c0", "applied_lt_shaped_delta_s": "#ef6c00", "gap_05_delta_s": "#2e7d32", "gap_10_delta_s": "#9c27b0", "gap_20_delta_s": "#b71c1c"}
    labels = {"measured_gt_shaped_delta_s": "measured > shaped", "overspeed_braking_branch_delta_s": "overspeed branch", "applied_lt_shaped_delta_s": "applied < shaped", "gap_05_delta_s": "gap >= .05", "gap_10_delta_s": "gap >= .10", "gap_20_delta_s": "gap >= .20"}
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">', '<rect width="100%" height="100%" fill="white"/>', '<style>text{font-family:Arial,sans-serif;fill:#202020}</style>', '<text x="72" y="23" font-size="17">Transition event timing from each profile rise onset</text>']
    lines.append(f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#444"/>')
    for tick in (0.0, 2.0, 4.0, 6.0, 8.0, 10.0):
        xx = x(tick)
        lines.append(f'<line x1="{xx:.1f}" y1="{top}" x2="{xx:.1f}" y2="{height-bottom}" stroke="#dddddd"/>')
        lines.append(f'<text x="{xx:.1f}" y="{height-bottom+18}" text-anchor="middle" font-size="12">{tick:g}s</text>')
    for i, row in enumerate(rows):
        yy = top + y_step * (i + 0.5)
        name = f'{row["scenario"]} {row["run"][-6:]} {row["from_mps"]:g}->{row["to_mps"]:g}'
        lines.append(f'<text x="{left-10}" y="{yy+4:.1f}" text-anchor="end" font-size="11">{xml(name)}</text>')
        for key, color in colors.items():
            value = number(row.get(key))
            if math.isfinite(value):
                xx = x(value)
                lines.append(f'<circle cx="{xx:.1f}" cy="{yy:.1f}" r="4" fill="{color}"/>')
    for i, key in enumerate(colors):
        xx = width - 360 + (i % 3) * 115
        yy = 22 + (i // 3) * 18
        lines.append(f'<circle cx="{xx}" cy="{yy-4}" r="4" fill="{colors[key]}"/>')
        lines.append(f'<text x="{xx+8}" y="{yy}" font-size="11">{labels[key]}</text>')
    lines.append('</svg>')
    path.write_text("\n".join(lines))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repo", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    all_samples: list[dict[str, object]] = []
    all_timing: list[dict[str, object]] = []
    aligned_samples: list[dict[str, object]] = []
    run_summaries: list[dict[str, object]] = []
    provenance: list[dict[str, object]] = []
    platform_rows: list[dict[str, object]] = []
    platform_values: dict[tuple[str, float], list[dict[str, float]]] = {}

    for scenario, run_names in RUNS.items():
        for run_name in run_names:
            run = args.root / run_name
            data_path = run / "data.csv"
            active, load_meta = load_active(run, scenario)
            data_hash = sha256(data_path)
            manifest_hash = sha256(run / "run_manifest.json")
            metadata_hash = sha256(run / "run_metadata.txt")
            provenance.append({
                "scenario": scenario,
                "run": run_name,
                "run_relative_path": str(run.relative_to(args.root.parent.parent.parent.parent.parent)),
                "data_csv_sha256": data_hash,
                "run_manifest_sha256": manifest_hash,
                "run_metadata_sha256": metadata_hash,
                "active_rows": len(active),
                "active_start_cmd_time_s": number(load_meta["active_start_cmd_time_s"]),
                "tracking_lead_mps": number(load_meta["tracking_lead_mps"]),
            })
            for sample in active:
                all_samples.append({"scenario": scenario, "run": run_name, **sample})
            timings = transition_timing(active, scenario, run_name)
            all_timing.extend(timings)
            for start, endpoint, from_mps, to_mps in TRANSITIONS[scenario]:
                for offset in (0.0, 0.25, 0.50, 1.0):
                    target_time = start + offset
                    row = min(active, key=lambda item: abs(item["t_rel_s"] - target_time))
                    aligned_samples.append({
                        "scenario": scenario,
                        "run": run_name,
                        "transition_s": start,
                        "transition_endpoint_s": endpoint,
                        "from_mps": from_mps,
                        "to_mps": to_mps,
                        "offset_s": offset,
                        "row_t_rel_s": row["t_rel_s"],
                        "alignment_abs_error_s": abs(row["t_rel_s"] - target_time),
                        "t_rel_s": row["t_rel_s"],
                        **{key: value for key, value in row.items() if key != "t_rel_s"},
                    })
            run_summaries.append({
                "scenario": scenario,
                "run": run_name,
                "active_rows": len(active),
                "active_duration_s": active[-1]["t_rel_s"],
                "data_csv_sha256": data_hash,
                "tracking_lead_mps": active[0]["tracking_lead_mps"],
                "period_min_s": min(row["period_s"] for row in active),
                "period_max_s": max(row["period_s"] for row in active),
                "duty_min": min(row["duty"] for row in active),
                "duty_max": max(row["duty"] for row in active),
            })
            for target, start, stop in PLATFORMS[scenario]:
                platform_values.setdefault((scenario, target), []).extend(
                    [row for row in active if start + 4.0 <= row["t_rel_s"] < stop]
                )
                platform_rows.append(stats_for(active, "run", scenario, run_name, target, start, stop))

    aggregate_rows: list[dict[str, object]] = []
    for (scenario, target), rows in platform_values.items():
        start, stop = next((start, stop) for value, start, stop in PLATFORMS[scenario] if value == target)
        pooled = stats_for(rows, "repeat_pool", scenario, "3_repeats", target, start - 4.0, stop)
        pooled["steady_window_start_s"] = start + 4.0
        pooled["steady_window_end_s"] = stop
        per_run = [row for row in platform_rows if row["scenario"] == scenario and row["platform_target_mps"] == target]
        pooled["repeat_count"] = len(per_run)
        pooled["measured_median_repeat_min_mps"] = min(number(row["measured_median_mps"]) for row in per_run)
        pooled["measured_median_repeat_max_mps"] = max(number(row["measured_median_mps"]) for row in per_run)
        pooled["planner_abs_error_repeat_median_min_mps"] = min(number(row["abs_error_planner_median_mps"]) for row in per_run)
        pooled["planner_abs_error_repeat_median_max_mps"] = max(number(row["abs_error_planner_median_mps"]) for row in per_run)
        pooled["stance_abs_error_repeat_median_min_mps"] = min(number(row["abs_error_stance_median_mps"]) for row in per_run)
        pooled["stance_abs_error_repeat_median_max_mps"] = max(number(row["abs_error_stance_median_mps"]) for row in per_run)
        aggregate_rows.append(pooled)

    write_csv(args.out / "active_velocity_semantics.csv", all_samples, SAMPLE_FIELDS)
    write_csv(args.out / "platform_steady_stats.csv", platform_rows + aggregate_rows)
    write_csv(args.out / "transition_causal_timing.csv", all_timing)
    write_csv(args.out / "transition_aligned_samples.csv", aligned_samples, ALIGNED_FIELDS)
    write_csv(args.out / "raw_run_sha256.csv", provenance)
    (args.out / "run_summaries.json").write_text(json.dumps(json_value(run_summaries), indent=2) + "\n")
    (args.out / "platform_steady_stats.json").write_text(json.dumps(json_value(platform_rows + aggregate_rows), indent=2) + "\n")
    (args.out / "transition_causal_timing.json").write_text(json.dumps(json_value(all_timing), indent=2) + "\n")
    (args.out / "transition_aligned_samples.json").write_text(json.dumps(json_value(aligned_samples), indent=2) + "\n")
    (args.out / "raw_run_sha256.json").write_text(json.dumps(json_value(provenance), indent=2) + "\n")
    (args.out / "history_evidence.json").write_text(json.dumps(json_value(history_evidence(args.repo)), indent=2) + "\n")
    write_steady_svg(args.out / "steady_speed_semantics.svg", aggregate_rows)
    write_timing_svg(args.out / "transition_causal_timing.svg", all_timing)

    summary = {
        "analysis": "Phase1 velocity semantic and governor causality audit",
        "controller_reproduction_sha": "a1d4e294092a39c8e649eda6f285ea2cfe01b05d",
        "source_branch": "research/phase1-velocity-failure-attribution-20260913",
        "source_commit": "cb0b7eb33d8ea2ae87ee92ada353ad814a5f087c",
        "run_count": len(run_summaries),
        "active_row_count": len(all_samples),
        "platform_steady_window": "last 4 seconds of each 8 second constant platform: [start+4, stop); varying 0.6 uses [4,8)",
        "transition_event_window": "profile rise onset to next rise onset; endpoint is recorded separately because the profile sampler linearly interpolates",
        "tracking_lead_source": "run_metadata argv, expected 0.20 m/s",
        "overspeed_branch_source_condition": "shaped > 0.90 and measured - shaped > tracking_lead",
        "outputs": sorted(path.name for path in args.out.iterdir() if path.is_file()),
    }
    (args.out / "analysis_summary.json").write_text(json.dumps(json_value(summary), indent=2) + "\n")
    print(json.dumps(json_value(summary), indent=2))


if __name__ == "__main__":
    main()
