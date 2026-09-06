#!/usr/bin/env python3
"""Read-only dynamic quality audit for a B1 stance-ablation raw run.

This is an evidence script, not an acceptance analyzer.  It joins the two
streams on the exact LowState tick/ground-truth physics time, reports every
cycle's planned and measured topology, separates the scheduler's modal
running pair from its transition/brake cycles, and integrates the exact
per-foot non-top terrain force.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import json
import math
import pathlib
import statistics
from typing import Iterable


LEGS = ("FR", "FL", "RR", "RL")
DIAGONAL_MASKS = (9, 6)  # FR+RL, FL+RR, source leg order in simulator CSV.
CONTACT_FORCE_N = 5.0  # existing diagnostic convention; not a v2 gate.
STALL_SPEED_MPS = 0.10  # reported descriptive stop/stall band; not a v2 gate.
STEP_X_MIN_M = 0.70  # phase2_step_5cm.xml: center .95, half-size .25.
STEP_X_MAX_M = 1.20


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or not rows[0]:
        raise ValueError(f"empty CSV: {path}")
    return rows


def number(row: dict[str, str], key: str) -> float:
    return float(row[key])


def percentile(values: Iterable[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = (len(ordered) - 1) * q / 100.0
    lo, hi = math.floor(index), math.ceil(index)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def median_or_nan(values: Iterable[float]) -> float:
    items = list(values)
    return statistics.median(items) if items else math.nan


def mask_from_truth(row: dict[str, str]) -> int:
    mask = 0
    for bit, leg in enumerate(LEGS):
        if number(row, f"{leg}_foot_contact_grf_world_z_N") >= CONTACT_FORCE_N:
            mask |= 1 << bit
    return mask


def top_mask_from_truth(row: dict[str, str]) -> int:
    mask = 0
    for bit, leg in enumerate(LEGS):
        if number(row, f"{leg}_terrain_top_grf_world_z_N") >= CONTACT_FORCE_N:
            mask |= 1 << bit
    return mask


def total_contact_force_norm(row: dict[str, str]) -> float:
    """Return all robot/environment contact force, including non-foot geoms."""
    return math.sqrt(
        sum(
            number(row, f"total_contact_grf_world_{axis}_N") ** 2
            for axis in ("x", "y", "z")
        )
    )


def is_aerial(row: dict[str, str]) -> bool:
    """Aerial is no total contact force, not merely no foot support."""
    return total_contact_force_norm(row) < CONTACT_FORCE_N


def mode_pair(rows: Iterable[dict[str, str]]) -> tuple[float, float]:
    pairs = collections.Counter(
        (
            round(number(row, "velocity_command_gait_period_s"), 6),
            round(number(row, "velocity_command_gait_duty"), 6),
        )
        for row in rows
        if row.get("velocity_command_gait_regime", "") == "continuous-trot"
        and int(float(row.get("motion_stage", "-1"))) == 2
    )
    if not pairs:
        return math.nan, math.nan
    return pairs.most_common(1)[0][0]


def consecutive_duration(rows: list[dict[str, str]], predicate) -> float:
    """Maximum duration of a predicate run using row timestamps."""
    best = 0.0
    start = None
    previous = None
    for row in rows:
        t = number(row, "time_s")
        if predicate(row):
            if start is None:
                start = t
            previous = t
        elif start is not None:
            best = max(best, previous - start)
            start = previous = None
    if start is not None:
        best = max(best, previous - start)
    return best


def force_summary(rows: list[dict[str, str]], key: str) -> dict[str, float]:
    values = [max(0.0, number(row, key)) for row in rows]
    if not values:
        return {"max_N": math.nan, "impulse_Ns": math.nan, "positive_samples": 0}
    # Ground truth is a fixed 2 ms physics stream in this run.  Use the actual
    # adjacent dt so the script exposes a changed sample clock instead of
    # silently assuming it.
    impulse = 0.0
    previous_t = None
    for row, value in zip(rows, values):
        t = number(row, "time_s")
        if previous_t is not None:
            impulse += value * (t - previous_t)
        previous_t = t
    return {
        "max_N": max(values),
        "impulse_Ns": impulse,
        "positive_samples": sum(value > 0.0 for value in values),
    }


def cycle_summary(
    cycle: int,
    controller_rows: list[dict[str, str]],
    truth_rows: list[dict[str, str]],
) -> dict[str, object]:
    state_times = [number(row, "state_tick_s") for row in controller_rows]
    start_s, end_s = min(state_times), max(state_times)
    gt = [row for row in truth_rows if start_s <= number(row, "time_s") <= end_s]
    unique_masks = [mask_from_truth(row) for row in gt]
    top_masks = [top_mask_from_truth(row) for row in gt]
    wbc_request = collections.Counter(
        int(float(row["wbc_scheduled_contact_mask"])) for row in controller_rows
    )
    terrain_planned = collections.Counter(
        int(float(row["terrain_execution_planned_contact_mask"]))
        for row in controller_rows
    )
    actual = collections.Counter(unique_masks)
    top = collections.Counter(top_masks)
    diagonal_sequence = [mask for mask in unique_masks if mask in DIAGONAL_MASKS]
    by_state_tick = {}
    for row in controller_rows:
        by_state_tick.setdefault(number(row, "state_tick_s"), row)
    matched_wbc = [
        (
            mask_from_truth(row),
            int(float(by_state_tick[number(row, "time_s")]["wbc_scheduled_contact_mask"])),
        )
        for row in gt
        if number(row, "time_s") in by_state_tick
    ]
    matched_terrain_plan = [
        (
            mask_from_truth(row),
            int(float(by_state_tick[number(row, "time_s")]["terrain_execution_planned_contact_mask"])),
        )
        for row in gt
        if number(row, "time_s") in by_state_tick
    ]
    rows_out: dict[str, object] = {
        "cycle_index": cycle,
        "controller_state_time_s": [start_s, end_s],
        "truth_rows": len(gt),
        "phase_min": min(number(row, "phase") for row in controller_rows),
        "phase_max": max(number(row, "phase") for row in controller_rows),
        "period_s": median_or_nan(
            number(row, "velocity_command_gait_period_s") for row in controller_rows
        ),
        "duty_factor": median_or_nan(
            number(row, "velocity_command_gait_duty") for row in controller_rows
        ),
        "regimes": dict(collections.Counter(
            row.get("velocity_command_gait_regime", "")
            for row in controller_rows
        )),
        "applied_speed_mps": {
            "p05": percentile(
                (number(row, "velocity_command_applied_mps") for row in controller_rows),
                5,
            ),
            "p50": percentile(
                (number(row, "velocity_command_applied_mps") for row in controller_rows),
                50,
            ),
            "p95": percentile(
                (number(row, "velocity_command_applied_mps") for row in controller_rows),
                95,
            ),
        },
        "measured_speed_mps": {
            "p05": percentile(
                (number(row, "velocity_command_measured_mps") for row in controller_rows),
                5,
            ),
            "p50": percentile(
                (number(row, "velocity_command_measured_mps") for row in controller_rows),
                50,
            ),
            "p95": percentile(
                (number(row, "velocity_command_measured_mps") for row in controller_rows),
                95,
            ),
        },
        "world_speed_gt_mps": {
            "p05": percentile(
                (number(row, "base_qvel_world_x_mps") for row in gt), 5
            ),
            "p50": percentile(
                (number(row, "base_qvel_world_x_mps") for row in gt), 50
            ),
            "p95": percentile(
                (number(row, "base_qvel_world_x_mps") for row in gt), 95
            ),
            "min": min((number(row, "base_qvel_world_x_mps") for row in gt), default=math.nan),
            "max": max((number(row, "base_qvel_world_x_mps") for row in gt), default=math.nan),
        },
        "wbc_contact_request_mask_counts": dict(wbc_request),
        "planned_terrain_mask_counts": dict(terrain_planned),
        "actual_foot_contact_mask_counts": dict(actual),
        "actual_top_support_mask_counts": dict(top),
        "wbc_request_vs_actual_exact_mask_fraction": (
            sum(actual_mask == planned_mask for actual_mask, planned_mask in matched_wbc)
            / max(1, len(matched_wbc))
        ),
        "terrain_plan_vs_actual_exact_mask_fraction": (
            sum(actual_mask == planned_mask for actual_mask, planned_mask in matched_terrain_plan)
            / max(1, len(matched_terrain_plan))
        ),
        "diagonal_alternations": sum(
            a != b for a, b in zip(diagonal_sequence, diagonal_sequence[1:])
        ),
        "diagonal_sequence_samples": len(diagonal_sequence),
        "diagonal_alternation_fraction": sum(
            a != b for a, b in zip(diagonal_sequence, diagonal_sequence[1:])
        ) / max(1, len(diagonal_sequence) - 1),
        "actual_diagonal_fraction": {
            str(mask): unique_masks.count(mask) / max(1, len(unique_masks))
            for mask in DIAGONAL_MASKS
        },
        "aerial_fraction": sum(is_aerial(row) for row in gt) / max(1, len(gt)),
        "single_contact_fraction": sum(mask in (1, 2, 4, 8) for mask in unique_masks)
        / max(1, len(unique_masks)),
        "non_top_force": {
            leg: force_summary(gt, f"{leg}_terrain_nontop_contact_force_N")
            for leg in LEGS
        },
    }
    return rows_out


def stable_metrics(rows: list[dict[str, str]], truth_rows: list[dict[str, str]]) -> dict[str, object]:
    if not rows:
        return {"controller_rows": 0, "truth_rows": 0}
    start_s = min(number(row, "state_tick_s") for row in rows)
    end_s = max(number(row, "state_tick_s") for row in rows)
    gt = [row for row in truth_rows if start_s <= number(row, "time_s") <= end_s]
    masks = [mask_from_truth(row) for row in gt]
    top_masks = [top_mask_from_truth(row) for row in gt]
    diagonal_sequence = [mask for mask in masks if mask in DIAGONAL_MASKS]
    transitions = sum(a != b for a, b in zip(diagonal_sequence, diagonal_sequence[1:]))
    by_state_tick = {}
    for row in rows:
        by_state_tick.setdefault(number(row, "state_tick_s"), row)
    matched_wbc = [
        (
            mask_from_truth(gt_row),
            int(float(by_state_tick[number(gt_row, "time_s")]["wbc_scheduled_contact_mask"])),
        )
        for gt_row in gt
        if number(gt_row, "time_s") in by_state_tick
    ]
    matched_terrain_plan = [
        (
            mask_from_truth(gt_row),
            int(float(by_state_tick[number(gt_row, "time_s")]["terrain_execution_planned_contact_mask"])),
        )
        for gt_row in gt
        if number(gt_row, "time_s") in by_state_tick
    ]
    result: dict[str, object] = {
        "controller_time_s": [start_s, end_s],
        "controller_rows": len(rows),
        "truth_rows": len(gt),
        "command_applied_mps": {
            "p05": percentile((number(r, "velocity_command_applied_mps") for r in rows), 5),
            "p50": percentile((number(r, "velocity_command_applied_mps") for r in rows), 50),
            "p95": percentile((number(r, "velocity_command_applied_mps") for r in rows), 95),
        },
        "measured_command_mps": {
            "p05": percentile((number(r, "velocity_command_measured_mps") for r in rows), 5),
            "p50": percentile((number(r, "velocity_command_measured_mps") for r in rows), 50),
            "p95": percentile((number(r, "velocity_command_measured_mps") for r in rows), 95),
        },
        "ground_truth_forward_speed_mps": {
            "p05": percentile((number(r, "base_qvel_world_x_mps") for r in gt), 5),
            "p50": percentile((number(r, "base_qvel_world_x_mps") for r in gt), 50),
            "p95": percentile((number(r, "base_qvel_world_x_mps") for r in gt), 95),
            "min": min((number(r, "base_qvel_world_x_mps") for r in gt), default=math.nan),
            "max": max((number(r, "base_qvel_world_x_mps") for r in gt), default=math.nan),
            "stall_band_fraction": sum(
                abs(number(r, "base_qvel_world_x_mps")) <= STALL_SPEED_MPS for r in gt
            ) / max(1, len(gt)),
            "distance_m": (
                number(gt[-1], "base_pos_world_x_m") - number(gt[0], "base_pos_world_x_m")
                if gt else math.nan
            ),
        },
        "actual_contact_mask_counts": dict(collections.Counter(masks)),
        "wbc_contact_request_mask_counts": dict(
            collections.Counter(item[1] for item in matched_wbc)
        ),
        "terrain_planned_mask_counts": dict(
            collections.Counter(item[1] for item in matched_terrain_plan)
        ),
        "wbc_request_vs_actual_exact_mask_fraction": (
            sum(actual == planned for actual, planned in matched_wbc)
            / max(1, len(matched_wbc))
        ),
        "terrain_plan_vs_actual_exact_mask_fraction": (
            sum(actual == planned for actual, planned in matched_terrain_plan)
            / max(1, len(matched_terrain_plan))
        ),
        "actual_contact_fraction_by_leg": {
            leg: sum(mask & (1 << bit) != 0 for mask in masks) / max(1, len(masks))
            for bit, leg in enumerate(LEGS)
        },
        "actual_diagonal_fraction": {
            str(mask): sum(item == mask for item in masks) / max(1, len(masks))
            for mask in DIAGONAL_MASKS
        },
        "actual_top_support_fraction_by_leg": {
            leg: sum(mask & (1 << bit) != 0 for mask in top_masks) / max(1, len(top_masks))
            for bit, leg in enumerate(LEGS)
        },
        "top_force_by_leg": {
            leg: force_summary(gt, f"{leg}_terrain_top_grf_world_z_N")
            for leg in LEGS
        },
        "longest_top_support_s_by_leg": {
            leg: consecutive_duration(
                gt,
                lambda row, name=f"{leg}_terrain_top_grf_world_z_N": number(row, name)
                >= CONTACT_FORCE_N,
            )
            for leg in LEGS
        },
        "aerial_fraction": sum(is_aerial(row) for row in gt) / max(1, len(gt)),
        "single_contact_fraction": sum(mask in (1, 2, 4, 8) for mask in masks)
        / max(1, len(masks)),
        "diagonal_alternations": transitions,
        "diagonal_sequence_samples": len(diagonal_sequence),
        "diagonal_alternation_fraction": transitions
        / max(1, len(diagonal_sequence) - 1),
        "longest_all_feet_aerial_s": consecutive_duration(gt, is_aerial),
        "longest_swing_s_by_leg": {
            leg: consecutive_duration(
                gt,
                lambda row, bit=bit: not (
                    mask_from_truth(row) & (1 << bit)
                ),
            )
            for bit, leg in enumerate(LEGS)
        },
        "non_top_force_by_leg": {
            leg: force_summary(gt, f"{leg}_terrain_nontop_contact_force_N")
            for leg in LEGS
        },
        "max_non_top_force_N": max(
            (
                number(row, f"{leg}_terrain_nontop_contact_force_N")
                for row in gt
                for leg in LEGS
            ),
            default=math.nan,
        ),
        "max_top_force_N": max(
            (
                number(row, f"{leg}_terrain_top_grf_world_z_N")
                for row in gt
                for leg in LEGS
            ),
            default=math.nan,
        ),
    }
    return result


def cycle_quality_summary(
    cycle_reports: list[dict[str, object]], stable_ids: list[int]
) -> dict[str, object]:
    selected = [
        item for item in cycle_reports if item["cycle_index"] in stable_ids
    ]
    fields = (
        "aerial_fraction",
        "single_contact_fraction",
        "terrain_plan_vs_actual_exact_mask_fraction",
        "actual_diagonal_fraction",
    )
    summary: dict[str, object] = {"cycle_count": len(selected)}
    for field in fields:
        if field == "actual_diagonal_fraction":
            for diagonal in ("6", "9"):
                values = [
                    float(item[field][diagonal])
                    for item in selected
                ]
                summary[f"diagonal_{diagonal}_fraction"] = {
                    "min": min(values, default=math.nan),
                    "p05": percentile(values, 5),
                    "p50": percentile(values, 50),
                    "p95": percentile(values, 95),
                    "max": max(values, default=math.nan),
                }
        else:
            values = [float(item[field]) for item in selected]
            summary[field] = {
                "min": min(values, default=math.nan),
                "p05": percentile(values, 5),
                "p50": percentile(values, 50),
                "p95": percentile(values, 95),
                "max": max(values, default=math.nan),
            }
    return summary


def first_at_or_after(rows: list[dict[str, str]], column: str, value: float) -> float | None:
    for row in rows:
        if number(row, column) >= value:
            return number(row, "time_s")
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--json-out", type=pathlib.Path)
    args = parser.parse_args()
    data = read_csv(args.run_dir / "data.csv")
    truth = read_csv(args.run_dir / "contact_ground_truth.csv")

    # The controller's state tick is exactly representable in the truth stream
    # for this run.  Refuse to silently interpolate or row-join if that changes.
    truth_by_time = {number(row, "time_s"): row for row in truth}
    state_rows = [row for row in data if number(row, "has_state") > 0.5]
    truth_times = sorted(truth_by_time)
    association_errors = []
    for row in state_rows:
        tick = number(row, "state_tick_s")
        index = bisect.bisect_left(truth_times, tick)
        candidates = truth_times[max(0, index - 1):min(len(truth_times), index + 1)]
        association_errors.append(min((abs(tick - item) for item in candidates), default=math.nan))
    unique_state_ticks = sorted({number(row, "state_tick_s") for row in state_rows})
    gt_missing_state_ticks = [tick for tick in unique_state_ticks if tick not in truth_by_time]

    cycle_rows: dict[int, list[dict[str, str]]] = collections.defaultdict(list)
    for row in data:
        if int(float(row.get("motion_stage", "-1"))) == 2:
            cycle_rows[int(float(row["cycle_index"]))].append(row)

    all_continuous = [
        row
        for row in data
        if int(float(row.get("motion_stage", "-1"))) == 2
        and row.get("velocity_command_gait_regime", "") == "continuous-trot"
    ]
    modal_period, modal_duty = mode_pair(data)
    stable_cycle_ids = []
    for cycle, rows in sorted(cycle_rows.items()):
        pair_values = {
            (
                round(number(row, "velocity_command_gait_period_s"), 6),
                round(number(row, "velocity_command_gait_duty"), 6),
            )
            for row in rows
        }
        regimes = {row.get("velocity_command_gait_regime", "") for row in rows}
        if pair_values == {(modal_period, modal_duty)} and regimes == {"continuous-trot"}:
            stable_cycle_ids.append(cycle)

    stable_rows = [row for cycle in stable_cycle_ids for row in cycle_rows[cycle]]
    transition_ids = [cycle for cycle in cycle_rows if cycle < min(stable_cycle_ids, default=10**9)]
    brake_ids = [cycle for cycle in cycle_rows if cycle > max(stable_cycle_ids, default=-1)]

    base_crossing = {
        "step_x_interval_m": [STEP_X_MIN_M, STEP_X_MAX_M],
        "front_edge_time_s": first_at_or_after(truth, "base_pos_world_x_m", STEP_X_MIN_M),
        "rear_edge_time_s": first_at_or_after(truth, "base_pos_world_x_m", STEP_X_MAX_M),
        "all_feet_rear_edge_time_s": first(
            (
                number(row, "time_s")
                for row in truth
                if all(number(row, f"{leg}_pos_world_x_m") >= STEP_X_MAX_M for leg in LEGS)
            ),
            None,
        ),
    }
    crossing_start = base_crossing["front_edge_time_s"]
    crossing_end = base_crossing["all_feet_rear_edge_time_s"]
    crossing_rows = []
    if crossing_start is not None and crossing_end is not None:
        crossing_rows = [
            row
            for row in data
            if number(row, "has_state") > 0.5
            and crossing_start <= number(row, "state_tick_s") <= crossing_end
        ]
    crossing_gt = []
    if crossing_start is not None and crossing_end is not None:
        crossing_gt = [
            row
            for row in truth
            if crossing_start <= number(row, "time_s") <= crossing_end
        ]
    terrain_columns = [
        f"{leg}_{suffix}"
        for leg in LEGS
        for suffix in (
            "terrain_top_grf_world_z_N",
            "terrain_nontop_contact_force_N",
        )
    ]
    terrain_contact_rows = [
        row
        for row in truth
        if any(number(row, column) > 0.0 for column in terrain_columns)
    ]
    terrain_interaction_start = (
        number(terrain_contact_rows[0], "time_s") if terrain_contact_rows else None
    )
    terrain_interaction_end = (
        number(terrain_contact_rows[-1], "time_s") if terrain_contact_rows else None
    )
    terrain_interaction_controller_rows = [
        row
        for row in data
        if number(row, "has_state") > 0.5
        and terrain_interaction_start is not None
        and terrain_interaction_start <= number(row, "state_tick_s") <= terrain_interaction_end
    ]

    cycle_reports = [
        cycle_summary(cycle, cycle_rows[cycle], truth)
        for cycle in sorted(cycle_rows)
    ]
    result: dict[str, object] = {
        "run": args.run_dir.name,
        "contact_force_convention_N": CONTACT_FORCE_N,
        "stall_band_convention_mps": STALL_SPEED_MPS,
        "csv": {
            "data_rows": len(data),
            "truth_rows": len(truth),
            "data_time_s": [number(data[0], "cmd_time_s"), number(data[-1], "cmd_time_s")],
            "truth_time_s": [number(truth[0], "time_s"), number(truth[-1], "time_s")],
            "truth_dt_s": median_or_nan(
                number(b, "time_s") - number(a, "time_s") for a, b in zip(truth, truth[1:])
            ),
            "state_tick_unique": len(unique_state_ticks),
            "state_tick_reused_rows": len(state_rows) - len(unique_state_ticks),
            "state_to_truth_max_error_s": max(association_errors, default=math.nan),
            "state_ticks_missing_in_truth": gt_missing_state_ticks,
        },
        "step_crossing": base_crossing,
        "gait_segmentation": {
            "modal_running_period_s": modal_period,
            "modal_running_duty_factor": modal_duty,
            "transition_cycle_indices": transition_ids,
            "stable_running_cycle_indices": stable_cycle_ids,
            "brake_or_tail_cycle_indices": brake_ids,
            "continuous_rows_before_brake": len(all_continuous),
            "continuous_period_range_s": [
                min((number(row, "velocity_command_gait_period_s") for row in all_continuous), default=math.nan),
                max((number(row, "velocity_command_gait_period_s") for row in all_continuous), default=math.nan),
            ],
            "continuous_duty_range": [
                min((number(row, "velocity_command_gait_duty") for row in all_continuous), default=math.nan),
                max((number(row, "velocity_command_gait_duty") for row in all_continuous), default=math.nan),
            ],
        },
        "cycles": cycle_reports,
        "stable_cycle_quality": cycle_quality_summary(
            cycle_reports, stable_cycle_ids
        ),
        "physical_crossing_window": {
            "controller_rows": len(crossing_rows),
            "controller_cycle_indices": sorted({
                int(float(row["cycle_index"])) for row in crossing_rows
            }),
            "controller_regimes": dict(collections.Counter(
                row.get("velocity_command_gait_regime", "")
                for row in crossing_rows
            )),
            "controller_period_range_s": [
                min((number(row, "velocity_command_gait_period_s") for row in crossing_rows), default=math.nan),
                max((number(row, "velocity_command_gait_period_s") for row in crossing_rows), default=math.nan),
            ],
            "controller_duty_range": [
                min((number(row, "velocity_command_gait_duty") for row in crossing_rows), default=math.nan),
                max((number(row, "velocity_command_gait_duty") for row in crossing_rows), default=math.nan),
            ],
            "truth_rows": len(crossing_gt),
            "truth_metrics": stable_metrics(crossing_rows, truth),
        },
        "terrain_interaction_window": {
            "truth_time_s": [terrain_interaction_start, terrain_interaction_end],
            "controller_rows": len(terrain_interaction_controller_rows),
            "controller_cycle_indices": sorted({
                int(float(row["cycle_index"]))
                for row in terrain_interaction_controller_rows
            }),
            "truth_metrics": stable_metrics(
                terrain_interaction_controller_rows, truth
            ),
        },
        "stable_running": stable_metrics(stable_rows, truth),
    }
    def finite_json(value):
        if isinstance(value, float) and not math.isfinite(value): return None
        if isinstance(value, dict): return {k:finite_json(v) for k,v in value.items()}
        if isinstance(value, list): return [finite_json(v) for v in value]
        return value
    result=finite_json(result)
    if args.json_out:
        with args.json_out.open("x",encoding="utf-8") as output:
            output.write(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def first(values: Iterable[float], default: float | None) -> float | None:
    return next(iter(values), default)


if __name__ == "__main__":
    raise SystemExit(main())
