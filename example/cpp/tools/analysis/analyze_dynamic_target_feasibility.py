#!/usr/bin/env python3
"""Classify projected contact-wrench target failures by phase and component."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

TARGET_FIELDS = (
    "desired_force_x_n",
    "desired_force_y_n",
    "desired_force_z_n",
    "desired_moment_x_nm",
    "desired_moment_y_nm",
    "desired_moment_z_nm",
)
ACHIEVED_FIELDS = (
    "achieved_force_x_n",
    "achieved_force_y_n",
    "achieved_force_z_n",
    "achieved_moment_x_nm",
    "achieved_moment_y_nm",
    "achieved_moment_z_nm",
)
RESIDUAL_FIELDS = (
    "wrench_residual_force_x_n",
    "wrench_residual_force_y_n",
    "wrench_residual_force_z_n",
    "wrench_residual_moment_x_nm",
    "wrench_residual_moment_y_nm",
    "wrench_residual_moment_z_nm",
)
COMPONENT_NAMES = (
    "force_x_N",
    "force_y_N",
    "force_z_N",
    "moment_x_Nm",
    "moment_y_Nm",
    "moment_z_Nm",
)
REQUIRED_FIELDS = {
    "row_number",
    "cmd_time_s",
    "phase",
    "selected_contact_count",
    "reduced_task",
    "wrench_satisfied",
    "task_satisfied",
    "task_residual_norm",
    "residual_norm",
    "constraint_feasible",
    "max_radial_friction_ratio",
    "min_contact_normal_force",
    *TARGET_FIELDS,
    *ACHIEVED_FIELDS,
    *RESIDUAL_FIELDS,
}


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile_abs(values: list[float], fraction: float = 0.95) -> float:
    if not values:
        return 0.0
    ordered = sorted(abs(value) for value in values)
    return ordered[int(fraction * (len(ordered) - 1))]


def maximum_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def phase_bin(phase: float, bins: int) -> int:
    return min(bins - 1, max(0, int(phase * bins)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--phase-bins", type=int, default=8)
    parser.add_argument("--consistency-tolerance", type=float, default=1e-6)
    args = parser.parse_args()
    if args.phase_bins <= 0 or args.consistency_tolerance < 0.0:
        print("validation=FAIL: invalid analysis parameters")
        return 2

    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or [])
            missing = sorted(REQUIRED_FIELDS - fields)
            if missing:
                raise ValueError(
                    "replay CSV missing fields: " + ",".join(missing)
                )
            rows = list(reader)
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    invalid_rows = 0
    wrench_norm_consistency_errors: list[float] = []
    failed_rows: list[dict[str, object]] = []
    dominant_components: Counter[str] = Counter()
    grouped: defaultdict[tuple[int, int], list[dict[str, object]]] = defaultdict(list)
    all_residuals = {name: [] for name in COMPONENT_NAMES}
    failed_task_residuals = {name: [] for name in COMPONENT_NAMES}
    task_unsatisfied_rows = 0
    reduced_task_unsatisfied_rows = 0
    full_task_unsatisfied_rows = 0
    wrench_unsatisfied_rows = 0
    task_unsatisfied_with_feasible_constraints = 0

    for row in rows:
        try:
            row_number = int(float(row["row_number"]))
            cmd_time_s = finite(row, "cmd_time_s")
            phase = finite(row, "phase")
            contacts = int(round(finite(row, "selected_contact_count")))
            reduced_task = finite(row, "reduced_task") >= 0.5
            wrench_satisfied = finite(row, "wrench_satisfied") >= 0.5
            task_satisfied = finite(row, "task_satisfied") >= 0.5
            task_residual_norm = finite(row, "task_residual_norm")
            residual_norm = finite(row, "residual_norm")
            constraint_feasible = finite(row, "constraint_feasible") >= 0.5
            friction_ratio = finite(row, "max_radial_friction_ratio")
            min_normal = finite(row, "min_contact_normal_force")
            target = tuple(finite(row, name) for name in TARGET_FIELDS)
            achieved = tuple(
                finite(row, name) for name in ACHIEVED_FIELDS
            )
            residual = tuple(
                finite(row, name) for name in RESIDUAL_FIELDS
            )
        except (KeyError, ValueError, OverflowError):
            invalid_rows += 1
            continue

        arithmetic_residual = tuple(
            achieved[index] - target[index]
            for index in range(len(target))
        )
        max_arithmetic_error = max(
            abs(arithmetic_residual[index] - residual[index])
            for index in range(len(residual))
        )
        if max_arithmetic_error > args.consistency_tolerance:
            invalid_rows += 1
        wrench_norm_consistency_errors.append(
            abs(math.sqrt(sum(value * value for value in residual)) - residual_norm)
        )

        for name, value in zip(COMPONENT_NAMES, residual):
            all_residuals[name].append(value)
        if not wrench_satisfied:
            wrench_unsatisfied_rows += 1
        if task_satisfied:
            continue

        task_unsatisfied_rows += 1
        if reduced_task:
            reduced_task_unsatisfied_rows += 1
        else:
            full_task_unsatisfied_rows += 1
        if constraint_feasible:
            task_unsatisfied_with_feasible_constraints += 1

        dominant_index = max(
            range(len(residual)),
            key=lambda index: abs(residual[index]),
        )
        dominant = COMPONENT_NAMES[dominant_index]
        dominant_components[dominant] += 1
        for name, value in zip(COMPONENT_NAMES, residual):
            failed_task_residuals[name].append(value)

        record: dict[str, object] = {
            "row_number": row_number,
            "cmd_time_s": cmd_time_s,
            "phase": phase,
            "selected_contact_count": contacts,
            "phase_bin": phase_bin(phase, args.phase_bins),
            "reduced_task": int(reduced_task),
            "wrench_satisfied": int(wrench_satisfied),
            "task_satisfied": int(task_satisfied),
            "task_residual_norm": task_residual_norm,
            "residual_norm": residual_norm,
            "constraint_feasible": int(constraint_feasible),
            "max_radial_friction_ratio": friction_ratio,
            "min_contact_normal_force": min_normal,
            "dominant_residual_component": dominant,
        }
        for name, value in zip(TARGET_FIELDS, target):
            record[name] = value
        for name, value in zip(ACHIEVED_FIELDS, achieved):
            record[name] = value
        for name, value in zip(RESIDUAL_FIELDS, residual):
            record[name] = value
        grouped[(contacts, record["phase_bin"])].append(record)
        failed_rows.append(record)


    detail_fields = [
        "row_number",
        "cmd_time_s",
        "phase",
        "selected_contact_count",
        "phase_bin",
        "reduced_task",
        "wrench_satisfied",
        "task_satisfied",
        "task_residual_norm",
        "residual_norm",
        "constraint_feasible",
        "max_radial_friction_ratio",
        "min_contact_normal_force",
        "dominant_residual_component",
        *TARGET_FIELDS,
        *ACHIEVED_FIELDS,
        *RESIDUAL_FIELDS,
    ]
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(failed_rows)

    lines = [
        "dynamic target feasibility classification",
        f"replay_rows={len(rows)}",
        f"invalid_rows={invalid_rows}",
        f"task_unsatisfied_rows={task_unsatisfied_rows}",
        f"full_task_unsatisfied_rows={full_task_unsatisfied_rows}",
        f"reduced_task_unsatisfied_rows={reduced_task_unsatisfied_rows}",
        f"wrench_unsatisfied_rows={wrench_unsatisfied_rows}",
        "task_unsatisfied_with_feasible_constraints="
        f"{task_unsatisfied_with_feasible_constraints}",
        f"wrench_norm_consistency_max_abs="
        f"{maximum_abs(wrench_norm_consistency_errors):.9g}",
        "all_wrench_residual_component_p95_abs:",
    ]
    for name in COMPONENT_NAMES:
        lines.append(
            f"  {name}={percentile_abs(all_residuals[name]):.9g}"
        )
    lines.append("all_wrench_residual_component_max_abs:")
    for name in COMPONENT_NAMES:
        lines.append(
            f"  {name}={maximum_abs(all_residuals[name]):.9g}"
        )
    lines.append("task_residual_component_p95_abs:")
    for name in COMPONENT_NAMES:
        lines.append(
            f"  {name}={percentile_abs(failed_task_residuals[name]):.9g}"
        )
    lines.append("task_residual_component_max_abs:")
    for name in COMPONENT_NAMES:
        lines.append(
            f"  {name}={maximum_abs(failed_task_residuals[name]):.9g}"
        )
    lines.append("dominant_residual_component_counts:")
    for name in COMPONENT_NAMES:
        lines.append(f"  {name}={dominant_components[name]}")
    lines.append("failure_groups:")
    for (contacts, bin_index), group in sorted(grouped.items()):
        phases = [float(row["phase"]) for row in group]
        lines.append(
            f"  contacts={contacts},phase_bin={bin_index},"
            f"phase_range=[{bin_index / args.phase_bins:.3f},"
            f"{(bin_index + 1) / args.phase_bins:.3f}),"
            f"rows={len(group)},phase_min={min(phases):.9g},"
            f"phase_max={max(phases):.9g}"
        )
    task_feasibility_pass = task_unsatisfied_rows == 0
    arithmetic_pass = (
        invalid_rows == 0
        and maximum_abs(wrench_norm_consistency_errors)
        <= args.consistency_tolerance
    )
    lines.extend(
        [
            "arithmetic_validation=" + ("PASS" if arithmetic_pass else "FAIL"),
            "task_feasibility_validation="
            + ("PASS" if task_feasibility_pass else "FAIL"),
            "validation=" + ("PASS" if arithmetic_pass else "FAIL"),
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if arithmetic_pass else 1


if __name__ == "__main__":
    sys.exit(main())
