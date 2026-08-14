#!/usr/bin/env python3
"""Audit bounded dynamic-acceleration target shaping."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

AXES = ("x", "y", "z")


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile_abs(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(abs(value) for value in values)
    index = min(len(ordered) - 1, int(percentile * (len(ordered) - 1)))
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--correction-limit-mps2", type=float, required=True)
    parser.add_argument("--slew-limit-mps3", type=float, required=True)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    if (
        not math.isfinite(args.correction_limit_mps2)
        or args.correction_limit_mps2 < 0.0
        or not math.isfinite(args.slew_limit_mps3)
        or args.slew_limit_mps3 < 0.0
    ):
        raise SystemExit("limits must be finite and non-negative")

    with Path(args.replay_csv).open(newline="") as stream:
        rows = list(csv.DictReader(stream))

    required = {
        "cmd_time_s",
        "dynamic_accel_target_active",
        "dynamic_accel_reference_held_for_duplicate_time",
        "dynamic_accel_slew_limited",
    }
    for axis in AXES:
        required.update(
            {
                f"dynamic_accel_correction_raw_{axis}_mps2",
                f"dynamic_accel_correction_applied_{axis}_mps2",
                f"dynamic_accel_correction_slack_{axis}_mps2",
            }
        )
    missing = sorted(required - set(rows[0] if rows else {}))
    if missing:
        print("validation=FAIL: missing fields " + ",".join(missing))
        return 1

    active_rows = 0
    invalid_rows = 0
    duplicate_pairs = 0
    positive_pairs = 0
    limit_violations = 0
    slew_violations = 0
    slack_consistency_violations = 0
    held_reference_violations = 0
    slew_limited_rows = 0
    raw: dict[str, list[float]] = {axis: [] for axis in AXES}
    applied: dict[str, list[float]] = {axis: [] for axis in AXES}
    slack: dict[str, list[float]] = {axis: [] for axis in AXES}
    previous_time: float | None = None
    previous_applied: dict[str, float] | None = None

    for row in rows:
        if finite(row, "dynamic_accel_target_active") < 0.5:
            continue
        active_rows += 1
        try:
            time_s = finite(row, "cmd_time_s")
            current_raw = {
                axis: finite(
                    row, f"dynamic_accel_correction_raw_{axis}_mps2"
                )
                for axis in AXES
            }
            current_applied = {
                axis: finite(
                    row, f"dynamic_accel_correction_applied_{axis}_mps2"
                )
                for axis in AXES
            }
            current_slack = {
                axis: finite(
                    row, f"dynamic_accel_correction_slack_{axis}_mps2"
                )
                for axis in AXES
            }
            held = finite(
                row, "dynamic_accel_reference_held_for_duplicate_time"
            ) >= 0.5
            slew_limited = finite(row, "dynamic_accel_slew_limited") >= 0.5
        except (KeyError, ValueError):
            invalid_rows += 1
            continue

        if any(abs(current_applied[axis]) > args.correction_limit_mps2 + args.tolerance for axis in AXES):
            limit_violations += 1
        if any(
            abs(
                current_raw[axis]
                - current_applied[axis]
                - current_slack[axis]
            )
            > args.tolerance
            for axis in AXES
        ):
            slack_consistency_violations += 1

        if previous_time is not None and previous_applied is not None:
            dt_s = time_s - previous_time
            if dt_s < -args.tolerance:
                invalid_rows += 1
            elif dt_s <= args.tolerance:
                duplicate_pairs += 1
                if not held or any(
                    abs(current_applied[axis] - previous_applied[axis])
                    > args.tolerance
                    for axis in AXES
                ):
                    held_reference_violations += 1
            else:
                positive_pairs += 1
                if any(
                    abs(current_applied[axis] - previous_applied[axis])
                    > args.slew_limit_mps3 * dt_s + args.tolerance
                    for axis in AXES
                ):
                    slew_violations += 1

        for axis in AXES:
            raw[axis].append(current_raw[axis])
            applied[axis].append(current_applied[axis])
            slack[axis].append(current_slack[axis])
        if slew_limited:
            slew_limited_rows += 1
        previous_time = time_s
        previous_applied = current_applied

    lines = [
        "bounded dynamic-acceleration target audit",
        "interpretation=reference shaping before contact-wrench allocation",
        "interpretation=shadow_only_not_main_controller",
        f"active_rows={active_rows}",
        f"invalid_rows={invalid_rows}",
        f"duplicate_timestamp_pairs={duplicate_pairs}",
        f"positive_dt_pairs={positive_pairs}",
        f"limit_violations={limit_violations}",
        f"slew_violations={slew_violations}",
        f"slack_consistency_violations={slack_consistency_violations}",
        f"held_reference_violations={held_reference_violations}",
        f"slew_limited_rows={slew_limited_rows}",
        "absolute_quantiles_mps2:",
    ]
    for axis in AXES:
        lines.append(
            f"  {axis}_raw_p95={percentile_abs(raw[axis], 0.95):.9g},"
            f"{axis}_raw_max={percentile_abs(raw[axis], 1.0):.9g},"
            f"{axis}_applied_p95={percentile_abs(applied[axis], 0.95):.9g},"
            f"{axis}_applied_max={percentile_abs(applied[axis], 1.0):.9g},"
            f"{axis}_slack_p95={percentile_abs(slack[axis], 0.95):.9g},"
            f"{axis}_slack_max={percentile_abs(slack[axis], 1.0):.9g}"
        )
    validation = (
        invalid_rows == 0
        and limit_violations == 0
        and slew_violations == 0
        and slack_consistency_violations == 0
        and held_reference_violations == 0
    )
    lines.append(f"validation={'PASS' if validation else 'FAIL'}")
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0 if validation else 1


if __name__ == "__main__":
    raise SystemExit(main())
