#!/usr/bin/env python3
"""Audit the sequence-constrained lexicographic/slack shadow output."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

TIME_EPSILON_S = 1e-12
RESIDUAL_TOLERANCE = 1e-5


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[int(fraction * (len(ordered) - 1))]


def stats(values: list[float]) -> str:
    return (
        f"count={len(values)},"
        f"p50={percentile(values, 0.50):.9g},"
        f"p95={percentile(values, 0.95):.9g},"
        f"max={max(values, default=0.0):.9g}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    required_fields = {
        "row_number",
        "cmd_time_s",
        "selected_contact_count",
        "shadow_active",
        "shadow_policy_satisfied",
        "shadow_fallback_to_force_solution",
        "shadow_max_force_excess_n",
        "shadow_max_moment_excess_nm",
        "shadow_torque_rate_task_active",
        "shadow_torque_rate_satisfied",
        "shadow_max_torque_rate_excess_nm",
    }
    rows: list[dict[str, object]] = []
    invalid_rows = 0
    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or [])
            missing = sorted(required_fields - fields)
            if missing:
                raise ValueError(
                    "replay CSV missing fields: " + ",".join(missing)
                )
            for raw in reader:
                try:
                    rows.append(
                        {
                            "row_number": int(float(raw["row_number"])),
                            "time_s": finite(raw, "cmd_time_s"),
                            "contacts": int(
                                round(finite(raw, "selected_contact_count"))
                            ),
                            "active": finite(raw, "shadow_active") >= 0.5,
                            "policy": finite(
                                raw, "shadow_policy_satisfied"
                            )
                            >= 0.5,
                            "fallback": finite(
                                raw, "shadow_fallback_to_force_solution"
                            )
                            >= 0.5,
                            "force_excess": finite(
                                raw, "shadow_max_force_excess_n"
                            ),
                            "moment_excess": finite(
                                raw, "shadow_max_moment_excess_nm"
                            ),
                            "rate_active": finite(
                                raw, "shadow_torque_rate_task_active"
                            )
                            >= 0.5,
                            "rate_satisfied": finite(
                                raw, "shadow_torque_rate_satisfied"
                            )
                            >= 0.5,
                            "rate_excess": finite(
                                raw, "shadow_max_torque_rate_excess_nm"
                            ),
                        }
                    )
                except (KeyError, ValueError, OverflowError):
                    invalid_rows += 1
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not rows:
        print("validation=FAIL: no valid replay rows")
        return 1

    active_rows = 0
    rate_active_rows = 0
    duplicate_pairs = 0
    positive_dt_pairs = 0
    negative_dt_pairs = 0
    policy_unsatisfied_rows = 0
    rate_unsatisfied_rows = 0
    fallback_rows = 0
    force_excesses: list[float] = []
    moment_excesses: list[float] = []
    rate_excesses: list[float] = []
    grouped: defaultdict[int, list[dict[str, object]]] = defaultdict(list)

    previous_time: float | None = None
    for index, row in enumerate(rows):
        time_s = row["time_s"]
        expected_rate_active = False
        if previous_time is not None:
            dt_s = time_s - previous_time
            if dt_s < -TIME_EPSILON_S:
                negative_dt_pairs += 1
            elif dt_s <= TIME_EPSILON_S:
                duplicate_pairs += 1
            else:
                positive_dt_pairs += 1
                expected_rate_active = True

        active = row["active"]
        policy = row["policy"]
        rate_active = row["rate_active"]
        rate_satisfied = row["rate_satisfied"]
        fallback = row["fallback"]
        rate_excess = row["rate_excess"]

        if not active:
            invalid_rows += 1
        if rate_active != expected_rate_active:
            invalid_rows += 1
        if not rate_active and (
            not rate_satisfied or abs(rate_excess) > RESIDUAL_TOLERANCE
        ):
            invalid_rows += 1
        if rate_active and (
            rate_satisfied != (rate_excess <= RESIDUAL_TOLERANCE)
        ):
            invalid_rows += 1
        if rate_active and not rate_satisfied and policy:
            invalid_rows += 1

        active_rows += int(active)
        rate_active_rows += int(rate_active)
        policy_unsatisfied_rows += int(not policy)
        rate_unsatisfied_rows += int(rate_active and not rate_satisfied)
        fallback_rows += int(fallback)
        force_excesses.append(row["force_excess"])
        moment_excesses.append(row["moment_excess"])
        rate_excesses.append(rate_excess)
        grouped[row["contacts"]].append(row)
        previous_time = time_s

    lines = [
        "sequence-constrained shadow allocator audit",
        "interpretation=previous_candidate_torque_is_an_allocator_rate_task",
        "interpretation=shadow_only_not_main_controller",
        f"replay_csv={args.replay_csv}",
        f"replay_rows={len(rows)}",
        f"invalid_rows={invalid_rows}",
        f"negative_dt_pairs={negative_dt_pairs}",
        f"duplicate_timestamp_pairs={duplicate_pairs}",
        f"positive_dt_pairs={positive_dt_pairs}",
        f"shadow_active_rows={active_rows}",
        f"torque_rate_task_active_rows={rate_active_rows}",
        f"policy_unsatisfied_rows={policy_unsatisfied_rows}",
        f"torque_rate_unsatisfied_rows={rate_unsatisfied_rows}",
        f"fallback_to_force_solution_rows={fallback_rows}",
        f"max_force_excess_n={max(force_excesses, default=0.0):.9g}",
        f"max_moment_excess_nm={max(moment_excesses, default=0.0):.9g}",
        f"torque_rate_excess_nm={stats(rate_excesses)}",
        "per_contact_group:",
    ]
    for contacts, group in sorted(grouped.items()):
        group_rate_excess = [row["rate_excess"] for row in group]
        lines.append(
            f"  contacts={contacts},rows={len(group)},"
            f"rate_active_rows={sum(row['rate_active'] for row in group)},"
            f"rate_unsatisfied_rows={sum(row['rate_active'] and not row['rate_satisfied'] for row in group)},"
            f"policy_unsatisfied_rows={sum(not row['policy'] for row in group)},"
            f"fallback_rows={sum(row['fallback'] for row in group)},"
            f"max_rate_excess_nm={max(group_rate_excess, default=0.0):.9g}"
        )

    validation_pass = (
        invalid_rows == 0
        and negative_dt_pairs == 0
        and active_rows == len(rows)
    )
    lines.append("validation=" + ("PASS" if validation_pass else "FAIL"))
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if validation_pass else 1


if __name__ == "__main__":
    sys.exit(main())
