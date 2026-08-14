#!/usr/bin/env python3
"""Audit the explicit contact-conditioned lexicographic/slack shadow output."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

FORCE_SLACK_FIELDS = (
    "shadow_force_slack_x_n",
    "shadow_force_slack_y_n",
    "shadow_force_slack_z_n",
)
MOMENT_SLACK_FIELDS = (
    "shadow_moment_slack_x_nm",
    "shadow_moment_slack_y_nm",
    "shadow_moment_slack_z_nm",
)
REQUIRED_FIELDS = {
    "selected_contact_count",
    "max_abs_torque",
    "shadow_active",
    "shadow_policy_satisfied",
    "shadow_moment_task_active",
    "shadow_fallback_to_force_solution",
    "shadow_max_force_excess_n",
    "shadow_max_moment_excess_nm",
    *FORCE_SLACK_FIELDS,
    *MOMENT_SLACK_FIELDS,
}


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile_abs(values: list[float], fraction: float = 0.95) -> float:
    ordered = sorted(abs(value) for value in values)
    if not ordered:
        return 0.0
    return ordered[int(fraction * (len(ordered) - 1))]


def maximum_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--moment-required-at-or-above-contacts",
        type=int,
        default=4,
    )
    args = parser.parse_args()
    if args.moment_required_at_or_above_contacts <= 0:
        print("validation=FAIL: invalid contact threshold")
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
    active_rows = 0
    policy_unsatisfied_rows = 0
    fallback_rows = 0
    force_excesses: list[float] = []
    moment_excesses: list[float] = []
    torque_peaks: list[float] = []
    force_slacks = {name: [] for name in FORCE_SLACK_FIELDS}
    moment_slacks = {name: [] for name in MOMENT_SLACK_FIELDS}
    grouped: defaultdict[int, list[dict[str, object]]] = defaultdict(list)

    for row in rows:
        try:
            contacts = int(round(finite(row, "selected_contact_count")))
            active = finite(row, "shadow_active") >= 0.5
            policy_satisfied = (
                finite(row, "shadow_policy_satisfied") >= 0.5
            )
            moment_active = (
                finite(row, "shadow_moment_task_active") >= 0.5
            )
            fallback = (
                finite(row, "shadow_fallback_to_force_solution") >= 0.5
            )
            force_excess = finite(row, "shadow_max_force_excess_n")
            moment_excess = finite(row, "shadow_max_moment_excess_nm")
            max_abs_torque = finite(row, "max_abs_torque")
            force_slack = {
                name: finite(row, name) for name in FORCE_SLACK_FIELDS
            }
            moment_slack = {
                name: finite(row, name) for name in MOMENT_SLACK_FIELDS
            }
        except (KeyError, ValueError, OverflowError):
            invalid_rows += 1
            continue

        expected_moment_active = contacts >= (
            args.moment_required_at_or_above_contacts
        )
        if not active or moment_active != expected_moment_active:
            invalid_rows += 1
        if active:
            active_rows += 1
            if not policy_satisfied:
                policy_unsatisfied_rows += 1
            if fallback:
                fallback_rows += 1
            force_excesses.append(force_excess)
            moment_excesses.append(moment_excess)
            torque_peaks.append(max_abs_torque)
            for name, value in force_slack.items():
                force_slacks[name].append(value)
            for name, value in moment_slack.items():
                moment_slacks[name].append(value)

        grouped[contacts].append(
            {
                "moment_active": moment_active,
                "policy_satisfied": policy_satisfied,
                "fallback": fallback,
                "force_excess": force_excess,
                "moment_excess": moment_excess,
                "max_abs_torque": max_abs_torque,
            }
        )

    if not rows:
        print("validation=FAIL: no replay rows")
        return 1

    lines = [
        "contact-conditioned shadow policy audit",
        "interpretation=shadow_allocator_only_not_main_controller",
        "interpretation=force_primary_with_hinge_moment_slack",
        f"replay_rows={len(rows)}",
        f"invalid_rows={invalid_rows}",
        f"moment_required_at_or_above_contacts={args.moment_required_at_or_above_contacts}",
        f"shadow_active_rows={active_rows}",
        f"policy_unsatisfied_rows={policy_unsatisfied_rows}",
        f"fallback_to_force_solution_rows={fallback_rows}",
        f"max_force_excess_n={max(force_excesses, default=0.0):.9g}",
        f"max_moment_excess_nm={max(moment_excesses, default=0.0):.9g}",
        f"candidate_max_abs_torque_nm={max(torque_peaks, default=0.0):.9g}",
        "per_contact_group:",
    ]
    for contacts, group in sorted(grouped.items()):
        lines.append(
            f"  contacts={contacts},rows={len(group)},"
            f"moment_active_rows={sum(item['moment_active'] for item in group)},"
            f"policy_unsatisfied_rows={sum(not item['policy_satisfied'] for item in group)},"
            f"fallback_rows={sum(item['fallback'] for item in group)},"
            f"max_force_excess_n={max(item['force_excess'] for item in group):.9g},"
            f"max_moment_excess_nm={max(item['moment_excess'] for item in group):.9g},"
            f"max_abs_torque_nm={max(item['max_abs_torque'] for item in group):.9g}"
        )
    lines.append("force_slack_abs_p95_max_n:")
    for name in FORCE_SLACK_FIELDS:
        lines.append(
            f"  {name}={percentile_abs(force_slacks[name]):.9g},"
            f"max={maximum_abs(force_slacks[name]):.9g}"
        )
    lines.append("moment_slack_abs_p95_max_nm:")
    for name in MOMENT_SLACK_FIELDS:
        lines.append(
            f"  {name}={percentile_abs(moment_slacks[name]):.9g},"
            f"max={maximum_abs(moment_slacks[name]):.9g}"
        )

    validation_pass = invalid_rows == 0
    lines.append("validation=" + ("PASS" if validation_pass else "FAIL"))
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if validation_pass else 1


if __name__ == "__main__":
    sys.exit(main())
