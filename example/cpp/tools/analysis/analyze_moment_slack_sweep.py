#!/usr/bin/env python3
"""Measure moment slack required by an existing contact-wrench replay."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

MOMENT_FIELDS = (
    "wrench_residual_moment_x_nm",
    "wrench_residual_moment_y_nm",
    "wrench_residual_moment_z_nm",
)
FORCE_FIELDS = (
    "wrench_residual_force_x_n",
    "wrench_residual_force_y_n",
    "wrench_residual_force_z_n",
)
REQUIRED_FIELDS = {
    "selected_contact_count",
    *MOMENT_FIELDS,
    *FORCE_FIELDS,
}


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--force-tolerance-n", type=float, default=1e-3)
    parser.add_argument(
        "--moment-slack-nm",
        type=float,
        nargs="+",
        default=[0.01, 0.05, 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 40.0],
    )
    parser.add_argument("--moment-required-at-or-above-contacts", type=int, default=4)
    args = parser.parse_args()
    if (
        args.force_tolerance_n < 0.0
        or args.moment_required_at_or_above_contacts <= 0
        or not args.moment_slack_nm
        or any(value < 0.0 for value in args.moment_slack_nm)
    ):
        print("validation=FAIL: invalid slack parameters")
        return 2

    invalid_rows = 0
    moment_residuals: list[float] = []
    force_residuals: list[float] = []
    contact_moment_residuals: dict[int, list[float]] = {}
    contact_counts: list[int] = []

    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or [])
            missing = sorted(REQUIRED_FIELDS - fields)
            if missing:
                raise ValueError(
                    "replay CSV missing fields: " + ",".join(missing)
                )
            for row in reader:
                try:
                    contacts = int(round(finite(row, "selected_contact_count")))
                    moment = max(abs(finite(row, name)) for name in MOMENT_FIELDS)
                    force = max(abs(finite(row, name)) for name in FORCE_FIELDS)
                except (KeyError, ValueError, OverflowError):
                    invalid_rows += 1
                    continue
                moment_residuals.append(moment)
                force_residuals.append(force)
                contact_moment_residuals.setdefault(contacts, []).append(moment)
                contact_counts.append(contacts)
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not moment_residuals:
        print("validation=FAIL: no valid replay rows")
        return 1

    lines = [
        "moment slack sweep",
        "interpretation=acceptance_audit_only_no_solver_change",
        f"replay_rows={len(moment_residuals)}",
        f"invalid_rows={invalid_rows}",
        f"force_tolerance_n={args.force_tolerance_n:.9g}",
        "moment_residual_abs_quantiles_nm:",
        f"moment_required_at_or_above_contacts={args.moment_required_at_or_above_contacts}",
        f"  p50={percentile(moment_residuals, 0.50):.9g}",
        f"  p90={percentile(moment_residuals, 0.90):.9g}",
        f"  p95={percentile(moment_residuals, 0.95):.9g}",
        f"  p99={percentile(moment_residuals, 0.99):.9g}",
        f"  max={max(moment_residuals):.9g}",
        "force_residual_abs_quantiles_n:",
        f"  p50={percentile(force_residuals, 0.50):.9g}",
        f"  p90={percentile(force_residuals, 0.90):.9g}",
        f"  p95={percentile(force_residuals, 0.95):.9g}",
        f"  p99={percentile(force_residuals, 0.99):.9g}",
        f"  max={max(force_residuals):.9g}",
        "contact_moment_residual_p95_nm:",
    ]
    for contacts, values in sorted(contact_moment_residuals.items()):
        lines.append(
            f"  contacts={contacts},rows={len(values)},"
            f"p95={percentile(values, 0.95):.9g},max={max(values):.9g}"
        )

    lines.append("slack_rows:")
    for slack in sorted(set(args.moment_slack_nm)):
        moment_pass = sum(value <= slack for value in moment_residuals)
        force_and_moment_pass = sum(
            moment <= slack and force <= args.force_tolerance_n
            for moment, force in zip(moment_residuals, force_residuals)
        )
        contact_conditioned_pass = sum(
            force <= args.force_tolerance_n
            and (
                contacts < args.moment_required_at_or_above_contacts
                or moment <= slack
            )
            for moment, force, contacts in zip(
                moment_residuals, force_residuals, contact_counts
            )
        )
        lines.append(
            f"  moment_slack_nm={slack:.9g},"
            f"moment_rows={moment_pass},"
            f"moment_pct={100.0 * moment_pass / len(moment_residuals):.6f},"
            f"force_and_moment_rows={force_and_moment_pass},"
            f"contact_conditioned_rows={contact_conditioned_pass}"
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
