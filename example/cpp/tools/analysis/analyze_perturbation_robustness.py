#!/usr/bin/env python3
"""Audit the intersection of replay feasibility across perturbation variants."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

REQUIRED = {
    "row_number",
    "cmd_time_s",
    "task_satisfied",
    "shadow_policy_satisfied",
    "shadow_torque_rate_task_active",
    "shadow_torque_rate_satisfied",
    "shadow_max_moment_excess_nm",
    "shadow_max_torque_rate_excess_nm",
}


def finite(row: dict[str, str], field: str) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field}")
    return value


def parse_spec(spec: str) -> tuple[str, Path]:
    label, separator, path = spec.partition("=")
    if not separator or not label or not path:
        raise ValueError("replay must use label=path")
    return label, Path(path)


def read_replay(path: Path) -> dict[int, dict[str, float | bool]]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        missing = sorted(REQUIRED - fields)
        if missing:
            raise ValueError(f"{path}: missing fields {','.join(missing)}")
        rows: dict[int, dict[str, float | bool]] = {}
        for raw in reader:
            row_number = int(finite(raw, "row_number"))
            if row_number in rows:
                raise ValueError(f"{path}: duplicate row_number {row_number}")
            rate_active = finite(raw, "shadow_torque_rate_task_active") >= 0.5
            rows[row_number] = {
                "time_s": finite(raw, "cmd_time_s"),
                "task_ok": finite(raw, "task_satisfied") >= 0.5,
                "policy_ok": finite(raw, "shadow_policy_satisfied") >= 0.5,
                "rate_ok": (
                    not rate_active
                    or finite(raw, "shadow_torque_rate_satisfied") >= 0.5
                ),
                "moment_excess": finite(raw, "shadow_max_moment_excess_nm"),
                "rate_excess": finite(raw, "shadow_max_torque_rate_excess_nm"),
            }
        return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--replay",
        action="append",
        required=True,
        help="label=path; repeat for every perturbation variant",
    )
    parser.add_argument("--out", required=True)
    parser.add_argument("--time-tolerance-s", type=float, default=1e-9)
    args = parser.parse_args()

    if args.time_tolerance_s < 0.0 or not math.isfinite(args.time_tolerance_s):
        raise SystemExit("time-tolerance-s must be finite and non-negative")

    variants: list[tuple[str, dict[int, dict[str, float | bool]]]] = []
    labels: set[str] = set()
    try:
        for spec in args.replay:
            label, path = parse_spec(spec)
            if label in labels:
                raise ValueError(f"duplicate label {label}")
            labels.add(label)
            variants.append((label, read_replay(path)))
    except (OSError, ValueError) as error:
        raise SystemExit(str(error)) from error

    if not variants:
        raise SystemExit("at least one replay is required")
    reference_label, reference = variants[0]
    reference_keys = set(reference)
    mismatch_rows = 0
    invalid_rows = 0
    robust_policy_rows = 0
    robust_task_rows = 0
    robust_rate_rows = 0
    robust_rows = 0
    max_moment = (-math.inf, None, None)
    max_rate = (-math.inf, None, None)
    per_variant = []

    for label, rows in variants:
        if set(rows) != reference_keys:
            mismatch_rows += 1
            continue
        task_pass = 0
        policy_pass = 0
        rate_pass = 0
        for key in reference_keys:
            if abs(float(rows[key]["time_s"]) - float(reference[key]["time_s"])) > args.time_tolerance_s:
                mismatch_rows += 1
                continue
            task_pass += bool(rows[key]["task_ok"])
            policy_pass += bool(rows[key]["policy_ok"])
            rate_pass += bool(rows[key]["rate_ok"])
        per_variant.append((label, len(rows), task_pass, policy_pass, rate_pass))

    if mismatch_rows == 0:
        for key in sorted(reference_keys):
            row_values = [rows[key] for _, rows in variants]
            task_ok = all(bool(row["task_ok"]) for row in row_values)
            policy_ok = all(bool(row["policy_ok"]) for row in row_values)
            rate_ok = all(bool(row["rate_ok"]) for row in row_values)
            robust_task_rows += task_ok
            robust_policy_rows += policy_ok
            robust_rate_rows += rate_ok
            robust_rows += policy_ok and rate_ok
            for label, row in zip((name for name, _ in variants), row_values):
                moment = float(row["moment_excess"])
                rate = float(row["rate_excess"])
                if moment > max_moment[0]:
                    max_moment = (moment, key, label)
                if rate > max_rate[0]:
                    max_rate = (rate, key, label)

    lines = [
        "perturbation robustness intersection audit",
        "interpretation=all_variants_must_pass_same_replay_row",
        "interpretation=shadow_only_not_main_controller",
        f"reference_variant={reference_label}",
        f"variant_count={len(variants)}",
        f"reference_rows={len(reference)}",
        f"mismatch_rows={mismatch_rows}",
        f"invalid_rows={invalid_rows}",
        "per_variant:",
    ]
    for label, row_count, task_pass, policy_pass, rate_pass in per_variant:
        lines.append(
            f"  {label}:rows={row_count},task_pass={task_pass},"
            f"policy_pass={policy_pass},rate_pass={rate_pass}"
        )
    lines.extend(
        [
            f"robust_task_rows={robust_task_rows}",
            f"robust_policy_rows={robust_policy_rows}",
            f"robust_rate_rows={robust_rate_rows}",
            f"robust_policy_and_rate_rows={robust_rows}",
            f"max_moment_excess_nm={max_moment[0]:.9g},row={max_moment[1]},variant={max_moment[2]}",
            f"max_rate_excess_nm={max_rate[0]:.9g},row={max_rate[1]},variant={max_rate[2]}",
        ]
    )
    validation = mismatch_rows == 0 and invalid_rows == 0
    lines.append(f"validation={'PASS' if validation else 'FAIL'}")
    output_path = Path(args.out)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0 if validation else 1


if __name__ == "__main__":
    raise SystemExit(main())
