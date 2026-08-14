#!/usr/bin/env python3
"""Validate full MuJoCo generalized-force dynamics closure."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

MASS_PREFIX = "full_mass_qacc_qfrc_qcoord_"
SMOOTH_PREFIX = "full_qfrc_smooth_qcoord_"
CONSTRAINT_PREFIX = "full_qfrc_constraint_qcoord_"
ACTUATOR_PREFIX = "full_qfrc_actuator_qcoord_"
BASE_SUFFIXES = (
    "trans_x_N",
    "trans_y_N",
    "trans_z_N",
    "rot_x_Nm",
    "rot_y_Nm",
    "rot_z_Nm",
)
BASE_LABELS = (
    "base_qcoord_trans_x",
    "base_qcoord_trans_y",
    "base_qcoord_trans_z",
    "base_qcoord_rot_x",
    "base_qcoord_rot_y",
    "base_qcoord_rot_z",
)


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def maximum_abs(values: list[float]) -> float:
    return max(abs(value) for value in values)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--closure-p95-tolerance",
        type=float,
        default=1e-6,
    )
    parser.add_argument(
        "--closure-max-tolerance",
        type=float,
        default=1e-4,
    )
    parser.add_argument(
        "--base-mirror-tolerance",
        type=float,
        default=1e-8,
    )
    args = parser.parse_args()

    tolerances = (
        args.closure_p95_tolerance,
        args.closure_max_tolerance,
        args.base_mirror_tolerance,
    )
    if any(tolerance < 0.0 for tolerance in tolerances):
        print("validation=FAIL: tolerances must be non-negative")
        return 2

    prefixes = (MASS_PREFIX, SMOOTH_PREFIX, CONSTRAINT_PREFIX, ACTUATOR_PREFIX)
    closure_errors: dict[str, list[float]] = {}
    mirror_errors: dict[str, list[float]] = {
        "mass": [],
        "smooth": [],
        "constraint": [],
        "actuator": [],
    }
    previous_time = -math.inf
    rows = 0

    try:
        with args.ground_truth_csv.open(
            newline="", encoding="utf-8"
        ) as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames or []
            if len(fields) != len(set(fields)):
                raise ValueError("duplicate CSV fields")
            labels = [
                field[len(MASS_PREFIX):]
                for field in fields
                if field.startswith(MASS_PREFIX)
            ]
            if not labels:
                raise ValueError("no full mass qacc fields")
            if len(labels) != len(set(labels)):
                raise ValueError("duplicate full generalized-force labels")
            for prefix in prefixes:
                missing = [
                    prefix + label
                    for label in labels
                    if prefix + label not in fields
                ]
                if missing:
                    raise ValueError(
                        "missing full generalized-force fields: "
                        + ",".join(missing)
                    )
            closure_errors = {label: [] for label in labels}
            for row_number, row in enumerate(reader, start=2):
                time_s = finite(row, "time_s")
                if time_s <= previous_time:
                    raise ValueError(
                        f"row {row_number}: time is not strictly increasing"
                    )
                previous_time = time_s
                rows += 1
                for label in labels:
                    mass = finite(row, MASS_PREFIX + label)
                    smooth = finite(row, SMOOTH_PREFIX + label)
                    constraint = finite(row, CONSTRAINT_PREFIX + label)
                    finite(row, ACTUATOR_PREFIX + label)
                    closure_errors[label].append(
                        mass - smooth - constraint
                    )

                if all(label in labels for label in BASE_LABELS):
                    for kind, full_prefix, base_prefix in (
                        (
                            "mass",
                            MASS_PREFIX,
                            "base_mass_qacc_qfrc_qcoord_",
                        ),
                        (
                            "smooth",
                            SMOOTH_PREFIX,
                            "base_qfrc_smooth_qcoord_",
                        ),
                        (
                            "constraint",
                            CONSTRAINT_PREFIX,
                            "base_qfrc_constraint_",
                        ),
                        (
                            "actuator",
                            ACTUATOR_PREFIX,
                            "base_qfrc_actuator_qcoord_",
                        ),
                    ):
                        for label, suffix in zip(
                            BASE_LABELS, BASE_SUFFIXES
                        ):
                            mirror_errors[kind].append(
                                finite(row, full_prefix + label)
                                - finite(row, base_prefix + suffix)
                            )
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if rows == 0:
        print("validation=FAIL: empty ground-truth CSV")
        return 1

    closure_pass = all(
        percentile([abs(value) for value in errors], 0.95)
        <= args.closure_p95_tolerance
        and maximum_abs(errors) <= args.closure_max_tolerance
        for errors in closure_errors.values()
    )
    base_mirror_pass = all(
        maximum_abs(errors) <= args.base_mirror_tolerance
        for errors in mirror_errors.values()
    )
    all_abs_errors = [
        abs(value)
        for errors in closure_errors.values()
        for value in errors
    ]

    lines = [
        "full generalized-force dynamics closure audit",
        f"rows={rows}",
        f"dof_count={len(labels)}",
        "dof_order=" + ",".join(labels),
        "q_coordinate_note=free_joint_translation_then_rotation_then_actuated_joints",
        "closure_equation=full_M_qacc_minus_full_qfrc_smooth_minus_full_qfrc_constraint",
        "closure_p95_tolerance=%.9g" % args.closure_p95_tolerance,
        "closure_max_tolerance=%.9g" % args.closure_max_tolerance,
        "base_mirror_tolerance=%.9g" % args.base_mirror_tolerance,
        "overall_closure_p95_abs=%.9g" % percentile(all_abs_errors, 0.95),
        "overall_closure_max_abs=%.9g" % max(all_abs_errors),
    ]
    for label, errors in closure_errors.items():
        lines.extend(
            [
                f"closure_{label}_p95_abs=%.9g"
                % percentile([abs(value) for value in errors], 0.95),
                f"closure_{label}_max_abs=%.9g"
                % maximum_abs(errors),
            ]
        )
    for kind, errors in mirror_errors.items():
        lines.append(
            f"base_mirror_{kind}_max_abs=%.9g" % maximum_abs(errors)
        )
    lines.extend(
        [
            "closure_validation=" + ("PASS" if closure_pass else "FAIL"),
            "base_mirror_validation="
            + ("PASS" if base_mirror_pass else "FAIL"),
            "interpretation=ground_truth_observation_only_no_control_injection",
            "validation="
            + ("PASS" if closure_pass and base_mirror_pass else "FAIL"),
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if closure_pass and base_mirror_pass else 1


if __name__ == "__main__":
    sys.exit(main())
