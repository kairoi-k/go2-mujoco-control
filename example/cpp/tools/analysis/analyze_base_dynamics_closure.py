#!/usr/bin/env python3
"""Validate MuJoCo base generalized-force dynamics closure."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

SUFFIXES = (
    "trans_x_N",
    "trans_y_N",
    "trans_z_N",
    "rot_x_Nm",
    "rot_y_Nm",
    "rot_z_Nm",
)
PREFIXES = {
    "mass_accel": "base_mass_qacc_qfrc_qcoord",
    "smooth": "base_qfrc_smooth_qcoord",
    "bias": "base_qfrc_bias_qcoord",
    "passive": "base_qfrc_passive_qcoord",
    "actuator": "base_qfrc_actuator_qcoord",
    "applied": "base_qfrc_applied_qcoord",
    "residual": "base_dynamics_residual_qcoord",
    "constraint": "base_qfrc_constraint",
}


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
        "--recorded-residual-tolerance",
        type=float,
        default=1e-8,
    )
    parser.add_argument(
        "--smooth-decomposition-tolerance",
        type=float,
        default=1e-6,
    )
    args = parser.parse_args()

    tolerances = (
        args.closure_p95_tolerance,
        args.closure_max_tolerance,
        args.recorded_residual_tolerance,
        args.smooth_decomposition_tolerance,
    )
    if any(tolerance < 0.0 for tolerance in tolerances):
        print("validation=FAIL: tolerances must be non-negative")
        return 2

    required = {"time_s"}
    for prefix in PREFIXES.values():
        required.update(f"{prefix}_{suffix}" for suffix in SUFFIXES)

    closure_errors = [[] for _ in SUFFIXES]
    recorded_residual_errors = [[] for _ in SUFFIXES]
    decomposition_errors = [[] for _ in SUFFIXES]
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
            missing = sorted(required - set(fields))
            if missing:
                raise ValueError("missing fields: " + ",".join(missing))
            for row_number, row in enumerate(reader, start=2):
                time_s = finite(row, "time_s")
                if time_s <= previous_time:
                    raise ValueError(
                        f"row {row_number}: time is not strictly increasing"
                    )
                previous_time = time_s
                rows += 1
                for index, suffix in enumerate(SUFFIXES):
                    mass_accel = finite(
                        row, f"{PREFIXES['mass_accel']}_{suffix}"
                    )
                    smooth = finite(row, f"{PREFIXES['smooth']}_{suffix}")
                    constraint = finite(
                        row, f"{PREFIXES['constraint']}_{suffix}"
                    )
                    recorded_residual = finite(
                        row, f"{PREFIXES['residual']}_{suffix}"
                    )
                    bias = finite(row, f"{PREFIXES['bias']}_{suffix}")
                    passive = finite(row, f"{PREFIXES['passive']}_{suffix}")
                    actuator = finite(
                        row, f"{PREFIXES['actuator']}_{suffix}"
                    )
                    applied = finite(row, f"{PREFIXES['applied']}_{suffix}")
                    closure = mass_accel - smooth - constraint
                    decomposition = (
                        applied + actuator + passive - bias
                    )
                    closure_errors[index].append(closure)
                    recorded_residual_errors[index].append(
                        recorded_residual - closure
                    )
                    decomposition_errors[index].append(
                        smooth - decomposition
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
        for errors in closure_errors
    )
    recorded_pass = all(
        maximum_abs(errors) <= args.recorded_residual_tolerance
        for errors in recorded_residual_errors
    )
    decomposition_pass = all(
        maximum_abs(errors) <= args.smooth_decomposition_tolerance
        for errors in decomposition_errors
    )

    lines = [
        "base generalized-force dynamics closure audit",
        f"rows={rows}",
        "q_coordinate_note=free_joint_translation_then_rotation",
        "closure_equation=M_qacc_minus_qfrc_smooth_minus_qfrc_constraint",
        "smooth_equation=qfrc_applied_plus_qfrc_actuator_plus_qfrc_passive_minus_qfrc_bias",
        "closure_p95_tolerance=%.9g" % args.closure_p95_tolerance,
        "closure_max_tolerance=%.9g" % args.closure_max_tolerance,
        "recorded_residual_tolerance=%.9g"
        % args.recorded_residual_tolerance,
        "smooth_decomposition_tolerance=%.9g"
        % args.smooth_decomposition_tolerance,
    ]
    for index, suffix in enumerate(SUFFIXES):
        lines.extend(
            [
                f"closure_{suffix}_p95_abs=%.9g"
                % percentile(
                    [abs(value) for value in closure_errors[index]],
                    0.95,
                ),
                f"closure_{suffix}_max_abs=%.9g"
                % maximum_abs(closure_errors[index]),
                f"recorded_residual_{suffix}_max_abs_error=%.9g"
                % maximum_abs(recorded_residual_errors[index]),
                f"smooth_decomposition_{suffix}_max_abs=%.9g"
                % maximum_abs(decomposition_errors[index]),
            ]
        )
    lines.extend(
        [
            "closure_validation=" + ("PASS" if closure_pass else "FAIL"),
            "recorded_residual_validation="
            + ("PASS" if recorded_pass else "FAIL"),
            "smooth_decomposition_validation="
            + ("PASS" if decomposition_pass else "FAIL"),
            "interpretation=shadow_observation_only_no_control_injection",
            "validation="
            + (
                "PASS"
                if closure_pass and recorded_pass and decomposition_pass
                else "FAIL"
            ),
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if closure_pass and recorded_pass and decomposition_pass else 1


if __name__ == "__main__":
    sys.exit(main())
