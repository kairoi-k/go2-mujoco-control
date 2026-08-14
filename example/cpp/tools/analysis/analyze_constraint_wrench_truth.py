#!/usr/bin/env python3
"""Validate MuJoCo qfrc_constraint against the logged contact wrench."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

AXES = ("x", "y", "z")


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--force-tolerance-n", type=float, default=1e-6)
    parser.add_argument(
        "--moment-p95-tolerance-nm",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--moment-max-tolerance-nm",
        type=float,
        default=2.0,
    )
    args = parser.parse_args()

    if (
        args.force_tolerance_n < 0.0
        or args.moment_p95_tolerance_nm < 0.0
        or args.moment_max_tolerance_nm < 0.0
    ):
        print("validation=FAIL: tolerances must be non-negative")
        return 2

    try:
        with args.ground_truth_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            required = {"time_s"}
            required.update(
                f"base_qfrc_constraint_trans_{axis}_N" for axis in AXES
            )
            required.update(
                f"base_qfrc_constraint_rot_{axis}_Nm" for axis in AXES
            )
            required.update(
                f"total_contact_grf_world_{axis}_N" for axis in AXES
            )
            required.update(
                f"total_contact_moment_world_{axis}_Nm" for axis in AXES
            )
            missing = sorted(required - fields)
            if missing:
                raise ValueError("missing fields: " + ",".join(missing))
            rows = list(reader)
    except (OSError, ValueError, KeyError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not rows:
        print("validation=FAIL: empty ground-truth CSV")
        return 1

    force_errors = [[] for _ in AXES]
    moment_errors = [[] for _ in AXES]
    previous_time = -math.inf
    try:
        for row_number, row in enumerate(rows, start=2):
            time_s = finite(row, "time_s")
            if time_s <= previous_time:
                raise ValueError(
                    f"row {row_number}: time is not strictly increasing"
                )
            previous_time = time_s
            for index, axis in enumerate(AXES):
                force_errors[index].append(
                    finite(row, f"base_qfrc_constraint_trans_{axis}_N")
                    - finite(row, f"total_contact_grf_world_{axis}_N")
                )
                moment_errors[index].append(
                    finite(row, f"base_qfrc_constraint_rot_{axis}_Nm")
                    - finite(row, f"total_contact_moment_world_{axis}_Nm")
                )
    except (KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 1

    force_max_abs = max(
        abs(value) for component in force_errors for value in component
    )
    moment_abs = [
        abs(value) for component in moment_errors for value in component
    ]
    force_pass = force_max_abs <= args.force_tolerance_n
    moment_pass = (
        percentile(moment_abs, 0.95) <= args.moment_p95_tolerance_nm
        and max(moment_abs) <= args.moment_max_tolerance_nm
    )

    lines = [
        "constraint wrench truth audit",
        f"rows={len(rows)}",
        "force_tolerance_n=%.9g" % args.force_tolerance_n,
        "moment_p95_tolerance_nm=%.9g" % args.moment_p95_tolerance_nm,
        "moment_max_tolerance_nm=%.9g" % args.moment_max_tolerance_nm,
    ]
    for index, axis in enumerate(AXES):
        lines.extend(
            [
                "force_error_%s_max_abs_n=%.9g"
                % (axis, max(abs(value) for value in force_errors[index])),
                "moment_error_%s_p95_abs_nm=%.9g"
                % (axis, percentile(
                    [abs(value) for value in moment_errors[index]], 0.95
                )),
                "moment_error_%s_max_abs_nm=%.9g"
                % (axis, max(abs(value) for value in moment_errors[index])),
                "moment_error_%s_rms_nm=%.9g"
                % (axis, rms(moment_errors[index])),
            ]
        )
    lines.extend(
        [
            "force_mapping_validation=" + ("PASS" if force_pass else "FAIL"),
            "moment_mapping_validation=" + (
                "PASS" if moment_pass else "FAIL"
            ),
            "qfrc_constraint_frame_note="
            "free_joint_q_coordinate_matches_logged_world_contact_wrench",
            "validation=" + ("PASS" if force_pass and moment_pass else "FAIL"),
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if force_pass and moment_pass else 1


if __name__ == "__main__":
    sys.exit(main())
