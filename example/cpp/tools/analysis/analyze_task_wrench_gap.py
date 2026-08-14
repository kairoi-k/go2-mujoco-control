#!/usr/bin/env python3
"""Compare desired task wrench with MuJoCo contact constraint truth."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from bisect import bisect_left
from pathlib import Path

AXES = ("x", "y", "z")
MASS_KG = 15.206408
GRAVITY_MPS2 = 9.81


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def world_to_body(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid base quaternion")
    w, x, y, z = (
        quaternion[0] / norm,
        -quaternion[1] / norm,
        -quaternion[2] / norm,
        -quaternion[3] / norm,
    )
    vx, vy, vz = vector
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values))


def correlation(left: list[float], right: list[float]) -> float:
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    denominator = math.sqrt(
        sum((value - left_mean) ** 2 for value in left)
        * sum((value - right_mean) ** 2 for value in right)
    )
    if denominator <= 1e-15:
        return 0.0
    return sum(
        (left_value - left_mean) * (right_value - right_mean)
        for left_value, right_value in zip(left, right)
    ) / denominator


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-csv", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--match-tolerance-s", type=float, default=0.0011)
    parser.add_argument("--mass-kg", type=float, default=MASS_KG)
    parser.add_argument("--gravity-mps2", type=float, default=GRAVITY_MPS2)
    args = parser.parse_args()

    if (
        args.match_tolerance_s <= 0.0
        or args.mass_kg <= 0.0
        or args.gravity_mps2 <= 0.0
    ):
        print("validation=FAIL: invalid audit parameters")
        return 2

    try:
        with args.state_csv.open(newline="", encoding="utf-8") as handle:
            state_reader = csv.DictReader(handle)
            state_fields = set(state_reader.fieldnames or ())
            state_required = {
                "state_tick_s",
                "motion_stage",
                "contact_count",
                "wbc_shadow_desired_force_x_n",
            }
            missing = sorted(state_required - state_fields)
            if missing:
                raise ValueError(
                    "state CSV missing fields: " + ",".join(missing)
                )
            state_rows = list(state_reader)
        with args.ground_truth_csv.open(
            newline="", encoding="utf-8"
        ) as handle:
            truth_reader = csv.DictReader(handle)
            truth_fields = set(truth_reader.fieldnames or ())
            truth_required = {
                "time_s",
                "base_quat_w",
                "base_quat_x",
                "base_quat_y",
                "base_quat_z",
            }
            truth_required.update(
                f"base_qfrc_constraint_trans_{axis}_N" for axis in AXES
            )
            truth_required.update(
                f"base_qfrc_constraint_rot_{axis}_Nm" for axis in AXES
            )
            missing = sorted(truth_required - truth_fields)
            if missing:
                raise ValueError(
                    "ground-truth CSV missing fields: " + ",".join(missing)
                )
            truth_rows = list(truth_reader)
    except (OSError, ValueError, KeyError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    truth_times = [finite(row, "time_s") for row in truth_rows]
    if not truth_times:
        print("validation=FAIL: empty ground-truth CSV")
        return 1

    desired_force_x = []
    actual_force_body = [[] for _ in AXES]
    actual_moment_body = [[] for _ in AXES]
    gaps_force = [[] for _ in AXES]
    gaps_moment = [[] for _ in AXES]
    time_errors = []
    time_cursor = 0
    walking_rows = 0
    match_failures = 0
    by_contact = {
        count: {"desired": [], "actual": [], "gap": []}
        for count in (2, 4)
    }

    for row in state_rows:
        if finite(row, "motion_stage") != 2.0:
            continue
        walking_rows += 1
        try:
            state_time = finite(row, "state_tick_s")
            desired_x = finite(row, "wbc_shadow_desired_force_x_n")
            contact_count = int(finite(row, "contact_count"))
        except (KeyError, ValueError):
            match_failures += 1
            continue
        next_cursor = bisect_left(truth_times, state_time, time_cursor)
        candidates = [
            index
            for index in (next_cursor - 1, next_cursor)
            if 0 <= index < len(truth_rows)
        ]
        if not candidates:
            match_failures += 1
            continue
        time_cursor = min(
            candidates,
            key=lambda index: abs(truth_times[index] - state_time),
        )
        time_error = truth_times[time_cursor] - state_time
        if abs(time_error) > args.match_tolerance_s:
            match_failures += 1
            continue

        truth = truth_rows[time_cursor]
        quaternion = tuple(
            finite(truth, f"base_quat_{axis}")
            for axis in ("w", "x", "y", "z")
        )
        force_world = tuple(
            finite(
                truth,
                f"base_qfrc_constraint_trans_{axis}_N",
            )
            for axis in AXES
        )
        moment_world = tuple(
            finite(
                truth,
                f"base_qfrc_constraint_rot_{axis}_Nm",
            )
            for axis in AXES
        )
        force_body = world_to_body(quaternion, force_world)
        moment_body = world_to_body(quaternion, moment_world)
        desired_force = (desired_x, 0.0, args.mass_kg * args.gravity_mps2)
        desired_moment = (0.0, 0.0, 0.0)
        desired_force_x.append(desired_x)
        time_errors.append(abs(time_error))
        for index in range(3):
            actual_force_body[index].append(force_body[index])
            actual_moment_body[index].append(moment_body[index])
            gaps_force[index].append(force_body[index] - desired_force[index])
            gaps_moment[index].append(
                moment_body[index] - desired_moment[index]
            )
        if contact_count in by_contact:
            bucket = by_contact[contact_count]
            bucket["desired"].append(desired_x)
            bucket["actual"].append(force_body[0])
            bucket["gap"].append(force_body[0] - desired_x)

    if not desired_force_x:
        print("validation=FAIL: no matched walking rows")
        return 1

    lines = [
        "task wrench gap audit",
        f"state_rows={len(state_rows)}",
        f"truth_rows={len(truth_rows)}",
        f"walking_rows={walking_rows}",
        f"matched_rows={len(desired_force_x)}",
        f"time_match_failures={match_failures}",
        "max_time_error_s=%.9g" % max(time_errors),
        "desired_force_x_min_n=%.9g" % min(desired_force_x),
        "desired_force_x_max_n=%.9g" % max(desired_force_x),
        "actual_force_body_x_min_n=%.9g" % min(actual_force_body[0]),
        "actual_force_body_x_max_n=%.9g" % max(actual_force_body[0]),
        "desired_vs_actual_force_x_correlation=%.9g"
        % correlation(desired_force_x, actual_force_body[0]),
    ]
    for contact_count, bucket in by_contact.items():
        if not bucket["desired"]:
            continue
        lines.extend(
            [
                "contact_%d_rows=%d" % (contact_count, len(bucket["desired"])),
                "contact_%d_force_x_correlation=%.9g" % (contact_count, correlation(bucket["desired"], bucket["actual"])),
                "contact_%d_force_x_gap_p95_abs_n=%.9g" % (contact_count, percentile([abs(value) for value in bucket["gap"]], 0.95)),
                "contact_%d_force_x_gap_rms_n=%.9g" % (contact_count, rms(bucket["gap"])),
            ]
        )
    for index, axis in enumerate(AXES):
        lines.extend(
            [
                "force_gap_body_%s_bias_n=%.9g"
                % (axis, statistics.fmean(gaps_force[index])),
                "force_gap_body_%s_rms_n=%.9g"
                % (axis, rms(gaps_force[index])),
                "force_gap_body_%s_p95_abs_n=%.9g"
                % (axis, percentile(
                    [abs(value) for value in gaps_force[index]], 0.95
                )),
                "force_gap_body_%s_max_abs_n=%.9g"
                % (axis, max(abs(value) for value in gaps_force[index])),
                "actual_moment_body_%s_p95_abs_nm=%.9g"
                % (axis, percentile(
                    [abs(value) for value in actual_moment_body[index]], 0.95
                )),
                "actual_moment_body_%s_max_abs_nm=%.9g"
                % (axis, max(abs(value) for value in actual_moment_body[index])),
                "zero_moment_gap_body_%s_rms_nm=%.9g"
                % (axis, rms(gaps_moment[index])),
            ]
        )
    lines.extend(
        [
            "interpretation=diagnostic_only_desired_body_wrench_vs_truth",
            "validation=PASS",
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
