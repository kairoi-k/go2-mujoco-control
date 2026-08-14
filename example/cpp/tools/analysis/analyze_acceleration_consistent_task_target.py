#!/usr/bin/env python3
"""Compare static and acceleration-adjusted task-force targets offline."""

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


def percentile_abs(values: list[float], fraction: float = 0.95) -> float:
    ordered = sorted(abs(value) for value in values)
    return ordered[int(fraction * (len(ordered) - 1))]


def rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values))


def maximum_abs(values: list[float]) -> float:
    return max(abs(value) for value in values)


def correlation(first: list[float], second: list[float]) -> float:
    first_mean = statistics.fmean(first)
    second_mean = statistics.fmean(second)
    numerator = sum(
        (left - first_mean) * (right - second_mean)
        for left, right in zip(first, second)
    )
    first_norm = math.sqrt(
        sum((value - first_mean) ** 2 for value in first)
    )
    second_norm = math.sqrt(
        sum((value - second_mean) ** 2 for value in second)
    )
    if first_norm <= 1e-12 or second_norm <= 1e-12:
        return 0.0
    return numerator / (first_norm * second_norm)


def body_to_world(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid base quaternion")
    w, x, y, z = (value / norm for value in quaternion)
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-csv", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
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
            reader = csv.DictReader(handle)
            state_fields = reader.fieldnames or []
            state_required = {
                "state_tick_s",
                "motion_stage",
                "contact_count",
                "wbc_shadow_desired_force_x_n",
            }
            missing = sorted(state_required - set(state_fields))
            if missing:
                raise ValueError(
                    "state CSV missing fields: " + ",".join(missing)
                )
            state_rows = list(reader)
        with args.ground_truth_csv.open(
            newline="", encoding="utf-8"
        ) as handle:
            reader = csv.DictReader(handle)
            truth_fields = reader.fieldnames or []
            truth_required = {
                "time_s",
                "base_quat_w",
                "base_quat_x",
                "base_quat_y",
                "base_quat_z",
            }
            truth_required.update(
                f"base_qacc_world_{axis}_mps2" for axis in AXES
            )
            truth_required.update(
                f"total_contact_grf_world_{axis}_N" for axis in AXES
            )
            missing = sorted(truth_required - set(truth_fields))
            if missing:
                raise ValueError(
                    "ground-truth CSV missing fields: "
                    + ",".join(missing)
                )
            truth_rows = list(reader)
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not truth_rows:
        print("validation=FAIL: empty ground-truth CSV")
        return 1

    truth_times = [finite(row, "time_s") for row in truth_rows]
    if any(
        truth_times[index] <= truth_times[index - 1]
        for index in range(1, len(truth_times))
    ):
        print("validation=FAIL: truth time is not strictly increasing")
        return 1

    static_errors = {axis: [] for axis in AXES}
    adjusted_errors = {axis: [] for axis in AXES}
    acceleration_force = {axis: [] for axis in AXES}
    by_contact: dict[int, dict[str, list[float]]] = {}
    detail_rows: list[dict[str, object]] = []
    walking_rows = 0
    matched_rows = 0
    match_failures = 0
    invalid_rows = 0
    time_errors: list[float] = []

    for state in state_rows:
        try:
            if finite(state, "motion_stage") != 2.0:
                continue
            walking_rows += 1
            state_time = finite(state, "state_tick_s")
            contact_count = int(finite(state, "contact_count"))
            desired_force_x = finite(
                state, "wbc_shadow_desired_force_x_n"
            )
        except (KeyError, ValueError):
            invalid_rows += 1
            continue

        truth_index = bisect_left(truth_times, state_time)
        candidates = [
            index
            for index in (truth_index - 1, truth_index)
            if 0 <= index < len(truth_rows)
        ]
        if not candidates:
            match_failures += 1
            continue
        truth_index = min(
            candidates,
            key=lambda index: abs(truth_times[index] - state_time),
        )
        time_error = truth_times[truth_index] - state_time
        if abs(time_error) > args.match_tolerance_s:
            match_failures += 1
            continue

        try:
            truth = truth_rows[truth_index]
            quaternion = tuple(
                finite(truth, f"base_quat_{axis}")
                for axis in ("w", "x", "y", "z")
            )
            static_target = body_to_world(
                quaternion,
                (desired_force_x, 0.0, args.mass_kg * args.gravity_mps2),
            )
            measured_accel = tuple(
                finite(truth, f"base_qacc_world_{axis}_mps2")
                for axis in AXES
            )
            actual_force = tuple(
                finite(truth, f"total_contact_grf_world_{axis}_N")
                for axis in AXES
            )
        except (KeyError, ValueError):
            invalid_rows += 1
            continue

        adjusted_target = tuple(
            static_target[index] + args.mass_kg * measured_accel[index]
            for index in range(3)
        )
        static_error = tuple(
            actual_force[index] - static_target[index]
            for index in range(3)
        )
        adjusted_error = tuple(
            actual_force[index] - adjusted_target[index]
            for index in range(3)
        )
        matched_rows += 1
        time_errors.append(abs(time_error))
        contact_bucket = by_contact.setdefault(
            contact_count,
            {
                "static_x": [],
                "static_y": [],
                "static_z": [],
                "adjusted_x": [],
                "adjusted_y": [],
                "adjusted_z": [],
            },
        )
        for index, axis in enumerate(AXES):
            static_errors[axis].append(static_error[index])
            adjusted_errors[axis].append(adjusted_error[index])
            acceleration_force[axis].append(
                args.mass_kg * measured_accel[index]
            )
            contact_bucket[f"static_{axis}"].append(static_error[index])
            contact_bucket[f"adjusted_{axis}"].append(adjusted_error[index])
        detail = {
            "state_time_s": state_time,
            "truth_time_s": truth_times[truth_index],
            "time_error_s": time_error,
            "contact_count": contact_count,
            "desired_force_x_body_n": desired_force_x,
        }
        for index, axis in enumerate(AXES):
            detail[f"base_qacc_world_{axis}_mps2"] = measured_accel[index]
            detail[f"actual_contact_force_world_{axis}_N"] = actual_force[index]
            detail[f"static_target_force_world_{axis}_N"] = static_target[index]
            detail[
                f"acceleration_adjusted_target_force_world_{axis}_N"
            ] = adjusted_target[index]
            detail[f"static_error_world_{axis}_N"] = static_error[index]
            detail[f"acceleration_adjusted_error_world_{axis}_N"] = (
                adjusted_error[index]
            )
        detail_rows.append(detail)

    if matched_rows == 0:
        print("validation=FAIL: no matched walking rows")
        return 1

    detail_fields = list(detail_rows[0])
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(detail_rows)

    arithmetic_pass = (
        match_failures == 0
        and invalid_rows == 0
        and max(time_errors) <= args.match_tolerance_s
    )
    lines = [
        "acceleration-consistent task-target diagnostic",
        f"state_rows={len(state_rows)}",
        f"truth_rows={len(truth_rows)}",
        f"walking_rows={walking_rows}",
        f"matched_rows={matched_rows}",
        f"match_failures={match_failures}",
        f"invalid_rows={invalid_rows}",
        "static_target_note=body_task_force_plus_gravity_rotated_to_world",
        "adjusted_target_note=static_target_plus_mass_times_measured_linear_acceleration",
        "adjusted_target_note_status=oracle_only_not_a_controller_command",
        "max_time_error_s=%.9g" % max(time_errors),
        "mass_kg=%.9g" % args.mass_kg,
        "gravity_mps2=%.9g" % args.gravity_mps2,
    ]
    for axis in AXES:
        lines.extend(
            [
                f"static_error_world_{axis}_p95_abs_N=%.9g"
                % percentile_abs(static_errors[axis]),
                f"static_error_world_{axis}_rms_N=%.9g"
                % rms(static_errors[axis]),
                f"static_error_world_{axis}_max_abs_N=%.9g"
                % maximum_abs(static_errors[axis]),
                f"adjusted_error_world_{axis}_p95_abs_N=%.9g"
                % percentile_abs(adjusted_errors[axis]),
                f"adjusted_error_world_{axis}_rms_N=%.9g"
                % rms(adjusted_errors[axis]),
                f"adjusted_error_world_{axis}_max_abs_N=%.9g"
                % maximum_abs(adjusted_errors[axis]),
                f"measured_acceleration_force_world_{axis}_p95_abs_N=%.9g"
                % percentile_abs(acceleration_force[axis]),
                f"static_error_vs_acceleration_force_{axis}_correlation=%.9g"
                % correlation(static_errors[axis], acceleration_force[axis]),
            ]
        )
    for contact_count, bucket in sorted(by_contact.items()):
        lines.append(f"contact_{contact_count}_rows={len(bucket['static_x'])}")
        for axis in AXES:
            lines.extend(
                [
                    f"contact_{contact_count}_static_error_{axis}_p95_abs_N=%.9g"
                    % percentile_abs(bucket[f"static_{axis}"]),
                    f"contact_{contact_count}_adjusted_error_{axis}_p95_abs_N=%.9g"
                    % percentile_abs(bucket[f"adjusted_{axis}"]),
                ]
            )
    lines.extend(
        [
            "interpretation=oracle_diagnostic_only_no_wbc_injection",
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
