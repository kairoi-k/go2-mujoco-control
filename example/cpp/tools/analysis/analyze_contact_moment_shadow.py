#!/usr/bin/env python3
"""Audit contact moment and base angular-acceleration signals offline."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

AXES = ("x", "y", "z")
LEGS = ("FR", "FL", "RR", "RL")


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(fraction * (len(ordered) - 1))
    return ordered[index]


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


def correlation(left: list[float], right: list[float]) -> float:
    if not left or len(left) != len(right):
        return 0.0
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
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
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--contact-force-threshold-n", type=float, default=5.0)
    args = parser.parse_args()

    if args.contact_force_threshold_n <= 0.0:
        print("validation=FAIL: contact threshold must be positive")
        return 2

    try:
        with args.ground_truth_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            required = {
                "time_s",
                "subtree_com_world_x_m",
                "subtree_com_world_y_m",
                "subtree_com_world_z_m",
                "base_quat_w",
                "base_quat_x",
                "base_quat_y",
                "base_quat_z",
            }
            required.update(
                f"base_angacc_body_{axis}_radps2" for axis in AXES
            )
            for leg in LEGS:
                required.update(
                    f"{leg}_pos_world_{axis}_m" for axis in AXES
                )
                required.update(
                    f"{leg}_foot_contact_grf_world_{axis}_N"
                    for axis in AXES
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

    moments = [[] for _ in AXES]
    angular_accelerations = [[] for _ in AXES]
    moment_norms = []
    contact_histogram = {count: 0 for count in range(len(LEGS) + 1)}
    previous_time = -math.inf

    try:
        for row_number, row in enumerate(rows, start=2):
            time_s = finite(row, "time_s")
            if time_s <= previous_time:
                raise ValueError(
                    f"row {row_number}: time is not strictly increasing"
                )
            previous_time = time_s

            com = tuple(
                finite(row, f"subtree_com_world_{axis}_m")
                for axis in AXES
            )
            quaternion = tuple(
                finite(row, f"base_quat_{axis}")
                for axis in ("w", "x", "y", "z")
            )
            moment_world = [0.0, 0.0, 0.0]
            contact_count = 0
            for leg in LEGS:
                foot_position = tuple(
                    finite(row, f"{leg}_pos_world_{axis}_m")
                    for axis in AXES
                )
                force_world = tuple(
                    finite(
                        row,
                        f"{leg}_foot_contact_grf_world_{axis}_N",
                    )
                    for axis in AXES
                )
                if vector_norm(force_world) >= args.contact_force_threshold_n:
                    contact_count += 1
                lever = tuple(
                    foot_position[index] - com[index] for index in range(3)
                )
                moment_world[0] += (
                    lever[1] * force_world[2]
                    - lever[2] * force_world[1]
                )
                moment_world[1] += (
                    lever[2] * force_world[0]
                    - lever[0] * force_world[2]
                )
                moment_world[2] += (
                    lever[0] * force_world[1]
                    - lever[1] * force_world[0]
                )

            moment_body = world_to_body(quaternion, tuple(moment_world))
            angular_acceleration = tuple(
                finite(row, f"base_angacc_body_{axis}_radps2")
                for axis in AXES
            )
            for index in range(3):
                moments[index].append(moment_body[index])
                angular_accelerations[index].append(
                    angular_acceleration[index]
                )
            moment_norms.append(vector_norm(moment_body))
            contact_histogram[contact_count] += 1
    except (KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 1

    lines = [
        "contact moment shadow audit",
        f"rows={len(rows)}",
        "contact_force_threshold_n=%.9g" % args.contact_force_threshold_n,
    ]
    lines.extend(
        f"contact_count_{count}={contact_histogram[count]}"
        for count in sorted(contact_histogram)
    )
    for index, axis in enumerate(AXES):
        moment = moments[index]
        angular_acceleration = angular_accelerations[index]
        lines.extend(
            [
                "moment_body_%s_p95_abs_nm=%.9g"
                % (axis, percentile([abs(value) for value in moment], 0.95)),
                "moment_body_%s_max_abs_nm=%.9g"
                % (axis, max(abs(value) for value in moment)),
                "base_angacc_body_%s_p95_abs_radps2=%.9g"
                % (
                    axis,
                    percentile(
                        [abs(value) for value in angular_acceleration],
                        0.95,
                    ),
                ),
                "moment_vs_base_angacc_%s_correlation=%.9g"
                % (axis, correlation(moment, angular_acceleration)),
            ]
        )
    lines.extend(
        [
            "moment_norm_p95_nm=%.9g" % percentile(moment_norms, 0.95),
            "moment_norm_max_nm=%.9g" % max(moment_norms),
            "structural_validation=PASS",
            "interpretation=diagnostic_only_no_full_body_inertia_inference",
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
