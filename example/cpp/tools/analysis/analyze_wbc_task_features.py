#!/usr/bin/env python3
"""Audit the task signals already driving the IK/PD locomotion chain."""

import argparse
import csv
import math
from pathlib import Path


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("%s has no CSV header" % path)
        return list(reader), set(reader.fieldnames)


def values(rows, name):
    return [float(row[name]) for row in rows]


def correlation(left, right):
    if not left or len(left) != len(right):
        return 0.0
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    denominator = math.sqrt(
        sum((value - left_mean) ** 2 for value in left)
        * sum((value - right_mean) ** 2 for value in right)
    )
    if denominator == 0.0:
        return 0.0
    return sum(
        (a - left_mean) * (b - right_mean)
        for a, b in zip(left, right)
    ) / denominator


def mean_abs(rows, name):
    data = [abs(float(row[name])) for row in rows]
    return sum(data) / len(data) if data else 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--target-speed", type=float, required=True)
    parser.add_argument("--direction-sign", type=float, default=1.0)
    args = parser.parse_args()

    rows, fields = load_rows(args.csv)
    required = {
        "motion_stage",
        "body_velocity_x_mps",
        "wbc_shadow_desired_force_x_n",
        "imu_roll_rad",
        "imu_pitch_rad",
        "attitude_feedback_x_m",
        "attitude_feedback_y_m",
        "contact_count",
    }
    missing = sorted(required - fields)
    if missing:
        raise ValueError("missing required fields: %s" % ", ".join(missing))

    walking = [
        row for row in rows
        if float(row["motion_stage"]) == 2.0
    ]
    if not walking:
        raise ValueError("no walking rows")
    raw_gyro_fields = [
        "imu_gyro_x_radps", "imu_gyro_y_radps", "imu_gyro_z_radps"
    ]
    body_gyro_fields = [
        "imu_gyro_body_x_radps", "imu_gyro_body_y_radps",
        "imu_gyro_body_z_radps"
    ]
    has_gyro = all(field in fields for field in raw_gyro_fields)
    has_body_gyro = all(field in fields for field in body_gyro_fields)
    target_velocity = args.direction_sign * args.target_speed
    body_velocity = values(walking, "body_velocity_x_mps")
    velocity_error = [target_velocity - value for value in body_velocity]
    desired_force = values(walking, "wbc_shadow_desired_force_x_n")
    pitch = values(walking, "imu_pitch_rad")
    roll = values(walking, "imu_roll_rad")
    attitude_x = values(walking, "attitude_feedback_x_m")
    attitude_y = values(walking, "attitude_feedback_y_m")

    lines = [
        "wbc task feature audit",
        "imu_gyro_body_fields_present=%d" % int(has_body_gyro),
        "imu_gyro_fields_present=%d" % int(has_gyro),
        "input_rows=%d" % len(rows),
        "walking_rows=%d" % len(walking),
        "target_velocity_mps=%.6f" % target_velocity,
        "body_velocity_min_mps=%.6f" % min(body_velocity),
        "body_velocity_max_mps=%.6f" % max(body_velocity),
        "desired_force_min_n=%.6f" % min(desired_force),
        "desired_force_max_n=%.6f" % max(desired_force),
        "desired_force_velocity_error_correlation=%.6f" % (
            correlation(desired_force, velocity_error)
        ),
        "attitude_x_pitch_correlation=%.6f" % (
            correlation(attitude_x, pitch)
        ),
        "attitude_y_roll_correlation=%.6f" % (
            correlation(attitude_y, roll)
        ),
        "mean_abs_pitch_rad=%.6f" % mean_abs(walking, "imu_pitch_rad"),
        "mean_abs_roll_rad=%.6f" % mean_abs(walking, "imu_roll_rad"),
        "mean_abs_attitude_x_m=%.6f" % mean_abs(
            walking, "attitude_feedback_x_m"
        ),
        "mean_abs_attitude_y_m=%.6f" % mean_abs(
            walking, "attitude_feedback_y_m"
        ),
    ]

    if has_gyro:
        for field in raw_gyro_fields:
            data = [abs(float(row[field])) for row in walking]
            lines.extend(
                [
                    "%s_mean_abs=%.6f" % (field, sum(data) / len(data)),
                    "%s_max_abs=%.6f" % (field, max(data)),
                ]
            )

    if has_body_gyro:
        for field in body_gyro_fields:
            data = [abs(float(row[field])) for row in walking]
            lines.extend(
                [
                    "%s_mean_abs=%.6f" % (field, sum(data) / len(data)),
                    "%s_max_abs=%.6f" % (field, max(data)),
                ]
            )
    for contact_count in (2, 4):
        phase_rows = [
            row for row in walking
            if int(float(row["contact_count"])) == contact_count
        ]
        if not phase_rows:
            continue
        lines.extend(
            [
                "contact_%d_rows=%d" % (contact_count, len(phase_rows)),
                "contact_%d_mean_desired_force_n=%.6f" % (
                    contact_count,
                    sum(
                        float(row["wbc_shadow_desired_force_x_n"])
                        for row in phase_rows
                    ) / len(phase_rows),
                ),
                "contact_%d_mean_abs_roll_rad=%.6f" % (
                    contact_count, mean_abs(phase_rows, "imu_roll_rad")
                ),
                "contact_%d_mean_abs_pitch_rad=%.6f" % (
                    contact_count, mean_abs(phase_rows, "imu_pitch_rad")
                ),
            ]
        )

    lines.append("validation=PASS")
    report = "\n".join(lines) + "\n"
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")


if __name__ == "__main__":
    main()
