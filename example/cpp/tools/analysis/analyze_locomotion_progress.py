#!/usr/bin/env python3
"""Audit measured locomotion speed against the commanded walking target."""

from __future__ import annotations

import argparse
import csv
import math
import sys


def finite(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--target-speed", type=float, required=True)
    parser.add_argument("--direction-sign", type=float, default=1.0)
    parser.add_argument("--min-speed-ratio", type=float, default=0.0)
    parser.add_argument("--min-cycle", type=int, default=-1)
    parser.add_argument("--last-seconds", type=float, default=0.0)
    args = parser.parse_args()

    if (
        not math.isfinite(args.target_speed)
        or args.target_speed <= 0.0
        or abs(abs(args.direction_sign) - 1.0) > 1e-9
        or not math.isfinite(args.min_speed_ratio)
        or args.min_speed_ratio < 0.0
    ):
        print("validation=FAIL: invalid speed audit parameters")
        return 2

    try:
        with open(args.csv_path, newline="") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                print("validation=FAIL: missing CSV header")
                return 2
            required = {
                "state_tick_s",
                "motion_stage",
                "cycle_index",
                "world_base_x_m",
                "world_base_y_m",
            }
            missing = sorted(required.difference(reader.fieldnames))
            if missing:
                print("validation=FAIL: missing columns=" + ",".join(missing))
                return 2
            rows = list(reader)
    except OSError as exc:
        print(f"validation=FAIL: cannot read CSV: {exc}")
        return 2

    walking = []
    try:
        for row_number, row in enumerate(rows, start=2):
            if int(row["motion_stage"]) != 2 or int(row["cycle_index"]) < 0:
                continue
            walking.append(
                (
                    finite(row, "state_tick_s"),
                    finite(row, "world_base_x_m"),
                    finite(row, "world_base_y_m"),
                    int(row["cycle_index"]),
                )
            )
    except (TypeError, ValueError) as exc:
        print(f"validation=FAIL: invalid locomotion row: {exc}")
        return 2

    if len(walking) < 2:
        print("validation=FAIL: fewer than two walking samples")
        return 2

    walking.sort()
    if args.min_cycle >= 0:
        walking = [sample for sample in walking if sample[3] >= args.min_cycle]
    if args.last_seconds > 0.0:
        end_t = walking[-1][0] if walking else 0.0
        walking = [
            sample
            for sample in walking
            if sample[0] >= end_t - args.last_seconds
        ]
    if len(walking) < 2:
        print("validation=FAIL: fewer than two walking samples after window")
        return 2

    start_t, start_x, start_y, _start_cycle = walking[0]
    end_t, end_x, end_y, _end_cycle = walking[-1]
    duration = end_t - start_t
    if duration <= 0.0:
        print("validation=FAIL: non-positive walking duration")
        return 2

    mean_t = sum(sample[0] for sample in walking) / len(walking)
    mean_x = sum(sample[1] for sample in walking) / len(walking)
    denominator = sum((sample[0] - mean_t) ** 2 for sample in walking)
    if denominator <= 0.0:
        print("validation=FAIL: zero time variance")
        return 2

    regression_slope = sum(
        (sample[0] - mean_t) * (sample[1] - mean_x)
        for sample in walking
    ) / denominator
    measured_speed = args.direction_sign * regression_slope
    endpoint_speed = args.direction_sign * (end_x - start_x) / duration
    speed_ratio = measured_speed / args.target_speed
    lateral_drift = max(
        abs(sample[2] - start_y) for sample in walking
    )

    validation = (
        math.isfinite(measured_speed)
        and speed_ratio >= args.min_speed_ratio
    )
    print(f"walking_rows={len(walking)}")
    print(f"walking_duration_s={duration:.6f}")
    print(f"walking_distance_m={args.direction_sign * (end_x - start_x):.6f}")
    print(f"target_speed_mps={args.target_speed:.6f}")
    print(f"measured_speed_mps={measured_speed:.6f}")
    print(f"endpoint_speed_mps={endpoint_speed:.6f}")
    print(f"speed_ratio={speed_ratio:.6f}")
    print(f"max_lateral_drift_m={lateral_drift:.6f}")
    print(f"min_speed_ratio={args.min_speed_ratio:.6f}")
    print(f"validation={'PASS' if validation else 'FAIL'}")
    return 0 if validation else 1


if __name__ == "__main__":
    sys.exit(main())
