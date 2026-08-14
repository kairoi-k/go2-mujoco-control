#!/usr/bin/env python3
"""Validate MuJoCo ground-truth foot-force CSV output."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys

LEGS = ("FR", "FL", "RR", "RL")
AXES = ("x", "y", "z")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--touch-threshold", type=float, default=5.0)
    args = parser.parse_args()

    try:
        with open(args.csv_path, newline="") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                print("validation=FAIL: missing CSV header")
                return 2
            rows = list(reader)
    except OSError as exc:
        print(f"validation=FAIL: cannot read CSV: {exc}")
        return 2

    required = {"time_s", "step_index"}
    for leg in LEGS:
        required.update(
            f"{leg}_sensor_force_world_{axis}_N" for axis in AXES
        )
        required.update(f"{leg}_contact_grf_world_{axis}_N" for axis in AXES)
        required.update(f"{leg}_pos_world_{axis}_m" for axis in AXES)
        required.add(f"{leg}_touch_N")

    missing = sorted(required.difference(reader.fieldnames))
    if missing:
        print("validation=FAIL: missing columns=" + ",".join(missing))
        return 2
    if not rows:
        print("validation=FAIL: CSV has no samples")
        return 2

    errors = []
    times = []
    steps = []
    contact_grf_z = []
    negative_contact_grf_z = 0
    max_contact_grf_norm = 0.0

    for row_number, row in enumerate(rows, start=2):
        try:
            time_s = float(row["time_s"])
            step_index = int(row["step_index"])
        except (TypeError, ValueError) as exc:
            errors.append(f"row {row_number}: invalid time or step index: {exc}")
            continue

        if not math.isfinite(time_s):
            errors.append(f"row {row_number}: non-finite time")
        if times and time_s <= times[-1]:
            errors.append(f"row {row_number}: time is not strictly increasing")
        if steps and step_index != steps[-1] + 1:
            errors.append(
                f"row {row_number}: step index jump "
                f"{steps[-1]}->{step_index}"
            )
        times.append(time_s)
        steps.append(step_index)

        for leg in LEGS:
            contact_grf_values = {}
            try:
                for axis in AXES:
                    raw = float(
                        row[f"{leg}_sensor_force_world_{axis}_N"]
                    )
                    contact_grf = float(row[f"{leg}_contact_grf_world_{axis}_N"])
                    contact_grf_values[axis] = contact_grf
                    if not math.isfinite(raw) or not math.isfinite(contact_grf):
                        errors.append(
                            f"row {row_number} {leg}: non-finite force"
                        )
                touch = float(row[f"{leg}_touch_N"])
                if all(math.isfinite(contact_grf_values[axis]) for axis in AXES):
                    max_contact_grf_norm = max(
                        max_contact_grf_norm,
                        math.sqrt(sum(contact_grf_values[axis] ** 2 for axis in AXES)),
                    )
            except (TypeError, ValueError) as exc:
                errors.append(f"row {row_number} {leg}: invalid force: {exc}")
                continue

            if touch > args.touch_threshold:
                grf_z = contact_grf_values["z"]
                contact_grf_z.append(grf_z)
                if grf_z < -1e-6:
                    negative_contact_grf_z += 1

        if len(errors) > 20:
            break

    if errors:
        print("validation=FAIL")
        for error in errors[:20]:
            print(error)
        return 1

    dts = [right - left for left, right in zip(times, times[1:])]
    median_dt = statistics.median(dts) if dts else 0.0
    min_contact_z = min(contact_grf_z) if contact_grf_z else float("nan")
    max_contact_z = max(contact_grf_z) if contact_grf_z else float("nan")
    print(f"rows={len(rows)}")
    print(f"time_span_s={times[-1] - times[0]:.9g}")
    print(f"median_dt_s={median_dt:.9g}")
    print(f"contact_samples={len(contact_grf_z)}")
    print(f"min_contact_grf_z_N={min_contact_z:.9g}")
    print(f"max_contact_grf_z_N={max_contact_z:.9g}")
    print(f"max_contact_grf_norm_N={max_contact_grf_norm:.9g}")
    print(f"negative_contact_grf_z_samples={negative_contact_grf_z}")
    print("validation=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
