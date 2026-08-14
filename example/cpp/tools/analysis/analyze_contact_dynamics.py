#!/usr/bin/env python3
"""Audit discrete translational Newton balance using MuJoCo state samples."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys

AXES = ("x", "y", "z")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument(
        "--balance-tolerance-n",
        type=float,
        default=10.0,
        help="allowed 95th-percentile force-balance residual",
    )
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

    required = {"time_s", "step_index", "total_mass_kg"}
    required.update(
        f"subtree_linvel_world_{axis}_mps" for axis in AXES
    )
    required.update(
        f"gravity_world_{axis}_mps2" for axis in AXES
    )
    required.update(
        f"total_contact_grf_world_{axis}_N" for axis in AXES
    )

    missing = sorted(required.difference(reader.fieldnames))
    if missing:
        print("validation=FAIL: missing columns=" + ",".join(missing))
        return 2
    if not rows:
        print("validation=FAIL: CSV has no samples")
        return 2
    if args.balance_tolerance_n < 0.0:
        print("validation=FAIL: tolerance must be non-negative")
        return 2

    errors = []
    times = []
    steps = []
    velocities = []
    masses = []
    gravity_samples = []
    force_samples = []

    for row_number, row in enumerate(rows, start=2):
        try:
            time_s = float(row["time_s"])
            step_index = int(row["step_index"])
            mass = float(row["total_mass_kg"])
            velocity = {
                axis: float(row[f"subtree_linvel_world_{axis}_mps"])
                for axis in AXES
            }
            gravity = {
                axis: float(row[f"gravity_world_{axis}_mps2"])
                for axis in AXES
            }
            grf = {
                axis: float(row[f"total_contact_grf_world_{axis}_N"])
                for axis in AXES
            }
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"row {row_number}: invalid dynamics sample: {exc}")
            continue

        values = [
            time_s,
            mass,
            *velocity.values(),
            *gravity.values(),
            *grf.values(),
        ]
        if not all(math.isfinite(value) for value in values):
            errors.append(f"row {row_number}: non-finite dynamics sample")
            continue
        if mass <= 0.0:
            errors.append(f"row {row_number}: non-positive total mass")
        if times and time_s <= times[-1]:
            errors.append(f"row {row_number}: time is not strictly increasing")
        if steps and step_index != steps[-1] + 1:
            errors.append(
                f"row {row_number}: step index jump "
                f"{steps[-1]}->{step_index}"
            )

        times.append(time_s)
        steps.append(step_index)
        velocities.append(velocity)
        masses.append(mass)
        gravity_samples.append(gravity)
        force_samples.append(grf)

        if len(errors) > 20:
            break

    if errors:
        print("validation=FAIL")
        for error in errors[:20]:
            print(error)
        return 1
    if len(times) < 2:
        print("validation=FAIL: need at least two samples for forward differences")
        return 1

    residual_norms = []
    residuals = []
    for index in range(0, len(times) - 1):
        dt = times[index + 1] - times[index]
        if dt <= 0.0:
            print("validation=FAIL: non-positive forward-difference interval")
            return 1
        acceleration = {
            axis: (
                velocities[index + 1][axis] - velocities[index][axis]
            ) / dt
            for axis in AXES
        }
        expected_grf = {
            axis: masses[index] * (
                acceleration[axis] - gravity_samples[index][axis]
            )
            for axis in AXES
        }
        residual = {
            axis: force_samples[index][axis] - expected_grf[axis]
            for axis in AXES
        }
        residuals.append((times[index], residual))
        residual_norms.append(
            math.sqrt(sum(residual[axis] ** 2 for axis in AXES))
        )

    dts = [right - left for left, right in zip(times, times[1:])]
    median_dt = statistics.median(dts) if dts else 0.0
    sorted_norms = sorted(residual_norms)
    p95_index = int(0.95 * (len(sorted_norms) - 1))
    p95_residual_n = sorted_norms[p95_index]
    max_index = max(
        range(len(residual_norms)), key=residual_norms.__getitem__
    )
    max_residual_norm_n = residual_norms[max_index]
    max_residual_time_s, max_residual_components = residuals[max_index]
    max_abs_residual_n = max(
        abs(value)
        for _, residual in residuals
        for value in residual.values()
    )
    rms_residual_n = math.sqrt(
        sum(value ** 2 for value in residual_norms) / len(residual_norms)
    )
    gravity = gravity_samples[0]
    gravity_drift = max(
        abs(sample[axis] - gravity[axis])
        for sample in gravity_samples
        for axis in AXES
    )
    balance_pass = p95_residual_n <= args.balance_tolerance_n

    print(f"rows={len(rows)}")
    print(f"balance_samples={len(residual_norms)}")
    print(f"time_span_s={times[-1] - times[0]:.9g}")
    print(f"median_dt_s={median_dt:.9g}")
    print(f"total_mass_min_kg={min(masses):.9g}")
    print(f"total_mass_max_kg={max(masses):.9g}")
    print(
        "gravity_world_mps2="
        + ",".join(f"{gravity[axis]:.9g}" for axis in AXES)
    )
    print(f"gravity_drift_mps2={gravity_drift:.9g}")
    print(f"p95_force_balance_residual_N={p95_residual_n:.9g}")
    print(f"rms_force_balance_residual_N={rms_residual_n:.9g}")
    print(f"max_force_balance_component_residual_N={max_abs_residual_n:.9g}")
    print(f"max_force_balance_residual_norm_N={max_residual_norm_n:.9g}")
    print(f"max_residual_time_s={max_residual_time_s:.9g}")
    print(
        "max_residual_components_N="
        + ",".join(
            f"{max_residual_components[axis]:.9g}" for axis in AXES
        )
    )
    print(f"force_balance_tolerance_N={args.balance_tolerance_n:.9g}")
    print(f"force_balance_validation={'PASS' if balance_pass else 'FAIL'}")
    print("validation=PASS" if balance_pass else "validation=FAIL")
    return 0 if balance_pass else 1


if __name__ == "__main__":
    sys.exit(main())
