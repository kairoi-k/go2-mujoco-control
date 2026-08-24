#!/usr/bin/env python3
"""Strict acceptance check for a normal, non-dragging 1 m/s WBC run."""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import statistics
import sys


LEGS = ("FR", "FL", "RR", "RL")


def percentile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    index = (len(values) - 1) * q / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return values[lower]
    weight = index - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"missing header: {path}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows


def floats(rows: list[dict[str, str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--min-cruise-speed", type=float, default=1.0)
    parser.add_argument("--cruise-trim-start-s", type=float, default=1.0)
    parser.add_argument("--cruise-trim-end-s", type=float, default=1.5)
    args = parser.parse_args()

    run_dir = args.run_dir
    metadata = run_dir / "run_metadata.txt"
    controller_log = run_dir / "controller.log"
    data_path = run_dir / "data.csv"
    truth_path = run_dir / "contact_ground_truth.csv"
    failures: list[str] = []

    if not metadata.exists() or not controller_log.exists():
        print("validation=FAIL")
        print("failure=missing run metadata or controller log")
        return 2

    metadata_values: dict[str, str] = {}
    for line in metadata.read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metadata_values[key] = value
    for key in (
        "controller_status",
        "safety_status",
        "quality_status",
        "analysis_status",
        "ground_truth_status",
        "dynamics_status",
        "completion_status",
    ):
        if metadata_values.get(key) != "0":
            failures.append(f"{key}={metadata_values.get(key, 'missing')}")

    log_text = controller_log.read_text(errors="replace")
    cycle_count = len(re.findall(r"Trot cycle .* health:", log_text))
    if cycle_count < 40:
        failures.append(f"cycle_health_count={cycle_count}<40")
    if "Trot pre-stop brake: reducing gait reference" not in log_text:
        failures.append("pre_stop_brake_not_recorded")
    if "Trot stopping; returning to stand" not in log_text:
        failures.append("controlled_stop_not_recorded")
    if re.search(r"Trot (?:hard|cycle quality guard|safety rejected)", log_text):
        failures.append("controller_log_contains_rejection")

    try:
        data = read_csv(data_path)
        truth = read_csv(truth_path)
    except (OSError, ValueError, KeyError) as exc:
        print("validation=FAIL")
        print(f"failure={exc}")
        return 2

    required_data = {"state_tick_s", "motion_stage"}
    required_truth = {
        "time_s",
        "base_qvel_world_x_mps",
        "base_pos_world_z_m",
        "base_angvel_body_x_radps",
        "base_angvel_body_y_radps",
    }
    required_truth.update(f"{leg}_pos_world_z_m" for leg in LEGS)
    missing_data = sorted(required_data.difference(data[0]))
    missing_truth = sorted(required_truth.difference(truth[0]))
    if missing_data:
        failures.append("data_missing=" + ",".join(missing_data))
    if missing_truth:
        failures.append("truth_missing=" + ",".join(missing_truth))
    if missing_data or missing_truth:
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1

    stage2 = [row for row in data if int(float(row["motion_stage"])) == 2]
    if not stage2:
        failures.append("no_motion_stage_2_samples")
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1
    stage2_start = min(float(row["state_tick_s"]) for row in stage2)
    stage2_end = max(float(row["state_tick_s"]) for row in stage2)
    cruise_start = stage2_start + args.cruise_trim_start_s
    cruise_end = stage2_end - args.cruise_trim_end_s
    cruise = [
        row for row in truth
        if cruise_start <= float(row["time_s"]) < cruise_end
    ]
    if len(cruise) < 100:
        failures.append(f"cruise_samples={len(cruise)}<100")
        cruise = [row for row in truth if stage2_start <= float(row["time_s"]) <= stage2_end]

    speed = floats(cruise, "base_qvel_world_x_mps")
    base_z = floats(cruise, "base_pos_world_z_m")
    foot_z = {
        leg: floats(cruise, f"{leg}_pos_world_z_m") for leg in LEGS
    }
    speed_p05 = percentile(speed, 5.0)
    speed_p50 = percentile(speed, 50.0)
    speed_p95 = percentile(speed, 95.0)
    base_z_p01 = percentile(base_z, 1.0)
    base_z_p50 = percentile(base_z, 50.0)
    base_z_p99 = percentile(base_z, 99.0)
    foot_p95 = {leg: percentile(foot_z[leg], 95.0) for leg in LEGS}
    foot_p99 = {leg: percentile(foot_z[leg], 99.0) for leg in LEGS}
    all_feet_low_fraction = sum(
        all(float(row[f"{leg}_pos_world_z_m"]) <= 0.035 for leg in LEGS)
        for row in cruise
    ) / max(1, len(cruise))

    if not math.isfinite(speed_p50) or speed_p50 < args.min_cruise_speed:
        failures.append(
            f"speed_median={speed_p50:.4f}<{args.min_cruise_speed:.4f}"
        )
    if not math.isfinite(speed_p05) or speed_p05 < 0.85:
        failures.append(f"speed_p05={speed_p05:.4f}<0.8500")
    if not (0.33 <= base_z_p01 and base_z_p99 <= 0.40):
        failures.append(
            f"base_z_quantiles=[{base_z_p01:.4f},{base_z_p99:.4f}]"
        )
    for leg in LEGS:
        if foot_p95[leg] < 0.045:
            failures.append(f"{leg}_swing_clearance_p95={foot_p95[leg]:.4f}<0.0450")
    if all_feet_low_fraction > 0.35:
        failures.append(
            f"all_feet_low_fraction={all_feet_low_fraction:.4f}>0.3500"
        )

    truth_time = floats(truth, "time_s")
    stop_start = max(stage2_end, truth_time[0])
    stop_rows = [row for row in truth if float(row["time_s"]) >= stop_start + 1.5]
    if len(stop_rows) < 100:
        failures.append(f"stop_tail_samples={len(stop_rows)}<100")
    else:
        stop_speed = [abs(float(row["base_qvel_world_x_mps"])) for row in stop_rows]
        stop_roll_rate = [abs(float(row["base_angvel_body_x_radps"])) for row in stop_rows]
        stop_pitch_rate = [abs(float(row["base_angvel_body_y_radps"])) for row in stop_rows]
        if percentile(stop_speed, 95.0) > 0.10:
            failures.append(f"stop_speed_p95={percentile(stop_speed, 95.0):.4f}>0.1000")
        if percentile(stop_roll_rate, 95.0) > 0.60 or percentile(stop_pitch_rate, 95.0) > 0.60:
            failures.append("stop_body_rate_p95>0.60")

    print(f"stage2_window_s={stage2_start:.3f}..{stage2_end:.3f}")
    print(f"cruise_window_s={cruise_start:.3f}..{cruise_end:.3f}")
    print(f"cycle_health_count={cycle_count}")
    print(f"speed_p05_mps={speed_p05:.6f}")
    print(f"speed_median_mps={speed_p50:.6f}")
    print(f"speed_p95_mps={speed_p95:.6f}")
    print(f"base_z_p01_m={base_z_p01:.6f}")
    print(f"base_z_median_m={base_z_p50:.6f}")
    print(f"base_z_p99_m={base_z_p99:.6f}")
    print("foot_z_p95_m=" + ",".join(f"{leg}:{foot_p95[leg]:.6f}" for leg in LEGS))
    print("foot_z_p99_m=" + ",".join(f"{leg}:{foot_p99[leg]:.6f}" for leg in LEGS))
    print(f"all_feet_low_fraction={all_feet_low_fraction:.6f}")
    print(f"stop_tail_samples={len(stop_rows)}")
    if failures:
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1
    print("validation=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
