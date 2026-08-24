#!/usr/bin/env python3
"""Acceptance check for the validated 1 m/s natural-trot profile.

The check uses MuJoCo ground truth for the realized motion and the controller
log only for lifecycle/safety evidence.  It deliberately evaluates the
steady locomotion window, not the stand-up or return-to-stand handoff.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import statistics
import sys


LEGS = ("FR", "FL", "RR", "RL")
DIAGONALS = (("FR", "RL"), ("FL", "RR"))
CROSS_PAIRS = (("FR", "FL"), ("FR", "RR"), ("RL", "FL"), ("RL", "RR"))


def percentile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * q / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"missing header: {path}")
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty CSV: {path}")
    return rows


def value(row: dict[str, str], key: str) -> float:
    return float(row[key])


def quat_roll_pitch(row: dict[str, str]) -> tuple[float, float]:
    w = value(row, "base_quat_w")
    x = value(row, "base_quat_x")
    y = value(row, "base_quat_y")
    z = value(row, "base_quat_z")
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    return roll, pitch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--min-cruise-speed", type=float, default=1.0)
    parser.add_argument(
        "--min-speed-p05",
        type=float,
        default=0.85,
        help="minimum 5th-percentile forward speed in the cruise window",
    )
    parser.add_argument(
        "--min-cycle-count",
        type=int,
        default=40,
        help="minimum number of controller cycle-health records",
    )
    parser.add_argument("--cruise-trim-start-s", type=float, default=1.0)
    parser.add_argument("--cruise-trim-end-s", type=float, default=1.5)
    args = parser.parse_args()

    run_dir = args.run_dir
    metadata_path = run_dir / "run_metadata.txt"
    controller_log_path = run_dir / "controller.log"
    data_path = run_dir / "data.csv"
    truth_path = run_dir / "contact_ground_truth.csv"
    failures: list[str] = []

    if not metadata_path.exists() or not controller_log_path.exists():
        print("validation=FAIL")
        print("failure=missing run metadata or controller log")
        return 2

    metadata: dict[str, str] = {}
    for line in metadata_path.read_text().splitlines():
        if "=" in line:
            key, item = line.split("=", 1)
            metadata[key] = item
    for key in (
        "controller_status",
        "safety_status",
        "quality_status",
        "analysis_status",
        "ground_truth_status",
        "dynamics_status",
        "completion_status",
    ):
        if metadata.get(key) != "0":
            failures.append(f"{key}={metadata.get(key, 'missing')}")

    log_text = controller_log_path.read_text(errors="replace")
    cycle_count = len(re.findall(r"Trot cycle .* health:", log_text))
    if cycle_count < args.min_cycle_count:
        failures.append(
            f"cycle_health_count={cycle_count}<{args.min_cycle_count}"
        )
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
        "base_quat_w",
        "base_quat_x",
        "base_quat_y",
        "base_quat_z",
    }
    required_truth.update(f"{leg}_pos_world_z_m" for leg in LEGS)
    required_truth.update(f"{leg}_touch_N" for leg in LEGS)
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
    stage2_start = min(value(row, "state_tick_s") for row in stage2)
    stage2_end = max(value(row, "state_tick_s") for row in stage2)
    cruise_start = stage2_start + args.cruise_trim_start_s
    cruise_end = stage2_end - args.cruise_trim_end_s
    cruise = [
        row for row in truth if cruise_start <= value(row, "time_s") < cruise_end
    ]
    if len(cruise) < 100:
        failures.append(f"cruise_samples={len(cruise)}<100")
        cruise = [row for row in truth if stage2_start <= value(row, "time_s") <= stage2_end]

    speed = [value(row, "base_qvel_world_x_mps") for row in cruise]
    base_z = [value(row, "base_pos_world_z_m") for row in cruise]
    foot_z = {
        leg: [value(row, f"{leg}_pos_world_z_m") for row in cruise]
        for leg in LEGS
    }
    roll_deg = []
    pitch_deg = []
    for row in cruise:
        roll, pitch = quat_roll_pitch(row)
        roll_deg.append(abs(math.degrees(roll)))
        pitch_deg.append(abs(math.degrees(pitch)))

    speed_p05 = percentile(speed, 5.0)
    speed_p50 = percentile(speed, 50.0)
    speed_p95 = percentile(speed, 95.0)
    base_z_p01 = percentile(base_z, 1.0)
    base_z_p50 = percentile(base_z, 50.0)
    base_z_p99 = percentile(base_z, 99.0)
    roll_p95 = percentile(roll_deg, 95.0)
    pitch_p95 = percentile(pitch_deg, 95.0)
    foot_p95 = {leg: percentile(foot_z[leg], 95.0) for leg in LEGS}
    foot_p99 = {leg: percentile(foot_z[leg], 99.0) for leg in LEGS}
    all_feet_low_fraction = sum(
        all(value(row, f"{leg}_pos_world_z_m") <= 0.035 for leg in LEGS)
        for row in cruise
    ) / max(1, len(cruise))

    contacts = {
        leg: [value(row, f"{leg}_touch_N") > 5.0 for row in cruise]
        for leg in LEGS
    }
    diagonal_sync = [
        sum(a == b for a, b in zip(contacts[first], contacts[second])) /
        max(1, len(cruise))
        for first, second in DIAGONALS
    ]
    cross_anti = [
        sum(a != b for a, b in zip(contacts[first], contacts[second])) /
        max(1, len(cruise))
        for first, second in CROSS_PAIRS
    ]

    if not math.isfinite(speed_p50) or speed_p50 < args.min_cruise_speed:
        failures.append(f"speed_median={speed_p50:.4f}<{args.min_cruise_speed:.4f}")
    if not math.isfinite(speed_p05) or speed_p05 < args.min_speed_p05:
        failures.append(
            f"speed_p05={speed_p05:.4f}<{args.min_speed_p05:.4f}"
        )
    if not math.isfinite(speed_p95) or speed_p95 > 1.35:
        failures.append(f"speed_p95={speed_p95:.4f}>1.3500")
    if not (0.33 <= base_z_p01 and base_z_p99 <= 0.40):
        failures.append(f"base_z_quantiles=[{base_z_p01:.4f},{base_z_p99:.4f}]")
    if roll_p95 > 8.0 or pitch_p95 > 8.0:
        failures.append(f"body_angle_p95_deg=[{roll_p95:.3f},{pitch_p95:.3f}]>8.000")
    for leg in LEGS:
        if foot_p95[leg] < 0.055:
            failures.append(f"{leg}_swing_clearance_p95={foot_p95[leg]:.4f}<0.0550")
    if all_feet_low_fraction > 0.35:
        failures.append(f"all_feet_low_fraction={all_feet_low_fraction:.4f}>0.3500")
    if min(diagonal_sync) < 0.65:
        failures.append(f"diagonal_contact_sync_min={min(diagonal_sync):.4f}<0.6500")
    if min(cross_anti) < 0.65:
        failures.append(f"cross_diagonal_contact_anti_min={min(cross_anti):.4f}<0.6500")

    truth_time = [value(row, "time_s") for row in truth]
    stop_start = max(stage2_end, truth_time[0])
    stop_rows = [row for row in truth if value(row, "time_s") >= stop_start + 1.5]
    if len(stop_rows) < 100:
        failures.append(f"stop_tail_samples={len(stop_rows)}<100")
    else:
        stop_speed = [abs(value(row, "base_qvel_world_x_mps")) for row in stop_rows]
        stop_roll_rate = [abs(value(row, "base_angvel_body_x_radps")) for row in stop_rows]
        stop_pitch_rate = [abs(value(row, "base_angvel_body_y_radps")) for row in stop_rows]
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
    print(f"body_angle_p95_deg=roll:{roll_p95:.6f},pitch:{pitch_p95:.6f}")
    print("foot_z_p95_m=" + ",".join(f"{leg}:{foot_p95[leg]:.6f}" for leg in LEGS))
    print("foot_z_p99_m=" + ",".join(f"{leg}:{foot_p99[leg]:.6f}" for leg in LEGS))
    print(f"all_feet_low_fraction={all_feet_low_fraction:.6f}")
    print("diagonal_contact_sync=" + ",".join(f"{a}+{b}:{s:.6f}" for (a, b), s in zip(DIAGONALS, diagonal_sync)))
    print(f"cross_diagonal_contact_anti_min={min(cross_anti):.6f}")
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
