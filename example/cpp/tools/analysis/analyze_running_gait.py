#!/usr/bin/env python3
"""Strict acceptance check for the 1 m/s running-trot profile.

The cruise window is measured from MuJoCo ground truth before the scheduled
terminal emergency-stop event.  Lifecycle and stop evidence comes from the
controller log and run metadata.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
import sys


LEGS = ("FR", "FL", "RR", "RL")
DIAGONALS = (("FR", "RL"), ("FL", "RR"))
CROSS_PAIRS = (("FR", "FL"), ("FR", "RR"), ("RL", "FL"), ("RL", "RR"))


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * q / 100.0
    lo, hi = math.floor(index), math.ceil(index)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or not rows[0]:
        raise ValueError(f"empty or invalid CSV: {path}")
    return rows


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def roll_pitch(row: dict[str, str]) -> tuple[float, float]:
    w, x, y, z = (f(row, key) for key in ("base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--min-cruise-speed", type=float, default=1.0)
    parser.add_argument("--trim-start-s", type=float, default=1.0)
    parser.add_argument("--trim-end-s", type=float, default=0.5)
    args = parser.parse_args()
    run_dir = args.run_dir
    failures: list[str] = []

    metadata_path = run_dir / "run_metadata.txt"
    log_path = run_dir / "controller.log"
    data_path = run_dir / "data.csv"
    truth_path = run_dir / "contact_ground_truth.csv"
    if not all(path.exists() for path in (metadata_path, log_path, data_path, truth_path)):
        print("validation=FAIL")
        print("failure=missing required run artifact")
        return 2

    metadata = {}
    for line in metadata_path.read_text().splitlines():
        if "=" in line:
            key, item = line.split("=", 1)
            metadata[key] = item
    for key in ("controller_status", "safety_status", "quality_status", "analysis_status", "ground_truth_status", "dynamics_status", "completion_status"):
        if metadata.get(key) != "0":
            failures.append(f"{key}={metadata.get(key, 'missing')}")

    log_text = log_path.read_text(errors="replace")
    cycle_count = len(re.findall(r"Trot cycle .* health:", log_text))
    if cycle_count < 25:
        failures.append(f"cycle_health_count={cycle_count}<25")
    if "Emergency stop latched; holding WBC stance" not in log_text:
        failures.append("emergency_stop_latch_not_recorded")
    if "Emergency stop hold complete; ending in WBC stance" not in log_text:
        failures.append("emergency_stop_hold_not_recorded")
    if re.search(r"Trot (?:hard|cycle quality guard|safety rejected)", log_text):
        failures.append("controller_log_contains_rejection")

    try:
        data = read_csv(data_path)
        truth = read_csv(truth_path)
    except (OSError, ValueError) as exc:
        print("validation=FAIL")
        print(f"failure={exc}")
        return 2

    required_data = {"state_tick_s", "motion_stage", "event_active", "event_type"}
    required_truth = {
        "time_s", "base_qvel_world_x_mps", "base_pos_world_z_m", "base_quat_w",
        "base_quat_x", "base_quat_y", "base_quat_z",
    }
    required_truth.update(f"{leg}_pos_world_z_m" for leg in LEGS)
    required_truth.update(f"{leg}_touch_N" for leg in LEGS)
    missing = sorted(required_data.difference(data[0]))
    missing += sorted(required_truth.difference(truth[0]))
    if missing:
        failures.append("missing=" + ",".join(missing))
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
    stage2_start = min(f(row, "state_tick_s") for row in stage2)
    stage2_end = max(f(row, "state_tick_s") for row in stage2)

    event_rows = []
    for row in data:
        event_type = row.get("event_type", "").strip().lower()
        active = float(row.get("event_active", "0") or 0.0)
        if active > 0.5 or event_type not in ("", "0", "none", "normal"):
            event_rows.append(row)
    if not event_rows:
        failures.append("emergency_event_not_visible_in_data")
        event_time = stage2_end
    else:
        event_time = min(f(row, "state_tick_s") for row in event_rows)
    cruise_start = stage2_start + args.trim_start_s
    cruise_end = event_time - args.trim_end_s
    cruise = [row for row in truth if cruise_start <= f(row, "time_s") < cruise_end]
    if len(cruise) < 100:
        failures.append(f"cruise_samples={len(cruise)}<100")
        cruise = [row for row in truth if stage2_start <= f(row, "time_s") < event_time]

    speed = [f(row, "base_qvel_world_x_mps") for row in cruise]
    base_z = [f(row, "base_pos_world_z_m") for row in cruise]
    roll_deg, pitch_deg = [], []
    foot_z = {leg: [f(row, f"{leg}_pos_world_z_m") for row in cruise] for leg in LEGS}
    contacts = {leg: [f(row, f"{leg}_touch_N") > 5.0 for row in cruise] for leg in LEGS}
    for row in cruise:
        roll, pitch = roll_pitch(row)
        roll_deg.append(abs(math.degrees(roll)))
        pitch_deg.append(abs(math.degrees(pitch)))
    speed_p05, speed_p50, speed_p95 = (percentile(speed, q) for q in (5.0, 50.0, 95.0))
    z_p01, z_p50, z_p99 = (percentile(base_z, q) for q in (1.0, 50.0, 99.0))
    roll_p95, pitch_p95 = percentile(roll_deg, 95.0), percentile(pitch_deg, 95.0)
    foot_p95 = {leg: percentile(foot_z[leg], 95.0) for leg in LEGS}
    low_fraction = sum(all(f(row, f"{leg}_pos_world_z_m") <= 0.035 for leg in LEGS) for row in cruise) / max(1, len(cruise))
    aerial_fraction = sum(not any(f(row, f"{leg}_touch_N") > 5.0 for leg in LEGS) for row in cruise) / max(1, len(cruise))
    diagonal_sync = [sum(a == b for a, b in zip(contacts[first], contacts[second])) / max(1, len(cruise)) for first, second in DIAGONALS]
    cross_anti = [sum(a != b for a, b in zip(contacts[first], contacts[second])) / max(1, len(cruise)) for first, second in CROSS_PAIRS]

    if not math.isfinite(speed_p50) or speed_p50 < args.min_cruise_speed:
        failures.append(f"speed_median={speed_p50:.4f}<{args.min_cruise_speed:.4f}")
    if not math.isfinite(speed_p05) or speed_p05 < 0.85:
        failures.append(f"speed_p05={speed_p05:.4f}<0.8500")
    if not math.isfinite(speed_p95) or speed_p95 > 1.35:
        failures.append(f"speed_p95={speed_p95:.4f}>1.3500")
    if not (0.33 <= z_p01 and z_p99 <= 0.40):
        failures.append(f"base_z_quantiles=[{z_p01:.4f},{z_p99:.4f}]")
    if roll_p95 > 8.0 or pitch_p95 > 8.0:
        failures.append(f"body_angle_p95_deg=[{roll_p95:.3f},{pitch_p95:.3f}]>8.000")
    for leg in LEGS:
        if foot_p95[leg] < 0.055:
            failures.append(f"{leg}_swing_clearance_p95={foot_p95[leg]:.4f}<0.0550")
    if low_fraction > 0.35:
        failures.append(f"all_feet_low_fraction={low_fraction:.4f}>0.3500")
    if aerial_fraction < 0.02:
        failures.append(f"aerial_fraction={aerial_fraction:.4f}<0.0200")
    if min(diagonal_sync) < 0.75:
        failures.append(f"diagonal_contact_sync_min={min(diagonal_sync):.4f}<0.7500")
    if min(cross_anti) < 0.70:
        failures.append(f"cross_diagonal_contact_anti_min={min(cross_anti):.4f}<0.7000")

    # The event is latched at the start of the terminal brake.  Evaluate the
    # settled tail after the documented 1.5 s post-hold, not during braking.
    stop_rows = [row for row in truth if f(row, "time_s") >= event_time + 1.5]
    if len(stop_rows) < 100:
        failures.append(f"stop_tail_samples={len(stop_rows)}<100")
    else:
        stop_speed = [abs(f(row, "base_qvel_world_x_mps")) for row in stop_rows]
        if percentile(stop_speed, 95.0) > 0.10:
            failures.append(f"stop_speed_p95={percentile(stop_speed, 95.0):.4f}>0.1000")

    print(f"stage2_window_s={stage2_start:.3f}..{stage2_end:.3f}")
    print(f"event_onset_s={event_time:.3f}")
    print(f"cruise_window_s={cruise_start:.3f}..{cruise_end:.3f}")
    print(f"cycle_health_count={cycle_count}")
    print(f"speed_p05_mps={speed_p05:.6f}")
    print(f"speed_median_mps={speed_p50:.6f}")
    print(f"speed_p95_mps={speed_p95:.6f}")
    print(f"base_z_p01_m={z_p01:.6f}")
    print(f"base_z_median_m={z_p50:.6f}")
    print(f"base_z_p99_m={z_p99:.6f}")
    print(f"body_angle_p95_deg=roll:{roll_p95:.6f},pitch:{pitch_p95:.6f}")
    print("foot_z_p95_m=" + ",".join(f"{leg}:{foot_p95[leg]:.6f}" for leg in LEGS))
    print(f"all_feet_low_fraction={low_fraction:.6f}")
    print(f"aerial_fraction={aerial_fraction:.6f}")
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
