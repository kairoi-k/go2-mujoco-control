#!/usr/bin/env python3
"""Strict acceptance check for the wall-clock 3 m/s running-trot profile.

This is deliberately separate from the diagonal-trot sprint checker.  A
running gait must prove both the speed/stop envelope and the gait signature:
diagonal-pair synchrony, a bounded aerial fraction, and nontrivial foot
clearance.  The run's MuJoCo ground-truth log is the source of truth.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import re
from collections import Counter


LEGS = ("FR", "FL", "RR", "RL")
PAIRS = (("FR", "RL"), ("FL", "RR"))
STATUS_KEYS = (
    "controller_status",
    "safety_status",
    "quality_status",
    "analysis_status",
    "ground_truth_status",
    "dynamics_status",
    "completion_status",
)


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not rows[0]:
        raise ValueError(f"empty or invalid CSV: {path}")
    return rows


def num(row: dict[str, str], key: str) -> float:
    return float(row[key])


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = (len(ordered) - 1) * q / 100.0
    lo, hi = math.floor(index), math.ceil(index)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def angles_deg(row: dict[str, str]) -> tuple[float, float]:
    w, x, y, z = (num(row, key) for key in (
        "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"))
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return abs(math.degrees(roll)), abs(math.degrees(pitch))


def longest_good_window(
    rows: list[dict[str, str]],
    min_speed: float,
    max_speed: float,
) -> tuple[float, float, float]:
    best_duration = 0.0
    best_start = float("nan")
    best_end = float("nan")
    current_start = float("nan")
    previous_time = float("nan")
    previous_good = False
    for row in rows:
        time_s = num(row, "time_s")
        speed = num(row, "base_qvel_world_x_mps")
        good = min_speed <= speed <= max_speed
        contiguous = (
            previous_good and math.isfinite(previous_time) and
            0.0 <= time_s - previous_time <= 0.02
        )
        if good and not contiguous:
            current_start = time_s
        if good:
            duration = max(0.0, time_s - current_start)
            if duration > best_duration:
                best_duration = duration
                best_start = current_start
                best_end = time_s
        previous_good = good
        previous_time = time_s
    return best_duration, best_start, best_end


def metadata(path: pathlib.Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=pathlib.Path)
    parser.add_argument("--min-cruise-window", type=float, default=20.0)
    parser.add_argument("--min-speed", type=float, default=2.90)
    parser.add_argument("--max-speed", type=float, default=3.80)
    parser.add_argument("--max-angle-deg", type=float, default=5.0)
    parser.add_argument("--min-aerial-fraction", type=float, default=0.15)
    parser.add_argument("--max-aerial-fraction", type=float, default=0.40)
    parser.add_argument("--min-pair-sync", type=float, default=0.75)
    parser.add_argument("--min-two-contact-fraction", type=float, default=0.30)
    parser.add_argument("--max-three-contact-fraction", type=float, default=0.10)
    parser.add_argument("--min-foot-clearance-m", type=float, default=0.08)
    parser.add_argument("--max-all-feet-low-fraction", type=float, default=0.05)
    parser.add_argument("--max-stop-speed", type=float, default=0.10)
    args = parser.parse_args()
    run_dir = args.run_dir
    failures: list[str] = []

    metadata_path = run_dir / "run_metadata.txt"
    log_path = run_dir / "controller.log"
    data_path = run_dir / "data.csv"
    truth_path = run_dir / "contact_ground_truth.csv"
    required = (metadata_path, log_path, data_path, truth_path)
    if not all(path.exists() for path in required):
        print("validation=FAIL")
        print("failure=missing required run artifact")
        return 2

    meta = metadata(metadata_path)
    for key in STATUS_KEYS:
        if meta.get(key) != "0":
            failures.append(f"{key}={meta.get(key, 'missing')}")
    argv = meta.get("argv", "")
    if "--gait-pattern running-trot" not in argv:
        failures.append("running_trot_pattern_not_in_argv")
    if "--wall-clock-motion" not in argv:
        failures.append("wall_clock_motion_not_in_argv")

    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    cycle_health_count = len(re.findall(r"Trot cycle .* health:", log_text))
    if cycle_health_count < 100:
        failures.append(f"cycle_health_count={cycle_health_count}<100")
    if "High-speed stop: brake complete; entering WBC four-contact hold" not in log_text:
        failures.append("brake_complete_not_recorded")
    if "High-speed stop: WBC four-contact hold complete; finished in WBC stance" not in log_text:
        failures.append("wbc_stop_hold_not_recorded")
    if re.search(r"Trot (?:hard|cycle quality guard|safety rejected)", log_text):
        failures.append("controller_log_contains_rejection")

    try:
        data = read_csv(data_path)
        truth = read_csv(truth_path)
    except (OSError, ValueError) as exc:
        print("validation=FAIL")
        print(f"failure={exc}")
        return 2

    stage2 = [row for row in data if int(float(row["motion_stage"])) == 2]
    stage3 = [row for row in data if int(float(row["motion_stage"])) == 3]
    if not stage2:
        failures.append("no_motion_stage_2_samples")
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1
    if not stage3:
        failures.append("no_motion_stage_3_stop_samples")
        stop_start = max(num(row, "cmd_time_s") for row in stage2)
    else:
        stop_start = min(num(row, "cmd_time_s") for row in stage3)
    stage2_start = min(num(row, "cmd_time_s") for row in stage2)
    stage2_end = max(num(row, "cmd_time_s") for row in stage2)
    cruise_start = stage2_start + 2.0
    cruise_end = min(stage2_end - 2.0, stop_start - 2.0)
    cruise = [
        row for row in truth
        if cruise_start <= num(row, "time_s") <= cruise_end
    ]
    if len(cruise) < 1000:
        failures.append(f"cruise_samples={len(cruise)}<1000")

    speed = [num(row, "base_qvel_world_x_mps") for row in cruise]
    base_z = [num(row, "base_pos_world_z_m") for row in cruise]
    roll = [angles_deg(row)[0] for row in cruise]
    pitch = [angles_deg(row)[1] for row in cruise]
    foot_z = {
        leg: [num(row, f"{leg}_pos_world_z_m") for row in cruise]
        for leg in LEGS
    }
    contacts = {
        leg: [num(row, f"{leg}_touch_N") > 5.0 for row in cruise]
        for leg in LEGS
    }
    contact_counts = Counter(
        sum(contacts[leg][index] for leg in LEGS)
        for index in range(len(cruise))
    )
    n = max(1, len(cruise))
    aerial_fraction = contact_counts[0] / n
    two_contact_fraction = contact_counts[2] / n
    three_contact_fraction = (
        contact_counts[3] + contact_counts[4]
    ) / n
    all_feet_low_fraction = sum(
        all(num(row, f"{leg}_pos_world_z_m") <= 0.035 for leg in LEGS)
        for row in cruise
    ) / n
    pair_sync = {
        f"{first}+{second}": sum(
            a == b for a, b in zip(contacts[first], contacts[second])
        ) / n
        for first, second in PAIRS
    }
    speed_window, window_start, window_end = longest_good_window(
        cruise, args.min_speed, args.max_speed)
    min_clearance = min(
        percentile(foot_z[leg], 95.0) for leg in LEGS
    ) if cruise else float("nan")
    final_row = truth[-1]
    final_speed = abs(num(final_row, "base_qvel_world_x_mps"))
    final_roll, final_pitch = angles_deg(final_row)
    # The controller log records the four-contact-hold completion, but does
    # not timestamp that line in the CSV. Evaluate the final 1.5 s of the
    # recorded run, while still requiring it to be at least 1.5 s after the
    # stage-3 transition. This excludes the physical braking transient and
    # avoids dependence on simulator shutdown latency.
    final_time = num(truth[-1], "time_s")
    settled_start = max(stop_start + 1.5, final_time - 1.5)
    stop_tail = [
        row for row in truth
        if num(row, "time_s") >= settled_start
    ]
    stop_tail_speed_p95 = percentile(
        [abs(num(row, "base_qvel_world_x_mps")) for row in stop_tail], 95.0
    )
    if speed_window < args.min_cruise_window:
        failures.append(
            f"good_window_s={speed_window:.3f}<{args.min_cruise_window:.3f}"
        )
    if not speed or percentile(speed, 50.0) < args.min_speed:
        failures.append(f"speed_median={percentile(speed, 50.0):.4f}")
    if speed and percentile(speed, 95.0) > args.max_speed:
        failures.append(f"speed_p95={percentile(speed, 95.0):.4f}")
    if base_z and (percentile(base_z, 1.0) < 0.33 or percentile(base_z, 99.0) > 0.40):
        failures.append(
            f"base_z_p01_p99={percentile(base_z, 1.0):.4f},{percentile(base_z, 99.0):.4f}"
        )
    if roll and percentile(roll, 95.0) > args.max_angle_deg:
        failures.append(f"roll_p95={percentile(roll, 95.0):.4f}")
    if pitch and percentile(pitch, 95.0) > args.max_angle_deg:
        failures.append(f"pitch_p95={percentile(pitch, 95.0):.4f}")
    if min_clearance < args.min_foot_clearance_m:
        failures.append(f"foot_clearance_p95_min={min_clearance:.4f}")
    if aerial_fraction < args.min_aerial_fraction or aerial_fraction > args.max_aerial_fraction:
        failures.append(f"aerial_fraction={aerial_fraction:.4f}")
    if two_contact_fraction < args.min_two_contact_fraction:
        failures.append(f"two_contact_fraction={two_contact_fraction:.4f}")
    if three_contact_fraction > args.max_three_contact_fraction:
        failures.append(f"three_contact_fraction={three_contact_fraction:.4f}")
    if all_feet_low_fraction > args.max_all_feet_low_fraction:
        failures.append(f"all_feet_low_fraction={all_feet_low_fraction:.4f}")
    if min(pair_sync.values(), default=0.0) < args.min_pair_sync:
        failures.append(f"pair_sync_min={min(pair_sync.values()):.4f}")
    if not stop_tail or stop_tail_speed_p95 > args.max_stop_speed:
        failures.append(f"stop_tail_speed_p95={stop_tail_speed_p95:.4f}")
    if final_speed > args.max_stop_speed:
        failures.append(f"final_speed={final_speed:.4f}")
    if max(final_roll, final_pitch) > args.max_angle_deg:
        failures.append(f"final_angle_deg={max(final_roll, final_pitch):.4f}")

    print(f"stage2_window_s={stage2_start:.3f}..{stage2_end:.3f}")
    print(f"stop_start_s={stop_start:.3f}")
    print(f"cruise_window_s={cruise_start:.3f}..{cruise_end:.3f}")
    print(f"cycle_health_count={cycle_health_count}")
    print(f"speed_p05_mps={percentile(speed, 5.0):.6f}")
    print(f"speed_median_mps={percentile(speed, 50.0):.6f}")
    print(f"speed_p95_mps={percentile(speed, 95.0):.6f}")
    print(f"good_speed_window_s={speed_window:.6f} ({window_start:.3f}..{window_end:.3f})")
    print(f"base_z_p01_p99_m={percentile(base_z, 1.0):.6f},{percentile(base_z, 99.0):.6f}")
    print(f"body_angle_p95_deg=roll:{percentile(roll, 95.0):.6f},pitch:{percentile(pitch, 95.0):.6f}")
    print("foot_z_p95_m=" + ",".join(
        f"{leg}:{percentile(foot_z[leg], 95.0):.6f}" for leg in LEGS
    ))
    print(f"aerial_fraction={aerial_fraction:.6f}")
    print(f"two_contact_fraction={two_contact_fraction:.6f}")
    print(f"three_contact_fraction={three_contact_fraction:.6f}")
    print(f"all_feet_low_fraction={all_feet_low_fraction:.6f}")
    print("pair_sync=" + ",".join(f"{key}:{value:.6f}" for key, value in pair_sync.items()))
    print(f"stop_tail_start_s={settled_start:.3f}")
    print(f"stop_tail_samples={len(stop_tail)}")
    print(f"stop_tail_speed_p95_mps={stop_tail_speed_p95:.6f}")
    print(f"final_speed_mps={final_speed:.6f}")
    print(f"final_angle_deg={max(final_roll, final_pitch):.6f}")
    if failures:
        print("validation=FAIL")
        for failure in failures:
            print("failure=" + failure)
        return 1
    print("validation=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
