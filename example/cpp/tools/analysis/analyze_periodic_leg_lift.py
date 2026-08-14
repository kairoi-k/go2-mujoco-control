#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


JOINTS = tuple(
    f"{leg}_{joint}"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("hip", "thigh", "calf")
)


def load_csv(path):
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    if "cycle_index" not in data.dtype.names:
        raise SystemExit(
            "CSV has no cycle_index column; run the periodic-capable controller"
        )
    return data


def require(mask, label):
    if not np.any(mask):
        raise SystemExit(f"CSV has no samples for {label}")
    return mask


def max_abs(data, columns, mask):
    return max(float(np.max(np.abs(data[column][mask]))) for column in columns)


def analyze(path, leg):
    foot_clearance_col = f"{leg}_foot_ground_clearance_m"
    foot_force_col = f"{leg}_foot_force"
    data = load_csv(path)
    startup = require(data["motion_stage"] == 1, "startup stand settle")
    startup_tail = startup & (
        data["cmd_time_s"] >= np.max(data["cmd_time_s"][startup]) - 0.25
    )
    initial_x = float(np.mean(data["base_world_x_m"][startup_tail]))
    initial_y = float(np.mean(data["base_world_y_m"][startup_tail]))
    initial_yaw = float(np.mean(data["imu_yaw_rad"][startup_tail]))
    first_support_x = None
    first_support_y = None

    cycle_numbers = sorted(
        {
            int(value)
            for value in data["cycle_index"]
            if value >= 1 and np.isfinite(value)
        }
    )
    results = []
    for cycle in cycle_numbers:
        cycle_mask = data["cycle_index"] == cycle
        baseline = require(
            cycle_mask
            & (
                (data["motion_stage"] == 3)
                | (data["motion_stage"] == 11)
            ),
            f"cycle {cycle} shifted baseline",
        )
        baseline_tail = baseline & (
            data["cmd_time_s"]
            >= np.max(data["cmd_time_s"][baseline]) - 0.25
        )
        hold = require(
            cycle_mask & (data["motion_stage"] == 5),
            f"cycle {cycle} lift hold",
        )
        motion = require(
            cycle_mask
            & (data["motion_stage"] >= 2)
            & (data["motion_stage"] <= 11)
            & (data["motion_stage"] != 10),
            f"cycle {cycle} motion",
        )

        baseline_clearance = float(
            np.mean(data[foot_clearance_col][baseline])
        )
        hold_clearance = float(
            np.mean(data[foot_clearance_col][hold])
        )
        support_x = float(np.mean(data["base_world_x_m"][baseline_tail]))
        support_y = float(np.mean(data["base_world_y_m"][baseline_tail]))
        if first_support_x is None:
            first_support_x = support_x
            first_support_y = support_y

        neutral = cycle_mask & (data["motion_stage"] == 9)
        final_x = float("nan")
        final_y = float("nan")
        final_yaw = float("nan")
        if np.any(neutral):
            final_x = float(np.mean(data["base_world_x_m"][neutral]))
            final_y = float(np.mean(data["base_world_y_m"][neutral]))
            final_yaw = float(np.mean(data["imu_yaw_rad"][neutral]))

        results.append(
            {
                "cycle": cycle,
                "lift_hold_force_mean": float(
                    np.mean(data[foot_force_col][hold])
                ),
                "lift_hold_force_max": float(
                    np.max(data[foot_force_col][hold])
                ),
                "lift_zero_force_fraction": float(
                    np.mean(data[foot_force_col][hold] == 0.0)
                ),
                "lift_clearance_hold_mm": 1000.0 * hold_clearance,
                "lift_clearance_increase_mm": 1000.0
                * (hold_clearance - baseline_clearance),
                "max_abs_roll_deg": math.degrees(
                    float(np.max(np.abs(data["imu_roll_rad"][motion])))
                ),
                "max_abs_pitch_deg": math.degrees(
                    float(np.max(np.abs(data["imu_pitch_rad"][motion])))
                ),
                "support_xy_drift_mm": 1000.0
                * math.hypot(
                    support_x - first_support_x,
                    support_y - first_support_y,
                ),
                "final_xy_drift_mm": (
                    1000.0
                    * math.hypot(final_x - initial_x, final_y - initial_y)
                    if np.isfinite(final_x)
                    else float("nan")
                ),
                "final_yaw_drift_deg": (
                    math.degrees(abs(final_yaw - initial_yaw))
                    if np.isfinite(final_yaw)
                    else float("nan")
                ),
                "max_abs_q_error_rad": max_abs(
                    data,
                    [f"{joint}_q_error" for joint in JOINTS],
                    motion,
                ),
                "max_abs_dq_rad_s": max_abs(
                    data,
                    [f"{joint}_dq_state" for joint in JOINTS],
                    motion,
                ),
                "max_abs_tau_est": max_abs(
                    data,
                    [f"{joint}_tau_est" for joint in JOINTS],
                    motion,
                ),
            }
        )
    return data, results


def write_summary(results, path):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)


def plot(data, results, path, leg):
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    time = data["cmd_time_s"]
    foot_clearance_col = f"{leg}_foot_ground_clearance_m"
    foot_force_col = f"{leg}_foot_force"

    axes[0, 0].plot(
        time,
        1000.0 * data[foot_clearance_col],
        color="#2f6f9f",
        label=f"{leg} clearance",
    )
    axes[0, 0].set_ylabel("millimeters")
    axes[0, 0].set_title(f"{leg} foot world clearance")

    axes[0, 1].plot(
        time, data[foot_force_col], color="#d98b2b", label=f"{leg} force"
    )
    axes[0, 1].set_title(f"{leg} foot force")

    axes[1, 0].plot(
        time,
        np.degrees(data["imu_roll_rad"]),
        color="#6f8f3d",
        label="roll",
    )
    axes[1, 0].plot(
        time,
        np.degrees(data["imu_pitch_rad"]),
        color="#9a5b8e",
        label="pitch",
    )
    axes[1, 0].set_ylabel("degrees")
    axes[1, 0].set_title("Body attitude")

    cycles = [result["cycle"] for result in results]
    axes[1, 1].plot(
        cycles,
        [result["support_xy_drift_mm"] for result in results],
        marker="o",
        label="shifted-support drift (mm)",
    )
    axes[1, 1].set_xticks(cycles)
    axes[1, 1].set_xlabel("cycle")
    axes[1, 1].set_title("Accumulated support drift")

    for axis in axes.flat:
        axis.grid(True, color="#d7dce0", alpha=0.7)
        axis.legend()
    axes[1, 0].set_xlabel("time (s)")
    figure.suptitle(f"Go2 periodic {leg} foot lift", fontsize=14)
    figure.savefig(path, dpi=160, facecolor="white")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--leg", choices=("FR", "FL", "RR", "RL"), default="FR")
    args = parser.parse_args()

    output_dir = args.output_dir or args.csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    data, results = analyze(args.csv_path, args.leg)
    if not results:
        raise SystemExit("No completed periodic lift cycles found")

    write_summary(results, output_dir / "periodic_summary.csv")
    plot(data, results, output_dir / "periodic_overview.png", args.leg)
    for result in results:
        print(
            f"cycle {result['cycle']}: "
            f"{args.leg} clearance={result['lift_clearance_hold_mm']:.1f} mm, "
            f"force={result['lift_hold_force_mean']:.1f}, "
            f"roll/pitch={result['max_abs_roll_deg']:.2f}/"
            f"{result['max_abs_pitch_deg']:.2f} deg, "
            f"support drift={result['support_xy_drift_mm']:.1f} mm"
        )
    final = results[-1]
    print(
        f"final return drift={final['final_xy_drift_mm']:.1f} mm, "
        f"yaw drift={final['final_yaw_drift_deg']:.2f} deg"
    )


if __name__ == "__main__":
    main()
