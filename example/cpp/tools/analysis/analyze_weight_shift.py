#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LEGS = ("FR", "FL", "RR", "RL")


def load_csv(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return np.atleast_1d(data)


def mean(data, column, mask):
    return float(np.mean(data[column][mask]))


def analyze_run(path):
    data = load_csv(path)
    baseline = (data["motion_stage"] == 1) & (data["cmd_time_s"] >= 3.2)
    hold = (data["motion_stage"] == 3) & (data["cmd_time_s"] >= 6.0)

    baseline_force = {
        leg: mean(data, f"{leg}_foot_force", baseline) for leg in LEGS
    }
    hold_force = {
        leg: mean(data, f"{leg}_foot_force", hold) for leg in LEGS
    }

    estimated_shift = []
    for leg in LEGS:
        foot_dx = (
            mean(data, f"{leg}_foot_x_state_m", hold)
            - mean(data, f"{leg}_foot_x_state_m", baseline)
        )
        foot_dy = (
            mean(data, f"{leg}_foot_y_state_m", hold)
            - mean(data, f"{leg}_foot_y_state_m", baseline)
        )
        estimated_shift.append((-foot_dx, -foot_dy))

    baseline_roll = mean(data, "imu_roll_rad", baseline)
    baseline_pitch = mean(data, "imu_pitch_rad", baseline)
    hold_roll = mean(data, "imu_roll_rad", hold)
    hold_pitch = mean(data, "imu_pitch_rad", hold)

    return {
        "path": path,
        "data": data,
        "command_shift_x_m": float(data["body_shift_x_target_m"][-1]),
        "command_shift_y_m": float(data["body_shift_y_target_m"][-1]),
        "estimated_shift_x_m": float(np.mean([value[0] for value in estimated_shift])),
        "estimated_shift_y_m": float(np.mean([value[1] for value in estimated_shift])),
        "fr_force_baseline": baseline_force["FR"],
        "fr_force_hold": hold_force["FR"],
        "fr_unload_percent": 100.0
        * (1.0 - hold_force["FR"] / baseline_force["FR"]),
        "roll_delta_deg": math.degrees(hold_roll - baseline_roll),
        "pitch_delta_deg": math.degrees(hold_pitch - baseline_pitch),
        "max_abs_roll_deg": math.degrees(
            float(np.max(np.abs(data["imu_roll_rad"][hold])))
        ),
        "max_abs_pitch_deg": math.degrees(
            float(np.max(np.abs(data["imu_pitch_rad"][hold])))
        ),
        "hold_force": hold_force,
    }


def write_summary(results, path):
    fields = [
        "run",
        "command_shift_x_m",
        "command_shift_y_m",
        "estimated_shift_x_m",
        "estimated_shift_y_m",
        "fr_force_baseline",
        "fr_force_hold",
        "fr_unload_percent",
        "roll_delta_deg",
        "pitch_delta_deg",
        "max_abs_roll_deg",
        "max_abs_pitch_deg",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            row = {field: result.get(field) for field in fields}
            row["run"] = result["path"].parent.name
            writer.writerow(row)


def plot_results(results, path):
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)

    for result in results:
        data = result["data"]
        label = (
            f"x={result['command_shift_x_m']:.3f}, "
            f"y={result['command_shift_y_m']:.3f} m"
        )
        axes[0, 0].plot(data["cmd_time_s"], data["FR_foot_force"], label=label)
        axes[0, 1].plot(
            data["cmd_time_s"], np.degrees(data["imu_roll_rad"]), label=label
        )
        axes[1, 0].plot(
            data["cmd_time_s"], np.degrees(data["imu_pitch_rad"]), label=label
        )

    run_labels = [result["path"].parent.name for result in results]
    x = np.arange(len(results))
    width = 0.2
    for index, leg in enumerate(LEGS):
        axes[1, 1].bar(
            x + (index - 1.5) * width,
            [result["hold_force"][leg] for result in results],
            width,
            label=leg,
        )

    axes[0, 0].set_title("Front-right foot force")
    axes[0, 0].set_ylabel("LowState foot force")
    axes[0, 1].set_title("Body roll")
    axes[0, 1].set_ylabel("degrees")
    axes[1, 0].set_title("Body pitch")
    axes[1, 0].set_ylabel("degrees")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_title("Mean foot force during final hold")
    axes[1, 1].set_xticks(x, run_labels, rotation=15, ha="right")

    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.set_xlim(2.5, 9.0)
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    axes[0, 0].legend()
    axes[0, 1].legend()
    axes[1, 0].legend()
    axes[1, 1].legend()

    figure.savefig(path, dpi=160)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_dir", type=Path)
    args = parser.parse_args()

    csv_paths = sorted(args.experiment_dir.glob("*/data.csv"))
    if not csv_paths:
        raise SystemExit("No */data.csv files found")

    results = [analyze_run(path) for path in csv_paths]
    write_summary(results, args.experiment_dir / "summary.csv")
    plot_results(results, args.experiment_dir / "comparison.png")

    for result in results:
        print(
            result["path"].parent.name,
            f"FR unload={result['fr_unload_percent']:.1f}%",
            f"estimated shift=({result['estimated_shift_x_m']:.3f},"
            f" {result['estimated_shift_y_m']:.3f}) m",
            f"max roll/pitch=({result['max_abs_roll_deg']:.2f},"
            f" {result['max_abs_pitch_deg']:.2f}) deg",
        )


if __name__ == "__main__":
    main()
