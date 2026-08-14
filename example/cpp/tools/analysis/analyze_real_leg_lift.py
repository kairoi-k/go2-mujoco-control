#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LEGS = ("FR", "FL", "RR", "RL")


def load_csv(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))


def mean(data, column, mask):
    return float(np.mean(data[column][mask]))


def analyze_run(path):
    data = load_csv(path)
    baseline = (data["motion_stage"] == 3) & (data["cmd_time_s"] >= 5.7)
    hold = (data["motion_stage"] == 5) & (data["cmd_time_s"] >= 7.2)
    final_stage = np.max(data["motion_stage"])
    final = (data["motion_stage"] == final_stage) & (
        data["cmd_time_s"] >= np.max(data["cmd_time_s"]) - 0.8
    )

    fr_force_baseline = mean(data, "FR_foot_force", baseline)
    fr_force_hold = mean(data, "FR_foot_force", hold)
    target_lift = (
        mean(data, "FR_foot_z_target_m", hold)
        - mean(data, "FR_foot_z_target_m", baseline)
    )
    actual_lift = (
        mean(data, "FR_foot_z_state_m", hold)
        - mean(data, "FR_foot_z_state_m", baseline)
    )

    return {
        "path": path,
        "data": data,
        "command_lift_m": float(np.max(data["FR_foot_lift_target_m"])),
        "target_lift_m": target_lift,
        "actual_lift_m": actual_lift,
        "fr_force_baseline": fr_force_baseline,
        "fr_force_hold": fr_force_hold,
        "fr_unload_percent": 100.0
        * (1.0 - fr_force_hold / fr_force_baseline),
        "fr_force_hold_min": float(np.min(data["FR_foot_force"][hold])),
        "max_abs_roll_deg": math.degrees(
            float(np.max(np.abs(data["imu_roll_rad"][hold])))
        ),
        "max_abs_pitch_deg": math.degrees(
            float(np.max(np.abs(data["imu_pitch_rad"][hold])))
        ),
        "fr_force_final": mean(data, "FR_foot_force", final),
        "hold_forces": {
            leg: mean(data, f"{leg}_foot_force", hold) for leg in LEGS
        },
    }


def write_summary(results, path):
    fields = [
        "run",
        "command_lift_m",
        "target_lift_m",
        "actual_lift_m",
        "fr_force_baseline",
        "fr_force_hold",
        "fr_unload_percent",
        "fr_force_hold_min",
        "max_abs_roll_deg",
        "max_abs_pitch_deg",
        "fr_force_final",
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
        label = f"{result['command_lift_m'] * 100:.0f} cm"
        baseline_mask = (
            (data["motion_stage"] == 3) & (data["cmd_time_s"] >= 5.7)
        )
        baseline_target_z = float(
            np.mean(data["FR_foot_z_target_m"][baseline_mask])
        )
        baseline_state_z = float(
            np.mean(data["FR_foot_z_state_m"][baseline_mask])
        )

        axes[0, 0].plot(
            data["cmd_time_s"], data["FR_foot_force"], label=label
        )
        axes[0, 1].plot(
            data["cmd_time_s"],
            1000.0 * (data["FR_foot_z_target_m"] - baseline_target_z),
            linestyle="--",
            label=f"{label} target",
        )
        axes[0, 1].plot(
            data["cmd_time_s"],
            1000.0 * (data["FR_foot_z_state_m"] - baseline_state_z),
            label=f"{label} state",
        )
        axes[1, 0].plot(
            data["cmd_time_s"],
            np.degrees(data["imu_roll_rad"]),
            label=f"{label} roll",
        )
        axes[1, 0].plot(
            data["cmd_time_s"],
            np.degrees(data["imu_pitch_rad"]),
            linestyle="--",
            label=f"{label} pitch",
        )

    labels = [result["path"].parent.name for result in results]
    x = np.arange(len(results))
    width = 0.2
    for index, leg in enumerate(LEGS):
        axes[1, 1].bar(
            x + (index - 1.5) * width,
            [result["hold_forces"][leg] for result in results],
            width,
            label=leg,
        )

    axes[0, 0].set_title("Front-right foot force")
    axes[0, 0].set_ylabel("LowState foot force")
    axes[0, 1].set_title("Front-right foot lift relative to pre-lift state")
    axes[0, 1].set_ylabel("millimeters")
    axes[1, 0].set_title("Body attitude")
    axes[1, 0].set_ylabel("degrees")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_title("Mean foot force during lift hold")
    axes[1, 1].set_xticks(x, labels)

    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.set_xlim(5.0, 10.5)
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend()

    figure.savefig(path, dpi=160)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_dir", type=Path)
    args = parser.parse_args()

    paths = sorted(args.experiment_dir.glob("lift_*/data.csv"))
    if not paths:
        raise SystemExit("No lift_*/data.csv files found")

    results = [analyze_run(path) for path in paths]
    write_summary(results, args.experiment_dir / "summary.csv")
    plot_results(results, args.experiment_dir / "comparison.png")

    for result in results:
        print(
            result["path"].parent.name,
            f"actual lift={result['actual_lift_m'] * 1000:.1f} mm",
            f"FR unload={result['fr_unload_percent']:.1f}%",
            f"FR hold force={result['fr_force_hold']:.1f}",
            f"max roll/pitch=({result['max_abs_roll_deg']:.2f},"
            f" {result['max_abs_pitch_deg']:.2f}) deg",
        )


if __name__ == "__main__":
    main()
