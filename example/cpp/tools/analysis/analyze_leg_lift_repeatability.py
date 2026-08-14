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
    return np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))


def analyze_run(path):
    data = load_csv(path)
    baseline = (data["motion_stage"] == 3) & (data["cmd_time_s"] >= 5.7)
    hold = (data["motion_stage"] == 5) & (data["cmd_time_s"] >= 7.2)
    motion = (data["motion_stage"] >= 2) & (data["motion_stage"] <= 6)

    baseline_z = float(np.mean(data["FR_foot_z_state_m"][baseline]))
    hold_z = float(np.mean(data["FR_foot_z_state_m"][hold]))
    fr_hold_force = data["FR_foot_force"][hold]
    clearance_before_mm = (
        1000.0 * float(np.mean(data["FR_foot_ground_clearance_m"][baseline]))
    )
    clearance_hold_mm = (
        1000.0 * float(np.mean(data["FR_foot_ground_clearance_m"][hold]))
    )

    return {
        "run": path.parent.name,
        "path": path,
        "data": data,
        "fr_force_before_lift": float(
            np.mean(data["FR_foot_force"][baseline])
        ),
        "fr_force_hold_mean": float(np.mean(fr_hold_force)),
        "fr_force_hold_max": float(np.max(fr_hold_force)),
        "fr_zero_force_fraction": float(np.mean(fr_hold_force == 0.0)),
        "actual_lift_mm": 1000.0 * (hold_z - baseline_z),
        "fr_clearance_before_mm": clearance_before_mm,
        "fr_clearance_hold_mm": clearance_hold_mm,
        "fr_clearance_increase_mm": clearance_hold_mm - clearance_before_mm,
        "max_abs_roll_deg": math.degrees(
            float(np.max(np.abs(data["imu_roll_rad"][hold])))
        ),
        "max_abs_pitch_deg": math.degrees(
            float(np.max(np.abs(data["imu_pitch_rad"][hold])))
        ),
        "max_abs_q_error_rad": max(
            float(np.max(np.abs(data[f"{joint}_q_error"][motion])))
            for joint in JOINTS
        ),
        "max_abs_dq_rad_s": max(
            float(np.max(np.abs(data[f"{joint}_dq_state"][motion])))
            for joint in JOINTS
        ),
        "max_abs_tau_est": max(
            float(np.max(np.abs(data[f"{joint}_tau_est"][motion])))
            for joint in JOINTS
        ),
    }


def write_summary(results, path):
    fields = [key for key in results[0] if key not in ("path", "data")]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({field: result[field] for field in fields})


def plot_results(results, path):
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.6), constrained_layout=True)
    colors = ("#2f6f9f", "#d98b2b", "#6f8f3d")

    for index, result in enumerate(results):
        data = result["data"]
        axes[0].plot(
            data["cmd_time_s"],
            data["FR_foot_force"],
            color=colors[index % len(colors)],
            label=result["run"],
        )
    axes[0].axhline(0.0, color="#444444", linewidth=1.0)
    axes[0].set_xlim(5.0, 10.5)
    axes[0].set_title("Front-right foot force")
    axes[0].set_xlabel("time (s)")
    axes[0].set_ylabel("LowState foot force")
    axes[0].legend()

    labels = [result["run"] for result in results]
    x = np.arange(len(results))
    axes[1].bar(
        x - 0.18,
        [result["fr_clearance_hold_mm"] for result in results],
        0.36,
        color="#2f6f9f",
        label="ground clearance (mm)",
    )
    axes[1].bar(
        x + 0.18,
        [result["fr_force_hold_mean"] for result in results],
        0.36,
        color="#d98b2b",
        label="mean FR force",
    )
    axes[1].set_xticks(x, labels)
    axes[1].set_title("Lift repeatability")
    axes[1].legend()

    for axis in axes:
        axis.grid(True, color="#d7dce0", alpha=0.7)
        axis.set_facecolor("#fbfcfd")
    figure.suptitle(
        "Go2 front-right foot lift: three independent repetitions",
        fontsize=14,
    )
    figure.savefig(path, dpi=160, facecolor="white")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_dir", type=Path)
    args = parser.parse_args()
    paths = sorted(args.experiment_dir.glob("repeat_*/data.csv"))
    if not paths:
        raise SystemExit("No repeat_*/data.csv files found")

    results = [analyze_run(path) for path in paths]
    write_summary(results, args.experiment_dir / "summary.csv")
    plot_results(results, args.experiment_dir / "repeatability.png")

    for result in results:
        print(
            result["run"],
            f"FR hold={result['fr_force_hold_mean']:.1f}",
            f"zero fraction={result['fr_zero_force_fraction']:.1%}",
            f"lift={result['actual_lift_mm']:.1f} mm",
            f"clearance={result['fr_clearance_hold_mm']:.1f} mm",
            f"roll/pitch={result['max_abs_roll_deg']:.2f}/"
            f"{result['max_abs_pitch_deg']:.2f} deg",
        )


if __name__ == "__main__":
    main()
