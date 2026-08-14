#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LEGS = ("FR", "FL", "RR", "RL")
ATTITUDE_LIMIT_DEG = 2.0
RECOMMENDED_SHIFT_CM = (7.5, 6.0)


def load_csv(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))


def analyze_run(path):
    data = load_csv(path)
    baseline = (data["motion_stage"] == 1) & (data["cmd_time_s"] >= 3.2)
    hold = (data["motion_stage"] == 3) & (data["cmd_time_s"] >= 5.7)
    if not np.any(baseline) or not np.any(hold):
        raise ValueError(f"Missing baseline or hold samples in {path}")

    forces = {
        leg: float(np.mean(data[f"{leg}_foot_force"][hold])) for leg in LEGS
    }
    fr_baseline = float(np.mean(data["FR_foot_force"][baseline]))
    max_roll = float(np.degrees(np.max(np.abs(data["imu_roll_rad"][hold]))))
    max_pitch = float(
        np.degrees(np.max(np.abs(data["imu_pitch_rad"][hold])))
    )
    shift_x_cm = -100.0 * float(data["body_shift_x_target_m"][-1])
    shift_y_cm = 100.0 * float(data["body_shift_y_target_m"][-1])

    return {
        "run": path.parent.name,
        "shift_back_cm": shift_x_cm,
        "shift_left_cm": shift_y_cm,
        "fr_force_baseline": fr_baseline,
        "fr_force_hold": forces["FR"],
        "fr_unload_percent": 100.0 * (1.0 - forces["FR"] / fr_baseline),
        "max_abs_roll_deg": max_roll,
        "max_abs_pitch_deg": max_pitch,
        "max_attitude_deg": max(max_roll, max_pitch),
        "safe_under_2deg": max(max_roll, max_pitch) <= ATTITUDE_LIMIT_DEG,
        **{f"{leg.lower()}_force_hold": forces[leg] for leg in LEGS},
    }


def write_summary(results, path):
    fields = list(results[0].keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)


def is_recommended(result):
    return (
        abs(result["shift_back_cm"] - RECOMMENDED_SHIFT_CM[0]) < 1e-6
        and abs(result["shift_left_cm"] - RECOMMENDED_SHIFT_CM[1]) < 1e-6
    )


def plot_results(results, path):
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    safe = [result for result in results if result["safe_under_2deg"]]
    unsafe = [result for result in results if not result["safe_under_2deg"]]
    color_min = min(result["fr_force_hold"] for result in results)
    color_max = max(result["fr_force_hold"] for result in results)

    safe_plot = axes[0].scatter(
        [result["shift_back_cm"] for result in safe],
        [result["shift_left_cm"] for result in safe],
        c=[result["fr_force_hold"] for result in safe],
        cmap="Blues_r",
        vmin=color_min,
        vmax=color_max,
        s=90,
        edgecolors="#18324a",
        linewidths=0.8,
        label="max |roll/pitch| ≤ 2°",
    )
    if unsafe:
        axes[0].scatter(
            [result["shift_back_cm"] for result in unsafe],
            [result["shift_left_cm"] for result in unsafe],
            c=[result["fr_force_hold"] for result in unsafe],
            cmap="Blues_r",
            vmin=color_min,
            vmax=color_max,
            marker="X",
            s=100,
            edgecolors="#7a3d00",
            linewidths=0.9,
            label="attitude limit exceeded",
        )

    recommended = next(result for result in results if is_recommended(result))
    axes[0].scatter(
        [recommended["shift_back_cm"]],
        [recommended["shift_left_cm"]],
        marker="*",
        s=260,
        facecolors="#f2a900",
        edgecolors="#4a3000",
        linewidths=1.0,
        label="recommended",
        zorder=5,
    )
    axes[0].annotate(
        f"recommended\nFR={recommended['fr_force_hold']:.1f}",
        (recommended["shift_back_cm"], recommended["shift_left_cm"]),
        xytext=(8, -34),
        textcoords="offset points",
        fontsize=9,
    )
    axes[0].set_title("Tested body-shift targets")
    axes[0].set_xlabel("backward shift (cm)")
    axes[0].set_ylabel("leftward shift (cm)")
    axes[0].legend(loc="upper left", fontsize=8)
    colorbar = figure.colorbar(safe_plot, ax=axes[0])
    colorbar.set_label("FR foot force before lift")

    axes[1].scatter(
        [result["max_attitude_deg"] for result in safe],
        [result["fr_force_hold"] for result in safe],
        color="#2f6f9f",
        edgecolors="#18324a",
        s=70,
        label="within limit",
    )
    if unsafe:
        axes[1].scatter(
            [result["max_attitude_deg"] for result in unsafe],
            [result["fr_force_hold"] for result in unsafe],
            color="#d98b2b",
            marker="X",
            s=80,
            label="limit exceeded",
        )
    axes[1].scatter(
        [recommended["max_attitude_deg"]],
        [recommended["fr_force_hold"]],
        marker="*",
        s=260,
        facecolors="#f2a900",
        edgecolors="#4a3000",
        linewidths=1.0,
        zorder=5,
    )
    axes[1].axvline(
        ATTITUDE_LIMIT_DEG,
        color="#444444",
        linestyle="--",
        linewidth=1.2,
        label="2° attitude limit",
    )
    axes[1].set_title("Unload versus body attitude")
    axes[1].set_xlabel("max |roll or pitch| during hold (degrees)")
    axes[1].set_ylabel("FR foot force before lift")
    axes[1].legend(fontsize=8)

    for axis in axes:
        axis.grid(True, color="#d7dce0", alpha=0.7)
        axis.set_facecolor("#fbfcfd")
    figure.suptitle(
        f"Go2 front-right foot weight-shift scan ({len(results)} valid runs)",
        fontsize=14,
    )
    figure.savefig(path, dpi=160, facecolor="white")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment_dir", type=Path)
    args = parser.parse_args()

    paths = sorted(args.experiment_dir.glob("*/data.csv"))
    if not paths:
        raise SystemExit("No */data.csv files found")

    results = [analyze_run(path) for path in paths]
    write_summary(results, args.experiment_dir / "summary.csv")
    plot_results(results, args.experiment_dir / "scan_overview.png")

    recommended = next(result for result in results if is_recommended(result))
    print(
        "recommended",
        f"back={recommended['shift_back_cm']:.1f} cm",
        f"left={recommended['shift_left_cm']:.1f} cm",
        f"FR force={recommended['fr_force_hold']:.1f}",
        f"unload={recommended['fr_unload_percent']:.1f}%",
        f"max attitude={recommended['max_attitude_deg']:.2f} deg",
    )


if __name__ == "__main__":
    main()
