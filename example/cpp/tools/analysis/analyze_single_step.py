#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LEGS = ("FR", "FL", "RR", "RL")


def load_csv(path):
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    required = {
        "motion_stage",
        "base_world_x_m",
        "base_world_y_m",
    }
    missing = sorted(required - set(data.dtype.names or ()))
    if missing:
        raise SystemExit(f"CSV is missing required columns: {', '.join(missing)}")
    return data


def require(mask, label):
    if not np.any(mask):
        raise SystemExit(f"CSV has no samples for {label}")
    return mask


def tail(mask, time, duration=0.25):
    selected_time = time[mask]
    return mask & (time >= np.max(selected_time) - duration)


def mean(data, column, mask):
    return float(np.mean(data[column][mask]))


def analyze(path, leg):
    data = load_csv(path)
    time = data["cmd_time_s"]
    foot_world_x_col = f"{leg}_foot_world_x_m"
    foot_world_y_col = f"{leg}_foot_world_y_m"
    foot_clearance_col = f"{leg}_foot_ground_clearance_m"
    foot_force_col = f"{leg}_foot_force"

    startup = require(data["motion_stage"] == 1, "stand settle")
    shift = require(data["motion_stage"] == 3, "weight-shift settle")
    swing = require(data["motion_stage"] == 5, "foot swing")
    final = require(data["motion_stage"] == 10, "final neutral hold")
    motion = require(
        (data["motion_stage"] >= 2) & (data["motion_stage"] <= 9),
        "single-step motion",
    )

    startup_tail = tail(startup, time)
    shift_tail = tail(shift, time)
    final_tail = tail(final, time)

    initial_body_x = mean(data, "base_world_x_m", startup_tail)
    initial_body_y = mean(data, "base_world_y_m", startup_tail)
    final_body_x = mean(data, "base_world_x_m", final_tail)
    final_body_y = mean(data, "base_world_y_m", final_tail)
    start_foot_x = mean(data, foot_world_x_col, shift_tail)
    start_foot_y = mean(data, foot_world_y_col, shift_tail)
    final_foot_x = mean(data, foot_world_x_col, final_tail)
    final_foot_y = mean(data, foot_world_y_col, final_tail)

    support_legs = [candidate for candidate in LEGS if candidate != leg]
    support_drifts = []
    for support_leg in support_legs:
        x_col = f"{support_leg}_foot_world_x_m"
        y_col = f"{support_leg}_foot_world_y_m"
        start_x = mean(data, x_col, shift_tail)
        start_y = mean(data, y_col, shift_tail)
        final_x = mean(data, x_col, final_tail)
        final_y = mean(data, y_col, final_tail)
        support_drifts.append(
            1000.0 * math.hypot(final_x - start_x, final_y - start_y)
        )

    return {
        "path": path,
        "data": data,
        "leg": leg,
        "target_swing_x_m": float(
            np.max(data[f"{leg}_foot_swing_x_target_m"])
        ),
        "target_swing_y_m": float(
            np.max(data[f"{leg}_foot_swing_y_target_m"])
        ),
        "actual_swing_x_m": final_foot_x - start_foot_x,
        "actual_swing_y_m": final_foot_y - start_foot_y,
        "body_displacement_x_m": final_body_x - initial_body_x,
        "body_displacement_y_m": final_body_y - initial_body_y,
        "swing_clearance_mean_mm": 1000.0 * mean(
            data, foot_clearance_col, swing
        ),
        "swing_clearance_max_mm": 1000.0
        * float(np.max(data[foot_clearance_col][swing])),
        "swing_force_mean": mean(data, foot_force_col, swing),
        "swing_force_max": float(np.max(data[foot_force_col][swing])),
        "swing_zero_force_fraction": float(
            np.mean(data[foot_force_col][swing] == 0.0)
        ),
        "landing_force_mean": mean(
            data,
            foot_force_col,
            require(data["motion_stage"] == 7, "post-landing settle"),
        ),
        "max_abs_roll_deg": math.degrees(
            float(np.max(np.abs(data["imu_roll_rad"][motion])))
        ),
        "max_abs_pitch_deg": math.degrees(
            float(np.max(np.abs(data["imu_pitch_rad"][motion])))
        ),
        "max_support_drift_mm": max(support_drifts),
        "max_abs_q_error_rad": max(
            float(
                np.max(
                    np.abs(
                        data[f"{leg_name}_q_error"][motion]
                    )
                )
            )
            for leg_name in (
                f"{candidate}_{joint}"
                for candidate in LEGS
                for joint in ("hip", "thigh", "calf")
            )
        ),
        "max_abs_tau_est": max(
            float(
                np.max(
                    np.abs(
                        data[f"{leg_name}_tau_est"][motion]
                    )
                )
            )
            for leg_name in (
                f"{candidate}_{joint}"
                for candidate in LEGS
                for joint in ("hip", "thigh", "calf")
            )
        ),
    }


def write_summary(result, path):
    fields = [
        "leg",
        "target_swing_x_m",
        "target_swing_y_m",
        "actual_swing_x_m",
        "actual_swing_y_m",
        "body_displacement_x_m",
        "body_displacement_y_m",
        "swing_clearance_mean_mm",
        "swing_clearance_max_mm",
        "swing_force_mean",
        "swing_force_max",
        "swing_zero_force_fraction",
        "landing_force_mean",
        "max_abs_roll_deg",
        "max_abs_pitch_deg",
        "max_support_drift_mm",
        "max_abs_q_error_rad",
        "max_abs_tau_est",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerow({field: result[field] for field in fields})


def plot(result, path):
    data = result["data"]
    leg = result["leg"]
    time = data["cmd_time_s"]
    foot_x = data[f"{leg}_foot_world_x_m"]
    foot_y = data[f"{leg}_foot_world_y_m"]
    foot_force = data[f"{leg}_foot_force"]
    clearance = data[f"{leg}_foot_ground_clearance_m"]
    baseline_x = mean(
        data,
        f"{leg}_foot_world_x_m",
        data["motion_stage"] == 3,
    )
    baseline_y = mean(
        data,
        f"{leg}_foot_world_y_m",
        data["motion_stage"] == 3,
    )
    initial_body_x = mean(
        data,
        "base_world_x_m",
        data["motion_stage"] == 1,
    )

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(
        time,
        1000.0 * (foot_x - baseline_x),
        label=f"{leg} world x",
    )
    axes[0, 0].plot(
        time,
        1000.0 * (foot_y - baseline_y),
        label=f"{leg} world y",
    )
    axes[0, 0].set_title("Swing foot world displacement")
    axes[0, 0].set_ylabel("millimeters")

    force_axis = axes[0, 1]
    clearance_axis = force_axis.twinx()
    force_axis.plot(time, foot_force, color="#d98b2b", label="foot force")
    clearance_axis.plot(
        time,
        1000.0 * clearance,
        color="#2f6f9f",
        label="world clearance",
    )
    force_axis.set_title("Foot contact and clearance")
    force_axis.set_ylabel("force")
    clearance_axis.set_ylabel("millimeters")

    axes[1, 0].plot(
        time,
        1000.0 * (data["base_world_x_m"] - initial_body_x),
        color="#6f8f3d",
        label="body world x",
    )
    axes[1, 0].set_title("Body forward displacement")
    axes[1, 0].set_ylabel("millimeters")
    axes[1, 0].set_xlabel("time (s)")

    axes[1, 1].plot(
        time,
        np.degrees(data["imu_roll_rad"]),
        label="roll",
    )
    axes[1, 1].plot(
        time,
        np.degrees(data["imu_pitch_rad"]),
        label="pitch",
    )
    axes[1, 1].set_title("Body attitude")
    axes[1, 1].set_ylabel("degrees")
    axes[1, 1].set_xlabel("time (s)")

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend()
    figure.suptitle(f"Go2 single step with {leg}", fontsize=14)
    figure.savefig(path, dpi=160, facecolor="white")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--leg", choices=LEGS, default="FR")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    output_dir = args.output_dir or args.csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    result = analyze(args.csv_path, args.leg)
    write_summary(result, output_dir / "single_step_summary.csv")
    plot(result, output_dir / "single_step_overview.png")

    print(
        f"{args.leg} target swing="
        f"({result['target_swing_x_m'] * 1000:.1f}, "
        f"{result['target_swing_y_m'] * 1000:.1f}) mm, "
        f"actual swing="
        f"({result['actual_swing_x_m'] * 1000:.1f}, "
        f"{result['actual_swing_y_m'] * 1000:.1f}) mm"
    )
    print(
        f"body displacement="
        f"({result['body_displacement_x_m'] * 1000:.1f}, "
        f"{result['body_displacement_y_m'] * 1000:.1f}) mm, "
        f"clearance={result['swing_clearance_mean_mm']:.1f} mm, "
        f"force={result['swing_force_mean']:.1f}"
    )
    print(
        f"roll/pitch={result['max_abs_roll_deg']:.2f}/"
        f"{result['max_abs_pitch_deg']:.2f} deg, "
        f"support drift={result['max_support_drift_mm']:.1f} mm"
    )


if __name__ == "__main__":
    main()
