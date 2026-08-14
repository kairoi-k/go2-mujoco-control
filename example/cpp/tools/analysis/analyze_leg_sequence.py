#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


LEGS = ("FR", "FL", "RR", "RL")


def load_csv(path):
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True))
    required = {
        "cmd_time_s",
        "motion_stage",
        "cycle_index",
        "active_leg_index",
        "base_world_x_m",
        "base_world_y_m",
        "imu_roll_rad",
        "imu_pitch_rad",
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


def max_abs(data, column, mask):
    return float(np.max(np.abs(data[column][mask])))


def infer_leg(data, step_rows, swing):
    active_indices = data["active_leg_index"][step_rows & swing]
    active_indices = active_indices[active_indices >= 0]
    if active_indices.size == 0:
        raise SystemExit("CSV has no active leg samples during foot swing")
    index = int(round(float(np.median(active_indices))))
    if index < 0 or index >= len(LEGS):
        raise SystemExit(f"Invalid active leg index in CSV: {index}")
    return LEGS[index]


def analyze_step(data, step, start_mask, target_start_x, target_start_y, last_step):
    time = data["cmd_time_s"]
    stage = data["motion_stage"]
    cycle = data["cycle_index"]
    step_rows = cycle == step
    shift = require(step_rows & (stage == 3), f"step {step} weight-shift settle")
    swing = require(step_rows & (stage == 5), f"step {step} foot swing")
    landing = require(step_rows & (stage == 7), f"step {step} landing settle")
    end = step_rows & (stage == 9)
    if last_step:
        end |= step_rows & (stage == 10)
    end = require(end, f"step {step} neutral end")
    motion = step_rows & (stage >= 2) & (stage <= 9)
    if last_step:
        motion |= step_rows & (stage == 10)

    start_tail = tail(start_mask, time)
    shift_tail = tail(shift, time)
    end_tail = tail(end, time)
    leg = infer_leg(data, step_rows, swing)
    foot_x = f"{leg}_foot_world_x_m"
    foot_y = f"{leg}_foot_world_y_m"
    foot_clearance = f"{leg}_foot_ground_clearance_m"
    foot_force = f"{leg}_foot_force"

    start_body_x = mean(data, "base_world_x_m", start_tail)
    start_body_y = mean(data, "base_world_y_m", start_tail)
    end_body_x = mean(data, "base_world_x_m", end_tail)
    end_body_y = mean(data, "base_world_y_m", end_tail)
    start_foot_x = mean(data, foot_x, shift_tail)
    start_foot_y = mean(data, foot_y, shift_tail)
    end_foot_x = mean(data, foot_x, end_tail)
    end_foot_y = mean(data, foot_y, end_tail)

    support_drifts = []
    for support_leg in LEGS:
        if support_leg == leg:
            continue
        support_x = f"{support_leg}_foot_world_x_m"
        support_y = f"{support_leg}_foot_world_y_m"
        drift_x = mean(data, support_x, end_tail) - mean(
            data, support_x, shift_tail
        )
        drift_y = mean(data, support_y, end_tail) - mean(
            data, support_y, shift_tail
        )
        support_drifts.append(1000.0 * math.hypot(drift_x, drift_y))

    q_error_columns = [
        f"{candidate}_{joint}_q_error"
        for candidate in LEGS
        for joint in ("hip", "thigh", "calf")
    ]
    tau_columns = [
        f"{candidate}_{joint}_tau_est"
        for candidate in LEGS
        for joint in ("hip", "thigh", "calf")
    ]
    target_end_x = mean(data, "body_advance_x_target_m", end_tail)
    target_end_y = mean(data, "body_advance_y_target_m", end_tail)

    return {
        "step": int(step),
        "leg": leg,
        "target_swing_x_m": float(
            np.max(data[f"{leg}_foot_swing_x_target_m"][step_rows])
        ),
        "target_swing_y_m": float(
            np.max(data[f"{leg}_foot_swing_y_target_m"][step_rows])
        ),
        "target_body_advance_x_m": target_end_x - target_start_x,
        "target_body_advance_y_m": target_end_y - target_start_y,
        "actual_swing_x_m": end_foot_x - start_foot_x,
        "actual_swing_y_m": end_foot_y - start_foot_y,
        "body_displacement_x_m": end_body_x - start_body_x,
        "body_displacement_y_m": end_body_y - start_body_y,
        "swing_clearance_mean_mm": 1000.0
        * mean(data, foot_clearance, swing),
        "swing_clearance_max_mm": 1000.0
        * float(np.max(data[foot_clearance][swing])),
        "swing_force_mean": mean(data, foot_force, swing),
        "swing_force_max": float(np.max(data[foot_force][swing])),
        "swing_zero_force_fraction": float(
            np.mean(data[foot_force][swing] == 0.0)
        ),
        "landing_force_mean": mean(data, foot_force, landing),
        "max_abs_roll_deg": math.degrees(
            max_abs(data, "imu_roll_rad", motion)
        ),
        "max_abs_pitch_deg": math.degrees(
            max_abs(data, "imu_pitch_rad", motion)
        ),
        "max_support_drift_mm": max(support_drifts),
        "max_abs_q_error_rad": max(
            max_abs(data, column, motion) for column in q_error_columns
        ),
        "max_abs_tau_est": max(
            max_abs(data, column, motion) for column in tau_columns
        ),
        "time_start_s": float(np.min(time[step_rows])),
        "time_end_s": float(np.max(time[step_rows])),
        "_end_tail": end_tail,
        "_target_end_x": target_end_x,
        "_target_end_y": target_end_y,
    }


def analyze(path):
    data = load_csv(path)
    stage = data["motion_stage"]
    cycle = data["cycle_index"]
    time = data["cmd_time_s"]
    startup = require(stage == 1, "stand settle")
    step_numbers = sorted(
        int(round(value))
        for value in np.unique(cycle[cycle > 0])
    )
    if not step_numbers:
        raise SystemExit("CSV has no completed step numbers")

    results = []
    start_mask = tail(startup, time)
    target_start_x = 0.0
    target_start_y = 0.0
    for index, step in enumerate(step_numbers):
        result = analyze_step(
            data,
            step,
            start_mask,
            target_start_x,
            target_start_y,
            step == step_numbers[-1],
        )
        results.append(result)
        start_mask = result["_end_tail"]
        target_start_x = result["_target_end_x"]
        target_start_y = result["_target_end_y"]
    return data, results


def write_summary(results, path):
    fields = [
        "step",
        "leg",
        "target_swing_x_m",
        "target_swing_y_m",
        "target_body_advance_x_m",
        "target_body_advance_y_m",
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
        writer = csv.DictWriter(
            handle, fieldnames=fields, lineterminator="\n"
        )
        writer.writeheader()
        for result in results:
            writer.writerow({field: result[field] for field in fields})


def plot(data, results, path, title):
    time = data["cmd_time_s"]
    startup = data["motion_stage"] == 1
    initial_body_x = mean(data, "base_world_x_m", startup)
    figure, axes = plt.subplots(
        2, 2, figsize=(12, 8), constrained_layout=True
    )
    body_axis, clearance_axis = axes[0]
    force_axis, attitude_axis = axes[1]

    body_axis.plot(
        time,
        1000.0 * (data["base_world_x_m"] - initial_body_x),
        color="#6f8f3d",
        label="body world x",
    )
    attitude_axis.plot(
        time,
        np.degrees(data["imu_roll_rad"]),
        label="roll",
    )
    attitude_axis.plot(
        time,
        np.degrees(data["imu_pitch_rad"]),
        label="pitch",
    )

    colors = ("#2f6f9f", "#d98b2b", "#8b5ca8", "#3c8c78")
    for result, color in zip(results, colors):
        step_rows = data["cycle_index"] == result["step"]
        leg = result["leg"]
        label = f"step {result['step']} {leg}"
        clearance_axis.plot(
            time[step_rows],
            1000.0 * data[f"{leg}_foot_ground_clearance_m"][step_rows],
            color=color,
            label=label,
        )
        force_axis.plot(
            time[step_rows],
            data[f"{leg}_foot_force"][step_rows],
            color=color,
            label=label,
        )
        body_axis.axvline(result["time_start_s"], color=color, alpha=0.3)
        body_axis.axvline(result["time_end_s"], color=color, alpha=0.3)

    body_axis.set_title("Body forward displacement")
    body_axis.set_ylabel("millimeters")
    clearance_axis.set_title("Active-foot world clearance")
    clearance_axis.set_ylabel("millimeters")
    force_axis.set_title("Active-foot contact force")
    force_axis.set_ylabel("force")
    attitude_axis.set_title("Body attitude")
    attitude_axis.set_ylabel("degrees")
    for axis in axes.flat:
        axis.set_xlabel("time (s)")
        axis.grid(True, alpha=0.25)
        axis.legend()
    figure.suptitle(title, fontsize=14)
    figure.savefig(path, dpi=160, facecolor="white")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--summary-name",
        default="two_step_summary.csv",
        help="summary filename inside the output directory",
    )
    parser.add_argument(
        "--plot-name",
        default="two_step_overview.png",
        help="plot filename inside the output directory",
    )
    parser.add_argument(
        "--title",
        default="Go2 two-step FR to FL sequence",
        help="plot title",
    )
    args = parser.parse_args()

    output_dir = args.output_dir or args.csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    data, results = analyze(args.csv_path)
    write_summary(results, output_dir / args.summary_name)
    plot(data, results, output_dir / args.plot_name, args.title)

    for result in results:
        print(
            f"step {result['step']} {result['leg']}: "
            f"target swing="
            f"({result['target_swing_x_m'] * 1000:.1f}, "
            f"{result['target_swing_y_m'] * 1000:.1f}) mm, "
            f"actual swing="
            f"({result['actual_swing_x_m'] * 1000:.1f}, "
            f"{result['actual_swing_y_m'] * 1000:.1f}) mm, "
            f"body delta="
            f"({result['body_displacement_x_m'] * 1000:.1f}, "
            f"{result['body_displacement_y_m'] * 1000:.1f}) mm, "
            f"clearance="
            f"{result['swing_clearance_mean_mm']:.1f}/"
            f"{result['swing_clearance_max_mm']:.1f} mm, "
            f"max attitude="
            f"{result['max_abs_roll_deg']:.2f}/"
            f"{result['max_abs_pitch_deg']:.2f} deg, "
            f"support drift="
            f"{result['max_support_drift_mm']:.1f} mm"
        )


if __name__ == "__main__":
    main()
