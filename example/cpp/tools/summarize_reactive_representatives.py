#!/usr/bin/env python3
"""Make compact quantitative evidence for the representative demo suite."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


EVENT_NAMES = {0: "none", 1: "emergency_stop", 2: "obstacle_left", 3: "obstacle_right", 4: "turn_left", 5: "turn_right", 6: "slip", 7: "low_friction", 8: "impact"}


def read_run(root: Path, name: str) -> list[dict[str, str]]:
    path = root / f"reactive_representative_{name}" / "data.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def xy(rows: list[dict[str, str]], key: str, clip_start: float = 3.5) -> tuple[list[float], list[float]]:
    pairs = [(float(r["cmd_time_s"]) - clip_start, float(r[key])) for r in rows if r.get(key) not in (None, "") and float(r["cmd_time_s"]) >= clip_start]
    return [p[0] for p in pairs], [p[1] for p in pairs]


def event_span(rows: list[dict[str, str]], code: int, clip_start: float = 3.5) -> tuple[float, float] | None:
    ts = [float(r["cmd_time_s"]) - clip_start for r in rows if int(float(r.get("event_type", "0"))) == code]
    return (min(ts), max(ts)) if ts else None


def shade(ax, span: tuple[float, float] | None, color: str, label: str) -> None:
    if span is not None:
        ax.axvspan(span[0], span[1], color=color, alpha=0.16, label=label)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-experiment-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    runs = {name: read_run(args.repo_experiment_root, name) for name in ["impact_strong_recovery", "slip_reference", "low_friction_physical", "turn_left_long", "obstacle_left_physical", "obstacle_to_turn_handoff"]}

    metrics: list[dict[str, object]] = []
    for name, rows in runs.items():
        event_codes = sorted({int(float(r.get("event_type", "0"))) for r in rows})
        event_rows = [r for r in rows if int(float(r.get("event_type", "0"))) != 0]
        gt_path = args.repo_experiment_root / f"reactive_representative_{name}" / "contact_ground_truth.csv"
        obstacle_force = obstacle_count = None
        if gt_path.exists():
            with gt_path.open(newline="", encoding="utf-8") as stream:
                gt = list(csv.DictReader(stream))
            obstacle_force = max(float(r.get("reactive_obstacle_contact_force_N", "0")) for r in gt)
            obstacle_count = max(float(r.get("reactive_obstacle_contact_count", "0")) for r in gt)
        metrics.append({
            "id": name,
            "duration_s": round(float(rows[-1]["cmd_time_s"]), 3),
            "event_types": "+".join(EVENT_NAMES[c] for c in event_codes if c),
            "event_rows": len(event_rows),
            "max_abs_yaw_rad": round(max(abs(float(r["imu_yaw_rad"])) for r in rows), 4),
            "max_abs_vy_mps": round(max(abs(float(r["world_velocity_y_mps"])) for r in rows), 4),
            "obstacle_contact_force_max_N": obstacle_force,
            "obstacle_contact_count_max": obstacle_count,
        })
    with (args.output_root / "metrics.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)

    slip, low = runs["slip_reference"], runs["low_friction_physical"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for rows, label, color in [(slip, "slip: actual vx", "tab:orange"), (low, "low friction: actual vx", "tab:blue")]:
        x, y = xy(rows, "world_velocity_x_mps")
        axes[0].plot(x, y, lw=1.2, label=label, color=color)
    x, y = xy(slip, "event_ref_vx_mps")
    axes[0].plot(x, y, lw=1.0, ls="--", color="tab:red", label="slip: vx reference")
    x, y = xy(low, "event_ref_vx_mps")
    axes[0].plot(x, y, lw=1.0, ls="--", color="tab:green", label="low friction: vx reference")
    shade(axes[0], event_span(slip, 6), "orange", "slip window")
    axes[0].axvspan(4.5, 8.5, color="blue", alpha=0.08, label="low friction: μ=0.02")
    axes[0].set_ylabel("vx (m/s)")
    axes[0].set_title("Slip vs physical low-friction: reference and measured velocity")
    axes[0].grid(alpha=0.25)
    axes[0].legend(ncol=2, fontsize=8)
    x, y = xy(slip, "imu_roll_rad")
    axes[1].plot(x, y, lw=1.0, color="tab:orange", label="slip roll")
    x, y = xy(low, "imu_roll_rad")
    axes[1].plot(x, y, lw=1.0, color="tab:blue", label="low-friction roll")
    axes[1].axvspan(4.5, 8.5, color="blue", alpha=0.08)
    axes[1].set_xlabel("time from walking clip (s)")
    axes[1].set_ylabel("roll (rad)")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output_root / "slip_vs_low_friction_metrics.png", dpi=160)
    plt.close(fig)

    obstacle, turn = runs["obstacle_left_physical"], runs["turn_left_long"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for rows, label, color in [(obstacle, "obstacle: measured vy", "tab:green"), (turn, "turn: measured vy", "tab:cyan")]:
        x, y = xy(rows, "world_velocity_y_mps")
        axes[0].plot(x, y, lw=1.2, label=label, color=color)
    x, y = xy(obstacle, "event_ref_vy_mps")
    axes[0].plot(x, y, ls="--", lw=1.0, color="darkgreen", label="obstacle vy reference")
    axes[0].set_ylabel("vy (m/s)")
    axes[0].set_title("Obstacle avoidance is lane change + yaw; turn is yaw-only")
    axes[0].grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    for rows, label, color in [(obstacle, "obstacle: measured yaw", "tab:green"), (turn, "turn: measured yaw", "tab:cyan")]:
        x, y = xy(rows, "imu_yaw_rad")
        axes[1].plot(x, y, lw=1.2, label=label, color=color)
    x, y = xy(obstacle, "event_ref_yaw_rate_radps")
    axes[1].plot(x, y, ls="--", lw=1.0, color="darkgreen", label="obstacle yaw-rate ref")
    x, y = xy(turn, "event_ref_yaw_rate_radps")
    axes[1].plot(x, y, ls="--", lw=1.0, color="teal", label="turn yaw-rate ref")
    axes[1].set_xlabel("time from walking clip (s)")
    axes[1].set_ylabel("yaw (rad) / yaw-rate ref")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output_root / "obstacle_vs_turn_metrics.png", dpi=160)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
