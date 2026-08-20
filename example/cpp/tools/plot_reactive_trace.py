#!/usr/bin/env python3
"""Render a compact, synchronized response trace for a reactive video."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from analyze_reactive_acceptance import analyze, number, read_rows


def series(rows, key):
    values = []
    for row in rows:
        value = number(row, key)
        values.append(value if math.isfinite(value) else math.nan)
    return values


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment")
    parser.add_argument("output")
    parser.add_argument("--time-offset", type=float, default=0.0)
    args = parser.parse_args()

    experiment = Path(args.experiment)
    result = analyze(str(experiment))
    rows = read_rows(experiment)
    times = [value - args.time_offset for value in series(rows, "cmd_time_s")]
    vx = series(rows, "world_velocity_x_mps")
    target_vx = series(rows, "event_target_vx_mps")
    yaw = series(rows, "imu_yaw_rad")
    finite_yaw = [value for value in yaw if math.isfinite(value)]
    yaw0 = finite_yaw[0] if finite_yaw else 0.0
    yaw_rel = [value - yaw0 if math.isfinite(value) else math.nan for value in yaw]
    event = result.get("scheduled_event") or {}
    start = event.get("start_s", math.nan)
    end = event.get("end_s", math.nan)
    start = float(start) if start is not None else math.nan
    end = float(end) if end is not None else math.nan
    start -= args.time_offset
    end -= args.time_offset
    event_name = str(event.get("type", "nominal")).upper()
    metrics = result.get("metrics", {})
    strict = "PASS" if result.get("strict_pass") else ("N/A" if not event else "CHECK")

    fig, axes = plt.subplots(
        2, 1, figsize=(3.0, 4.8), dpi=120, sharex=True,
        facecolor="#111827",
    )
    fig.subplots_adjust(left=0.17, right=0.96, top=0.82, bottom=0.12, hspace=0.30)
    for axis in axes:
        axis.set_facecolor("#111827")
        axis.tick_params(colors="#d1d5db", labelsize=7)
        for spine in axis.spines.values():
            spine.set_color("#6b7280")
        axis.grid(True, color="#374151", linewidth=0.5, alpha=0.7)
        if math.isfinite(start) and math.isfinite(end):
            axis.axvspan(start, end, color="#ef4444", alpha=0.22, linewidth=0)

    axes[0].plot(times, vx, color="#67e8f9", linewidth=1.25, label="measured vx")
    if any(math.isfinite(value) for value in target_vx):
        axes[0].plot(times, target_vx, color="#fbbf24", linewidth=0.9, linestyle="--", label="target vx")
    axes[0].axhline(0.0, color="#9ca3af", linewidth=0.6)
    axes[0].set_ylabel("vx (m/s)", color="#e5e7eb", fontsize=8)
    axes[0].legend(loc="upper right", fontsize=6, frameon=False, labelcolor="#e5e7eb")

    axes[1].plot(times, yaw_rel, color="#a7f3d0", linewidth=1.25, label="measured yaw")
    axes[1].axhline(0.0, color="#9ca3af", linewidth=0.6)
    axes[1].set_ylabel("yaw Δ (rad)", color="#e5e7eb", fontsize=8)
    axes[1].set_xlabel("controller time (s)", color="#e5e7eb", fontsize=8)
    axes[1].legend(loc="upper left", fontsize=6, frameon=False, labelcolor="#e5e7eb")
    axes[1].set_xlim(0.0, max(times) if times else 1.0)

    fig.suptitle(
        f"{event_name}  |  DATA GATE: {strict}",
        color="white", fontsize=11, fontweight="bold", y=0.95,
    )
    if math.isfinite(start) and math.isfinite(end):
        subtitle = f"event {start:.2f}–{end:.2f}s  |  red = active window"
    else:
        subtitle = "nominal run  |  no event injected"
    fig.text(0.5, 0.885, subtitle, ha="center", color="#fef08a", fontsize=7)
    summary = (
        f"yaw Δ {float(metrics.get('yaw_change_rad', math.nan)):+.3f} rad\n"
        f"vx drop {float(metrics.get('braking_drop_mps', math.nan)):.3f} m/s\n"
        f"jump {float(metrics.get('max_velocity_jump_mps', math.nan)):.3f} m/s"
    )
    fig.text(
        0.18, 0.845, summary, va="top", color="#e5e7eb", fontsize=7,
        family="DejaVu Sans Mono",
        bbox={"facecolor": "#1f2937", "edgecolor": "#4b5563", "alpha": 0.95, "pad": 4},
    )
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
