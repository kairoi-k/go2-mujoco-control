#!/usr/bin/env python3
"""Report contiguous high-speed acceptance windows from a trot CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def longest_window(rows, speed_min: float, angle_max_deg: float):
    best = (0.0, None, None)
    start = None
    previous = None
    for row in rows:
        try:
            sample = (
                float(row["cmd_time_s"]),
                float(row["world_velocity_x_mps"]),
                max(
                    abs(float(row["imu_roll_rad"])),
                    abs(float(row["imu_pitch_rad"])),
                )
                * 180.0
                / math.pi,
                int(row["motion_stage"]),
            )
        except (KeyError, TypeError, ValueError):
            continue
        good = (
            sample[3] == 2
            and sample[1] >= speed_min
            and sample[2] <= angle_max_deg
        )
        if good and start is None:
            start = sample[0]
        if not good and start is not None:
            if previous is not None and previous[0] - start > best[0]:
                best = (previous[0] - start, start, previous[0])
            start = None
        previous = sample
    if start is not None and previous is not None and previous[0] - start > best[0]:
        best = (previous[0] - start, start, previous[0])
    return best


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--angle-deg", type=float, default=10.0)
    args = parser.parse_args()
    with args.csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    duration, start, end = longest_window(rows, args.speed, args.angle_deg)
    if start is None:
        print(
            "window speed>=%.2f angle<=%.1fdeg: none"
            % (args.speed, args.angle_deg)
        )
        return
    print(
        "window speed>=%.2f angle<=%.1fdeg: duration=%.3fs start=%.3f end=%.3f"
        % (args.speed, args.angle_deg, duration, start, end)
    )


if __name__ == "__main__":
    main()
