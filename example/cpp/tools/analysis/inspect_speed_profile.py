#!/usr/bin/env python3
"""Print selected samples from a WBC speed run for timing diagnosis."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--every", type=int, default=10)
    args = parser.parse_args()
    with args.csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    selected = []
    last_cycle = None
    for row in rows:
        try:
            stage = int(row["motion_stage"])
            cycle = int(row["cycle_index"])
            velocity = float(row["world_velocity_x_mps"])
            if stage != 2 or cycle == last_cycle:
                continue
            last_cycle = cycle
            selected.append(
                (
                    float(row["cmd_time_s"]),
                    cycle,
                    velocity,
                    math.degrees(float(row["imu_roll_rad"])),
                    math.degrees(float(row["imu_pitch_rad"])),
                )
            )
        except (KeyError, TypeError, ValueError):
            continue
    for sample in selected[:: max(1, args.every)]:
        print(
            "t=%.3f cycle=%d v=%.3f roll=%.2f pitch=%.2f" % sample
        )
    if selected:
        print(
            "last t=%.3f cycle=%d v=%.3f roll=%.2f pitch=%.2f" % selected[-1]
        )


if __name__ == "__main__":
    main()
