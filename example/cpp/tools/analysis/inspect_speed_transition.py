#!/usr/bin/env python3
"""Print a compact high-speed transition trace from a run CSV."""

import argparse
import csv
import math


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--start", type=float, default=-math.inf)
    parser.add_argument("--end", type=float, default=math.inf)
    parser.add_argument("--every", type=float, default=0.10)
    args = parser.parse_args()

    with open(args.csv_path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    last_printed = -math.inf
    for row in rows:
        t = float(row["cmd_time_s"])
        if t < args.start or t > args.end:
            continue
        if t - last_printed + 1.0e-9 < args.every:
            continue
        last_printed = t
        deg = 180.0 / math.pi
        print(
            f"t={t:.3f} stage={row['motion_stage']} "
            f"v={float(row['world_velocity_x_mps']):.3f} "
            f"roll={float(row['imu_roll_rad']) * deg:.1f} "
            f"pitch={float(row['imu_pitch_rad']) * deg:.1f} "
            f"contacts={row.get('wbc_shadow_active_contacts', '')} "
            f"mask={row.get('wbc_shadow_contact_mask', '')} "
            f"q={row.get('wbc_shadow_task_satisfied', '')}"
        )


if __name__ == "__main__":
    main()
