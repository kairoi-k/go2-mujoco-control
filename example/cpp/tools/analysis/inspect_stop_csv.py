#!/usr/bin/env python3
"""Inspect the final WBC stop-hold samples in a run CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args()
    with args.csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    first_stage3 = next(
        (index for index, row in enumerate(rows)
         if row.get("motion_stage") == "3"),
        None,
    )
    if first_stage3 is not None:
        print("before_stage3")
        for row in rows[max(0, first_stage3 - 8):first_stage3 + 3]:
            print(
                "t=%.3f stage=%s v=%.3f roll=%.1f pitch=%.1f" % (
                    float(row["cmd_time_s"]), row.get("motion_stage"),
                    float(row["world_velocity_x_mps"]),
                    math.degrees(float(row["imu_roll_rad"])),
                    math.degrees(float(row["imu_pitch_rad"])),
                )
            )
    rows = [r for r in rows if r.get("motion_stage") == "3"]
    for row in rows[:: max(1, len(rows) // 12)]:
        def f(key: str) -> float:
            return float(row.get(key, "nan"))
        print(
            "t=%.3f stage=%s v=%.3f roll=%.1f pitch=%.1f contacts=%s "
            "solver=%s id=%s tau=%.1f qerr=%.3f" % (
                f("cmd_time_s"), row.get("motion_stage"),
                f("world_velocity_x_mps"), math.degrees(f("imu_roll_rad")),
                math.degrees(f("imu_pitch_rad")), row.get("contact_count"),
                row.get("wbc_shadow_solver_ok"), row.get("wbc_full_id_ok"),
                f("wbc_shadow_max_abs_tau"),
                max(
                    abs(f("%s_q_error" % leg))
                    for leg in ("FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf", "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf")
                ),
            )
        )
    print("stage3_rows=%d" % len(rows))


if __name__ == "__main__":
    main()
