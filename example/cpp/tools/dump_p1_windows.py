#!/usr/bin/env python3
"""Dump P1 walk-segment plan evidence from observe CSVs."""
import csv
import math
from pathlib import Path

ROOT = Path("/home/che/dev/go2-mujoco-control-terrain/example/cpp/experiments/_runs")
RUNS = [
    "p1_flat_n1_h2_2026-08-23",
    "p1_flat_n2_h2_2026-08-23",
    "p1_flat_n3_h2_2026-08-23",
    "p1_b10_n1_h2_2026-08-23",
    "p1_b10_n2_h2_2026-08-23",
    "p1_b10_n3_h2_2026-08-23",
    "p1_stair_n1_h2_2026-08-23",
    "p1_stair_n2_h2_2026-08-23",
    "p1_stair_n3_h2_2026-08-23",
]
STATUS = {
    -1: "none",
    0: "kValid",
    1: "kInvalidInput",
    2: "kUnknownSurface",
    3: "kNoSupportPatch",
    4: "kStepTooHigh",
    5: "kUnreachable",
}


def f(row, key):
    try:
        v = float(row[key])
    except (KeyError, ValueError):
        return None
    return v if math.isfinite(v) else None


for name in RUNS:
    path = ROOT / name / "data.csv"
    rows = list(csv.DictReader(path.open()))
    walk = [r for r in rows if int(float(r.get("motion_stage", "0") or 0)) == 2]
    if not walk:
        print(name, "NO_WALK")
        continue
    t0 = f(walk[0], "cmd_time_s")
    t1 = f(walk[-1], "cmd_time_s")
    first_elev = None
    max_look_z = 0.0
    for r in walk:
        z = f(r, "terrain_look_z_fr_m") or 0.0
        if z > max_look_z:
            max_look_z = z
        if first_elev is None and z >= 0.08:
            first_elev = (
                f(r, "cmd_time_s"),
                f(r, "world_base_x_m"),
                STATUS.get(int(float(r["terrain_look_status_fr"])), "?"),
                z,
            )
    print(
        f"{name} walk_t=[{t0:.2f},{t1:.2f}] n={len(walk)} "
        f"max_look_z={max_look_z:.3f} first_elev={first_elev}"
    )
