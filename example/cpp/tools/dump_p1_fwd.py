#!/usr/bin/env python3
import csv
import math
from collections import Counter
from pathlib import Path

ROOT = Path("/home/che/dev/go2-mujoco-control-terrain/example/cpp/experiments/_runs")
keys = [
    "terrain_fwd_z_20_m",
    "terrain_fwd_z_40_m",
    "terrain_fwd_z_60_m",
    "terrain_fwd_z_80_m",
]


def coarsen(v):
    if v is None or v < 0:
        return "unk"
    return f"{round(v / 0.05) * 0.05:.2f}"

for name in [
    "p1_stair_n1_h3_2026-08-23",
    "p1_stair_n2_h3_2026-08-23",
    "p1_stair_n3_h3_2026-08-23",
    "p1_b10_n1_h3_2026-08-23",
]:
    path = ROOT / name / "data.csv"
    if not path.exists():
        print(name, "MISSING")
        continue
    rows = list(csv.DictReader(path.open()))
    walk = [r for r in rows if int(float(r.get("motion_stage", "0") or 0)) == 2]
    profiles = []
    for r in walk:
        prof = []
        for k in keys:
            try:
                v = float(r[k])
            except (KeyError, ValueError):
                v = -1.0
            prof.append(coarsen(v))
        profiles.append(tuple(prof))
    counts = Counter(profiles)
    print(name, "n", len(walk), "top", counts.most_common(8))
