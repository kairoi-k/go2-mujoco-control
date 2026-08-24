#!/usr/bin/env python3
"""Summarize sensor-map foothold observations for P1 dual acceptance."""
from __future__ import annotations

import csv
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Optional

STATUS = {
    0: "kValid",
    1: "kInvalidInput",
    2: "kUnknownSurface",
    3: "kNoSupportPatch",
    4: "kStepTooHigh",
    5: "kUnreachable",
}


def finite(row: dict, key: str) -> float | None:
    raw = row.get(key, "")
    if raw is None or raw == "":
        return None
    try:
        value = float(raw)
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    return value


def coarsen(z: float | None, step: float = 0.02) -> float | None:
    if z is None:
        return None
    return round(z / step) * step


def summarize(csv_path: Path) -> dict:
    with csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    walk = [r for r in rows if int(float(r.get("motion_stage", "0") or 0)) == 2]
    if not walk:
        walk = rows
    look_seq = []
    front_seq = []
    for row in walk:
        st = int(float(row.get("terrain_look_status_fr", "-1") or -1))
        z = finite(row, "terrain_look_z_fr_m")
        look_seq.append((STATUS.get(st, str(st)), coarsen(z)))
        fst = int(float(row.get("terrain_plan_status_fr", "-1") or -1))
        fz = finite(row, "terrain_plan_z_fr_m")
        front_seq.append((STATUS.get(fst, str(fst)), coarsen(fz)))

    def unique_run(seq):
        out = []
        for item in seq:
            if not out or out[-1] != item:
                out.append(item)
        return out

    xs = [finite(r, "world_base_x_m") for r in walk]
    xs = [x for x in xs if x is not None]
    pitches = [abs(finite(r, "imu_pitch_rad") or 0.0) for r in walk]
    look_z = [finite(r, "terrain_look_z_fr_m") for r in walk]
    look_z = [z for z in look_z if z is not None]
    return {
        "n_walk": len(walk),
        "x_min": min(xs) if xs else None,
        "x_max": max(xs) if xs else None,
        "pitch_peak_deg": (max(pitches) * 180.0 / math.pi) if pitches else None,
        "look_z_min": min(look_z) if look_z else None,
        "look_z_max": max(look_z) if look_z else None,
        "look_status": Counter(s for s, _ in look_seq),
        "front_status": Counter(s for s, _ in front_seq),
        "look_runs": unique_run(look_seq)[:24],
        "front_runs": unique_run(front_seq)[:24],
    }


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: analyze_terrain_observe.py <data.csv> [...]", file=sys.stderr)
        return 2
    for arg in sys.argv[1:]:
        path = Path(arg)
        summary = summarize(path)
        print(f"== {path}")
        print(
            f"walk_rows={summary['n_walk']} x=[{summary['x_min']:.3f},{summary['x_max']:.3f}] "
            f"pitch_peak={summary['pitch_peak_deg']:.1f}deg "
            f"look_z=[{summary['look_z_min']},{summary['look_z_max']}]"
            if summary["x_min"] is not None
            else f"walk_rows={summary['n_walk']} (no x)"
        )
        print("look_status", dict(summary["look_status"]))
        print("front_status", dict(summary["front_status"]))
        print("look_runs", summary["look_runs"])
        print("front_runs", summary["front_runs"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
