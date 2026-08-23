#!/usr/bin/env python3
"""P2 5cm barrier cross: x past far edge and |pitch|<=12 deg in the window."""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

# barrier pos 0.58, half-x 0.14 -> [0.44, 0.72]
X_NEAR = 0.44
X_FAR = 0.90
PITCH_LIM = 12.0 * math.pi / 180.0


def f(row, key):
    try:
        v = float(row[key])
    except (KeyError, ValueError):
        return None
    return v if math.isfinite(v) else None


def analyze(path: Path) -> dict:
    rows = list(csv.DictReader(path.open()))
    walk = [r for r in rows if int(float(r.get("motion_stage", "0") or 0)) == 2]
    xs = [f(r, "world_base_x_m") for r in walk]
    xs = [x for x in xs if x is not None]
    crossed = bool(xs) and max(xs) >= X_FAR
    window_pitch = []
    phases = []
    for r in walk:
        x = f(r, "world_base_x_m")
        p = f(r, "imu_pitch_rad")
        if x is None or p is None:
            continue
        if X_NEAR <= x <= X_FAR + 0.20:
            window_pitch.append(abs(p))
        ph = r.get("terrain_fsm_phase") or ""
        if ph != "":
            try:
                phases.append(int(float(ph)))
            except (TypeError, ValueError):
                pass
    peak = max(window_pitch) if window_pitch else None
    pitch_ok = peak is not None and peak <= PITCH_LIM
    return {
        "n": len(walk),
        "x_max": max(xs) if xs else None,
        "crossed": crossed,
        "pitch_peak_deg": (peak * 180.0 / math.pi) if peak is not None else None,
        "pitch_ok": pitch_ok,
        "pass": crossed and pitch_ok,
        "phases": sorted(set(phases)),
    }


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: analyze_p2_cross.py data.csv", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    s = analyze(path)
    print(
        f"{path} n={s['n']} x_max={s['x_max']} crossed={s['crossed']} "
        f"pitch_peak={s['pitch_peak_deg']}deg pitch_ok={s['pitch_ok']} "
        f"phases={s['phases']} PASS={s['pass']}"
    )
    return 0 if s["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
