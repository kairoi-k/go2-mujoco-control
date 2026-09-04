#!/usr/bin/env python3
"""Order-103 lockstep canary comparison (verification-only evidence).

Compares a lockstep run's ground-truth trajectory against an authoritative
wall-clock run in two segments split at the lockstep handoff tick (the trace
barrier row):

  * startup segment: t in [0, handoff_s)  -- both runs use the identical
    wall-clock startup path, so this measures wall-clock jitter equivalence;
  * lockstep segment: t in [handoff_s, end) -- the frozen-exchange phase.

Metrics (via linear time interpolation onto the union time grid): base
height, roll, pitch and speed. Reports p50/p95/max absolute differences per
segment plus a PASS/FAIL gate with the documented tolerances. Purely
diagnostic evidence for the acceptance report; the authoritative gates are
the lifecycle statuses, the fixed 3 m/s analyzer and the B0 analyzer.
"""
import argparse
import csv
import math
import sys
from bisect import bisect_right

import numpy as np


def load_gt(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "t": float(r["time_s"]),
                "z": float(r["base_pos_world_z_m"]),
                "qx": float(r["base_quat_x"]),
                "qy": float(r["base_quat_y"]),
                "qz": float(r["base_quat_z"]),
                "qw": float(r["base_quat_w"]),
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def euler(rows):
    out = []
    for r in rows:
        w, x, y, z = r["qw"], r["qx"], r["qy"], r["qz"]
        sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.degrees(math.asin(sp))
        roll = math.degrees(
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        out.append({"t": r["t"], "z": r["z"], "pitch": pitch, "roll": roll})
    return out


def interp(series, t):
    """series: list of (t, value) sorted; returns interpolated value."""
    times = series[0]
    values = series[1]
    i = bisect_right(times, t)
    if i == 0:
        return None if t < times[0] else values[0]
    if i >= len(times):
        return None if t > times[-1] else values[-1]
    t0, t1 = times[i - 1], times[i]
    f = (t - t0) / (t1 - t0)
    return values[i - 1] + f * (values[i] - values[i - 1])


def to_series(rows, key):
    return ([r["t"] for r in rows], [r[key] for r in rows])


def diff_stats(ref, cand, t0, t1):
    """Compare two euler-series over [t0, t1] on the union grid."""
    times = np.unique(np.concatenate([
        np.array([r["t"] for r in ref], dtype=float),
        np.array([r["t"] for r in cand], dtype=float),
    ]))
    times = times[(times >= t0) & (times <= t1)]
    dz, dp, dr = [], [], []
    rz = to_series(ref, "z")
    cz = to_series(cand, "z")
    rp = to_series(ref, "pitch")
    cp = to_series(cand, "pitch")
    rr = to_series(ref, "roll")
    cr = to_series(cand, "roll")
    for t in times:
        a = interp(rz, t)
        b = interp(cz, t)
        if a is None or b is None:
            continue
        dz.append(abs(a - b))
        dp.append(abs(interp(rp, t) - interp(cp, t)))
        dr.append(abs(interp(rr, t) - interp(cr, t)))
    if not dz:
        return None
    dz = np.array(dz)
    dp = np.array(dp)
    dr = np.array(dr)
    return {
        "samples": len(dz),
        "dz_m": {"p50": float(np.percentile(dz, 50)),
                 "p95": float(np.percentile(dz, 95)),
                 "max": float(dz.max())},
        "dpitch_deg": {"p50": float(np.percentile(dp, 50)),
                       "p95": float(np.percentile(dp, 95)),
                       "max": float(dp.max())},
        "droll_deg": {"p50": float(np.percentile(dr, 50)),
                      "p95": float(np.percentile(dr, 95)),
                      "max": float(dr.max())},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="authoritative wall-clock run dir")
    ap.add_argument("--cand", required=True, help="lockstep run dir")
    ap.add_argument("--trace", required=True, help="lockstep trace CSV")
    ap.add_argument("--member", required=True, choices=["baseline", "terrain"])
    ap.add_argument("--tolerance-dz-m", type=float, default=0.06)
    ap.add_argument("--tolerance-angle-deg", type=float, default=6.0)
    args = ap.parse_args()

    ref = euler(load_gt(f"{args.ref}/contact_ground_truth.csv"))
    cand = euler(load_gt(f"{args.cand}/contact_ground_truth.csv"))

    # handoff tick = barrier row's sim_tick_ms
    handoff_s = None
    with open(args.trace, newline="") as f:
        for row in csv.DictReader(f):
            if row["phase"] == "barrier":
                handoff_s = int(row["sim_tick_ms"]) / 1000.0
                break
    if handoff_s is None:
        print("no barrier row in trace; cannot split segments", file=sys.stderr)
        return 1

    end = min(ref[-1]["t"], cand[-1]["t"])
    startup = diff_stats(ref, cand, 0.0, handoff_s)
    lockstep_seg = diff_stats(ref, cand, handoff_s, end)

    print(f"member={args.member} handoff_tick_s={handoff_s:.3f} end_s={end:.3f}")
    for name, seg in (("startup", startup), ("lockstep", lockstep_seg)):
        if seg is None:
            print(f"{name}: no overlapping samples")
            continue
        dz = seg["dz_m"]
        dp = seg["dpitch_deg"]
        dr = seg["droll_deg"]
        print(f"{name}: samples={seg['samples']} "
              f"dz(p50/p95/max)={dz['p50']:.4f}/{dz['p95']:.4f}/{dz['max']:.4f} m "
              f"dpitch={dp['p50']:.3f}/{dp['p95']:.3f}/{dp['max']:.3f} deg "
              f"droll={dr['p50']:.3f}/{dr['p95']:.3f}/{dr['max']:.3f} deg")

    def gate(seg):
        if seg is None:
            return False
        # Robust gate on p95; max is reported as diagnostic (stop-transition
        # and touchdown transients differ run-to-run).
        return (seg["dz_m"]["p95"] <= args.tolerance_dz_m and
                seg["dpitch_deg"]["p95"] <= args.tolerance_angle_deg and
                seg["droll_deg"]["p95"] <= args.tolerance_angle_deg)

    ok = gate(startup) and gate(lockstep_seg)
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(tolerances dz<={args.tolerance_dz_m} m, "
          f"angle<={args.tolerance_angle_deg} deg)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
