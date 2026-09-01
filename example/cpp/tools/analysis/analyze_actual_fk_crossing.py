#!/usr/bin/env python3
"""Harness-only, measured-state Go2 FK crossing evidence.

This tool deliberately consumes controller state and measured-contact witnesses
only.  Terrain execution/target/planned columns are never read.  The report is
development evidence, not a frozen acceptance analyzer.
"""
from __future__ import annotations
import argparse, csv, json, math, sys
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
# Frozen development geometry: floor z=0, 5 cm tabletop x=[.70,1.20].
EDGE_X, TABLE_END_X, TABLE_Z, PATCH_OFFSET = .70, 1.20, .05, .022
FORBIDDEN = ("terrain_exec_", "terrain_pending_", "planned", "target_world_",
             "terrain_target", "swing_target", "committed_touchdown")


def f(row, key):
    try:
        v = float(row[key])
        return v if math.isfinite(v) else None
    except (KeyError, TypeError, ValueError):
        return None


def mask(row):
    for key in ("wbc_measured_contact_mask", "terrain_event_sequencer_measured_contacts"):
        v = f(row, key)
        if v is not None:
            return int(round(v)) & 15
    return sum((int(round(f(row, "contact_" + leg) or 0)) & 1) << i
               for i, leg in enumerate(LEGS))


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cy*cp*cr + sy*sp*sr, cy*cp*sr - sy*sp*cr,
            cy*sp*cr + sy*cp*sr, sy*cp*cr - cy*sp*sr)


def rotate(q, p):
    w, x, y, z = q
    # R(q) p, avoiding a dependency on numpy.
    return ( (1-2*y*y-2*z*z)*p[0] + (2*x*y-2*z*w)*p[1] + (2*x*z+2*y*w)*p[2],
             (2*x*y+2*z*w)*p[0] + (1-2*x*x-2*z*z)*p[1] + (2*y*z-2*x*w)*p[2],
             (2*x*z-2*y*w)*p[0] + (2*y*z+2*x*w)*p[1] + (1-2*x*x-2*y*y)*p[2])


def foot_body(leg, qh, qt, qc):
    front, left = leg in ("FR", "FL"), leg in ("FL", "RL")
    s = 1.0 if left else -1.0
    hx, hy, hlink = (.1934 if front else -.1934), s*.0465, s*.0955
    lower = qt + qc
    lx = -.213*math.sin(qt) - .213*math.sin(lower)
    lz = -.213*math.cos(qt) - .213*math.cos(lower)
    return (hx + lx, hy + math.cos(qh)*hlink - math.sin(qh)*lz,
            math.sin(qh)*hlink + math.cos(qh)*lz)


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as h:
        r = csv.DictReader(h)
        if not r.fieldnames:
            raise ValueError(f"{path}: missing CSV header")
        return r.fieldnames, list(r)


def fk(row):
    def first(*keys):
        for key in keys:
            value = f(row, key)
            if value is not None: return value
        return None
    bx = first("world_base_x_m", "base_pos_world_x_m")
    by = first("world_base_y_m", "base_pos_world_y_m")
    bz = first("world_base_z_m", "base_pos_world_z_m")
    quat = tuple(f(row, k) for k in ("base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"))
    if all(v is not None for v in quat):
        q = quat
    else:
        rpy = tuple(first(k) for k in ("imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad"))
        if any(v is None for v in rpy): return None
        q = quat_from_rpy(*rpy)
    if None in (bx, by, bz):
        return None
    out = []
    for leg in LEGS:
        vals = [f(row, f"{leg}_{j}_q_state") for j in JOINTS]
        if any(v is None for v in vals): return None
        p = rotate(q, foot_body(leg, *vals))
        out.append((bx+p[0], by+p[1], bz+p[2]))
    return out


def gt_witness(row):
    """Read only GT contact mask/forces; GT positions are intentionally excluded."""
    m = f(row, "phase2_terrain_foot_contact_mask")
    forces = []
    for leg in LEGS:
        v = f(row, f"{leg}_foot_contact_grf_world_z_N")
        if v is None: v = f(row, f"{leg}_contact_grf_world_z_N")
        forces.append(v)
    if m is None and all(v is None for v in forces): return None
    return (int(round(m)) & 15 if m is not None else 0, forces)


def phase_report(name, samples, feet, witnesses):
    if not samples: return {"status": "missing", "reason": "no measured FK samples"}
    memberships = {leg: {"flat": 0, "tabletop": 0, "unknown": 0} for leg in LEGS}
    contact_witness = {leg: 0 for leg in LEGS}
    clearances = []
    collisions = 0
    for i, (row, ps) in enumerate(zip(samples, feet)):
        m = mask(row)
        if witnesses[i] is not None: m = witnesses[i][0]
        for n, (x, y, z) in zip(LEGS, ps):
            patch_z = z - PATCH_OFFSET
            if .70 <= x <= 1.20:
                label = "tabletop" if abs(patch_z - TABLE_Z) <= .035 else "unknown"
                clearances.append(patch_z - TABLE_Z)
            else:
                label = "flat" if abs(patch_z) <= .035 else "unknown"
                clearances.append(patch_z)
            memberships[n][label] += 1
            if m & (1 << LEGS.index(n)): contact_witness[n] += 1
        if any(z - PATCH_OFFSET < -.035 for _, _, z in ps): collisions += 1
    dominant = {leg: max(v, key=v.get) for leg, v in memberships.items()}
    force_witness = {leg: 0 for leg in LEGS}
    force_max = {leg: None for leg in LEGS}
    for witness in witnesses:
        if witness is None: continue
        gt_mask, forces = witness
        for i, leg in enumerate(LEGS):
            force = forces[i]
            if force is not None and force > 5.0:
                force_witness[leg] += 1
                force_max[leg] = max(force_max[leg] or force, force)
    measured = sum(contact_witness.values())
    status = "observed" if measured or any(sum(v.values()) for v in memberships.values()) else "ambiguous"
    return {"status": status, "samples": len(samples), "surface_membership": dominant,
            "surface_membership_counts": memberships, "measured_contact_witness_samples": contact_witness,
            "measured_contact_witness_total": measured, "measured_force_witness_samples": force_witness,
            "measured_force_max_normal_N": force_max, "clearance_min_m": min(clearances),
            "collision_diagnostic_samples": collisions,
            "uncertainty": "FK site-to-patch offset ±0.005 m; membership boundary ±0.035 m",
            "criteria_class": "development-only; frozen acceptance thresholds not applied"}


def analyze(data_path, gt_path=None):
    fields, rows = load_rows(data_path)
    forbidden_seen = sorted(k for k in fields if any(x in k.lower() for x in FORBIDDEN))
    required = ["state_tick_s"]
    missing = [k for k in required if k not in fields]
    base_ok = all(k in fields for k in ("world_base_x_m", "world_base_y_m", "world_base_z_m")) or all(
        k in fields for k in ("base_pos_world_x_m", "base_pos_world_y_m", "base_pos_world_z_m"))
    orientation_ok = all(k in fields for k in ("base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z")) or all(
        k in fields for k in ("imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad"))
    if not base_ok: missing.append("base_pose")
    if not orientation_ok: missing.append("base_orientation")
    gt_rows = []
    if gt_path:
        _, gt_rows = load_rows(gt_path)
    feet, valid_rows, witnesses = [], [], []
    for i, row in enumerate(rows):
        p = fk(row)
        if p is None: continue
        feet.append(p); valid_rows.append(row)
        witnesses.append(gt_witness(gt_rows[min(i, len(gt_rows)-1)]) if gt_rows else None)
    if not valid_rows:
        raise ValueError("no rows with complete measured pose, orientation, and 12 q_state joints")
    xs = [p[0][0] for p in feet] + [p[1][0] for p in feet]
    first_front = next((i for i,p in enumerate(feet) if max(p[0][0],p[1][0]) >= EDGE_X), None)
    first_rear = next((i for i,p in enumerate(feet) if max(p[2][0],p[3][0]) >= EDGE_X), None)
    rear_exit = next((i for i,p in enumerate(feet) if min(p[2][0],p[3][0]) > TABLE_END_X), None)
    cuts = [("approach", 0, first_front), ("front_ascent_first_touchdown", first_front, first_rear),
            ("tabletop_measured_support_posture", first_front, first_rear),
            ("rear_ascent_support_exchange", first_rear, rear_exit),
            ("rear_edge_descent", rear_exit, None), ("post_cross_recovery", rear_exit, None)]
    phases = {}
    for name, a, b in cuts:
        if a is None: phases[name] = {"status":"missing", "reason":"no FK crossing witness"}; continue
        a, b = max(0,a), b if b is not None and b > a else len(valid_rows)
        phases[name] = phase_report(name, valid_rows[a:b], feet[a:b], witnesses[a:b])
    # Posture is measured base orientation only; CoM is not inferred from target/planned state.
    roll = [f(r,"imu_roll_rad") for r in valid_rows]; pitch = [f(r,"imu_pitch_rad") for r in valid_rows]
    posture = {"roll_abs_max_rad": max((abs(v) for v in roll if v is not None), default=None),
               "pitch_abs_max_rad": max((abs(v) for v in pitch if v is not None), default=None),
               "com_progression": "missing (CoM is not an allowlisted measured field)",
               "body_pose_progression": {"base_x_start_m": f(valid_rows[0],"world_base_x_m"),
                                          "base_x_end_m": f(valid_rows[-1],"world_base_x_m")}}
    fk_fields = all(f"measured_fk_{leg}_foot_world_{axis}" in fields
                    for leg in LEGS for axis in ("x", "y", "z"))
    fk_errors = []
    if fk_fields:
        for row, predicted in zip(valid_rows, feet):
            for leg, p in zip(LEGS, predicted):
                logged = tuple(f(row, f"measured_fk_{leg}_foot_world_{axis}")
                               for axis in ("x", "y", "z"))
                if any(v is None for v in logged): continue
                fk_errors.append(max(abs(a-b) for a,b in zip(logged, p)))
    fk_check = {"status": "checked" if fk_errors else "missing",
                "samples": len(fk_errors),
                "max_abs_error_m": max(fk_errors, default=None),
                "reason": None if fk_errors else "historical replay has no complete measured_fk columns"}
    lifecycle_fields = sorted(k for k in fields if any(token in k.lower() for token in
                         ("safety", "lifecycle", "stop", "motion_stage", "phase")))
    return {"analyzer":"actual_fk_crossing", "evidence_class":"harness-only development evidence",
            "allowlist":{"state_tick_s":True,"base_pose_orientation":True,"q_state_12":True,
                         "measured_contact_mask_forces":True,"safety_lifecycle":True,
                         "frozen_terrain_geometry":{"floor_z_m":0.0,"tabletop_x_m":[.70,1.20],"tabletop_z_m":.05}},
            "safety_lifecycle":{"status":"observed" if lifecycle_fields else "missing",
                                "fields_present":lifecycle_fields}, 
            "forbidden_fields_present_but_ignored": forbidden_seen,
            "input_rows":len(rows), "valid_fk_rows":len(valid_rows), "missing_required":missing,
            "measured_fk_cross_check":fk_check,
            "body_posture_com":posture, "phases":phases,
            "overall_status":"observed" if first_front is not None else "ambiguous"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True); ap.add_argument("--ground-truth", type=Path)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    try: report = analyze(a.data, a.ground_truth)
    except (OSError, ValueError) as e: print(f"validation=FAIL: {e}"); return 2
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if a.out: a.out.parent.mkdir(parents=True, exist_ok=True); a.out.write_text(text, encoding="utf-8")
    print(text, end=""); return 0

if __name__ == "__main__": sys.exit(main())
