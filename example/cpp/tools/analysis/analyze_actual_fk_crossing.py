#!/usr/bin/env python3
"""Auditable, harness-only Go2 FK crossing evidence (C-007 development).

Only state_tick_s, measured base pose/orientation, q_state, measured contact
mask/forces, lifecycle/safety fields and the frozen terrain geometry drive the
crossing result.  Planned/target/execution fields are detected and ignored.
Historical ground-truth alignment is an explicit audit-only fallback for old
C002 logs that have no measured_fk columns; it never supplies measured state.
"""
from __future__ import annotations
import argparse, csv, json, math, statistics, sys
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
AXES = ("x", "y", "z")
BIT_MAP = {leg: 1 << i for i, leg in enumerate(LEGS)}
EDGE_X, TABLE_END_X, TABLE_Z, FLOOR_Z, PATCH_OFFSET = .70, 1.20, .05, 0.0, .022
CONTACT_FORCE_THRESHOLD_N = 5.0
PENETRATION_TOLERANCE_M = .035
FORBIDDEN = ("terrain_exec_", "terrain_pending_", "planned", "target_world_",
             "terrain_target", "swing_target", "committed_touchdown")


def number(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, TypeError, ValueError):
        return None


def first_number(row, *keys):
    for key in keys:
        value = number(row, key)
        if value is not None:
            return value
    return None


def measured_mask(row):
    for key in ("wbc_measured_contact_mask", "measured_contact_mask",
                "terrain_event_sequencer_measured_contacts"):
        value = number(row, key)
        if value is not None:
            return int(round(value)) & 15, key
    values = [number(row, "contact_" + leg) for leg in LEGS]
    if any(value is not None for value in values):
        return sum((int(round(value or 0)) & 1) * BIT_MAP[leg]
                   for leg, value in zip(LEGS, values)), "contact_FR/FL/RR/RL"
    return None, None


def measured_forces(row):
    values, source = [], None
    for leg in LEGS:
        value = first_number(row, f"measured_force_{leg}_N",
                             f"foot_force_{leg}", f"{leg}_force_N")
        values.append(value)
        if value is not None: source = source or "controller measured force estimate (N)"
    return values, source


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (cy*cp*cr + sy*sp*sr, cy*cp*sr - sy*sp*cr,
            cy*sp*cr + sy*cp*sr, sy*cp*cr - cy*sp*sr)


def rotate(q, p):
    w, x, y, z = q
    return ((1-2*y*y-2*z*z)*p[0] + (2*x*y-2*z*w)*p[1] + (2*x*z+2*y*w)*p[2],
            (2*x*y+2*z*w)*p[0] + (1-2*x*x-2*z*z)*p[1] + (2*y*z-2*x*w)*p[2],
            (2*x*z-2*y*w)*p[0] + (2*y*z+2*x*w)*p[1] + (1-2*x*x-2*y*y)*p[2])


def foot_body(leg, q_hip, q_thigh, q_calf):
    front, left = leg in ("FR", "FL"), leg in ("FL", "RL")
    side = 1.0 if left else -1.0
    hip_x, hip_y, hip_link = (.1934 if front else -.1934), side*.0465, side*.0955
    lower = q_thigh + q_calf
    leg_x = -.213*math.sin(q_thigh) - .213*math.sin(lower)
    leg_z = -.213*math.cos(q_thigh) - .213*math.cos(lower)
    return (hip_x + leg_x, hip_y + math.cos(q_hip)*hip_link - math.sin(q_hip)*leg_z,
            math.sin(q_hip)*hip_link + math.cos(q_hip)*leg_z)


def load_rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError(f"{path}: missing CSV header")
        return reader.fieldnames, list(reader)


def fk(row):
    bx = first_number(row, "world_base_x_m", "base_pos_world_x_m")
    by = first_number(row, "world_base_y_m", "base_pos_world_y_m")
    bz = first_number(row, "world_base_z_m", "base_pos_world_z_m")
    quat = tuple(number(row, key) for key in ("base_quat_w", "base_quat_x",
                                               "base_quat_y", "base_quat_z"))
    if all(value is not None for value in quat):
        rotation = quat
    else:
        rpy = tuple(first_number(row, key) for key in ("imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad"))
        if any(value is None for value in rpy):
            return None
        rotation = quat_from_rpy(*rpy)
    if None in (bx, by, bz):
        return None
    feet = []
    for leg in LEGS:
        values = [number(row, f"{leg}_{joint}_q_state") for joint in JOINTS]
        if any(value is None for value in values):
            return None
        px, py, pz = rotate(rotation, foot_body(leg, *values))
        feet.append((bx + px, by + py, bz + pz))
    return feet


def gt_record(row):
    """Read contact mask/forces and optional audit-only GT alignment values."""
    mask = number(row, "phase2_terrain_foot_contact_mask")
    forces = [first_number(row, f"{leg}_foot_contact_grf_world_z_N",
                           f"{leg}_contact_grf_world_z_N") for leg in LEGS]
    if mask is None and all(value is None for value in forces):
        return None
    positions = []
    for leg in LEGS:
        positions.append(tuple(number(row, f"{leg}_pos_world_{axis}_m") for axis in AXES))
    base = tuple(number(row, f"base_pos_world_{axis}_m") for axis in AXES)
    return {"mask": int(round(mask)) & 15 if mask is not None else 0,
            "forces": forces, "positions": positions, "base": base}


def tick_quality(rows, key="state_tick_s"):
    ticks = [number(row, key) for row in rows]
    ticks = [value for value in ticks if value is not None]
    deltas = [right-left for left, right in zip(ticks, ticks[1:])]
    positive = [delta for delta in deltas if delta > 0]
    return {"rows_with_tick": len(ticks), "monotonic": all(delta >= 0 for delta in deltas),
            "duplicates": sum(delta == 0 for delta in deltas),
            "backward": sum(delta < 0 for delta in deltas),
            "gaps": sum(delta > .002001 for delta in deltas),
            "dt_min_s": min(positive, default=None), "dt_median_s": statistics.median(positive) if positive else None,
            "dt_p95_s": percentile(positive, .95), "dt_max_s": max(positive, default=None),
            "dt_samples": len(positive), "nominal_dt_s": .002}


def percentile(values, fraction):
    if not values: return None
    ordered = sorted(values)
    return ordered[min(len(ordered)-1, int(math.ceil(fraction*len(ordered))-1))]


def nearest_index(times, value):
    if value is None or not times: return None
    return min(range(len(times)), key=lambda i: abs(times[i]-value))


def phase_report(name, rows, feet, gt):
    if not rows: return {"status": "missing", "reason": "no measured FK samples"}
    membership = {leg: {"flat": 0, "tabletop": 0, "unknown": 0} for leg in LEGS}
    contact = {leg: 0 for leg in LEGS}; force = {leg: 0 for leg in LEGS}
    penetration = {"contact_samples": 0, "max_m": 0.0, "within_tolerance": 0}
    swing = {"samples": 0, "collision_samples": 0, "clearance_min_m": None}
    for i, (row, points) in enumerate(zip(rows, feet)):
        row_mask, _ = measured_mask(row)
        if gt[i] is not None: row_mask = gt[i]["mask"]
        forces = gt[i]["forces"] if gt[i] is not None else measured_forces(row)[0]
        for index, (leg, point) in enumerate(zip(LEGS, points)):
            x, _, z = point; patch_z = z - PATCH_OFFSET
            surface_z = TABLE_Z if EDGE_X <= x <= TABLE_END_X else FLOOR_Z
            label = "tabletop" if EDGE_X <= x <= TABLE_END_X and abs(patch_z-surface_z) <= PENETRATION_TOLERANCE_M else "flat" if not (EDGE_X <= x <= TABLE_END_X) and abs(patch_z-surface_z) <= PENETRATION_TOLERANCE_M else "unknown"
            membership[leg][label] += 1
            touching = row_mask is not None and bool(row_mask & BIT_MAP[leg])
            force_touching = forces[index] is not None and forces[index] > CONTACT_FORCE_THRESHOLD_N
            if touching or force_touching: contact[leg] += 1
            if force_touching: force[leg] += 1
            penetration_m = max(0.0, surface_z-patch_z)
            if touching or force_touching:
                penetration["contact_samples"] += 1; penetration["max_m"] = max(penetration["max_m"], penetration_m)
                if penetration_m <= PENETRATION_TOLERANCE_M: penetration["within_tolerance"] += 1
            else:
                clearance = patch_z-surface_z; swing["samples"] += 1
                swing["clearance_min_m"] = clearance if swing["clearance_min_m"] is None else min(swing["clearance_min_m"], clearance)
                if clearance < -PENETRATION_TOLERANCE_M: swing["collision_samples"] += 1
    status = "observed" if any(sum(v.values()) for v in membership.values()) else "ambiguous"
    return {"status": status, "samples": len(rows), "surface_membership": {leg: max(values, key=values.get) for leg, values in membership.items()},
            "surface_membership_counts": membership, "measured_contact_witness_samples": contact,
            "measured_contact_witness_total": sum(contact.values()), "measured_force_witness_samples": force,
            "contact_penetration": penetration,
            "swing_clearance_collision": swing, "criteria_class": "development-only; frozen B1 thresholds not applied",
            "uncertainty": "FK site-to-contact-patch offset 0.022 m; membership/penetration tolerance ±0.035 m"}


def analyze(data_path, gt_path=None):
    fields, rows = load_rows(data_path)
    forbidden = sorted(key for key in fields if any(token in key.lower() for token in FORBIDDEN))
    gt_rows, gt_times = [], []
    if gt_path:
        gt_fields, gt_rows = load_rows(gt_path)
        gt_times = [number(row, "time_s") for row in gt_rows]
        gt_times = [value for value in gt_times if value is not None]
    feet, valid_rows, witnesses, align_deltas = [], [], [], []
    for row in rows:
        points = fk(row)
        if points is None: continue
        feet.append(points); valid_rows.append(row)
        index = nearest_index(gt_times, number(row, "state_tick_s")) if gt_rows else None
        witnesses.append(gt_record(gt_rows[index]) if index is not None else None)
        align_deltas.append(abs(gt_times[index]-number(row, "state_tick_s")) if index is not None else None)
    if not valid_rows: raise ValueError("no rows with complete measured pose, orientation, and 12 q_state joints")
    first_front = next((i for i, points in enumerate(feet) if max(points[0][0], points[1][0]) >= EDGE_X), None)
    first_rear = next((i for i, points in enumerate(feet) if max(points[2][0], points[3][0]) >= EDGE_X), None)
    rear_exit = next((i for i, points in enumerate(feet) if min(points[2][0], points[3][0]) > TABLE_END_X), None)
    cuts = [("approach", 0, first_front), ("front_ascent_first_touchdown", first_front, first_rear),
            ("tabletop_measured_support_posture", first_front, first_rear), ("rear_ascent_support_exchange", first_rear, rear_exit),
            ("rear_edge_descent", rear_exit, None), ("post_cross_recovery", rear_exit, None)]
    phases = {}
    for name, start, end in cuts:
        if start is None: phases[name] = {"status": "missing", "reason": "no FK crossing witness"}; continue
        finish = end if end is not None and end > start else len(valid_rows)
        phases[name] = phase_report(name, valid_rows[start:finish], feet[start:finish], witnesses[start:finish])
    fk_fields = all(f"measured_fk_{leg}_foot_world_{axis}" in fields for leg in LEGS for axis in AXES)
    per_leg = {leg: [] for leg in LEGS}
    if fk_fields:
        for row, points in zip(valid_rows, feet):
            for leg, point in zip(LEGS, points):
                logged = tuple(number(row, f"measured_fk_{leg}_foot_world_{axis}") for axis in AXES)
                if all(value is not None for value in logged): per_leg[leg].append(max(abs(a-b) for a,b in zip(logged, point)))
    fk_check = {"status": "checked" if any(per_leg.values()) else "unavailable", "source": "measured_fk_* (post-C005 only)", "per_leg_m": {leg: {"samples": len(values), "median": statistics.median(values) if values else None, "p95": percentile(values,.95), "max": max(values, default=None)} for leg, values in per_leg.items()}, "max_abs_error_m": max((max(values) for values in per_leg.values() if values), default=None), "reason": None if any(per_leg.values()) else "measured_fk fields unavailable; not a failed cross-check"}
    gt_aligned = sum(value is not None for value in align_deltas)
    gt_alignment = {"status": "checked" if gt_aligned else "unavailable", "mode": "nearest state_tick_s to GT time_s; audit-only positions/base, contact mask/forces remain witnesses", "rows_aligned": gt_aligned, "max_abs_dt_s": max((v for v in align_deltas if v is not None), default=None), "median_abs_dt_s": statistics.median([v for v in align_deltas if v is not None]) if gt_aligned else None, "reason": None if gt_aligned else "ground-truth alignment unavailable"}
    mask_sources = sorted({measured_mask(row)[1] for row in valid_rows if measured_mask(row)[1]})
    force_sources = sorted({measured_forces(row)[1] for row in valid_rows if measured_forces(row)[1]})
    return {"analyzer": "actual_fk_crossing", "evidence_class": "harness-only development evidence", "methodology": {"frame": "base_link x-forward/y-left/z-up; world = base_position + R(base_orientation)*foot_base", "quaternion_order": "w,x,y,z; RPY fallback roll,pitch,yaw intrinsic XYZ as production BodyToWorld convention", "leg_order": "FR,FL,RR,RL", "joint_order": "per leg hip,thigh,calf; q_state total 12", "fk_geometry_m": {"hip_x_front_rear": [.1934,-.1934], "hip_y_abs": .0465, "hip_link_y_abs": .0955, "thigh": .213, "calf": .213}}, "allowlist": {"state_tick_s": True, "base_pose_orientation": True, "q_state_12": True, "measured_contact_mask_forces": True, "safety_lifecycle": True, "frozen_terrain_geometry": {"floor_z_m": 0.0, "tabletop_x_m": [.70,1.20], "tabletop_z_m": .05}}, "forbidden_fields_present_but_ignored": forbidden, "input_rows": len(rows), "valid_fk_rows": len(valid_rows), "missing_required": [key for key in ("state_tick_s",) if key not in fields], "state_tick_quality": tick_quality(rows), "gt_alignment": gt_alignment, "contact_provenance": {"mask_sources": mask_sources, "force_sources": force_sources, "gt_mask_source": "phase2_terrain_foot_contact_mask (when supplied)", "gt_force_source": "<leg>_foot_contact_grf_world_z_N or <leg>_contact_grf_world_z_N (audit-only C002 witness)", "force_units": "N (normal/world-z estimate or GT GRF z)", "force_threshold_N": CONTACT_FORCE_THRESHOLD_N, "mask_bit_mapping": BIT_MAP, "temporal_alignment": "controller row state_tick_s; GT nearest time_s; no target/planned fields"}, "measured_fk_cross_check": fk_check, "body_posture_com": {"roll_abs_max_rad": max((abs(number(row,"imu_roll_rad")) for row in valid_rows if number(row,"imu_roll_rad") is not None), default=None), "pitch_abs_max_rad": max((abs(number(row,"imu_pitch_rad")) for row in valid_rows if number(row,"imu_pitch_rad") is not None), default=None), "com_progression": "unavailable (CoM is not allowlisted measured state)", "base_x_start_m": first_number(valid_rows[0],"world_base_x_m","base_pos_world_x_m"), "base_x_end_m": first_number(valid_rows[-1],"world_base_x_m","base_pos_world_x_m")}, "safety_lifecycle": {"status": "observed" if any(any(token in key.lower() for token in ("safety","lifecycle","stop","motion_stage","phase")) for key in fields) else "unavailable"}, "phases": phases, "overall_status": "observed" if first_front is not None else "ambiguous"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True); parser.add_argument("--ground-truth", type=Path); parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    try: report = analyze(args.data, args.ground_truth)
    except (OSError, ValueError) as exc: print(f"validation=FAIL: {exc}"); return 2
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.out: args.out.parent.mkdir(parents=True, exist_ok=True); args.out.write_text(text, encoding="utf-8")
    print(text, end=""); return 0

if __name__ == "__main__": sys.exit(main())
