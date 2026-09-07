#!/usr/bin/env python3
import csv, hashlib, json, math, re, sys
from pathlib import Path
WINDOW = (22.444, 23.244)
LEGS = ("FR", "FL", "RR", "RL")
def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None
def stats(vals):
    vals = [x for x in vals if x is not None]
    if not vals:
        return {"count": 0}
    return {"count": len(vals), "min": min(vals), "max": max(vals)}
def read_rows(path, time_key, lo=WINDOW[0], hi=WINDOW[1]):
    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        rows = []
        for row in reader:
            t = num(row.get(time_key))
            if t is not None and lo - 1e-9 <= t <= hi + 1e-9:
                rows.append(row)
        return rows
def main(run_dir, scene):
    run = Path(run_dir)
    out = {
        "schema": "b1-initial-support-vs-feasibility-audit-v1",
        "acceptance_claim": False,
        "window_s": list(WINDOW),
        "run_dir": str(run),
    }
    metadata = {}
    for line in (run / "run_metadata.txt").read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            metadata[k] = v
    out["run_metadata"] = {k: metadata[k] for k in ("git_head", "git_dirty", "scene_file", "argv") if k in metadata}
    gt = read_rows(run / "contact_ground_truth.csv", "time_s")
    ctl = read_rows(run / "data.csv", "state_tick_s")
    out["rows"] = {"ground_truth": len(gt), "controller": len(ctl)}
    # Foot-site center and measured touch sensor.  The phase2 terrain mask is
    # kept separately: it marks the step/top interaction, not ordinary floor.
    contacts = {}
    for leg in LEGS:
        zkey = f"{leg}_pos_world_z_m"
        tkey = f"{leg}_touch_N"
        nkey = f"{leg}_terrain_nontop_contact_force_N"
        rows = [r for r in gt if num(r.get(tkey)) is not None and num(r.get(tkey)) >= 5.0]
        near = [r for r in rows if num(r.get(zkey)) is not None and 0.0145 - 1e-9 <= num(r[zkey]) <= 0.0212 + 1e-9]
        contacts[leg] = {
            "touch_threshold_N": 5.0,
            "touch_rows": len(rows),
            "touch_site_z_m": stats([num(r.get(zkey)) for r in rows]),
            "touch_force_N": stats([num(r.get(tkey)) for r in rows]),
            "near_radius_rows": len(near),
            "near_radius_time_s": stats([num(r.get("time_s")) for r in near]),
            "near_radius_site_z_m": stats([num(r.get(zkey)) for r in near]),
            "near_radius_touch_N": stats([num(r.get(tkey)) for r in near]),
            "terrain_nontop_force_N": stats([num(r.get(nkey)) for r in gt]),
            "terrain_nontop_positive_rows": sum(1 for r in gt if (num(r.get(nkey)) or 0.0) > 1e-9),
        }
    out["ground_truth_contacts"] = contacts
    out["step_mask"] = {
        "positive_rows": sum(1 for r in gt if int(num(r.get("phase2_terrain_foot_contact_mask")) or 0) != 0),
        "first_positive_s": next((num(r["time_s"]) for r in gt if int(num(r.get("phase2_terrain_foot_contact_mask")) or 0) != 0), None),
    }
    # Compact controller state at each planner update / state row transitions.
    ctl_keys = ("state_tick_s", "phase", "terrain_plan_status", "terrain_plan_id",
                "terrain_plan_failure", "terrain_map_epoch", "terrain_plan_valid",
                "terrain_execution_plan_id", "terrain_execution_plan_usable",
                "terrain_execution_applied_mask", "terrain_execution_planned_contact_mask",
                "contact_FR", "contact_FL", "contact_RR", "contact_RL",
                "foot_force_FR", "foot_force_FL", "foot_force_RR", "foot_force_RL")
    selected = []
    last = None
    for r in ctl:
        sig = tuple(r.get(k) for k in ctl_keys[2:])
        if sig != last:
            selected.append({k: r.get(k) for k in ctl_keys})
            last = sig
    out["controller_transition_rows"] = selected
    # Planner stderr is the per-leg rejection evidence.  Parse only the exact
    # emitted aggregate line; no inferred candidate geometry is manufactured.
    text = (run / "controller.log").read_text(errors="replace")
    pat = re.compile(r"Terrain planner id=(\d+) state=([0-9.]+) leg=(\d+) regions=(\d+) valid=(\d+) evaluated=(\d+) feasible=(\d+)(.*) selected=(none|\([^\n]*\))")
    entries = []
    for m in pat.finditer(text):
        t = num(m.group(2))
        if t is None or not (WINDOW[0] - 1e-9 <= t <= WINDOW[1] + 1e-9):
            continue
        reasons = {}
        for name, count in re.findall(r"\s([a-z_]+)=(\d+)", m.group(8)):
            reasons[name] = int(count)
        entries.append({"plan_id": int(m.group(1)), "state_s": t, "leg": int(m.group(3)),
                        "regions": int(m.group(4)), "valid": int(m.group(5)),
                        "evaluated": int(m.group(6)), "feasible": int(m.group(7)),
                        "rejections": reasons, "selected": m.group(9)})
    out["planner_rejection_lines"] = entries
    aggregate = {}
    for e in entries:
        for k, v in e["rejections"].items(): aggregate[k] = aggregate.get(k, 0) + v
    out["planner_rejection_aggregate"] = aggregate
    # Scene/model contact geometry.  Importing MuJoCo only loads XML; no mj_step
    # or simulator is invoked.  Values are model-resolved defaults.
    model = {}
    try:
        import mujoco
        m = mujoco.MjModel.from_xml_path(str(scene))
        model = {"mujoco_version": mujoco.__version__, "timestep_s": float(m.opt.timestep),
                 "option_solref": m.opt.o_solref.tolist(), "option_solimp": m.opt.o_solimp.tolist()}
        geoms = []
        for i in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
            if name in ("phase2_floor", "phase2_step_5cm", "FL", "FR", "RR", "RL"):
                geoms.append({"name": name, "size": m.geom_size[i].tolist(),
                    "pos": m.geom_pos[i].tolist(), "margin_m": float(m.geom_margin[i]),
                    "gap_m": float(m.geom_gap[i]), "solref": m.geom_solref[i].tolist(),
                    "solimp": m.geom_solimp[i].tolist(), "priority": int(m.geom_priority[i]),
                    "contype": int(m.geom_contype[i]), "conaffinity": int(m.geom_conaffinity[i])})
        model["geoms"] = geoms
        sites = []
        for i in range(m.nsite):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, i) or ""
            if "foot_contact" in name:
                sites.append({"name": name, "size": m.site_size[i].tolist(), "pos": m.site_pos[i].tolist()})
        model["foot_sites"] = sites
    except Exception as exc:
        model = {"error": type(exc).__name__ + ": " + str(exc)}
    out["mujoco_model_resolved"] = model
    out["derived_geometry"] = {
        "foot_collision_radius_m": 0.022,
        "foot_geom_center_x_offset_m": -0.002,
        "site_to_contact_patch_offset_m": 0.022,
        "site_z_minus_radius_m_for_prompt_band": [-0.0075, -0.0008],
        "interpretation": "site center is below the collision sphere radius over the observed 0.0145..0.0212 m band; this is compliant contact penetration evidence, not a proof of step collision",
    }
    out["source_evidence"] = {
        "kinematics": "example/cpp/kinematics/go2_forward_kinematics.h:30-45",
        "feasibility_initial_geometry": "example/cpp/terrain/terrain_feasibility.h:467-548,563-588",
        "world_checker": "example/cpp/terrain/terrain_feasibility.h:1027-1112",
        "planner_start": "example/cpp/terrain/terrain_planner.h:304-310",
        "runtime_start_population": "example/cpp/trot/trot_experiment_control.cpp:167-201,260-270",
        "scene": "unitree_robots/go2/go2.xml:4,8,29-34,107-108,143-144; b1_v3_running_step_5cm.xml:8-10",
    }
    return out
if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: audit.py RUN_DIR SCENE_XML")
    result = main(sys.argv[1], sys.argv[2])
    print(json.dumps(result, sort_keys=True, allow_nan=False))
