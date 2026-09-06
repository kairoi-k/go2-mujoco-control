#!/usr/bin/env python3
"""Read-only physical summary of the retained B1 probes; NOT acceptance."""
import argparse, csv, hashlib, json
from pathlib import Path

RUNS = ["b1_research_baseline_5cbc547_20260907_0001",
        "b1_research_sensoronly_5cbc547_20260907_0003"]
LEGS = ("FR", "FL", "RR", "RL")

def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1048576), b""):
            h.update(chunk)
    return h.hexdigest()

def summarize(run):
    result = {"run": str(run), "rows": 0, "base_x_min_m": None,
              "base_x_max_m": None, "nonfoot_collision_rows": 0,
              "first_obstacle_foot_contact": None, "first_nonfoot_collision": None,
              "first_body_beyond_s": None, "first_body_and_all_feet_beyond_s": None}
    # Harness-only geometry from the frozen centered scene; never controller input.
    rear_edge_m = 1.2
    with (run / "contact_ground_truth.csv").open(newline="") as f:
        for row in csv.DictReader(f):
            t, x = float(row["time_s"]), float(row["base_pos_world_x_m"])
            result["rows"] += 1
            result["base_x_min_m"] = x if result["base_x_min_m"] is None else min(x, result["base_x_min_m"])
            result["base_x_max_m"] = x if result["base_x_max_m"] is None else max(x, result["base_x_max_m"])
            foot = int(row["phase2_terrain_foot_contact_mask"])
            collision = int(row["phase2_terrain_nonfoot_contact_count"]) > 0
            result["nonfoot_collision_rows"] += int(collision)
            for key, condition in (("first_obstacle_foot_contact", foot > 0),
                                   ("first_nonfoot_collision", collision)):
                if condition and result[key] is None:
                    fields = ["time_s", "base_pos_world_x_m", "base_pos_world_z_m",
                              "phase2_terrain_foot_contact_mask", "phase2_terrain_nonfoot_contact_count"]
                    fields += [leg + suffix for leg in LEGS for suffix in
                               ("_pos_world_x_m", "_pos_world_y_m", "_pos_world_z_m",
                                "_foot_contact_grf_world_x_N", "_foot_contact_grf_world_z_N")]
                    result[key] = {k: float(row[k]) for k in fields}
            if x > rear_edge_m and result["first_body_beyond_s"] is None:
                result["first_body_beyond_s"] = t
            if x > rear_edge_m and all(float(row[leg + "_pos_world_x_m"]) > rear_edge_m for leg in LEGS):
                if result["first_body_and_all_feet_beyond_s"] is None:
                    result["first_body_and_all_feet_beyond_s"] = t
    result["hashes"] = {p.name: sha(p) for p in sorted(run.iterdir()) if p.is_file()}
    result["manifest"] = json.loads((run / "run_manifest.json").read_text())
    return result

def pointwise_settling(run):
    with (run / "data.csv").open(newline="") as f:
        rows = [(float(r["cmd_time_s"]), float(r["velocity_command_measured_mps"]))
                for r in csv.DictReader(f) if float(r["velocity_command_active"]) > .5
                and r["velocity_command_gait_regime"] == "continuous-trot"]
    start = rows[0][0]
    result = []
    for left, right, target in ((32., 40., 2.), (48., 56., 3.)):
        window = [(t-start, v) for t, v in rows if left <= t-start <= right]
        tolerance = max(.15, .05*target)
        longest = 0.; first = None
        for t, v in window:
            if abs(v-target) <= tolerance:
                if first is None:
                    first = t
                longest = max(longest, t-first)
            else:
                first = None
        result.append({"start_s": left, "end_s": right, "target_mps": target,
                       "rows": len(window), "observed_first_s": window[0][0],
                       "observed_last_s": window[-1][0],
                       "abs_error_min_mps": min(abs(v-target) for t,v in window),
                       "abs_error_max_mps": max(abs(v-target) for t,v in window),
                       "rows_inside_tolerance": sum(abs(v-target)<=tolerance for t,v in window),
                       "longest_consecutive_inside_span_s": longest,
                       "tolerance_mps": tolerance})
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("repo", type=Path)
    args = parser.parse_args()
    base = args.repo / "example/cpp/experiments/_runs"
    print(json.dumps({"schema": "b1-physical-probe-audit-v1", "acceptance_claim": False,
                      "runs": [summarize(base / name) for name in RUNS],
                      "independent_flat_transition_pointwise": pointwise_settling(base / "phase2_b0_development_steps_r0_20260907_000505_terrain")}, indent=2, allow_nan=False))
