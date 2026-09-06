#!/usr/bin/env python3
"""Auditable analytical witnesses for the Stage C design review.

Python standard library only. These are synthetic/source-semantics checks,
NOT Go2 builds, Atlas replays, robot experiments, or B0/B1 acceptance tests.
Run: python analytical_checks.py --output analytical_results.json
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import struct

BASE = "f3b452d56b2bedd5ea02249d4e5087b6ca151c47"


def support_margin(a: tuple[float, float], b: tuple[float, float],
                   c: tuple[float, float], width: float = 0.04) -> float:
    """Two-contact branch equivalent to the audited SupportMargin2D formula."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    u = max(0.0, min(1.0, ((c[0]-a[0])*dx + (c[1]-a[1])*dy) /
                         (length*length))) if length > 1e-6 else 0.0
    distance = math.hypot(c[0]-(a[0]+u*dx), c[1]-(a[1]+u*dy))
    along = ((c[0]-a[0])*dx+(c[1]-a[1])*dy)/length if length > 1e-9 else 0.0
    return -distance if distance > width else min(along, length-along, width-distance)


def cross(a: tuple[float, float, float], b: tuple[float, float, float]):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def run_checks() -> dict:
    checks: list[dict] = []

    def record(name: str, condition: bool, result: dict, scope: str):
        checks.append({"id": name, "passed": bool(condition), "result": result, "scope": scope})

    resolution = struct.unpack("f", struct.pack("f", .05))[0]
    half_float = min(.035, .5*resolution-.025)
    half_exact = min(.035, .5*.05-.025)
    record("A01_region_degeneracy", 0 < half_float < 1e-8 and half_exact == 0,
           {"resolution_promoted_m": resolution, "half_width_float32_m": half_float,
            "full_width_float32_m": 2*half_float, "half_width_exact_m": half_exact},
           "Source arithmetic witness; does not prove every observed no-safe rejection has this cause.")

    high_state = True
    actual_anchor_initialized = False and high_state
    record("A02_shadow_anchor_input_asymmetry", not actual_anchor_initialized,
           {"shadow_enabled": True, "terrain_actuation": False,
            "have_high_state": high_state, "anchor_can_initialize_in_a_clean_run": actual_anchor_initialized},
           "Boolean guard witness from PublishTerrainControlSnapshot; no missing Atlas fields reconstructed.")

    x_bounds = (-2.0, -2.0 + 440*resolution)
    y_bounds = (-2.0, -2.0 + 80*resolution)
    base_x = 20.5
    cell_centers = [base_x-.45+(i+.5)*resolution for i in range(32)]
    in_map = sum(x_bounds[0] <= x < x_bounds[1] for x in cell_centers)
    record("A03_finite_map_window", in_map == 0,
           {"world_x_bounds_m": x_bounds, "world_y_bounds_m": y_bounds,
            "synthetic_base_x_m": base_x, "in_bounds_window_columns": in_map},
           "Possible zero-known-cells mechanism; actual H9 world position must still be checked on Atlas.")

    cases = [{"planner_nodes": n, "planner_dt_s": .02, "source_age_s": age,
              "mpc_intervals": 8, "mpc_dt_s": .03,
              "last_plan_node_s": (n-1)*.02,
              "last_requested_force_start_s": age+7*.03,
              "last_force_interval_end_s": age+8*.03}
             for n, age in [(8, 0.), (24, .3), (24, .1)]]
    for c in cases:
        c["covers_force_starts"] = c["last_plan_node_s"]+1e-12 >= c["last_requested_force_start_s"]
        c["covers_full_intervals"] = c["last_plan_node_s"]+1e-12 >= c["last_force_interval_end_s"]
    record("A04_horizon_is_not_ttl", not cases[0]["covers_force_starts"] and
           not cases[1]["covers_force_starts"] and cases[2]["covers_full_intervals"],
           {"cases": cases}, "Coverage arithmetic; the real solver's node/interval semantics must be declared, not silently changed.")

    indices = [math.floor((.04+k*.03)/.02+1e-9) for k in range(4)]
    record("A05_absolute_time_lookup", indices == [2, 3, 5, 6],
           {"absolute_indices": indices, "naive_indices": [2, 3, 4, 5]},
           "Synthetic time-grid witness; does not re-run the production H6 code.")

    pitch = math.radians(5)
    foot_x, foot_z = .2, -.35
    full_x = math.cos(pitch)*foot_x + math.sin(pitch)*foot_z
    error = abs(foot_x-full_x)
    record("A06_body_and_heading_frames", error > .025,
           {"synthetic_pitch_deg": 5, "full_rotation_x_m": full_x,
            "yaw_only_x_m": foot_x, "difference_m": error},
           "Body-frame FK needs full rotation; yaw-only rotation is appropriate only for a declared heading-aligned map frame.")

    m, g, cy, cz = 15., 9.81, .035, .35
    fz = m*g/2
    force = (0., fz*cy/cz, fz)
    moments = [cross((x, -cy, -cz), force) for x in [-.2, .2]]
    net_moment = [sum(v[i] for v in moments) for i in range(3)]
    margin = support_margin((-.2, 0), (.2, 0), (0, cy))
    record("A07_geometry_not_necessary_for_instantaneous_wrench", margin < .015 and
           max(abs(v) for v in net_moment) < 1e-10 and abs(force[1]) <= .6*force[2],
           {"legacy_margin_m": margin, "per_foot_force_N": force,
            "net_moment_Nm": net_moment, "resulting_lateral_accel_mps2": 2*force[1]/m},
           "Instantaneous wrench-feasible counterexample ONLY: not a proof of stable, viable, or safe locomotion over time.")

    centered_margin = support_margin((-.2, 0), (.2, 0), (0, 0))
    record("A08_geometry_not_sufficient_for_requested_dynamics", centered_margin >= .015 and 9 > .6*g,
           {"legacy_margin_m": centered_margin, "requested_horizontal_accel_mps2": 9,
            "friction_limited_accel_mps2_at_zero_vertical_accel": .6*g},
           "With zero vertical acceleration the requested horizontal acceleration violates friction despite a passing geometric margin.")

    choices_a = [((.2, -.065), 0.), ((.2, -.1), 1.)]
    choices_b = [((-.2, .135), 0.), ((-.2, .1), 1.)]
    combinations = [{"a": a, "b": b, "cost": ca+cb, "margin_m": support_margin(a, b, (0, 0))}
                    for (a, ca), (b, cb) in itertools.product(choices_a, choices_b)]
    feasible = [c for c in combinations if c["margin_m"] >= .015]
    best = min(feasible, key=lambda c: (c["cost"], -c["margin_m"]))
    record("A09_greedy_vs_joint_combination", combinations[0]["margin_m"] < .015 and len(feasible) > 0,
           {"combinations": combinations, "joint_selected": best},
           "Synthetic geometry example justifying joint consideration; not evidence that joint search solves any specific H9 input.")

    current_margin = support_margin((-.2, 0), (.2, 0), (0, .035))
    initial_results = [current_margin for _ in combinations]
    record("A10_fixed_initial_condition", all(v < .015 for v in initial_results),
           {"initial_margin_for_all_future_choices_m": initial_results},
           "Future footholds cannot repair a hard constraint on an already fixed current contact state; do not move x0 or skip its check.")

    contacts = [False, True, False, True, False, True]
    events = [k for k, value in enumerate(contacts) if value and (k == 0 or not contacts[k-1])]
    record("A11_repeated_touchdown_events", events == [1, 3, 5] and len(events[:1]) < len(events),
           {"true_rising_edges": events, "first_touchdown_only_representation": events[:1]},
           "Changing array length alone cannot create later foothold events.")

    contact_counts = [2, 2, 0, 2, 2, 0, 2, 2]
    transfer_gate = min(contact_counts) >= 2
    record("A12_aerial_transfer_contract_witness", not transfer_gate,
           {"running_trot_horizon_contact_counts": contact_counts,
            "frozen_analyzer_transfer_support_pass": transfer_gate},
           "Conditional conflict: when such a horizon is counted as a transfer MPC sample it fails >=2, despite aerial trot being allowed. Not proof all B1 cases are impossible.")

    shadow = {"flat": {"total": 6171, "valid": 441, "unchecked": 5730,
                       "no_plan": 5629, "support": 4105, "no_safe": 1524, "zero_known_no_safe": 1449},
              "step5": {"total": 6266, "valid": 389, "unchecked": 5877,
                        "no_plan": 5776, "support": 4209, "no_safe": 1567, "zero_known_no_safe": 1493}}
    for row in shadow.values():
        row["total_valid_fraction"] = row["valid"]/row["total"]
        row["reported_checked_valid_fraction"] = 1.
        row["support_fraction_of_no_plan_rows"] = row["support"]/row["no_plan"]
    record("A13_report_denominators", all(r["total_valid_fraction"] < .08 and
           r["valid"]+r["unchecked"] == r["total"] and r["support"]+r["no_safe"] == r["no_plan"]
           for r in shadow.values()),
           {"reported_counts_only": shadow,
            "zero_known_fraction_of_no_safe_rows": (1449+1493)/(1524+1567)},
           "Arithmetic on the committed H9 report; original CSVs were unavailable in this review environment.")

    velocity_command = 1.
    effective_reference = [(k+1)*.03*velocity_command for k in range(8)]
    record("A14_reference_vs_prediction", effective_reference[-1]-effective_reference[0] > .001,
           {"unchanged_input_velocity_command_mps": velocity_command,
            "effective_legacy_reference_x_m": effective_reference,
            "effective_reference_span_m": effective_reference[-1]-effective_reference[0]},
           "The existing SRBD solver integrates its nominal horizontal reference internally; do not confuse it with logged supplied-anchor span or optimized predicted states.")

    inherited = {"old_source_plan": 7, "new_proposal": 8, "event_id": "FR:cycle12",
                 "same_target": True, "same_event_time": True}
    record("A15_commitment_identity", inherited["old_source_plan"] != inherited["new_proposal"] and
           inherited["same_target"] and inherited["same_event_time"], inherited,
           "An inherited commitment may keep source provenance across new proposals; matching only plan IDs is neither required nor sufficient.")

    old_valid_until, now, new_rejected = 1.2, 1.1, True
    record("A16_candidate_rejection_vs_active_validity", new_rejected and now <= old_valid_until,
           {"new_candidate_rejected": new_rejected, "old_packet_still_within_validity": now <= old_valid_until,
            "old_packet_cannot_be_reused_after": old_valid_until},
           "Lifecycle distinction, not authorization to extend validity or suppress a frozen acceptance rejection.")

    return {"baseline_sha": BASE, "suite_kind": "analytical_source_semantics_and_synthetic_counterexamples",
            "atlas_experiments_run": False, "production_controller_built": False,
            "b0_or_b1_acceptance_claim": False,
            "passed": all(c["passed"] for c in checks), "checks_passed": sum(c["passed"] for c in checks),
            "checks_total": len(checks), "checks": checks}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("analytical_results.json"))
    args = parser.parse_args()
    result = run_checks()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(f"Analytical witnesses: {result['checks_passed']}/{result['checks_total']}; not robot acceptance.")
    return 0 if result["passed"] else 1

if __name__ == "__main__":
    raise SystemExit(main())
