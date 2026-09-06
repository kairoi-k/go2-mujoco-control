#!/usr/bin/env python3
"""Pre-registered V3 approach and cycle topology checks.

Physical boundaries, scene geometry, lifecycle and legacy gates remain owned
by analyze_b1_physical.py. These helpers only add the pre-registered stable
approach and complete-cycle topology evidence.
"""
from __future__ import annotations

import math
from collections import defaultdict

LEGS = ("FR", "FL", "RR", "RL")
DIAGONALS = (0b1001, 0b0110)
PERIOD_S = 0.14
PERIOD_TOL_S = 0.005
DUTY = 0.44
DUTY_TOL = 0.01
REQUESTED_SPEED_MPS = 1.0
REQUESTED_SPEED_TOL_MPS = 0.02
APPROACH_S = 0.80
APPROACH_SPEED_RANGE_MPS = (0.75, 1.25)
APPROACH_SPEED_FRACTION = 0.95
FORCE_WITNESS_N = 10.0  # Existing analyze_b1_physical.py policy.
TRUTH_GAP_S = 0.010  # Existing analyze_b1_physical.py policy.
CONTROLLER_GAP_S = 0.020  # Existing analyze_b1_physical.py policy.
DIAGONAL_SUPPORT_S = 0.010
AERIAL_FORCE_LIMIT_N = 10.0
AERIAL_WITNESS_S = 0.004


def _f(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else math.nan
    except (KeyError, TypeError, ValueError):
        return math.nan


def _norm(row, prefix):
    return math.sqrt(sum(_f(row, prefix + axis + "_N") ** 2 for axis in "xyz"))


def _quantile(values, q):
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        return None
    point = (len(ordered) - 1) * q
    low, high = math.floor(point), math.ceil(point)
    return ordered[low] if low == high else ordered[low] + (ordered[high] - ordered[low]) * (point - low)


def _span(rows, predicate, gap):
    """Observed duration only; never extrapolate past the last sample."""
    best = 0.0
    start = last = None
    for row in rows:
        time_s = _f(row, "time_s")
        good = math.isfinite(time_s) and predicate(row)
        if good and start is not None and time_s - last <= gap + 1e-9:
            last = time_s
        else:
            if start is not None:
                best = max(best, last - start)
            start = last = time_s if good else None
    if start is not None:
        best = max(best, last - start)
    return best


def _unique_state_rows(rows):
    by_tick = {}
    for row in rows:
        tick = _f(row, "state_tick_s")
        if math.isfinite(tick):
            by_tick[tick] = row
    return [by_tick[tick] for tick in sorted(by_tick)]

def _unique_truth_rows(rows):
    """Deduplicate exact GT timestamps while retaining every distinct sample."""
    by_time = {}
    for row in rows:
        time_s = _f(row, "time_s")
        if math.isfinite(time_s):
            by_time[time_s] = row
    return [by_time[time_s] for time_s in sorted(by_time)]


def _sequence_issues(rows, key, max_gap_s=None):
    values = [_f(row, key) for row in rows]
    issues = []
    if any(not math.isfinite(value) for value in values):
        issues.append(f"nonfinite_{key}")
    if any(a > b for a, b in zip(values, values[1:]) if math.isfinite(a) and math.isfinite(b)):
        issues.append(f"descending_{key}")
    if max_gap_s is not None and any(
        b - a > max_gap_s + 1e-9
        for a, b in zip(values, values[1:])
        if math.isfinite(a) and math.isfinite(b)
    ):
        issues.append(f"{key}_gap_gt_{round(max_gap_s * 1000):g}ms")
    return issues


def stable_approach(rows, first_step_contact_s):
    """Evaluate only the pre-contact 0.8 s stable-running approach.
    ``rows`` must already be joined on state_tick_s/time_s and contain the
    controller plus GT fields used below.  Speed and coverage are measured on
    GT ``time_s`` samples; controller tick reuse must not discard them.  The
    return object is evidence, not a verdict for the physical analyzer.
    """
    if first_step_contact_s is None or not math.isfinite(first_step_contact_s):
        return {"status": "NOT_CERTIFIED", "reasons": ["first_step_contact_missing"]}
    start = first_step_contact_s - APPROACH_S
    # The wrapper owns controller-to-GT age/join coverage.  Keep the helper's
    # independent ordering check, but do not turn state tick reuse into a
    # controller sampling gate here.
    reasons = _sequence_issues(rows, "state_tick_s")
    reasons += _sequence_issues(rows, "time_s", TRUTH_GAP_S)
    truth_rows = _unique_truth_rows(rows)
    # Contact belongs to the interaction boundary, so the approach is a
    # half-open interval and never counts the first-contact GT row.
    selected = [
        row for row in truth_rows
        if start <= _f(row, "time_s") < first_step_contact_s
    ]
    truth_times = [_f(row, "time_s") for row in selected]
    if not truth_times or truth_times[0] > start + TRUTH_GAP_S + 1e-9:
        reasons.append("approach_truth_coverage")
    if not truth_times or truth_times[-1] < first_step_contact_s - TRUTH_GAP_S - 1e-9:
        reasons.append("approach_first_contact_truth_tail_coverage")
    period_bad = sum(not math.isfinite(_f(row, "velocity_command_gait_period_s")) or abs(_f(row, "velocity_command_gait_period_s") - PERIOD_S) > PERIOD_TOL_S for row in selected)
    duty_bad = sum(not math.isfinite(_f(row, "velocity_command_gait_duty")) or abs(_f(row, "velocity_command_gait_duty") - DUTY) > DUTY_TOL for row in selected)
    request_bad = sum(not math.isfinite(_f(row, "velocity_command_requested_mps")) or abs(_f(row, "velocity_command_requested_mps") - REQUESTED_SPEED_MPS) > REQUESTED_SPEED_TOL_MPS for row in selected)
    stage_bad = sum(_f(row, "motion_stage") != 2 or _f(row, "velocity_command_active") != 1 for row in selected)
    if period_bad: reasons.append("approach_period_out_of_band")
    if duty_bad: reasons.append("approach_duty_out_of_band")
    if request_bad: reasons.append("approach_requested_speed_out_of_band")
    if stage_bad: reasons.append("approach_not_active_stage2")
    speeds = [_f(row, "base_qvel_world_x_mps") for row in selected]
    if any(not math.isfinite(speed) for speed in speeds):
        reasons.append("approach_speed_missing_or_nonfinite")
    in_band = sum(APPROACH_SPEED_RANGE_MPS[0] <= speed <= APPROACH_SPEED_RANGE_MPS[1] for speed in speeds if math.isfinite(speed))
    speed_fraction = in_band / len(speeds) if speeds else 0.0
    if speed_fraction < APPROACH_SPEED_FRACTION:
        reasons.append("approach_measured_speed_fraction")
    return {
        "status": "PASS" if not reasons else "NOT_CERTIFIED",
        "reasons": reasons,
        "evidence": {
            "interval_s": [start, first_step_contact_s], "rows": len(selected),
            "period_bad_rows": period_bad, "duty_bad_rows": duty_bad,
            "requested_speed_bad_rows": request_bad, "stage_bad_rows": stage_bad,
            "speed_in_band_fraction": speed_fraction,
            "speed_p05_mps": _quantile(speeds, .05),
        },
    }


def complete_cycle_topology(control_rows, truth_rows, interaction_start_s, interaction_end_s):
    """Return complete cycle topology and the registered V3 sliding result.

    A cycle is complete only when real adjacent cycle-index wraps bound its
    full state-tick interval inside the truth-owned interaction window. The
    supplied interaction bounds constrain the window but never fabricate a
    missing wrap. Phase samples are retained as evidence but are not used as a
    completeness proxy.
    The four-cycle edge rule is explicit: exactly four cycles use one edge
    window and require at least three good cycles; five or more use every
    consecutive five-cycle window and require three good cycles per window.
    """
    if (interaction_start_s is None or interaction_end_s is None
            or not math.isfinite(interaction_start_s)
            or not math.isfinite(interaction_end_s)
            or interaction_end_s <= interaction_start_s):
        return {"status": "NOT_CERTIFIED", "reasons": ["interaction_window_missing"]}
    reasons = _sequence_issues(control_rows, "state_tick_s", CONTROLLER_GAP_S)
    reasons += _sequence_issues(truth_rows, "time_s", TRUTH_GAP_S)
    for row in control_rows:
        cycle_value = _f(row, "cycle_index")
        if not math.isfinite(cycle_value):
            reasons.append("nonfinite_cycle_index")
        elif cycle_value < 0 or not cycle_value.is_integer():
            reasons.append("invalid_cycle_index")
        phase = _f(row, "phase")
        if not math.isfinite(phase):
            reasons.append("nonfinite_phase")
        elif not 0.0 <= phase <= 1.0:
            reasons.append("phase_out_of_range")
        if not math.isfinite(_f(row, "motion_stage")):
            reasons.append("nonfinite_motion_stage")
    truth_fields = [
        "total_contact_grf_world_x_N", "total_contact_grf_world_y_N",
        "total_contact_grf_world_z_N",
    ] + [
        f"{leg}_foot_contact_grf_world_{axis}_N"
        for leg in LEGS for axis in "xyz"
    ]
    for row in truth_rows:
        for field in truth_fields:
            if not math.isfinite(_f(row, field)):
                reasons.append(f"nonfinite_{field}")
    if any("_gap_gt_" not in reason for reason in reasons):
        return {"status": "NOT_CERTIFIED", "reasons": sorted(set(reasons)), "evidence": {"complete_cycles": 0, "cycles": [], "windows": []}}
    states = _unique_state_rows(control_rows)
    phase_by_cycle = {}
    for row in states:
        cycle_index = int(_f(row, "cycle_index"))
        phase = _f(row, "phase")
        previous_phase = phase_by_cycle.get(cycle_index)
        if previous_phase is not None and phase < previous_phase - 1e-9:
            reasons.append(f"phase_reversal_cycle_{cycle_index}")
        phase_by_cycle[cycle_index] = phase
    if any("_gap_gt_" not in reason for reason in reasons):
        return {"status": "NOT_CERTIFIED", "reasons": sorted(set(reasons)), "evidence": {"complete_cycles": 0, "cycles": [], "windows": []}}
    runs = []
    for row in states:
        cycle_index = int(_f(row, "cycle_index"))
        if _f(row, "motion_stage") != 2:
            continue
        if not runs or runs[-1]["cycle_index"] != cycle_index:
            runs.append({"cycle_index": cycle_index, "rows": []})
        runs[-1]["rows"].append(row)
    all_indexes = [run["cycle_index"] for run in runs]
    if any(b != a + 1 for a, b in zip(all_indexes, all_indexes[1:])):
        reasons.append("cycle_index_gap")
    if len(set(all_indexes)) != len(all_indexes):
        reasons.append("cycle_index_repeated_after_wrap")
    cycles = []
    for run_number, run in enumerate(runs):
        cycle_index, rows = run["cycle_index"], run["rows"]
        ticks = [_f(row, "state_tick_s") for row in rows]
        if not ticks:
            continue
        # Cycle-index transitions are the only wrap evidence.  Both a real
        # predecessor and successor run are required; interaction bounds only
        # decide whether that already-observed cycle lies in the window.
        if run_number == 0 or run_number + 1 >= len(runs):
            if ticks[-1] >= interaction_start_s - 1e-9 and ticks[0] < interaction_end_s - 1e-9:
                reasons.append(f"cycle_{cycle_index}_wrap_missing")
            continue
        previous_index = runs[run_number - 1]["cycle_index"]
        next_index = runs[run_number + 1]["cycle_index"]
        if previous_index != cycle_index - 1 or next_index != cycle_index + 1:
            reasons.append(f"cycle_{cycle_index}_adjacent_wrap_invalid")
            continue
        wrap_start = ticks[0]
        wrap_end = _f(runs[run_number + 1]["rows"][0], "state_tick_s")
        if wrap_start < interaction_start_s - 1e-9 or wrap_end > interaction_end_s + 1e-9 or wrap_end <= wrap_start:
            continue
        truth = [row for row in truth_rows if wrap_start <= _f(row, "time_s") < wrap_end - 1e-9]
        def contact_mask(row):
            mask = 0
            for bit, leg in enumerate(LEGS):
                if _norm(row, leg + "_foot_contact_grf_world_") >= FORCE_WITNESS_N:
                    mask |= 1 << bit
            return mask
        truth_times = [_f(row, "time_s") for row in truth]
        if (not truth or truth_times[0] - wrap_start > TRUTH_GAP_S + 1e-9
                or wrap_end - truth_times[-1] > TRUTH_GAP_S + 1e-9):
            reasons.append(f"cycle_{cycle_index}_truth_edge_coverage")
        if not truth:
            reasons.append(f"cycle_{cycle_index}_truth_coverage_missing")
        diagonal_s = {
            str(mask): _span(truth, lambda row, mask=mask: contact_mask(row) == mask, TRUTH_GAP_S)
            for mask in DIAGONALS
        }
        aerial_s = _span(truth, lambda row: _norm(row, "total_contact_grf_world_") < AERIAL_FORCE_LIMIT_N, TRUTH_GAP_S)
        plan_masks = sorted({int(_f(row, "terrain_execution_planned_contact_mask")) for row in rows if math.isfinite(_f(row, "terrain_execution_planned_contact_mask"))})
        good = all(value >= DIAGONAL_SUPPORT_S - 1e-9 for value in diagonal_s.values()) and aerial_s >= AERIAL_WITNESS_S - 1e-9
        cycles.append({
            "cycle_index": cycle_index, "interval_s": [wrap_start, wrap_end],
            "state_sample_interval_s": [ticks[0], ticks[-1]],
            "wrap_interval_s": [wrap_start, wrap_end],
            "truth_rows": len(truth), "period_s": _f(rows[len(rows) // 2], "velocity_command_gait_period_s"),
            "duty": _f(rows[len(rows) // 2], "velocity_command_gait_duty"),
            "diagonal_support_s": diagonal_s, "aerial_s": aerial_s,
            "planned_masks": plan_masks, "good_cycle": good,
        })
    indexes = [item["cycle_index"] for item in cycles]
    if len(cycles) < 4:
        reasons.append("complete_interaction_cycles_lt_4")
    if any(b != a + 1 for a, b in zip(indexes, indexes[1:])):
        reasons.append("cycle_index_gap")
    windows = []
    if len(cycles) == 4:
        candidates = [cycles]
    elif len(cycles) >= 5:
        candidates = [cycles[i:i + 5] for i in range(len(cycles) - 4)]
    else:
        candidates = []
    for window in candidates:
        good_count = sum(item["good_cycle"] for item in window)
        windows.append({"cycle_indices": [item["cycle_index"] for item in window],
                        "good_cycles": good_count, "required_good_cycles": 3,
                        "passed": good_count >= 3})
    if not windows:
        reasons.append("no_registered_cycle_window")
    elif any(not window["passed"] for window in windows):
        reasons.append("sliding_cycle_topology_below_three_good")
    return {"status": "PASS" if not reasons else "NOT_CERTIFIED", "reasons": reasons,
            "evidence": {"complete_cycles": len(cycles), "cycles": cycles, "windows": windows}}
