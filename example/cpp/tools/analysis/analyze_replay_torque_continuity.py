#!/usr/bin/env python3
"""Audit candidate J^T f torque continuity without calling it controller safety."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

TIME_EPSILON_S = 1e-12


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[int(fraction * (len(ordered) - 1))]


def stats(values: list[float]) -> str:
    return (
        f"count={len(values)},"
        f"p50={percentile(values, 0.50):.9g},"
        f"p95={percentile(values, 0.95):.9g},"
        f"max={max(values, default=0.0):.9g}"
    )


def format_event(event: dict[str, object]) -> str:
    return (
        f"prev_row={event['prev_row']},row={event['row']},"
        f"prev_time_s={event['prev_time_s']:.9g},time_s={event['time_s']:.9g},"
        f"prev_phase={event['prev_phase']:.9g},phase={event['phase']:.9g},"
        f"prev_contacts={event['prev_contacts']},contacts={event['contacts']},"
        f"joint={event['joint']},delta_nm={event['delta_nm']:.9g},"
        f"dt_s={event['dt_s']:.9g},rate_nm_s={event['rate_nm_s']:.9g}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--top-n", type=int, default=12)
    args = parser.parse_args()
    if args.top_n <= 0:
        print("validation=FAIL: --top-n must be positive")
        return 2

    invalid_rows = 0
    negative_dt_pairs = 0
    rows: list[dict[str, object]] = []
    candidate_fields: list[str] = []

    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fieldnames = reader.fieldnames or []
            candidate_fields = [
                name for name in fieldnames if name.endswith("_tau_ff_candidate")
            ]
            required = {
                "row_number",
                "cmd_time_s",
                "phase",
                "selected_contact_count",
                *candidate_fields,
            }
            missing = sorted(required - set(fieldnames))
            if not candidate_fields:
                raise ValueError("replay CSV has no *_tau_ff_candidate fields")
            if missing:
                raise ValueError("replay CSV missing fields: " + ",".join(missing))

            for raw in reader:
                try:
                    row = {
                        "row_number": int(float(raw["row_number"])),
                        "time_s": finite(raw, "cmd_time_s"),
                        "phase": finite(raw, "phase"),
                        "contacts": int(
                            round(finite(raw, "selected_contact_count"))
                        ),
                        "tau": tuple(finite(raw, name) for name in candidate_fields),
                    }
                except (KeyError, ValueError, OverflowError):
                    invalid_rows += 1
                    continue
                rows.append(row)
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if len(rows) < 2:
        print("validation=FAIL: fewer than two valid replay rows")
        return 1

    duplicate_pairs = 0
    positive_pairs = 0
    same_contact_pairs = 0
    contact_transition_pairs = 0
    duplicate_deltas: list[float] = []
    positive_dt_values: list[float] = []
    group_deltas: defaultdict[str, list[float]] = defaultdict(list)
    group_rates: defaultdict[str, list[float]] = defaultdict(list)
    joint_abs_torques: defaultdict[str, list[float]] = defaultdict(list)
    top_events: list[dict[str, object]] = []

    for row in rows:
        for name, value in zip(candidate_fields, row["tau"]):
            joint_abs_torques[name].append(abs(value))

    for previous, current in zip(rows, rows[1:]):
        dt_s = current["time_s"] - previous["time_s"]
        if dt_s < -TIME_EPSILON_S:
            negative_dt_pairs += 1
            continue

        deltas = [
            abs(current_value - previous_value)
            for current_value, previous_value in zip(
                current["tau"], previous["tau"]
            )
        ]
        max_index = max(range(len(deltas)), key=deltas.__getitem__)
        delta_nm = deltas[max_index]
        event = {
            "prev_row": previous["row_number"],
            "row": current["row_number"],
            "prev_time_s": previous["time_s"],
            "time_s": current["time_s"],
            "prev_phase": previous["phase"],
            "phase": current["phase"],
            "prev_contacts": previous["contacts"],
            "contacts": current["contacts"],
            "joint": candidate_fields[max_index],
            "delta_nm": delta_nm,
            "dt_s": dt_s,
            "rate_nm_s": (
                delta_nm / dt_s if dt_s > TIME_EPSILON_S else float("nan")
            ),
        }

        if dt_s <= TIME_EPSILON_S:
            duplicate_pairs += 1
            duplicate_deltas.append(delta_nm)
            continue

        positive_pairs += 1
        positive_dt_values.append(dt_s)
        group = (
            "contact_count_transition"
            if current["contacts"] != previous["contacts"]
            else "same_contact_count"
        )
        if group == "contact_count_transition":
            contact_transition_pairs += 1
        else:
            same_contact_pairs += 1
        group_deltas[group].append(delta_nm)
        group_rates[group].append(event["rate_nm_s"])
        group_deltas["all_positive_dt"].append(delta_nm)
        group_rates["all_positive_dt"].append(event["rate_nm_s"])
        top_events.append(event)

    top_events.sort(key=lambda event: event["delta_nm"], reverse=True)
    lines = [
        "candidate torque continuity audit",
        "interpretation=J_transpose_f_candidate_torque_only_not_controller_torque",
        "interpretation=duplicate_timestamps_are_reported_but_excluded_from_rate",
        "interpretation=no_safety_threshold_is_claimed_by_this_audit",
        f"replay_csv={args.replay_csv}",
        f"rows={len(rows)}",
        f"candidate_joint_count={len(candidate_fields)}",
        f"invalid_rows={invalid_rows}",
        f"negative_dt_pairs={negative_dt_pairs}",
        f"duplicate_timestamp_pairs={duplicate_pairs}",
        f"positive_dt_pairs={positive_pairs}",
        f"same_contact_count_positive_dt_pairs={same_contact_pairs}",
        f"contact_count_transition_positive_dt_pairs={contact_transition_pairs}",
        f"positive_dt_s={stats(positive_dt_values)}",
        f"duplicate_dt_delta_abs_nm={stats(duplicate_deltas)}",
        "positive_dt_delta_abs_nm:",
    ]
    for group in ("all_positive_dt", "same_contact_count", "contact_count_transition"):
        lines.append(f"  {group}={stats(group_deltas[group])}")
    lines.append("positive_dt_delta_rate_nm_s:")
    for group in ("all_positive_dt", "same_contact_count", "contact_count_transition"):
        lines.append(f"  {group}={stats(group_rates[group])}")
    lines.append("candidate_abs_torque_nm_by_joint:")
    for name in candidate_fields:
        lines.append(f"  {name}={stats(joint_abs_torques[name])}")
    lines.append(f"top_positive_dt_jumps_n={min(args.top_n, len(top_events))}:")
    for event in top_events[: args.top_n]:
        lines.append("  " + format_event(event))

    validation_pass = invalid_rows == 0 and negative_dt_pairs == 0
    lines.append("validation=" + ("PASS" if validation_pass else "FAIL"))
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if validation_pass else 1


if __name__ == "__main__":
    sys.exit(main())
