#!/usr/bin/env python3
"""Sweep a candidate torque rate limiter as an offline shadow experiment."""

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


def run_limit(
    rows: list[dict[str, object]],
    candidate_fields: list[str],
    rate_limit: float,
) -> dict[str, object]:
    previous_time = rows[0]["time_s"]
    previous_contacts = rows[0]["contacts"]
    limited_previous = list(rows[0]["tau"])
    positive_dt_pairs = 0
    duplicate_pairs = 0
    limited_pairs = 0
    same_contact_pairs = 0
    transition_pairs = 0
    tracking_errors: list[float] = []
    limited_deltas: list[float] = []
    group_errors: defaultdict[str, list[float]] = defaultdict(list)
    group_rates: defaultdict[str, list[float]] = defaultdict(list)
    top_events: list[dict[str, object]] = []

    for row in rows[1:]:
        dt_s = row["time_s"] - previous_time
        if dt_s <= TIME_EPSILON_S:
            duplicate_pairs += 1
            limited = list(row["tau"])
            tracking_error = 0.0
        else:
            positive_dt_pairs += 1
            group = (
                "contact_count_transition"
                if row["contacts"] != previous_contacts
                else "same_contact_count"
            )
            if group == "contact_count_transition":
                transition_pairs += 1
            else:
                same_contact_pairs += 1
            allowed_delta = rate_limit * dt_s
            limited = [
                previous + max(-allowed_delta, min(allowed_delta, target - previous))
                for previous, target in zip(limited_previous, row["tau"])
            ]
            deltas = [
                abs(value - previous)
                for value, previous in zip(limited, limited_previous)
            ]
            target_errors = [
                abs(value - target)
                for value, target in zip(limited, row["tau"])
            ]
            limited_delta = max(deltas, default=0.0)
            tracking_error = max(target_errors, default=0.0)
            if tracking_error > TIME_EPSILON_S:
                limited_pairs += 1
            limited_deltas.append(limited_delta)
            group_errors[group].append(tracking_error)
            group_rates[group].append(limited_delta / dt_s)
            group_errors["all_positive_dt"].append(tracking_error)
            group_rates["all_positive_dt"].append(limited_delta / dt_s)
            if tracking_error > 0.0:
                top_events.append(
                    {
                        "prev_time_s": previous_time,
                        "time_s": row["time_s"],
                        "prev_contacts": previous_contacts,
                        "contacts": row["contacts"],
                        "row": row["row_number"],
                        "dt_s": dt_s,
                        "joint": candidate_fields[
                            max(
                                range(len(target_errors)),
                                key=target_errors.__getitem__,
                            )
                        ],
                        "tracking_error": tracking_error,
                    }
                )

        tracking_errors.append(tracking_error)
        limited_previous = limited
        previous_time = row["time_s"]
        previous_contacts = row["contacts"]

    top_events.sort(
        key=lambda event: event["tracking_error"],
        reverse=True,
    )
    return {
        "positive_dt_pairs": positive_dt_pairs,
        "duplicate_pairs": duplicate_pairs,
        "limited_pairs": limited_pairs,
        "same_contact_pairs": same_contact_pairs,
        "transition_pairs": transition_pairs,
        "tracking_errors": tracking_errors,
        "limited_deltas": limited_deltas,
        "group_errors": group_errors,
        "group_rates": group_rates,
        "top_events": top_events,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--max-torque-rate-nm-s",
        type=float,
        nargs="+",
        default=[500.0, 1000.0, 2000.0, 3000.0, 4000.0, 6000.0, 8000.0],
    )
    parser.add_argument("--top-n", type=int, default=3)
    args = parser.parse_args()
    if (
        args.top_n <= 0
        or not args.max_torque_rate_nm_s
        or any(
            not math.isfinite(value) or value <= 0.0
            for value in args.max_torque_rate_nm_s
        )
    ):
        print("validation=FAIL: invalid rate-limit parameters")
        return 2

    invalid_rows = 0
    negative_dt_pairs = 0
    rows: list[dict[str, object]] = []
    candidate_fields: list[str] = []
    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames or []
            candidate_fields = [
                name for name in fields if name.endswith("_tau_ff_candidate")
            ]
            required = {
                "row_number",
                "cmd_time_s",
                "selected_contact_count",
                *candidate_fields,
            }
            missing = sorted(required - set(fields))
            if not candidate_fields:
                raise ValueError("replay CSV has no candidate torque fields")
            if missing:
                raise ValueError("replay CSV missing fields: " + ",".join(missing))
            for raw in reader:
                try:
                    rows.append(
                        {
                            "row_number": int(float(raw["row_number"])),
                            "time_s": finite(raw, "cmd_time_s"),
                            "contacts": int(
                                round(finite(raw, "selected_contact_count"))
                            ),
                            "tau": tuple(
                                finite(raw, name) for name in candidate_fields
                            ),
                        }
                    )
                except (KeyError, ValueError, OverflowError):
                    invalid_rows += 1
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    for previous, current in zip(rows, rows[1:]):
        if current["time_s"] < previous["time_s"] - TIME_EPSILON_S:
            negative_dt_pairs += 1

    if len(rows) < 2:
        print("validation=FAIL: fewer than two valid rows")
        return 1

    lines = [
        "candidate torque rate-limit shadow sweep",
        "interpretation=candidate_J_transpose_f_only_not_controller_torque",
        "interpretation=hard_rate_limiter_is_simulated_offline_no_wrench_recompute",
        f"replay_csv={args.replay_csv}",
        f"rows={len(rows)}",
        f"candidate_joint_count={len(candidate_fields)}",
        f"invalid_rows={invalid_rows}",
        f"negative_dt_pairs={negative_dt_pairs}",
    ]
    for rate_limit in sorted(set(args.max_torque_rate_nm_s)):
        result = run_limit(rows, candidate_fields, rate_limit)
        lines.extend(
            [
                f"rate_limit_nm_s={rate_limit:.9g}",
                f"  positive_dt_pairs={result['positive_dt_pairs']}",
                f"  duplicate_timestamp_pairs={result['duplicate_pairs']}",
                f"  rate_limited_pairs={result['limited_pairs']}",
                f"  same_contact_count_positive_dt_pairs={result['same_contact_pairs']}",
                f"  contact_count_transition_positive_dt_pairs={result['transition_pairs']}",
                f"  tracking_error_abs_nm={stats(result['tracking_errors'])}",
                f"  applied_delta_abs_nm={stats(result['limited_deltas'])}",
                "  tracking_error_by_group:",
            ]
        )
        for group in (
            "all_positive_dt",
            "same_contact_count",
            "contact_count_transition",
        ):
            lines.append(
                f"    {group}={stats(result['group_errors'][group])}"
            )
        lines.append("  applied_rate_nm_s_by_group:")
        for group in (
            "all_positive_dt",
            "same_contact_count",
            "contact_count_transition",
        ):
            lines.append(
                f"    {group}={stats(result['group_rates'][group])}"
            )
        lines.append(f"  top_tracking_error_n={min(args.top_n, len(result['top_events']))}:")
        for event in result["top_events"][: args.top_n]:
            lines.append(
                "    "
                f"row={event['row']},time_s={event['time_s']:.9g},"
                f"dt_s={event['dt_s']:.9g},"
                f"prev_contacts={event['prev_contacts']},"
                f"contacts={event['contacts']},"
                f"joint={event['joint']},"
                f"tracking_error_nm={event['tracking_error']:.9g}"
            )

    validation_pass = invalid_rows == 0 and negative_dt_pairs == 0
    lines.append("validation=" + ("PASS" if validation_pass else "FAIL"))
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if validation_pass else 1


if __name__ == "__main__":
    sys.exit(main())
