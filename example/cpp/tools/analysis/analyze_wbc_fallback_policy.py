#!/usr/bin/env python3
"""Audit a conservative offline fallback policy for WBC shadow replay."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def flag(row: dict[str, str], name: str) -> bool:
    return finite(row, name) >= 0.5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--fullbody-detail-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--time-tolerance-s", type=float, default=0.0011)
    args = parser.parse_args()
    if args.time_tolerance_s <= 0.0 or not math.isfinite(args.time_tolerance_s):
        raise SystemExit("time-tolerance-s must be finite and positive")

    try:
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            replay_reader = csv.DictReader(handle)
            replay_fields = replay_reader.fieldnames or []
            replay_rows = list(replay_reader)
        with args.fullbody_detail_csv.open(newline="", encoding="utf-8") as handle:
            fullbody_reader = csv.DictReader(handle)
            fullbody_fields = fullbody_reader.fieldnames or []
            fullbody_rows = list(fullbody_reader)
    except OSError as error:
        raise SystemExit(str(error))

    replay_required = {
        "cmd_time_s",
        "constraint_feasible",
        "contact_count_match",
        "contact_mask_match",
        "shadow_active",
        "shadow_policy_satisfied",
        "shadow_fallback_to_force_solution",
        "shadow_torque_rate_task_active",
        "shadow_torque_rate_satisfied",
    }
    fullbody_required = {
        "replay_time_s",
        "task_gate",
        "feasible_candidate",
        "constraint_feasible",
        "contact_count_match",
        "contact_mask_match",
    }
    missing_replay = sorted(replay_required - set(replay_fields))
    missing_fullbody = sorted(fullbody_required - set(fullbody_fields))
    if missing_replay or missing_fullbody:
        raise SystemExit(
            "missing fields: replay=" + ",".join(missing_replay)
            + " fullbody=" + ",".join(missing_fullbody)
        )
    if len(replay_rows) != len(fullbody_rows):
        raise SystemExit(
            f"row count mismatch: replay={len(replay_rows)} "
            f"fullbody={len(fullbody_rows)}"
        )

    detail_rows: list[dict[str, object]] = []
    counts = Counter()
    reason_counts = Counter()
    longest_hold_run = 0
    current_hold_run = 0
    hold_transition_count = 0
    previous_hold = False
    invalid_rows = 0

    for index, (replay, fullbody) in enumerate(zip(replay_rows, fullbody_rows)):
        reasons: list[str] = []
        try:
            replay_time = finite(replay, "cmd_time_s")
            fullbody_time = finite(fullbody, "replay_time_s")
            time_error = fullbody_time - replay_time
            if abs(time_error) > args.time_tolerance_s:
                reasons.append("time_match")
            shadow_active = flag(replay, "shadow_active")
            if not shadow_active:
                reasons.append("shadow_inactive")
            replay_constraint = flag(replay, "constraint_feasible")
            if not replay_constraint:
                reasons.append("replay_constraint")
            if not flag(replay, "contact_count_match"):
                reasons.append("contact_count")
            if not flag(replay, "contact_mask_match"):
                reasons.append("contact_mask")
            rate_active = flag(replay, "shadow_torque_rate_task_active")
            rate_satisfied = flag(replay, "shadow_torque_rate_satisfied")
            if rate_active and not rate_satisfied:
                reasons.append("torque_rate")
            if not flag(fullbody, "constraint_feasible"):
                reasons.append("fullbody_constraint")
            if not flag(fullbody, "contact_count_match"):
                reasons.append("fullbody_contact_count")
            if not flag(fullbody, "contact_mask_match"):
                reasons.append("fullbody_contact_mask")
            if not flag(fullbody, "task_gate"):
                reasons.append("fullbody_task_gate")
            if not flag(fullbody, "feasible_candidate"):
                reasons.append("fullbody_candidate")
            policy_satisfied = flag(replay, "shadow_policy_satisfied")
            fallback = flag(replay, "shadow_fallback_to_force_solution")
        except (KeyError, ValueError) as error:
            invalid_rows += 1
            reasons.append("invalid:" + str(error))
            replay_time = float("nan")
            fullbody_time = float("nan")
            time_error = float("nan")
            shadow_active = False
            rate_active = False
            rate_satisfied = False
            policy_satisfied = False
            fallback = False

        hard_failure = bool(reasons)
        if hard_failure:
            classification = "hold"
        elif policy_satisfied:
            classification = "pass"
        elif fallback:
            classification = "force_only"
        else:
            classification = "hold"
            reasons.append("no_force_fallback")
        reason_text = "|".join(reasons) if reasons else "none"
        counts[classification] += 1
        for reason in reasons:
            reason_counts[reason] += 1
        is_hold = classification == "hold"
        if is_hold:
            current_hold_run += 1
            longest_hold_run = max(longest_hold_run, current_hold_run)
            if not previous_hold:
                hold_transition_count += 1
        else:
            current_hold_run = 0
        previous_hold = is_hold
        detail_rows.append(
            {
                "row_index": index,
                "cmd_time_s": replay_time,
                "fullbody_time_s": fullbody_time,
                "time_error_s": time_error,
                "classification": classification,
                "reasons": reason_text,
                "shadow_policy_satisfied": int(policy_satisfied),
                "shadow_fallback_to_force_solution": int(fallback),
                "shadow_torque_rate_task_active": int(rate_active),
                "shadow_torque_rate_satisfied": int(rate_satisfied),
                "replay_constraint_feasible": int(
                    not any(reason == "replay_constraint" for reason in reasons)
                ),
                "fullbody_task_gate": int(
                    not any(reason == "fullbody_task_gate" for reason in reasons)
                ),
                "fullbody_feasible_candidate": int(
                    not any(reason == "fullbody_candidate" for reason in reasons)
                ),
            }
        )

    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(detail_rows[0]))
        writer.writeheader()
        writer.writerows(detail_rows)

    lines = [
        "offline WBC fallback policy audit",
        "policy=pass_requires_shadow_policy_rate_and_fullbody_feasibility",
        "policy=force_only_requires_no_hard_failure_and_allocator_force_fallback",
        "policy=hold_on_constraint_contact_rate_or_fullbody_failure",
        f"replay_rows={len(replay_rows)}",
        f"fullbody_rows={len(fullbody_rows)}",
        f"invalid_rows={invalid_rows}",
        f"pass_rows={counts['pass']}",
        f"force_only_rows={counts['force_only']}",
        f"hold_rows={counts['hold']}",
        f"longest_hold_run={longest_hold_run}",
        f"hold_transition_count={hold_transition_count}",
    ]
    for reason, count in sorted(reason_counts.items()):
        lines.append(f"reason_{reason}_rows={count}")
    lines.extend(
        [
            "validation="
            + (
                "PASS"
                if invalid_rows == 0 and sum(counts.values()) == len(replay_rows)
                else "FAIL"
            ),
            "interpretation=offline_diagnostic_not_runtime_safety_proof",
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if invalid_rows == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
