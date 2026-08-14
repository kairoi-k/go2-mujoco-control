#!/usr/bin/env python3
"""Audit an explicit rate-aware force-only fallback against a moment replay."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path


def read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)):
            raise ValueError(f"{path} has duplicate fields")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} is empty")
    return rows, fields


def finite(row: dict[str, str], field: str) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field}")
    return value


def flag(row: dict[str, str], field: str) -> bool:
    return finite(row, field) >= 0.5


def rate_ok(row: dict[str, str]) -> bool:
    return (
        not flag(row, "shadow_torque_rate_task_active")
        or flag(row, "shadow_torque_rate_satisfied")
    )


def candidate_ok(
    replay: dict[str, str],
    fullbody: dict[str, str],
) -> bool:
    return (
        flag(replay, "shadow_policy_satisfied")
        and rate_ok(replay)
        and flag(fullbody, "task_gate")
        and flag(fullbody, "feasible_candidate")
    )


def candidate_reasons(
    replay: dict[str, str],
    fullbody: dict[str, str],
    prefix: str,
) -> list[str]:
    reasons: list[str] = []
    if not flag(replay, "shadow_policy_satisfied"):
        reasons.append(prefix + "_shadow_policy")
    if not rate_ok(replay):
        reasons.append(prefix + "_torque_rate")
    if not flag(fullbody, "task_gate"):
        reasons.append(prefix + "_fullbody_task_gate")
    if not flag(fullbody, "feasible_candidate"):
        reasons.append(prefix + "_fullbody_candidate")
    return reasons


def longest_run(values: list[str], target: str) -> int:
    longest = 0
    current = 0
    for value in values:
        if value == target:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def transition_count(values: list[str]) -> int:
    return sum(
        values[index] != values[index - 1]
        for index in range(1, len(values))
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a six-dimensional rate replay with a separately "
            "solved force-only rate replay."
        )
    )
    parser.add_argument("--moment-replay-csv", type=Path, required=True)
    parser.add_argument(
        "--moment-fullbody-detail-csv",
        type=Path,
        required=True,
    )
    parser.add_argument("--force-only-replay-csv", type=Path, required=True)
    parser.add_argument(
        "--force-only-fullbody-detail-csv",
        type=Path,
        required=True,
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--time-tolerance-s", type=float, default=1e-9)
    args = parser.parse_args()

    if args.time_tolerance_s <= 0.0:
        print("validation=FAIL: time tolerance must be positive")
        return 2

    try:
        moment_rows, moment_fields = read_csv(args.moment_replay_csv)
        moment_full_rows, moment_full_fields = read_csv(
            args.moment_fullbody_detail_csv
        )
        force_rows, force_fields = read_csv(args.force_only_replay_csv)
        force_full_rows, force_full_fields = read_csv(
            args.force_only_fullbody_detail_csv
        )
        required_replay = {
            "row_number",
            "cmd_time_s",
            "shadow_policy_satisfied",
            "shadow_torque_rate_task_active",
            "shadow_torque_rate_satisfied",
        }
        required_fullbody = {
            "replay_time_s",
            "task_gate",
            "feasible_candidate",
        }
        for name, fields in (
            ("moment replay", moment_fields),
            ("force-only replay", force_fields),
        ):
            missing = sorted(required_replay - set(fields))
            if missing:
                raise ValueError(
                    f"{name} missing fields: {','.join(missing)}"
                )
        for name, fields in (
            ("moment fullbody", moment_full_fields),
            ("force-only fullbody", force_full_fields),
        ):
            missing = sorted(required_fullbody - set(fields))
            if missing:
                raise ValueError(
                    f"{name} missing fields: {','.join(missing)}"
                )
        lengths = {
            len(moment_rows),
            len(moment_full_rows),
            len(force_rows),
            len(force_full_rows),
        }
        if len(lengths) != 1:
            raise ValueError(
                "replay and fullbody detail row counts do not match"
            )
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    detail_rows: list[dict[str, object]] = []
    classifications: list[str] = []
    for index, (moment, moment_full, force, force_full) in enumerate(
        zip(moment_rows, moment_full_rows, force_rows, force_full_rows),
        start=1,
    ):
        try:
            moment_time = finite(moment, "cmd_time_s")
            force_time = finite(force, "cmd_time_s")
            moment_full_time = finite(moment_full, "replay_time_s")
            force_full_time = finite(force_full, "replay_time_s")
            if (
                abs(moment_time - force_time) > args.time_tolerance_s
                or abs(moment_time - moment_full_time)
                > args.time_tolerance_s
                or abs(force_time - force_full_time)
                > args.time_tolerance_s
            ):
                raise ValueError("time alignment mismatch")
            if moment["row_number"] != force["row_number"]:
                raise ValueError("replay row number mismatch")
            moment_pass = candidate_ok(moment, moment_full)
            force_pass = candidate_ok(force, force_full)
        except (KeyError, ValueError) as exc:
            print(f"validation=FAIL: row {index}: {exc}")
            return 2

        if moment_pass:
            classification = "moment"
        elif force_pass:
            classification = "force-only"
        else:
            classification = "hold"
        classifications.append(classification)
        reasons = []
        if not moment_pass:
            reasons.extend(
                candidate_reasons(moment, moment_full, "moment")
            )
        if classification == "hold":
            reasons.extend(
                candidate_reasons(force, force_full, "force_only")
            )
        detail_rows.append(
            {
                "row_number": moment["row_number"],
                "cmd_time_s": f"{moment_time:.17g}",
                "moment_pass": int(moment_pass),
                "force_only_pass": int(force_pass),
                "classification": classification,
                "reasons": "|".join(reasons),
                "moment_shadow_policy_satisfied": int(
                    flag(moment, "shadow_policy_satisfied")
                ),
                "moment_torque_rate_satisfied": int(
                    rate_ok(moment)
                ),
                "moment_fullbody_task_gate": int(
                    flag(moment_full, "task_gate")
                ),
                "moment_fullbody_candidate": int(
                    flag(moment_full, "feasible_candidate")
                ),
                "force_only_shadow_policy_satisfied": int(
                    flag(force, "shadow_policy_satisfied")
                ),
                "force_only_torque_rate_satisfied": int(
                    rate_ok(force)
                ),
                "force_only_fullbody_task_gate": int(
                    flag(force_full, "task_gate")
                ),
                "force_only_fullbody_candidate": int(
                    flag(force_full, "feasible_candidate")
                ),
            }
        )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    counts = Counter(classifications)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(detail_rows[0]))
        writer.writeheader()
        writer.writerows(detail_rows)
    with args.out.open("w", encoding="utf-8") as handle:
        handle.write("rate-aware fallback audit\n")
        handle.write(
            "policy=moment_first_then_explicit_force_only_then_hold\n"
        )
        handle.write(f"replay_rows={len(detail_rows)}\n")
        handle.write(f"moment_pass_rows={counts['moment']}\n")
        handle.write(f"force_only_rows={counts['force-only']}\n")
        handle.write(f"hold_rows={counts['hold']}\n")
        handle.write(
            f"longest_hold_run={longest_run(classifications, 'hold')}\n"
        )
        handle.write(
            f"hold_transition_count={transition_count(classifications)}\n"
        )
        handle.write("validation=PASS\n")
        handle.write(
            "interpretation=offline_policy_audit_not_runtime_safety_proof\n"
        )
    print("rate-aware fallback audit")
    print("policy=moment_first_then_explicit_force_only_then_hold")
    print(f"replay_rows={len(detail_rows)}")
    print(f"moment_pass_rows={counts['moment']}")
    print(f"force_only_rows={counts['force-only']}")
    print(f"hold_rows={counts['hold']}")
    print(f"longest_hold_run={longest_run(classifications, 'hold')}")
    print(f"hold_transition_count={transition_count(classifications)}")
    print("validation=PASS")
    print("interpretation=offline_policy_audit_not_runtime_safety_proof")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
