#!/usr/bin/env python3
"""Quantify task and actuator mismatch while a stateful policy holds torque."""

from __future__ import annotations

import argparse
import csv
from bisect import bisect_left
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
TORQUE_FIELDS = tuple(
    f"{leg}_{joint}_tau_ff_candidate"
    for leg in LEGS
    for joint in JOINTS
)
TRUTH_LABELS = tuple(
    f"{leg}_{joint}_joint"
    for leg in LEGS
    for joint in JOINTS
)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} is empty")
    return rows, fields


def finite(row, field):
    value = float(row[field])
    if not value == value or value in (float("inf"), float("-inf")):
        raise ValueError(f"non-finite {field}")
    return value


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(fraction * (len(ordered) - 1))
    return ordered[index]


def torque(row):
    return tuple(finite(row, field) for field in TORQUE_FIELDS)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stateful-detail-csv", type=Path, required=True)
    parser.add_argument("--moment-replay-csv", type=Path, required=True)
    parser.add_argument("--force-only-replay-csv", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--time-tolerance-s", type=float, default=1e-6)
    parser.add_argument("--hold-budget-contact-gap-nm", type=float)
    parser.add_argument("--hold-budget-actuator-delta-nm", type=float)
    args = parser.parse_args()
    if args.time_tolerance_s <= 0.0:
        print("validation=FAIL: invalid time tolerance")
        return 2

    if (
        (args.hold_budget_contact_gap_nm is None)
        != (args.hold_budget_actuator_delta_nm is None)
        or (
            args.hold_budget_contact_gap_nm is not None
            and (
                args.hold_budget_contact_gap_nm <= 0.0
                or args.hold_budget_actuator_delta_nm <= 0.0
            )
        )
    ):
        print("validation=FAIL: invalid hold budget")
        return 2
    hold_budget_enabled = args.hold_budget_contact_gap_nm is not None

    try:
        state_rows, state_fields = read_csv(args.stateful_detail_csv)
        moment_rows, moment_fields = read_csv(args.moment_replay_csv)
        force_rows, force_fields = read_csv(args.force_only_replay_csv)
        truth_rows, truth_fields = read_csv(args.ground_truth_csv)
        required_state = {"row_number", "cmd_time_s", "classification"}
        required_replay = {"row_number", *TORQUE_FIELDS}
        required_truth = {
            "time_s",
            *(
                f"full_mass_qacc_qfrc_qcoord_{label}"
                for label in TRUTH_LABELS
            ),
            *(
                f"full_qfrc_smooth_qcoord_{label}"
                for label in TRUTH_LABELS
            ),
            *(
                f"full_qfrc_constraint_qcoord_{label}"
                for label in TRUTH_LABELS
            ),
            *(
                f"full_qfrc_actuator_qcoord_{label}"
                for label in TRUTH_LABELS
            ),
        }
        missing = []
        if required_state - set(state_fields):
            missing.append(
                "state=" + ",".join(sorted(required_state - set(state_fields)))
            )
        if required_replay - set(moment_fields):
            missing.append(
                "moment=" + ",".join(sorted(required_replay - set(moment_fields)))
            )
        if required_replay - set(force_fields):
            missing.append(
                "force_only=" + ",".join(
                    sorted(required_replay - set(force_fields))
                )
            )
        if required_truth - set(truth_fields):
            missing.append(
                "truth=" + ",".join(sorted(required_truth - set(truth_fields)))
            )
        if missing:
            raise ValueError("missing fields: " + ";".join(missing))
        moment_by_row = {
            int(row["row_number"]): row for row in moment_rows
        }
        force_by_row = {
            int(row["row_number"]): row for row in force_rows
        }
        truth_times = [finite(row, "time_s") for row in truth_rows]
        if any(
            truth_times[i] <= truth_times[i - 1]
            for i in range(1, len(truth_times))
        ):
            raise ValueError("truth time is not strictly increasing")
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    previous_torque = None
    truth_detail = []
    all_contact_gap = []
    all_actuator_delta = []
    hold_contact_gap = []
    hold_actuator_delta = []
    hold_budget_checked_rows = 0
    hold_budget_fail_rows = 0

    for index, state in enumerate(state_rows, start=1):
        try:
            row_number = int(state["row_number"])
            time_s = finite(state, "cmd_time_s")
            moment = moment_by_row[row_number]
            force = force_by_row[row_number]
            classification = state["classification"]
            if classification == "moment":
                selected = torque(moment)
            elif classification == "force-only":
                selected = torque(force)
            elif classification == "hold":
                if previous_torque is None:
                    raise ValueError("hold before first selected torque")
                selected = previous_torque
            else:
                raise ValueError(f"unknown classification {classification}")
            truth_index = bisect_left(truth_times, time_s)
            candidates = [
                candidate
                for candidate in (truth_index - 1, truth_index)
                if 0 <= candidate < len(truth_rows)
            ]
            if not candidates:
                raise ValueError("no matching truth row")
            truth_index = min(
                candidates,
                key=lambda candidate: abs(truth_times[candidate] - time_s),
            )
            time_error = truth_times[truth_index] - time_s
            if abs(time_error) > args.time_tolerance_s:
                raise ValueError("truth time mismatch")
            truth = truth_rows[truth_index]
            contact_gaps = []
            actuator_deltas = []
            for index_joint, label in enumerate(TRUTH_LABELS):
                mass = finite(
                    truth, f"full_mass_qacc_qfrc_qcoord_{label}"
                )
                smooth = finite(truth, f"full_qfrc_smooth_qcoord_{label}")
                contact = finite(
                    truth, f"full_qfrc_constraint_qcoord_{label}"
                )
                actuator = finite(
                    truth, f"full_qfrc_actuator_qcoord_{label}"
                )
                contact_gaps.append(abs(contact - selected[index_joint]))
                required_actuator = mass - smooth - selected[index_joint]
                actuator_deltas.append(
                    abs(required_actuator - actuator)
                )
        except (KeyError, ValueError) as exc:
            print(f"validation=FAIL: row {index}: {exc}")
            return 2

        max_contact_gap = max(contact_gaps)
        max_actuator_delta = max(actuator_deltas)
        all_contact_gap.append(max_contact_gap)
        all_actuator_delta.append(max_actuator_delta)
        hold_budget_pass = ""
        if classification == "hold":
            hold_contact_gap.append(max_contact_gap)
            hold_actuator_delta.append(max_actuator_delta)
            if hold_budget_enabled:
                hold_budget_checked_rows += 1
                hold_budget_pass = int(
                    max_contact_gap <= args.hold_budget_contact_gap_nm
                    and max_actuator_delta
                    <= args.hold_budget_actuator_delta_nm
                )
                if not hold_budget_pass:
                    hold_budget_fail_rows += 1
        truth_detail.append(
            {
                "row_number": row_number,
                "cmd_time_s": f"{time_s:.17g}",
                "truth_time_s": f"{truth_times[truth_index]:.17g}",
                "time_error_s": f"{time_error:.17g}",
                "classification": classification,
                "hold_budget_pass": hold_budget_pass,
                "max_abs_contact_gap_nm": f"{max_contact_gap:.17g}",
                "max_abs_actuator_delta_nm": f"{max_actuator_delta:.17g}",
                "max_abs_selected_torque_nm": f"{max(abs(value) for value in selected):.17g}",
            }
        )
        previous_torque = selected

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(truth_detail[0]))
        writer.writeheader()
        writer.writerows(truth_detail)

    hold_task_not_certified = 1 if hold_contact_gap else 0
    hold_budget_contact_limit = (
        f"{args.hold_budget_contact_gap_nm:.17g}"
        if hold_budget_enabled
        else "none"
    )
    hold_budget_actuator_limit = (
        f"{args.hold_budget_actuator_delta_nm:.17g}"
        if hold_budget_enabled
        else "none"
    )
    hold_budget_summary_pass = (
        "not_evaluated"
        if not hold_budget_enabled
        else str(int(hold_budget_fail_rows == 0))
    )
    lines = [
        "stateful hold candidate audit",
        "interpretation=hold_rate_safe_does_not_imply_task_safe",
        f"rows={len(truth_detail)}",
        f"hold_rows={len(hold_contact_gap)}",
        f"hold_task_not_certified={hold_task_not_certified}",
        f"hold_budget_evaluated={int(hold_budget_enabled)}",
        f"hold_budget_contact_gap_limit_nm={hold_budget_contact_limit}",
        f"hold_budget_actuator_delta_limit_nm={hold_budget_actuator_limit}",
        f"hold_budget_checked_rows={hold_budget_checked_rows}",
        f"hold_budget_fail_rows={hold_budget_fail_rows}",
        f"hold_budget_pass={hold_budget_summary_pass}",
        "hold_budget_is_not_safety_proof=1",
        f"all_contact_gap_p95_nm={percentile(all_contact_gap, 0.95):.17g}",
        f"all_contact_gap_p99_nm={percentile(all_contact_gap, 0.99):.17g}",
        f"all_contact_gap_max_nm={max(all_contact_gap):.17g}",
        f"all_actuator_delta_p95_nm={percentile(all_actuator_delta, 0.95):.17g}",
        f"all_actuator_delta_p99_nm={percentile(all_actuator_delta, 0.99):.17g}",
        f"all_actuator_delta_max_nm={max(all_actuator_delta):.17g}",
        f"hold_contact_gap_p95_nm={percentile(hold_contact_gap, 0.95):.17g}",
        f"hold_contact_gap_p99_nm={percentile(hold_contact_gap, 0.99):.17g}",
        f"hold_contact_gap_max_nm={max(hold_contact_gap, default=0.0):.17g}",
        f"hold_actuator_delta_p95_nm={percentile(hold_actuator_delta, 0.95):.17g}",
        f"hold_actuator_delta_p99_nm={percentile(hold_actuator_delta, 0.99):.17g}",
        f"hold_actuator_delta_max_nm={max(hold_actuator_delta, default=0.0):.17g}",
        "validation=PASS",
    ]
    args.out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
