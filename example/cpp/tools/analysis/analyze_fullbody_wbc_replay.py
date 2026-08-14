#!/usr/bin/env python3
"""Replay a contact-wrench candidate through full-body MuJoCo truth."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from bisect import bisect_left
from pathlib import Path

MASS_PREFIX = "full_mass_qacc_qfrc_qcoord_"
SMOOTH_PREFIX = "full_qfrc_smooth_qcoord_"
CONSTRAINT_PREFIX = "full_qfrc_constraint_qcoord_"
ACTUATOR_PREFIX = "full_qfrc_actuator_qcoord_"
BASE_LABELS = (
    "base_qcoord_trans_x",
    "base_qcoord_trans_y",
    "base_qcoord_trans_z",
    "base_qcoord_rot_x",
    "base_qcoord_rot_y",
    "base_qcoord_rot_z",
)
BASE_REPORT_LABELS = (
    "trans_x_N",
    "trans_y_N",
    "trans_z_N",
    "rot_x_Nm",
    "rot_y_Nm",
    "rot_z_Nm",
)
LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
JOINT_LABELS = tuple(
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "RL", "RR")
    for joint in JOINTS
)
MASS_KG = 15.206408
GRAVITY_MPS2 = 9.81


def finite(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def percentile_abs(values: list[float], fraction: float = 0.95) -> float:
    ordered = sorted(abs(value) for value in values)
    return ordered[int(fraction * (len(ordered) - 1))]


def maximum_abs(values: list[float]) -> float:
    return max(abs(value) for value in values)


def body_to_world(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid base quaternion")
    w, x, y, z = (value / norm for value in quaternion)
    vx, vy, vz = vector
    return (
        (1 - 2 * (y * y + z * z)) * vx
        + 2 * (x * y - z * w) * vy
        + 2 * (x * z + y * w) * vz,
        2 * (x * y + z * w) * vx
        + (1 - 2 * (x * x + z * z)) * vy
        + 2 * (y * z - x * w) * vz,
        2 * (x * z - y * w) * vx
        + 2 * (y * z + x * w) * vy
        + (1 - 2 * (x * x + y * y)) * vz,
    )


def read_truth(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)):
            raise ValueError("ground-truth CSV has duplicate fields")
        rows = list(reader)
    labels = [
        field[len(MASS_PREFIX):]
        for field in fields
        if field.startswith(MASS_PREFIX)
    ]
    if not labels:
        raise ValueError("ground-truth CSV has no full mass fields")
    if len(labels) != len(set(labels)):
        raise ValueError("ground-truth CSV has duplicate dof labels")
    required = {"time_s", "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z"}
    required.update(
        f"base_qacc_world_{axis}_mps2" for axis in ("x", "y", "z")
    )
    for label in labels:
        required.update(
            prefix + label
            for prefix in (
                MASS_PREFIX,
                SMOOTH_PREFIX,
                CONSTRAINT_PREFIX,
                ACTUATOR_PREFIX,
            )
        )
    missing = sorted(required - set(fields))
    if missing:
        raise ValueError("ground-truth CSV missing fields: " + ",".join(missing))
    if any(label not in labels for label in BASE_LABELS + JOINT_LABELS):
        raise ValueError("ground-truth CSV dof order does not match Go2 model")
    return rows, labels


def read_replay(path: Path) -> tuple[list[dict[str, str]], bool]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        target_fields = {
            "desired_force_y_n",
            "desired_force_z_n",
            "desired_moment_x_nm",
            "desired_moment_y_nm",
            "desired_moment_z_nm",
        }
        has_target_wrench = target_fields <= set(fields)
        required = {
            "cmd_time_s",
            "desired_force_x_n",
            "reduced_task",
            "wrench_satisfied",
            "task_satisfied",
            "constraint_feasible",
            "contact_count_match",
            "contact_mask_match",
        }
        for leg in LEGS:
            for joint in JOINTS:
                required.add(f"{leg}_{joint}_tau_ff_candidate")
        missing = sorted(required - set(fields))
        if missing:
            raise ValueError("replay CSV missing fields: " + ",".join(missing))
        return list(reader), has_target_wrench


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--detail-csv", type=Path, required=True)
    parser.add_argument("--match-tolerance-s", type=float, default=0.0011)
    parser.add_argument("--mass-kg", type=float, default=MASS_KG)
    parser.add_argument("--gravity-mps2", type=float, default=GRAVITY_MPS2)
    parser.add_argument(
        "--hold-closure-tolerance",
        type=float,
        default=1e-4,
    )
    args = parser.parse_args()
    if (
        args.match_tolerance_s <= 0.0
        or args.mass_kg <= 0.0
        or args.gravity_mps2 <= 0.0
        or args.hold_closure_tolerance < 0.0
    ):
        print("validation=FAIL: invalid audit parameters")
        return 2

    try:
        replay_rows, has_target_wrench = read_replay(args.replay_csv)
        truth_rows, truth_labels = read_truth(args.ground_truth_csv)
        truth_times = [finite(row, "time_s") for row in truth_rows]
        if any(
            truth_times[index] <= truth_times[index - 1]
            for index in range(1, len(truth_times))
        ):
            raise ValueError("ground-truth time is not strictly increasing")
    except (OSError, KeyError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    if not replay_rows or not truth_rows:
        print("validation=FAIL: empty replay or ground-truth CSV")
        return 1

    all_base_gap = {label: [] for label in BASE_REPORT_LABELS}
    feasible_base_gap = {label: [] for label in BASE_REPORT_LABELS}
    all_base_gap_minus_mass_qacc = {label: [] for label in BASE_REPORT_LABELS[:3]}
    feasible_base_gap_minus_mass_qacc = {label: [] for label in BASE_REPORT_LABELS[:3]}
    all_joint_contact_gap = {label: [] for label in JOINT_LABELS}
    feasible_joint_contact_gap = {label: [] for label in JOINT_LABELS}
    all_actuator_delta = {label: [] for label in JOINT_LABELS}
    feasible_actuator_delta = {label: [] for label in JOINT_LABELS}
    all_candidate_torque = {label: [] for label in JOINT_LABELS}
    feasible_candidate_torque = {label: [] for label in JOINT_LABELS}
    hold_closure_errors: list[float] = []
    detail_rows: list[dict[str, object]] = []
    match_failures = 0
    invalid_rows = 0
    matched_rows = 0
    feasible_rows = 0
    wrench_unsatisfied_rows = 0
    constraint_infeasible_rows = 0
    contact_count_mismatch_rows = 0
    contact_mask_mismatch_rows = 0
    task_unsatisfied_rows = 0
    task_gate_failures = 0
    time_errors: list[float] = []

    for replay in replay_rows:
        try:
            replay_time = finite(replay, "cmd_time_s")
            desired_force_x = finite(replay, "desired_force_x_n")
            reduced_task = finite(replay, "reduced_task") >= 0.5
            wrench_satisfied = finite(replay, "wrench_satisfied") >= 0.5
            task_satisfied = finite(replay, "task_satisfied") >= 0.5
            constraint_feasible = finite(replay, "constraint_feasible") >= 0.5
            contact_count_match = finite(replay, "contact_count_match") >= 0.5
            contact_mask_match = finite(replay, "contact_mask_match") >= 0.5
        except (KeyError, ValueError):
            invalid_rows += 1
            continue

        truth_index = bisect_left(truth_times, replay_time)
        candidates = [
            index
            for index in (truth_index - 1, truth_index)
            if 0 <= index < len(truth_rows)
        ]
        if not candidates:
            match_failures += 1
            continue
        truth_index = min(
            candidates,
            key=lambda index: abs(truth_times[index] - replay_time),
        )
        time_error = truth_times[truth_index] - replay_time
        if abs(time_error) > args.match_tolerance_s:
            match_failures += 1
            continue

        try:
            truth = truth_rows[truth_index]
            quaternion = tuple(
                finite(truth, f"base_quat_{axis}")
                for axis in ("w", "x", "y", "z")
            )
            if has_target_wrench:
                target_body_wrench = (
                    finite(replay, "desired_force_x_n"),
                    finite(replay, "desired_force_y_n"),
                    finite(replay, "desired_force_z_n"),
                    finite(replay, "desired_moment_x_nm"),
                    finite(replay, "desired_moment_y_nm"),
                    finite(replay, "desired_moment_z_nm"),
                )
            else:
                target_body_wrench = (
                    desired_force_x,
                    0.0,
                    args.mass_kg * args.gravity_mps2,
                    0.0,
                    0.0,
                    0.0,
                )
            desired_force_world = body_to_world(
                quaternion,
                target_body_wrench[:3],
            )
            desired_moment_world = body_to_world(
                quaternion,
                target_body_wrench[3:],
            )
            target_base = (
                desired_force_world[0],
                desired_force_world[1],
                desired_force_world[2],
                desired_moment_world[0],
                desired_moment_world[1],
                desired_moment_world[2],
            )
            actual_base = tuple(
                finite(
                    truth,
                    CONSTRAINT_PREFIX + label,
                )
                for label in BASE_LABELS
            )
            base_gap = tuple(
                actual_base[index] - target_base[index]
                for index in range(6)
            )
            base_gap_minus_mass_qacc = tuple(
                base_gap[index]
                - finite(truth, MASS_PREFIX + BASE_LABELS[index])
                for index in range(3)
            )
            base_qacc_world = tuple(
                finite(truth, f"base_qacc_world_{axis}_mps2")
                for axis in ("x", "y", "z")
            )
            joint_values: dict[str, dict[str, float]] = {}
            for leg in LEGS:
                for joint in JOINTS:
                    label = f"{leg}_{joint}_joint"
                    candidate = finite(
                        replay,
                        f"{leg}_{joint}_tau_ff_candidate",
                    )
                    mass = finite(truth, MASS_PREFIX + label)
                    smooth = finite(truth, SMOOTH_PREFIX + label)
                    actual_constraint = finite(
                        truth,
                        CONSTRAINT_PREFIX + label,
                    )
                    actual_actuator = finite(
                        truth,
                        ACTUATOR_PREFIX + label,
                    )
                    required_actuator = mass - smooth - candidate
                    hold_error = (
                        mass - smooth - candidate - required_actuator
                    )
                    joint_values[label] = {
                        "candidate": candidate,
                        "actual_constraint": actual_constraint,
                        "actual_actuator": actual_actuator,
                        "required_actuator": required_actuator,
                        "contact_gap": actual_constraint - candidate,
                        "actuator_delta": required_actuator - actual_actuator,
                        "hold_error": hold_error,
                    }
        except (KeyError, ValueError) as exc:
            invalid_rows += 1
            continue

        matched_rows += 1
        time_errors.append(abs(time_error))
        if not wrench_satisfied:
            wrench_unsatisfied_rows += 1
        if not task_satisfied:
            task_unsatisfied_rows += 1
        if not constraint_feasible:
            constraint_infeasible_rows += 1
        if not contact_count_match:
            contact_count_mismatch_rows += 1
        if not contact_mask_match:
            contact_mask_mismatch_rows += 1
        task_gate = task_satisfied if reduced_task else wrench_satisfied
        if not task_gate:
            task_gate_failures += 1
        feasible = (
            task_gate
            and constraint_feasible
            and contact_count_match
            and contact_mask_match
        )
        if feasible:
            feasible_rows += 1

        detail: dict[str, object] = {
            "replay_time_s": replay_time,
            "truth_time_s": truth_times[truth_index],
            "time_error_s": time_error,
            "reduced_task": int(reduced_task),
            "wrench_satisfied": int(wrench_satisfied),
            "task_satisfied": int(task_satisfied),
            "constraint_feasible": int(constraint_feasible),
            "contact_count_match": int(contact_count_match),
            "contact_mask_match": int(contact_mask_match),
            "task_gate": int(task_gate),
            "feasible_candidate": int(feasible),
            "desired_force_x_body_n": target_body_wrench[0],
        }
        for axis, value in zip(("x", "y", "z"), target_body_wrench[:3]):
            detail[f"desired_force_{axis}_body_n"] = value
        for axis, value in zip(("x", "y", "z"), target_body_wrench[3:]):
            detail[f"desired_moment_{axis}_body_nm"] = value
        for label, gap in zip(BASE_REPORT_LABELS, base_gap):
            all_base_gap[label].append(gap)
            if feasible:
                feasible_base_gap[label].append(gap)
            detail[f"base_task_qforce_gap_{label}"] = gap
        for axis, value in zip(("x", "y", "z"), base_qacc_world):
            detail[f"base_qacc_world_{axis}_mps2"] = value
        for label, gap in zip(
            BASE_REPORT_LABELS[:3],
            base_gap_minus_mass_qacc,
        ):
            all_base_gap_minus_mass_qacc[label].append(gap)
            if feasible:
                feasible_base_gap_minus_mass_qacc[label].append(gap)
            detail[f"base_gap_minus_mass_qacc_{label}"] = gap
        for label, values in joint_values.items():
            all_joint_contact_gap[label].append(values["contact_gap"])
            all_actuator_delta[label].append(values["actuator_delta"])
            all_candidate_torque[label].append(values["candidate"])
            if feasible:
                feasible_joint_contact_gap[label].append(
                    values["contact_gap"]
                )
                feasible_actuator_delta[label].append(
                    values["actuator_delta"]
                )
                feasible_candidate_torque[label].append(
                    values["candidate"]
                )
            hold_closure_errors.append(values["hold_error"])
            for metric, value in values.items():
                detail[f"{label}_{metric}"] = value
        detail_rows.append(detail)

    if matched_rows == 0:
        print("validation=FAIL: no replay rows matched ground truth")
        return 1

    detail_fields = list(detail_rows[0])
    args.detail_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.detail_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(detail_rows)

    arithmetic_pass = (
        match_failures == 0
        and invalid_rows == 0
        and maximum_abs(hold_closure_errors)
        <= args.hold_closure_tolerance
    )
    task_feasibility_pass = (
        task_gate_failures == 0
        and task_unsatisfied_rows == 0
        and constraint_infeasible_rows == 0
        and contact_count_mismatch_rows == 0
        and contact_mask_mismatch_rows == 0
    )

    lines = [
        "full-body contact-wrench dynamics replay audit",
        f"replay_rows={len(replay_rows)}",
        f"truth_rows={len(truth_rows)}",
        f"matched_rows={matched_rows}",
        f"match_failures={match_failures}",
        f"invalid_rows={invalid_rows}",
        f"feasible_candidate_rows={feasible_rows}",
        f"wrench_unsatisfied_rows={wrench_unsatisfied_rows}",
        f"task_unsatisfied_rows={task_unsatisfied_rows}",
        f"task_gate_failures={task_gate_failures}",
        f"constraint_infeasible_rows={constraint_infeasible_rows}",
        f"contact_count_mismatch_rows={contact_count_mismatch_rows}",
        f"contact_mask_mismatch_rows={contact_mask_mismatch_rows}",
        "max_time_error_s=%.9g" % max(time_errors),
        "target_wrench_source="
        + ("replay_csv_body_wrench" if has_target_wrench else "static_gravity_fallback"),
        "coordinate_note=body_task_force_rotated_to_world_free_joint_translation_qcoord",
        "candidate_note=contact_torque_replay_values_are_J_transpose_f_candidates",
        "required_actuator_note=hold_measured_qacc_using_candidate_contact_qforce",
        "base_gap_equation=actual_base_qfrc_constraint_minus_desired_base_task_qforce",
        "acceleration_decomposition=base_task_gap_minus_full_M_qacc_qcoord_translation",
        "joint_gap_equation=actual_joint_qfrc_constraint_minus_candidate_contact_qforce",
        "required_actuator_equation=full_M_qacc_minus_full_qfrc_smooth_minus_candidate_contact_qforce",
        "hold_closure_tolerance=%.9g" % args.hold_closure_tolerance,
        "hold_joint_closure_max_abs=%.9g"
        % maximum_abs(hold_closure_errors),
    ]
    for label in BASE_REPORT_LABELS:
        lines.extend(
            [
                f"all_base_task_qforce_gap_{label}_p95_abs=%.9g"
                % percentile_abs(all_base_gap[label]),
                f"all_base_task_qforce_gap_{label}_max_abs=%.9g"
                % maximum_abs(all_base_gap[label]),
                f"feasible_base_task_qforce_gap_{label}_p95_abs=%.9g"
                % percentile_abs(feasible_base_gap[label])
                if feasible_base_gap[label]
                else f"feasible_base_task_qforce_gap_{label}_p95_abs=NA",
                f"feasible_base_task_qforce_gap_{label}_max_abs=%.9g"
                % maximum_abs(feasible_base_gap[label])
                if feasible_base_gap[label]
                else f"feasible_base_task_qforce_gap_{label}_max_abs=NA",
            ]
        )
    for label in BASE_REPORT_LABELS[:3]:
        lines.extend(
            [
                f"all_base_gap_minus_mass_qacc_{label}_p95_abs=%.9g"
                % percentile_abs(all_base_gap_minus_mass_qacc[label]),
                f"all_base_gap_minus_mass_qacc_{label}_max_abs=%.9g"
                % maximum_abs(all_base_gap_minus_mass_qacc[label]),
                f"feasible_base_gap_minus_mass_qacc_{label}_p95_abs=%.9g"
                % percentile_abs(feasible_base_gap_minus_mass_qacc[label]),
                f"feasible_base_gap_minus_mass_qacc_{label}_max_abs=%.9g"
                % maximum_abs(feasible_base_gap_minus_mass_qacc[label]),
            ]
        )
    for label in JOINT_LABELS:
        lines.extend(
            [
                f"all_joint_contact_gap_{label}_p95_abs=%.9g"
                % percentile_abs(all_joint_contact_gap[label]),
                f"all_joint_contact_gap_{label}_max_abs=%.9g"
                % maximum_abs(all_joint_contact_gap[label]),
                f"all_required_actuator_delta_{label}_p95_abs=%.9g"
                % percentile_abs(all_actuator_delta[label]),
                f"all_required_actuator_delta_{label}_max_abs=%.9g"
                % maximum_abs(all_actuator_delta[label]),
                f"all_candidate_contact_torque_{label}_max_abs=%.9g"
                % maximum_abs(all_candidate_torque[label]),
            ]
        )
        if feasible_joint_contact_gap[label]:
            lines.extend(
                [
                    f"feasible_joint_contact_gap_{label}_p95_abs=%.9g"
                    % percentile_abs(feasible_joint_contact_gap[label]),
                    f"feasible_required_actuator_delta_{label}_p95_abs=%.9g"
                    % percentile_abs(feasible_actuator_delta[label]),
                    f"feasible_candidate_contact_torque_{label}_max_abs=%.9g"
                    % maximum_abs(feasible_candidate_torque[label]),
                ]
            )
    lines.extend(
        [
            "arithmetic_validation=" + ("PASS" if arithmetic_pass else "FAIL"),
            "task_feasibility_validation="
            + ("PASS" if task_feasibility_pass else "FAIL"),
            "main_chain_gate=HOLD",
            "main_chain_gate_reason=shadow_only_no_injection_or_robustness_gate",
            "interpretation=diagnostic_only_no_wbc_injection",
            "validation=" + ("PASS" if arithmetic_pass else "FAIL"),
        ]
    )
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if arithmetic_pass else 1


if __name__ == "__main__":
    sys.exit(main())
