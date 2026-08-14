#!/usr/bin/env python3
"""Recompute contact wrench after an offline candidate-torque rate limit."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
TAU_FIELDS = tuple(
    f"{leg}_{joint}_tau_ff_candidate" for leg in LEGS for joint in JOINTS
)
CONTACT_FIELDS = tuple(f"contact_{leg}" for leg in LEGS)
Q_FIELDS = tuple(f"{leg}_{joint}_q_state" for leg in LEGS for joint in JOINTS)
TIME_EPSILON_S = 1e-12
MAPPING_TOLERANCE = 1e-5


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


def geometry(leg: int) -> tuple[float, float, float, float, float]:
    front = leg in (0, 1)
    left = leg in (1, 3)
    side_sign = 1.0 if left else -1.0
    return (
        0.1934 if front else -0.1934,
        side_sign * 0.0465,
        side_sign * 0.0955,
        0.213,
        0.213,
    )


def foot_position(leg: int, q: tuple[float, float, float]) -> tuple[float, float, float]:
    hip_x, hip_y, hip_link_y, thigh, calf = geometry(leg)
    q_hip, q_thigh, q_calf = q
    sin_hip, cos_hip = math.sin(q_hip), math.cos(q_hip)
    sin_thigh, cos_thigh = math.sin(q_thigh), math.cos(q_thigh)
    sin_lower = math.sin(q_thigh + q_calf)
    cos_lower = math.cos(q_thigh + q_calf)
    leg_x = -thigh * sin_thigh - calf * sin_lower
    leg_z = -thigh * cos_thigh - calf * cos_lower
    lateral_y = cos_hip * hip_link_y - sin_hip * leg_z
    lateral_z = sin_hip * hip_link_y + cos_hip * leg_z
    return hip_x + leg_x, hip_y + lateral_y, lateral_z


def foot_jacobian(
    leg: int,
    q: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    _, _, hip_link_y, thigh, calf = geometry(leg)
    q_hip, q_thigh, q_calf = q
    lower_angle = q_thigh + q_calf
    sin_hip, cos_hip = math.sin(q_hip), math.cos(q_hip)
    sin_thigh, cos_thigh = math.sin(q_thigh), math.cos(q_thigh)
    sin_lower, cos_lower = math.sin(lower_angle), math.cos(lower_angle)
    leg_z = -thigh * cos_thigh - calf * cos_lower
    lower_z = -calf * cos_lower
    d_leg_z_d_thigh = thigh * sin_thigh + calf * sin_lower
    d_leg_z_d_calf = calf * sin_lower
    lateral_y = cos_hip * hip_link_y - sin_hip * leg_z
    return (
        (0.0, leg_z, lower_z),
        (
            -sin_hip * hip_link_y - cos_hip * leg_z,
            -sin_hip * d_leg_z_d_thigh,
            -sin_hip * d_leg_z_d_calf,
        ),
        (
            lateral_y,
            cos_hip * d_leg_z_d_thigh,
            cos_hip * d_leg_z_d_calf,
        ),
    )


def solve_transpose(
    jacobian: tuple[tuple[float, float, float], ...],
    torque: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    matrix = tuple(
        tuple(jacobian[column][row] for column in range(3))
        for row in range(3)
    )
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(determinant) <= 1e-10:
        return None
    inverse = (
        (
            (e * i - f * h) / determinant,
            (c * h - b * i) / determinant,
            (b * f - c * e) / determinant,
        ),
        (
            (f * g - d * i) / determinant,
            (a * i - c * g) / determinant,
            (c * d - a * f) / determinant,
        ),
        (
            (d * h - e * g) / determinant,
            (b * g - a * h) / determinant,
            (a * e - b * d) / determinant,
        ),
    )
    return tuple(
        sum(inverse[row][column] * torque[column] for column in range(3))
        for row in range(3)
    )


def wrench_from_forces(
    positions: tuple[tuple[float, float, float], ...],
    forces: tuple[tuple[float, float, float], ...],
    contacts: tuple[bool, ...],
) -> tuple[float, float, float, float, float, float]:
    force_x = force_y = force_z = 0.0
    moment_x = moment_y = moment_z = 0.0
    for position, force, active in zip(positions, forces, contacts):
        if not active:
            continue
        px, py, pz = position
        fx, fy, fz = force
        force_x += fx
        force_y += fy
        force_z += fz
        moment_x += py * fz - pz * fy
        moment_y += pz * fx - px * fz
        moment_z += px * fy - py * fx
    return force_x, force_y, force_z, moment_x, moment_y, moment_z


def clamp_step(previous: float, target: float, limit: float) -> float:
    return previous + max(-limit, min(limit, target - previous))


def run_limit(
    rows: list[dict[str, object]],
    state_rows: list[dict[str, str]],
    rate_limit: float,
) -> dict[str, object]:
    previous_time = rows[0]["time_s"]
    previous_contacts = rows[0]["contacts"]
    limited_tau = list(rows[0]["tau"])
    tracking_errors: list[float] = []
    applied_rates: list[float] = []
    wrench_norms: list[float] = []
    force_norms: list[float] = []
    moment_norms: list[float] = []
    mapping_errors: list[float] = []
    group_wrench: defaultdict[str, list[float]] = defaultdict(list)
    group_tracking: defaultdict[str, list[float]] = defaultdict(list)
    rate_limited_pairs = 0
    duplicate_pairs = 0
    inverse_failures = 0
    top_events: list[dict[str, object]] = []

    for row in rows:
        state = state_rows[row["row_number"] - 1]
        q = tuple(
            tuple(
                finite(state, f"{leg}_{joint}_q_state")
                for joint in JOINTS
            )
            for leg in LEGS
        )
        contacts = tuple(
            finite(state, name) >= 0.5 for name in CONTACT_FIELDS
        )
        positions = tuple(
            foot_position(leg, q[leg]) for leg in range(len(LEGS))
        )
        candidate_tau = tuple(row["tau"])
        previous_limited_tau = list(limited_tau)
        dt_s = row["time_s"] - previous_time
        if row is rows[0]:
            limited_tau = list(candidate_tau)
        elif dt_s <= TIME_EPSILON_S:
            duplicate_pairs += 1
            limited_tau = list(candidate_tau)
        else:
            allowed_delta = rate_limit * dt_s
            limited_tau = [
                clamp_step(previous, target, allowed_delta)
                for previous, target in zip(limited_tau, candidate_tau)
            ]

        tracking_error = max(
            (
                abs(limited - candidate)
                for limited, candidate in zip(limited_tau, candidate_tau)
            ),
            default=0.0,
        )
        tracking_errors.append(tracking_error)
        if tracking_error > TIME_EPSILON_S and dt_s > TIME_EPSILON_S:
            rate_limited_pairs += 1

        forces = []
        inverse_ok = True
        for leg in range(len(LEGS)):
            torque = tuple(limited_tau[3 * leg : 3 * leg + 3])
            if not contacts[leg]:
                forces.append((0.0, 0.0, 0.0))
                continue
            force = solve_transpose(foot_jacobian(leg, q[leg]), torque)
            if force is None or not all(math.isfinite(value) for value in force):
                inverse_ok = False
                break
            forces.append(force)
        if not inverse_ok:
            inverse_failures += 1
            previous_time = row["time_s"]
            previous_contacts = row["contacts"]
            continue

        limited_wrench = wrench_from_forces(
            positions,
            tuple(forces),
            contacts,
        )
        achieved = tuple(row["achieved_wrench"])
        desired = tuple(row["desired_wrench"])
        original_wrench = wrench_from_forces(
            positions,
            tuple(
                solve_transpose(
                    foot_jacobian(leg, q[leg]),
                    tuple(candidate_tau[3 * leg : 3 * leg + 3]),
                )
                if contacts[leg]
                else (0.0, 0.0, 0.0)
                for leg in range(len(LEGS))
            ),
            contacts,
        )
        original_mapping_error = max(
            abs(a - b) for a, b in zip(original_wrench, achieved)
        )
        mapping_errors.append(original_mapping_error)
        residual = tuple(
            actual - target for actual, target in zip(limited_wrench, desired)
        )
        force_norm = math.sqrt(sum(value * value for value in residual[:3]))
        moment_norm = math.sqrt(sum(value * value for value in residual[3:]))
        wrench_norm = math.sqrt(sum(value * value for value in residual))
        force_norms.append(force_norm)
        moment_norms.append(moment_norm)
        wrench_norms.append(wrench_norm)
        if dt_s > TIME_EPSILON_S:
            applied_rate = max(
                abs(limited - previous)
                for limited, previous in zip(limited_tau, previous_limited_tau)
            ) / dt_s
            applied_rates.append(applied_rate)
            group = (
                "contact_count_transition"
                if row["contacts"] != previous_contacts
                else "same_contact_count"
            )
            group_wrench[group].append(wrench_norm)
            group_tracking[group].append(tracking_error)
            group_wrench["all_positive_dt"].append(wrench_norm)
            group_tracking["all_positive_dt"].append(tracking_error)
            top_events.append(
                {
                    "row": row["row_number"],
                    "time_s": row["time_s"],
                    "dt_s": dt_s,
                    "contacts": row["contacts"],
                    "prev_contacts": previous_contacts,
                    "tracking_error": tracking_error,
                    "wrench_norm": wrench_norm,
                }
            )

        previous_time = row["time_s"]
        previous_contacts = row["contacts"]
        limited_tau = list(limited_tau)

    top_events.sort(key=lambda event: event["tracking_error"], reverse=True)
    return {
        "rate_limited_pairs": rate_limited_pairs,
        "duplicate_pairs": duplicate_pairs,
        "inverse_failures": inverse_failures,
        "tracking_errors": tracking_errors,
        "applied_rates": applied_rates,
        "wrench_norms": wrench_norms,
        "force_norms": force_norms,
        "moment_norms": moment_norms,
        "mapping_errors": mapping_errors,
        "group_wrench": group_wrench,
        "group_tracking": group_tracking,
        "top_events": top_events,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-csv", type=Path, required=True)
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--max-torque-rate-nm-s",
        type=float,
        nargs="+",
        default=[2000.0, 3000.0, 4000.0, 6000.0],
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

    try:
        with args.state_csv.open(newline="", encoding="utf-8") as handle:
            state_reader = csv.DictReader(handle)
            state_fields = set(state_reader.fieldnames or [])
            missing = sorted(
                {"cmd_time_s", *CONTACT_FIELDS, *Q_FIELDS} - state_fields
            )
            if missing:
                raise ValueError(
                    "state CSV missing fields: " + ",".join(missing)
                )
            state_rows = list(state_reader)
        with args.replay_csv.open(newline="", encoding="utf-8") as handle:
            replay_reader = csv.DictReader(handle)
            replay_fields = set(replay_reader.fieldnames or [])
            missing = sorted(
                {
                    "row_number",
                    "cmd_time_s",
                    "selected_contact_count",
                    "desired_force_x_n",
                    "desired_force_y_n",
                    "desired_force_z_n",
                    "desired_moment_x_nm",
                    "desired_moment_y_nm",
                    "desired_moment_z_nm",
                    "achieved_force_x_n",
                    "achieved_force_y_n",
                    "achieved_force_z_n",
                    "achieved_moment_x_nm",
                    "achieved_moment_y_nm",
                    "achieved_moment_z_nm",
                    *TAU_FIELDS,
                }
                - replay_fields
            )
            if missing:
                raise ValueError(
                    "replay CSV missing fields: " + ",".join(missing)
                )
            replay_rows = list(replay_reader)
    except (OSError, ValueError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    invalid_rows = 0
    rows: list[dict[str, object]] = []
    negative_dt_pairs = 0
    previous_time = None
    for raw in replay_rows:
        try:
            row_number = int(float(raw["row_number"]))
            if row_number <= 0 or row_number > len(state_rows):
                raise ValueError("row_number outside state CSV")
            row = {
                "row_number": row_number,
                "time_s": finite(raw, "cmd_time_s"),
                "contacts": int(
                    round(finite(raw, "selected_contact_count"))
                ),
                "tau": tuple(finite(raw, name) for name in TAU_FIELDS),
                "desired_wrench": tuple(
                    finite(raw, name)
                    for name in (
                        "desired_force_x_n",
                        "desired_force_y_n",
                        "desired_force_z_n",
                        "desired_moment_x_nm",
                        "desired_moment_y_nm",
                        "desired_moment_z_nm",
                    )
                ),
                "achieved_wrench": tuple(
                    finite(raw, name)
                    for name in (
                        "achieved_force_x_n",
                        "achieved_force_y_n",
                        "achieved_force_z_n",
                        "achieved_moment_x_nm",
                        "achieved_moment_y_nm",
                        "achieved_moment_z_nm",
                    )
                ),
            }
        except (KeyError, ValueError, OverflowError):
            invalid_rows += 1
            continue
        if previous_time is not None and row["time_s"] < previous_time - TIME_EPSILON_S:
            negative_dt_pairs += 1
        previous_time = row["time_s"]
        rows.append(row)

    if len(rows) < 2:
        print("validation=FAIL: fewer than two valid replay rows")
        return 1

    lines = [
        "torque rate-limit wrench recomputation shadow",
        "interpretation=candidate_J_transpose_f_only_not_controller_torque",
        "interpretation=limited_torque_recovered_to_contact_force_via_inverse_J_transpose",
        f"state_csv={args.state_csv}",
        f"replay_csv={args.replay_csv}",
        f"state_rows={len(state_rows)}",
        f"replay_rows={len(rows)}",
        f"invalid_rows={invalid_rows}",
        f"negative_dt_pairs={negative_dt_pairs}",
    ]
    all_mapping_errors: list[float] = []
    any_inverse_failures = False
    for rate_limit in sorted(set(args.max_torque_rate_nm_s)):
        result = run_limit(state_rows=state_rows, rows=rows, rate_limit=rate_limit)
        all_mapping_errors.extend(result["mapping_errors"])
        any_inverse_failures = any_inverse_failures or bool(
            result["inverse_failures"]
        )
        lines.extend(
            [
                f"rate_limit_nm_s={rate_limit:.9g}",
                f"  rate_limited_pairs={result['rate_limited_pairs']}",
                f"  duplicate_timestamp_pairs={result['duplicate_pairs']}",
                f"  inverse_failures={result['inverse_failures']}",
                f"  torque_tracking_error_nm={stats(result['tracking_errors'])}",
                f"  recomputed_force_residual_norm_n={stats(result['force_norms'])}",
                f"  recomputed_moment_residual_norm_nm={stats(result['moment_norms'])}",
                f"  recomputed_wrench_residual_norm={stats(result['wrench_norms'])}",
                f"  applied_torque_rate_nm_s={stats(result['applied_rates'])}",
                f"  original_Jt_mapping_error={stats(result['mapping_errors'])}",
                "  wrench_residual_by_group:",
            ]
        )
        for group in (
            "all_positive_dt",
            "same_contact_count",
            "contact_count_transition",
        ):
            lines.append(
                f"    {group}={stats(result['group_wrench'][group])}"
            )
        lines.append("  tracking_error_by_group:")
        for group in (
            "all_positive_dt",
            "same_contact_count",
            "contact_count_transition",
        ):
            lines.append(
                f"    {group}={stats(result['group_tracking'][group])}"
            )
        lines.append(f"  top_tracking_error_n={min(args.top_n, len(result['top_events']))}:")
        for event in result["top_events"][: args.top_n]:
            lines.append(
                "    "
                f"row={event['row']},time_s={event['time_s']:.9g},"
                f"dt_s={event['dt_s']:.9g},"
                f"prev_contacts={event['prev_contacts']},"
                f"contacts={event['contacts']},"
                f"tracking_error_nm={event['tracking_error']:.9g},"
                f"wrench_residual_norm={event['wrench_norm']:.9g}"
            )

    lines.append(
        f"global_original_Jt_mapping_error={stats(all_mapping_errors)}"
    )
    validation_pass = (
        invalid_rows == 0
        and negative_dt_pairs == 0
        and not any_inverse_failures
        and max(all_mapping_errors, default=math.inf) <= MAPPING_TOLERANCE
    )
    lines.append(
        "mapping_validation="
        + ("PASS" if max(all_mapping_errors, default=math.inf) <= MAPPING_TOLERANCE else "FAIL")
    )
    lines.append("validation=" + ("PASS" if validation_pass else "FAIL"))
    report = "\n".join(lines) + "\n"
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report, end="")
    return 0 if validation_pass else 1


if __name__ == "__main__":
    sys.exit(main())
