#!/usr/bin/env python3
"""Replay MuJoCo contact forces into a Go2 J^T f torque audit."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
GEOMETRY = {
    "FR": (0.1934, -0.0465, -0.0955, 0.213, 0.213),
    "FL": (0.1934, 0.0465, 0.0955, 0.213, 0.213),
    "RR": (-0.1934, -0.0465, -0.0955, 0.213, 0.213),
    "RL": (-0.1934, 0.0465, 0.0955, 0.213, 0.213),
}


def finite_float(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def vector_norm(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def world_to_body(quaternion: tuple[float, float, float, float],
                  vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid base quaternion")
    w, x, y, z = (quaternion[0] / norm, -quaternion[1] / norm,
                  -quaternion[2] / norm, -quaternion[3] / norm)
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


def foot_jacobian(leg: str, q_hip: float, q_thigh: float,
                  q_calf: float) -> tuple[tuple[float, ...], ...]:
    _, _, hip_link_y, thigh_length, calf_length = GEOMETRY[leg]
    lower_angle = q_thigh + q_calf
    sin_hip, cos_hip = math.sin(q_hip), math.cos(q_hip)
    sin_thigh, cos_thigh = math.sin(q_thigh), math.cos(q_thigh)
    sin_lower, cos_lower = math.sin(lower_angle), math.cos(lower_angle)
    leg_z = -thigh_length * cos_thigh - calf_length * cos_lower
    lower_z = -calf_length * cos_lower
    d_leg_z_d_thigh = thigh_length * sin_thigh + calf_length * sin_lower
    d_leg_z_d_calf = calf_length * sin_lower
    lateral_y = cos_hip * hip_link_y - sin_hip * leg_z
    return (
        (0.0, leg_z, lower_z),
        (-sin_hip * hip_link_y - cos_hip * leg_z,
         -sin_hip * d_leg_z_d_thigh, -sin_hip * d_leg_z_d_calf),
        (lateral_y, cos_hip * d_leg_z_d_thigh,
         cos_hip * d_leg_z_d_calf),
    )


def torque_from_force(leg: str, angles: tuple[float, float, float],
                      force_body: tuple[float, float, float]) -> tuple[float, ...]:
    jacobian = foot_jacobian(leg, *angles)
    return tuple(
        sum(jacobian[row][joint] * force_body[row] for row in range(3))
        for joint in range(3)
    )


def read_ground_truth(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"time_s", "base_quat_w", "base_quat_x",
                    "base_quat_y", "base_quat_z"}
        for leg in LEGS:
            required.update(f"{leg}_foot_contact_grf_world_{axis}_N" for axis in "xyz")
        missing = sorted(required.difference(reader.fieldnames or ()))
        if missing:
            raise ValueError("ground-truth missing columns: " + ",".join(missing))
        samples = []
        previous_time = -math.inf
        for row_number, row in enumerate(reader, start=2):
            time_s = finite_float(row, "time_s")
            if time_s <= previous_time:
                raise ValueError(f"ground-truth row {row_number} time is not increasing")
            previous_time = time_s
            quaternion = tuple(
                finite_float(row, f"base_quat_{axis}")
                for axis in ("w", "x", "y", "z")
            )
            forces = {
                leg: tuple(
                    finite_float(row, f"{leg}_foot_contact_grf_world_{axis}_N")
                    for axis in "xyz"
                )
                for leg in LEGS
            }
            samples.append({"time": time_s, "quaternion": quaternion, "forces": forces})
    if not samples:
        raise ValueError("ground-truth CSV is empty")
    return samples


def nearest_sample(samples: list[dict[str, object]], target: float,
                   cursor: int) -> tuple[int, float]:
    while cursor + 1 < len(samples):
        current_error = abs(float(samples[cursor]["time"]) - target)
        next_error = abs(float(samples[cursor + 1]["time"]) - target)
        if next_error > current_error:
            break
        cursor += 1
    return cursor, float(samples[cursor]["time"]) - target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state-csv", required=True)
    parser.add_argument("--ground-truth-csv", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--time-offset-s", type=float, default=0.0)
    parser.add_argument("--match-tolerance-s", type=float, default=0.0011)
    parser.add_argument("--contact-force-threshold-n", type=float, default=5.0)
    parser.add_argument("--swing-force-tolerance-n", type=float, default=5.0)
    parser.add_argument("--mu", type=float, default=0.4)
    parser.add_argument("--max-torque-nm", type=float, default=100.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (args.match_tolerance_s <= 0 or args.contact_force_threshold_n <= 0
            or args.swing_force_tolerance_n <= 0
            or args.mu < 0 or args.max_torque_nm <= 0):
        print("validation=FAIL: invalid audit parameters")
        return 2
    try:
        ground_truth = read_ground_truth(Path(args.ground_truth_csv))
        with Path(args.state_csv).open(newline="") as handle:
            state_reader = csv.DictReader(handle)
            required = {"state_tick_s"}
            for leg in LEGS:
                required.add(f"contact_{leg}")
                required.update(f"{leg}_{joint}_q_state" for joint in JOINTS)
            missing = sorted(required.difference(state_reader.fieldnames or ()))
            if missing:
                raise ValueError("state CSV missing columns: " + ",".join(missing))
            state_rows = list(state_reader)
    except (OSError, ValueError, KeyError) as exc:
        print(f"validation=FAIL: {exc}")
        return 2

    summary = {
        "state_rows": len(state_rows), "matched_rows": 0, "skipped_rows": 0,
        "duplicate_state_rows": 0,
        "time_match_failures": 0, "contact_mask_mismatch_rows": 0,
        "force_on_state_swing_rows": 0, "friction_violation_rows": 0,
        "swing_force_violation_rows": 0,
        "torque_limit_violation_rows": 0, "max_time_error_s": 0.0,
        "max_force_on_state_swing_n": 0.0, "max_friction_ratio": 0.0,
        "max_abs_torque_nm": 0.0, "max_torque_violation_nm": 0.0,
    }
    output_path = Path(args.out)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_fields = [
        "state_row", "state_tick_s", "ground_truth_time_s", "time_error_s",
        "state_contact_count", "ground_truth_contact_count", "contact_mask_match",
        "force_on_state_swing_max_N", "max_friction_ratio", "friction_ok",
        "max_abs_torque_Nm", "torque_limits_ok",
    ] + [f"{leg}_{joint}_tau_candidate_Nm" for leg in LEGS for joint in JOINTS]

    cursor = 0
    previous_state_time = -math.inf
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=output_fields)
        writer.writeheader()
        for row_number, row in enumerate(state_rows, start=2):
            try:
                state_time = finite_float(row, "state_tick_s")
                if state_time == previous_state_time:
                    summary["duplicate_state_rows"] += 1
                    continue
                if state_time < previous_state_time:
                    raise ValueError("state time is not increasing")
                previous_state_time = state_time
                cursor, time_error = nearest_sample(
                    ground_truth, state_time + args.time_offset_s, cursor
                )
                if abs(time_error) > args.match_tolerance_s:
                    summary["time_match_failures"] += 1
                    continue
                sample = ground_truth[cursor]
                state_contact = {
                    leg: int(finite_float(row, f"contact_{leg}") >= 0.5)
                    for leg in LEGS
                }
                angles = {
                    leg: tuple(
                        finite_float(row, f"{leg}_{joint}_q_state")
                        for joint in JOINTS
                    )
                    for leg in LEGS
                }
            except (ValueError, KeyError):
                summary["skipped_rows"] += 1
                continue

            summary["matched_rows"] += 1
            summary["max_time_error_s"] = max(
                summary["max_time_error_s"], abs(time_error)
            )
            quaternion = sample["quaternion"]
            forces_world = sample["forces"]
            ground_contact = {}
            body_forces = {}
            force_on_swing = 0.0
            max_friction_ratio = 0.0
            friction_ok = True
            contact_mask_match = True
            for leg in LEGS:
                world_force = forces_world[leg]
                norm = vector_norm(world_force)
                ground_contact[leg] = int(norm >= args.contact_force_threshold_n)
                if ground_contact[leg] != state_contact[leg]:
                    contact_mask_match = False
                if not state_contact[leg]:
                    force_on_swing = max(force_on_swing, norm)
                if ground_contact[leg]:
                    normal = world_force[2]
                    tangent = math.hypot(world_force[0], world_force[1])
                    if normal <= 1e-9:
                        friction_ok = False
                        max_friction_ratio = math.inf
                    else:
                        ratio = tangent / normal
                        max_friction_ratio = max(max_friction_ratio, ratio)
                        if ratio > args.mu + 1e-9:
                            friction_ok = False
                    body_forces[leg] = world_to_body(quaternion, world_force)
                else:
                    body_forces[leg] = (0.0, 0.0, 0.0)

            if not contact_mask_match:
                summary["contact_mask_mismatch_rows"] += 1
            if force_on_swing > 1e-9:
                summary["force_on_state_swing_rows"] += 1
                summary["max_force_on_state_swing_n"] = max(
                    summary["max_force_on_state_swing_n"], force_on_swing
                )
                if force_on_swing > args.swing_force_tolerance_n:
                    summary["swing_force_violation_rows"] += 1
            summary["max_friction_ratio"] = max(
                summary["max_friction_ratio"], max_friction_ratio
            )
            if not friction_ok:
                summary["friction_violation_rows"] += 1

            torques = {
                leg: (torque_from_force(leg, angles[leg], body_forces[leg])
                      if ground_contact[leg] else (0.0, 0.0, 0.0))
                for leg in LEGS
            }
            max_abs_torque = max(abs(value) for leg in LEGS for value in torques[leg])
            max_violation = max(0.0, max_abs_torque - args.max_torque_nm)
            torque_ok = max_violation <= 1e-9
            if not torque_ok:
                summary["torque_limit_violation_rows"] += 1
                summary["max_torque_violation_nm"] = max(
                    summary["max_torque_violation_nm"], max_violation
                )
            summary["max_abs_torque_nm"] = max(
                summary["max_abs_torque_nm"], max_abs_torque
            )
            output_row = {
                "state_row": row_number, "state_tick_s": f"{state_time:.12g}",
                "ground_truth_time_s": f"{sample['time']:.12g}",
                "time_error_s": f"{time_error:.12g}",
                "state_contact_count": sum(state_contact.values()),
                "ground_truth_contact_count": sum(ground_contact.values()),
                "contact_mask_match": int(contact_mask_match),
                "force_on_state_swing_max_N": f"{force_on_swing:.12g}",
                "max_friction_ratio": f"{max_friction_ratio:.12g}",
                "friction_ok": int(friction_ok),
                "max_abs_torque_Nm": f"{max_abs_torque:.12g}",
                "torque_limits_ok": int(torque_ok),
            }
            for leg in LEGS:
                for joint, torque in zip(JOINTS, torques[leg]):
                    output_row[f"{leg}_{joint}_tau_candidate_Nm"] = f"{torque:.12g}"
            writer.writerow(output_row)

    structural_ok = (summary["matched_rows"] > 0
                     and summary["time_match_failures"] == 0
                     and summary["skipped_rows"] == 0)
    contact_ok = summary["contact_mask_mismatch_rows"] == 0
    swing_ok = summary["swing_force_violation_rows"] == 0
    friction_ok = summary["friction_violation_rows"] == 0
    torque_ok = summary["torque_limit_violation_rows"] == 0
    physical_ok = structural_ok and swing_ok and friction_ok and torque_ok
    print("ground-truth torque replay completed")
    for key, value in summary.items():
        if isinstance(value, float):
            print(f"{key}={value:.9g}")
        else:
            print(f"{key}={value}")
    print(f"contact_mask_validation={'PASS' if contact_ok else 'FAIL'}")
    print(f"contact_schedule_validation={'PASS' if contact_ok else 'FAIL'}")
    print(f"swing_force_validation={'PASS' if swing_ok else 'FAIL'}")
    print(f"friction_validation={'PASS' if friction_ok else 'FAIL'}")
    print(f"torque_limit_validation={'PASS' if torque_ok else 'FAIL'}")
    print(f"structural_validation={'PASS' if structural_ok else 'FAIL'}")
    print(f"physics_validation={'PASS' if physical_ok else 'FAIL'}")
    passed = physical_ok and contact_ok
    print(f"validation={'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
