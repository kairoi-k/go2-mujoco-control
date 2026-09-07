#!/usr/bin/env python3
"""Extract a small, provenance-preserving Stage-C feedback replay packet.

This tool only joins recorded CSV rows. It never invents missing state or
runs MuJoCo. The companion C++ tool consumes its output for model evaluation.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import pathlib
import xml.etree.ElementTree as ET

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
STATE_COLUMNS = [
    "cmd_time_s", "state_tick_s", "has_state", "motion_stage", "velocity_command_active",
    "motion_dt_s", "world_base_x_m", "world_base_y_m", "world_base_z_m",
    "imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad",
    "imu_gyro_body_x_radps", "imu_gyro_body_y_radps", "imu_gyro_body_z_radps",
    "terrain_map_valid", "terrain_map_source", "terrain_map_epoch",
    "terrain_map_age_s", "terrain_capture_stamp_s",
    "terrain_registration_stamp_s", "terrain_execution_map_epoch",
    "terrain_execution_plan_usable", "terrain_execution_applied_mask",
    "terrain_execution_planned_contact_mask", "wbc_measured_contact_mask",
    "wbc_scheduled_contact_mask", "wbc_terrain_planned_contact_mask",
    "terrain_surface_transition_active", "terrain_surface_transition_required_mask",
    "terrain_surface_transition_committed_mask", "terrain_exec_FR_valid",
    "terrain_exec_FL_valid", "terrain_exec_RR_valid", "terrain_exec_RL_valid",
]
for leg in LEGS:
    for joint in JOINTS:
        STATE_COLUMNS += [f"{leg}_{joint}_q_state", f"{leg}_{joint}_dq_state"]
GT_COLUMNS = [
    "time_s", "base_pos_world_x_m", "base_pos_world_y_m", "base_pos_world_z_m",
    "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z",
    "base_qvel_world_x_mps", "base_qvel_world_y_mps", "base_qvel_world_z_mps",
    "base_angvel_body_x_radps", "base_angvel_body_y_radps", "base_angvel_body_z_radps",
    "phase2_terrain_foot_contact_mask",
]
for leg in LEGS:
    GT_COLUMNS += [f"{leg}_pos_world_x_m", f"{leg}_pos_world_y_m", f"{leg}_pos_world_z_m"]


def fail(message: str) -> "NoReturn":
    raise SystemExit(f"ERROR: {message}")


def sha256(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path: pathlib.Path) -> tuple[list[str], list[dict[str, str]]]:
    try:
        with path.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            header = reader.fieldnames
            if not header or len(set(header)) != len(header):
                fail(f"invalid or duplicate CSV header: {path}")
            rows = []
            for number, row in enumerate(reader, start=2):
                if None in row or any(value is None for value in row.values()):
                    fail(f"malformed CSV row {number}: {path}")
                rows.append(row)
            return header, rows
    except OSError as exc:
        fail(f"cannot read {path}: {exc}")


def require_columns(header: list[str], required: list[str], label: str) -> None:
    missing = [column for column in required if column not in header]
    if missing:
        fail(f"{label} missing required columns: {','.join(missing)}")


def number(row: dict[str, str], column: str) -> float:
    value = row.get(column, "").strip()
    if not value:
        fail(f"empty numeric field {column}")
    try:
        result = float(value)
    except ValueError:
        fail(f"non-numeric field {column}: {value}")
    if not math.isfinite(result):
        fail(f"non-finite field {column}: {value}")
    return result


def integer(row: dict[str, str], column: str, minimum: int = 0, maximum: int | None = None) -> int:
    value = number(row, column)
    rounded = round(value)
    if abs(value - rounded) > 1e-9 or rounded < minimum or (maximum is not None and rounded > maximum):
        fail(f"invalid integer field {column}: {row.get(column)}")
    return int(rounded)


def resolve_path(value: str, repo: pathlib.Path) -> pathlib.Path:
    path = pathlib.Path(value)
    if path.is_absolute():
        return path
    candidate = repo / path
    return candidate if candidate.exists() else pathlib.Path.cwd() / path


def scene_plane(scene: pathlib.Path) -> tuple[float, str]:
    try:
        root = ET.parse(scene).getroot()
    except (OSError, ET.ParseError) as exc:
        fail(f"cannot parse scene source {scene}: {exc}")
    planes = [
        node for node in root.iter("geom")
        if node.get("name") == "phase2_floor" and node.get("type") == "plane"
    ]
    if len(planes) != 1:
        fail(f"scene must provide exactly one phase2_floor plane: {scene}")
    raw = planes[0].get("pos", "0 0 0").split()
    if len(raw) != 3:
        fail(f"invalid phase2_floor pos in {scene}")
    try:
        z = float(raw[2])
    except ValueError:
        fail(f"invalid phase2_floor z in {scene}")
    if not math.isfinite(z):
        fail(f"non-finite phase2_floor z in {scene}")
    return z, f"scene:{scene}:phase2_floor"


def nearest_ground_truth(times: list[float], rows: list[dict[str, str]], t: float) -> tuple[int, dict[str, str]]:
    position = bisect.bisect_left(times, t)
    candidates = []
    if position < len(times):
        candidates.append((abs(times[position] - t), position))
    if position:
        candidates.append((abs(times[position - 1] - t), position - 1))
    if not candidates:
        fail(f"no ground-truth row for state time {t}")
    distance, index = min(candidates)
    if distance > 1e-9:
        fail(f"ground-truth timestamp mismatch at {t}: {distance}")
    return index, rows[index]


def actual_columns() -> list[str]:
    return [
        "cmd_time_s", "state_tick_s", "world_base_x_m", "world_base_y_m", "world_base_z_m",
        "imu_roll_rad", "imu_pitch_rad", "imu_yaw_rad",
        "imu_gyro_body_x_radps", "imu_gyro_body_y_radps", "imu_gyro_body_z_radps",
    ] + [
        f"{leg}_{joint}_{suffix}"
        for leg in LEGS for joint in JOINTS
        for suffix in ("q_state", "dq_state")
    ]


def duplicate_consistent(first: dict[str, str], other: dict[str, str]) -> bool:
    for column in actual_columns():
        try:
            if abs(float(first[column]) - float(other[column])) > 1e-12:
                return False
        except (KeyError, ValueError):
            return False
    for column in ("has_state", "motion_stage", "velocity_command_active", "wbc_measured_contact_mask"):
        if first.get(column) != other.get(column):
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, help="run directory containing data.csv and contact_ground_truth.csv")
    parser.add_argument("--out", required=True, help="new extracted CSV; existing output is rejected")
    parser.add_argument("--start-time-s", type=float, default=18.0)
    parser.add_argument("--end-time-s", type=float, default=24.0)
    parser.add_argument("--sample-count", type=int, default=7)
    parser.add_argument("--repo", default=None, help="repository root, inferred from this script")
    args = parser.parse_args()
    if not math.isfinite(args.start_time_s) or not math.isfinite(args.end_time_s) or args.end_time_s < args.start_time_s:
        fail("invalid time window")
    if args.sample_count <= 0:
        fail("sample-count must be positive")
    repo = pathlib.Path(args.repo).resolve() if args.repo else pathlib.Path(__file__).resolve().parents[4]
    run_dir = resolve_path(args.run_dir, repo).resolve()
    data_path = run_dir / "data.csv"
    truth_path = run_dir / "contact_ground_truth.csv"
    manifest_path = run_dir / "run_manifest.json"
    out_path = pathlib.Path(args.out).resolve()
    if out_path.exists():
        fail(f"refusing to overwrite existing output: {out_path}")
    if not manifest_path.exists():
        fail(f"missing run_manifest.json: {manifest_path}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        repository = manifest["repository"]
        run_meta = manifest["run"]
        artifacts = manifest["artifacts"]
        commit = repository["git_commit"]
        scene_value = run_meta["scene_file"]
        controller_sha = artifacts["controller_sha256"]
        scenario_sha = artifacts["scenario_sha256"]
    except (OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
        fail(f"manifest lacks required provenance: {exc}")
    if not commit or not scene_value or len(controller_sha) != 64 or len(scenario_sha) != 64:
        fail("manifest provenance is empty or malformed")
    data_header, data_rows = read_csv(data_path)
    truth_header, truth_rows = read_csv(truth_path)
    require_columns(data_header, STATE_COLUMNS, "data.csv")
    require_columns(truth_header, GT_COLUMNS, "contact_ground_truth.csv")
    if not data_rows or not truth_rows:
        fail("raw CSV is empty")
    truth_times = [number(row, "time_s") for row in truth_rows]
    if any(b <= a for a, b in zip(truth_times, truth_times[1:])):
        fail("ground-truth timestamps are not strictly increasing")
    scene_path = resolve_path(scene_value, repo).resolve()
    surface_z, surface_source = scene_plane(scene_path)

    grouped: dict[tuple[int, int], list[tuple[int, dict[str, str], dict[str, str], int]]] = {}
    for data_index, row in enumerate(data_rows, start=2):
        t = number(row, "state_tick_s")
        if not args.start_time_s <= t <= args.end_time_s:
            continue
        if row.get("has_state") != "1" or row.get("motion_stage") not in ("2", "3") or row.get("velocity_command_active") != "1":
            continue
        dt = number(row, "motion_dt_s")
        if not 0.0 < dt <= 0.002000001:
            continue
        for column in actual_columns() + [
            "motion_dt_s", "terrain_map_age_s",
            "terrain_capture_stamp_s", "terrain_registration_stamp_s",
        ]:
            number(row, column)
        integer(row, "terrain_map_valid", maximum=1)
        integer(row, "terrain_map_epoch")
        integer(row, "terrain_execution_plan_usable", maximum=1)
        for column in (
            "terrain_execution_applied_mask", "terrain_execution_planned_contact_mask",
            "wbc_measured_contact_mask", "wbc_scheduled_contact_mask",
            "wbc_terrain_planned_contact_mask",
        ):
            integer(row, column, maximum=15)
        gt_index, gt = nearest_ground_truth(truth_times, truth_rows, t)
        for column in GT_COLUMNS:
            number(gt, column)
        integer(gt, "phase2_terrain_foot_contact_mask", maximum=15)
        quat_norm = math.sqrt(sum(number(gt, f"base_quat_{axis}") ** 2 for axis in ("w", "x", "y", "z")))
        if abs(quat_norm - 1.0) > 1e-6:
            fail(f"ground-truth quaternion is not normalized at {t}: {quat_norm}")
        cmd_time = number(row, "cmd_time_s")
        key = (round(t * 1e9), round(cmd_time * 1e9))
        grouped.setdefault(key, []).append((data_index, row, gt, gt_index))
    if not grouped:
        fail("no valid stage-2/3 active rows in requested window")
    unique = []
    for key in sorted(grouped):
        group = grouped[key]
        first = group[0]
        for other in group[1:]:
            if not duplicate_consistent(first[1], other[1]):
                fail(f"duplicate state timestamp has conflicting raw state: {first[1]['state_tick_s']}")
        unique.append(first)
    count = min(args.sample_count, len(unique))
    indices = sorted({round(i * (len(unique) - 1) / max(1, count - 1)) for i in range(count)})
    selected = [unique[index] for index in indices]
    fields = [
        "sample_index", "data_row_index", "ground_truth_row_index", "cmd_time_s", "state_tick_s", "motion_dt_s",
        "motion_stage", "velocity_command_active", "base_position_source", "orientation_source",
        "base_linear_velocity_source", "angular_velocity_source", "joint_state_source",
        "contact_provenance", "terrain_map_source", "terrain_map_epoch", "terrain_map_valid",
        "terrain_map_age_s", "terrain_capture_stamp_s", "terrain_registration_stamp_s",
        "terrain_execution_map_epoch", "terrain_execution_plan_usable", "terrain_execution_applied_mask",
        "terrain_execution_planned_contact_mask", "wbc_measured_contact_mask", "wbc_scheduled_contact_mask",
        "wbc_terrain_planned_contact_mask", "gt_contact_mask", "surface_plane_z_m", "surface_normal_x",
        "surface_normal_y", "surface_normal_z", "surface_provenance", "reference_provenance",
        "base_x_m", "base_y_m", "base_z_m", "base_quat_w", "base_quat_x", "base_quat_y", "base_quat_z",
        "base_vx_mps", "base_vy_mps", "base_vz_mps", "imu_gyro_body_x_radps", "imu_gyro_body_y_radps",
        "imu_gyro_body_z_radps",
    ]
    fields += [f"q_{i}" for i in range(12)] + [f"dq_{i}" for i in range(12)]
    for leg in LEGS:
        fields += [f"{leg}_gt_foot_x_m", f"{leg}_gt_foot_y_m", f"{leg}_gt_foot_z_m"]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="raise")
        writer.writeheader()
        for sample_index, (data_row_index, row, gt, gt_index) in enumerate(selected):
            values: dict[str, object] = {
                "sample_index": sample_index,
                "data_row_index": data_row_index,
                "ground_truth_row_index": gt_index + 2,
                "cmd_time_s": row["cmd_time_s"],
                "state_tick_s": row["state_tick_s"],
                "motion_dt_s": row["motion_dt_s"],
                "motion_stage": row["motion_stage"],
                "velocity_command_active": row["velocity_command_active"],
                "base_position_source": "data.csv:world_base_x/y/z_m",
                "orientation_source": "contact_ground_truth.csv:base_quat_w/x/y/z",
                "base_linear_velocity_source": "contact_ground_truth.csv:base_qvel_world_x/y/z_mps",
                "angular_velocity_source": "data.csv:imu_gyro_body_x/y/z_radps",
                "joint_state_source": "data.csv:*_q_state/*_dq_state",
                "contact_provenance": "data.csv:wbc_measured_contact_mask",
                "terrain_map_source": row["terrain_map_source"],
                "terrain_map_epoch": row["terrain_map_epoch"],
                "terrain_map_valid": row["terrain_map_valid"],
                "terrain_map_age_s": row["terrain_map_age_s"],
                "terrain_capture_stamp_s": row["terrain_capture_stamp_s"],
                "terrain_registration_stamp_s": row["terrain_registration_stamp_s"],
                "terrain_execution_map_epoch": row["terrain_execution_map_epoch"],
                "terrain_execution_plan_usable": row["terrain_execution_plan_usable"],
                "terrain_execution_applied_mask": row["terrain_execution_applied_mask"],
                "terrain_execution_planned_contact_mask": row["terrain_execution_planned_contact_mask"],
                "wbc_measured_contact_mask": row["wbc_measured_contact_mask"],
                "wbc_scheduled_contact_mask": row["wbc_scheduled_contact_mask"],
                "wbc_terrain_planned_contact_mask": row["wbc_terrain_planned_contact_mask"],
                "gt_contact_mask": gt["phase2_terrain_foot_contact_mask"],
                "surface_plane_z_m": f"{surface_z:.17g}", "surface_normal_x": "0",
                "surface_normal_y": "0", "surface_normal_z": "1",
                "surface_provenance": surface_source,
                "reference_provenance": "missing-from-raw; C++ requires explicit evaluator reference",
                "base_x_m": row["world_base_x_m"], "base_y_m": row["world_base_y_m"], "base_z_m": row["world_base_z_m"],
                "base_quat_w": gt["base_quat_w"], "base_quat_x": gt["base_quat_x"],
                "base_quat_y": gt["base_quat_y"], "base_quat_z": gt["base_quat_z"],
                "base_vx_mps": gt["base_qvel_world_x_mps"], "base_vy_mps": gt["base_qvel_world_y_mps"],
                "base_vz_mps": gt["base_qvel_world_z_mps"],
                "imu_gyro_body_x_radps": row["imu_gyro_body_x_radps"],
                "imu_gyro_body_y_radps": row["imu_gyro_body_y_radps"],
                "imu_gyro_body_z_radps": row["imu_gyro_body_z_radps"],
            }
            ordered_joints = [(leg, joint) for leg in LEGS for joint in JOINTS]
            for index, (leg, joint) in enumerate(ordered_joints):
                values[f"q_{index}"] = row[f"{leg}_{joint}_q_state"]
                values[f"dq_{index}"] = row[f"{leg}_{joint}_dq_state"]
            for leg in LEGS:
                values[f"{leg}_gt_foot_x_m"] = gt[f"{leg}_pos_world_x_m"]
                values[f"{leg}_gt_foot_y_m"] = gt[f"{leg}_pos_world_y_m"]
                values[f"{leg}_gt_foot_z_m"] = gt[f"{leg}_pos_world_z_m"]
            writer.writerow(values)
    metadata = {
        "schema": "stage_c_feedback_samples_v1",
        "run_dir": str(run_dir),
        "data_csv": str(data_path), "data_sha256": sha256(data_path),
        "ground_truth_csv": str(truth_path), "ground_truth_sha256": sha256(truth_path),
        "run_manifest": str(manifest_path), "run_manifest_sha256": sha256(manifest_path),
        "repository": repository, "controller_sha256": controller_sha,
        "scenario_sha256": scenario_sha, "scene_file": str(scene_path),
        "scene_sha256": artifacts.get("scenario_sha256"),
        "scene_plane_z_m": surface_z, "scene_plane_source": surface_source,
        "window": {"start_s": args.start_time_s, "end_s": args.end_time_s},
        "candidate_rows": len(unique), "emitted_rows": len(selected),
        "state_sources": {
            "position": "data.csv:world_base_x/y/z_m",
            "orientation": "contact_ground_truth.csv:base_quat_w/x/y/z",
            "linear_velocity": "contact_ground_truth.csv:base_qvel_world_x/y/z_mps",
            "angular_velocity": "data.csv:imu_gyro_body_x/y/z_radps",
            "joints": "data.csv:*_q_state/*_dq_state",
        },
        "missing_from_raw": [
            "centroidal_derivative_6", "foot_acceleration_12",
            "full_12d_force_reference", "contact_surface_normal_per_sample",
        ],
        "claim_boundary": "offline same-model sample evaluation only; no mj_step, contact evolution, final PD actuator, or B1 claim",
    }
    meta_path = out_path.with_suffix(out_path.suffix + ".meta.json")
    if meta_path.exists():
        fail(f"refusing to overwrite metadata output: {meta_path}")
    meta_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    main()