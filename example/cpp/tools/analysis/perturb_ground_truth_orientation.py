#!/usr/bin/env python3
"""Apply a deterministic world-frame SO(3) bias to logged base quaternions."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def normalize(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("quaternion norm must be finite and positive")
    return tuple(component / norm for component in quaternion)


def axis_quaternion(
    axis: str,
    angle_rad: float,
) -> tuple[float, float, float, float]:
    half = angle_rad * 0.5
    scalar = math.cos(half)
    vector = math.sin(half)
    if axis == "x":
        return (scalar, vector, 0.0, 0.0)
    if axis == "y":
        return (scalar, 0.0, vector, 0.0)
    if axis == "z":
        return (scalar, 0.0, 0.0, vector)
    raise ValueError(f"unsupported axis: {axis}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--time-column", default="time_s")
    parser.add_argument("--start-time-s", type=float)
    parser.add_argument("--end-time-s", type=float)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--pitch-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--quaternion-prefix", default="base_quat")
    args = parser.parse_args()

    angles_deg = (args.roll_deg, args.pitch_deg, args.yaw_deg)
    if any(not math.isfinite(angle) for angle in angles_deg):
        raise SystemExit("rotation angles must be finite")
    if all(abs(angle) <= 1e-15 for angle in angles_deg):
        raise SystemExit("at least one rotation angle must be nonzero")
    if (args.start_time_s is None) != (args.end_time_s is None):
        raise SystemExit("start-time-s and end-time-s must be provided together")
    if args.start_time_s is not None:
        if not math.isfinite(args.start_time_s) or not math.isfinite(args.end_time_s):
            raise SystemExit("time bounds must be finite")
        if args.start_time_s > args.end_time_s:
            raise SystemExit("start-time-s must not exceed end-time-s")
    if not args.time_column:
        raise SystemExit("time-column must not be empty")

    angle_rad = tuple(angle * math.pi / 180.0 for angle in angles_deg)
    q_delta = quaternion_multiply(
        quaternion_multiply(axis_quaternion("z", angle_rad[2]), axis_quaternion("y", angle_rad[1])),
        axis_quaternion("x", angle_rad[0]),
    )
    quaternion_columns = tuple(
        f"{args.quaternion_prefix}_{axis}" for axis in ("w", "x", "y", "z")
    )

    input_path = Path(args.input)
    output_path = Path(args.output)
    with input_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [column for column in quaternion_columns if column not in fieldnames]
        if missing:
            raise SystemExit("missing quaternion columns: " + ",".join(missing))
        if args.start_time_s is not None and args.time_column not in fieldnames:
            raise SystemExit("time-window perturbation requires " + args.time_column)
        rows = list(reader)

    selected_rows = 0
    changed_cells = 0
    for row in rows:
        if args.start_time_s is not None:
            raw_time = row.get(args.time_column, "")
            if raw_time == "":
                raise SystemExit("empty " + args.time_column + " in time-window perturbation")
            time_s = float(raw_time)
            if not math.isfinite(time_s):
                raise SystemExit("non-finite " + args.time_column + " in time-window perturbation")
            if not (args.start_time_s <= time_s <= args.end_time_s):
                continue
        selected_rows += 1
        try:
            quaternion = normalize(tuple(float(row[column]) for column in quaternion_columns))
        except (KeyError, ValueError) as error:
            raise SystemExit("invalid quaternion row: " + str(error))
        biased = normalize(quaternion_multiply(q_delta, quaternion))
        for column, value in zip(quaternion_columns, biased):
            row[column] = format(value, ".17g")
            changed_cells += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=reader.fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("ground-truth orientation estimate created")
    print(f"input={input_path}")
    print(f"output={output_path}")
    print("composition=world_delta_left_multiply_q_est=q_delta*q_true")
    print(f"roll_deg={args.roll_deg}")
    print(f"pitch_deg={args.pitch_deg}")
    print(f"yaw_deg={args.yaw_deg}")
    print(f"time_column={args.time_column}")
    print(f"start_time_s={args.start_time_s}")
    print(f"end_time_s={args.end_time_s}")
    print(f"rows={len(rows)}")
    print(f"selected_rows={selected_rows}")
    print(f"changed_cells={changed_cells}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
