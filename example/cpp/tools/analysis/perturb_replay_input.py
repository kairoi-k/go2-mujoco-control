#!/usr/bin/env python3
"""Create a deterministic replay-input variant for target-robustness audits."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--column", action="append")
    parser.add_argument(
        "--offset",
        "--offset-n",
        dest="offset",
        type=float,
    )
    parser.add_argument(
        "--set-column",
        action="append",
        default=[],
        metavar="COLUMN=VALUE",
    )
    parser.add_argument(
        "--scale-column",
        action="append",
        default=[],
        metavar="COLUMN=FACTOR",
    )
    parser.add_argument("--start-time-s", type=float)
    parser.add_argument("--end-time-s", type=float)
    parser.add_argument("--time-column", default="cmd_time_s")
    args = parser.parse_args()

    if args.offset is not None:
        if args.column is None and (args.set_column or args.scale_column):
            raise SystemExit(
                "set-column with offset requires explicit column"
            )
        offset_columns = args.column or [
            "wbc_shadow_desired_force_x_n"
        ]
    else:
        if args.column:
            raise SystemExit("column requires offset")
        offset_columns = []
    set_values = {}
    for spec in args.set_column:
        if "=" not in spec:
            raise SystemExit("set-column must use COLUMN=VALUE")
        column, raw_value = spec.split("=", 1)
        if not column or not raw_value:
            raise SystemExit("set-column must use COLUMN=VALUE")
        if column in offset_columns or column in set_values:
            raise SystemExit("duplicate perturbation column: " + column)
        try:
            value = float(raw_value)
        except ValueError:
            raise SystemExit("set-column value must be numeric")
        if not math.isfinite(value):
            raise SystemExit("set-column value must be finite")
        set_values[column] = value
    scale_values = {}
    for spec in args.scale_column:
        if "=" not in spec:
            raise SystemExit("scale-column must use COLUMN=FACTOR")
        column, raw_factor = spec.split("=", 1)
        if not column or not raw_factor:
            raise SystemExit("scale-column must use COLUMN=FACTOR")
        if column in offset_columns or column in set_values or column in scale_values:
            raise SystemExit("duplicate perturbation column: " + column)
        try:
            factor = float(raw_factor)
        except ValueError:
            raise SystemExit("scale-column factor must be numeric")
        if not math.isfinite(factor):
            raise SystemExit("scale-column factor must be finite")
        scale_values[column] = factor
    if args.offset is None and not set_values and not scale_values:
        raise SystemExit("provide offset, set-column or scale-column")
    operation_columns = (
        offset_columns + list(scale_values) + list(set_values)
    )

    if (args.start_time_s is None) != (args.end_time_s is None):
        raise SystemExit("start-time-s and end-time-s must be provided together")
    if args.start_time_s is not None:
        if not math.isfinite(args.start_time_s) or not math.isfinite(args.end_time_s):
            raise SystemExit("time bounds must be finite")
        if args.start_time_s > args.end_time_s:
            raise SystemExit("start-time-s must not exceed end-time-s")

    if args.offset is not None and not math.isfinite(args.offset):
        raise SystemExit("offset must be finite")

    input_path = Path(args.input)
    output_path = Path(args.output)
    with input_path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames or []
        missing = [
            column for column in operation_columns if column not in fieldnames
        ]
        if missing:
            raise SystemExit(
                "missing perturbation columns: " + ",".join(missing)
            )
        if args.start_time_s is not None and args.time_column not in fieldnames:
            raise SystemExit(
                "time-window perturbation requires " + args.time_column
            )
        rows = list(reader)

    changed_cells = 0
    empty_cells = 0
    selected_rows = 0
    for row in rows:
        if args.start_time_s is not None:
            raw_time = row.get(args.time_column, "")
            if raw_time == "":
                raise SystemExit(
                    "empty " + args.time_column + " in time-window perturbation"
                )
            time_s = float(raw_time)
            if not math.isfinite(time_s):
                raise SystemExit(
                    "non-finite " + args.time_column + " in time-window perturbation"
                )
            if not (args.start_time_s <= time_s <= args.end_time_s):
                continue
        selected_rows += 1
        for column in offset_columns:
            raw = row.get(column, "")
            if raw == "":
                empty_cells += 1
                continue
            value = float(raw)
            if not math.isfinite(value):
                raise SystemExit(f"non-finite value in {column}")
            row[column] = format(value + args.offset, ".17g")
            changed_cells += 1
        for column, factor in scale_values.items():
            raw = row.get(column, "")
            if raw == "":
                empty_cells += 1
                continue
            value = float(raw)
            if not math.isfinite(value):
                raise SystemExit(f"non-finite value in {column}")
            row[column] = format(value * factor, ".17g")
            changed_cells += 1
        for column, value in set_values.items():
            if row.get(column, "") == "":
                empty_cells += 1
                continue
            row[column] = format(value, ".17g")
            changed_cells += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=reader.fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print("replay input perturbation created")
    print(f"input={input_path}")
    print(f"output={output_path}")
    print(f"columns={','.join(operation_columns)}")
    print(f"offset={args.offset}")
    print(f"set_columns={','.join(set_values)}")
    print(f"scale_columns={','.join(scale_values)}")
    print(f"start_time_s={args.start_time_s}")
    print(f"end_time_s={args.end_time_s}")
    print(f"time_column={args.time_column}")
    print(f"rows={len(rows)}")
    print(f"selected_rows={selected_rows}")
    print(f"changed_cells={changed_cells}")
    print(f"empty_cells={empty_cells}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
