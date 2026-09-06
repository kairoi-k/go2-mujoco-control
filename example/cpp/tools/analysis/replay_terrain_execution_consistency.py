#!/usr/bin/env python3
"""Replay CSV evidence for the opt-in terrain planner-consumer shadow."""
import argparse
import csv
import json
import math
import pathlib
import sys

REQUIRED = (
    "state_tick_s", "terrain_plan_id", "terrain_plan_epoch",
    "terrain_plan_valid", "terrain_model_com_valid",
    "terrain_model_com_state_stamp_s",
    "wbc_terrain_execution_shadow_enabled",
    "wbc_terrain_execution_shadow_checked",
    "wbc_terrain_execution_snapshot_valid",
    "wbc_terrain_execution_shadow_rejection_code",
    "wbc_terrain_execution_shadow_plan_id",
    "wbc_terrain_execution_shadow_plan_epoch",
)


def number(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(name + " is not finite")
    return value


def inspect_csv(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = [name for name in REQUIRED if name not in fields]
        if missing:
            return {"status": "missing_fields", "path": str(path),
                    "missing": missing}
        rows = 0
        com_valid = 0
        com_timestamp_mismatch = 0
        shadow_enabled = 0
        shadow_checked = 0
        shadow_valid = 0
        shadow_rejected = 0
        errors = []
        for row in reader:
            rows += 1
            state_time = number(row, "state_tick_s")
            if int(number(row, "terrain_model_com_valid")):
                com_valid += 1
                com_time = number(row, "terrain_model_com_state_stamp_s")
                if abs(com_time - state_time) > 1.0e-6:
                    com_timestamp_mismatch += 1
            if not int(number(
                    row, "wbc_terrain_execution_shadow_enabled")):
                continue
            shadow_enabled += 1
            checked = int(number(
                row, "wbc_terrain_execution_shadow_checked"))
            valid = int(number(
                row, "wbc_terrain_execution_snapshot_valid"))
            rejection = int(number(
                row, "wbc_terrain_execution_shadow_rejection_code"))
            if checked:
                shadow_checked += 1
            if valid:
                shadow_valid += 1
                if not checked:
                    errors.append("valid sample was not checked")
                if number(row, "wbc_terrain_execution_shadow_plan_id") <= 0:
                    errors.append("valid sample has no plan id")
                if number(row, "wbc_terrain_execution_shadow_plan_epoch") <= 0:
                    errors.append("valid sample has no plan epoch")
            elif checked:
                shadow_rejected += 1
                if rejection == 0:
                    errors.append("checked invalid sample has no rejection")
            elif valid:
                errors.append("unchecked sample was marked valid")
        result = {
            "status": "pass" if not errors and
                com_timestamp_mismatch == 0 else "fail",
            "path": str(path), "rows": rows,
            "model_com_valid_rows": com_valid,
            "model_com_timestamp_mismatch_rows": com_timestamp_mismatch,
            "shadow_enabled_rows": shadow_enabled,
            "shadow_checked_rows": shadow_checked,
            "shadow_valid_rows": shadow_valid,
            "shadow_rejected_rows": shadow_rejected,
            "errors": errors,
        }
        if shadow_enabled and shadow_valid == 0:
            result["status"] = "fail"
            result["errors"].append(
                "shadow enabled but no checked sample passed")
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=pathlib.Path)
    parser.add_argument("--expect-missing-model-com", action="store_true")
    args = parser.parse_args()
    result = inspect_csv(args.csv)
    print(json.dumps(result, sort_keys=True))
    if result["status"] == "missing_fields":
        return 0 if args.expect_missing_model_com else 2
    return 0 if result["status"] == "pass" else 1


if __name__ == "__main__":
    sys.exit(main())
