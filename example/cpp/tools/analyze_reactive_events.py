#!/usr/bin/env python3
"""Summarize reactive WBC/MPC experiment evidence without external packages."""

import argparse
import csv
import json
import math
from pathlib import Path

EVENT_NAMES = {
    0: "none", 1: "emergency_stop", 2: "obstacle_left",
    3: "obstacle_right", 4: "turn_left", 5: "turn_right",
    6: "slip", 7: "low_friction", 8: "impact",
}


def read_metadata(path):
    values = {}
    file = path / "run_metadata.txt"
    if file.exists():
        for line in file.read_text().splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                values[key] = value
    return values


def finite_values(rows, key):
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def analyze(path):
    path = Path(path)
    with (path / "data.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    metadata = read_metadata(path)
    transitions = []
    previous = None
    for row in rows:
        try:
            event = int(float(row.get("event_type", 0)))
            time_s = float(row.get("cmd_time_s", 0.0))
        except (TypeError, ValueError):
            continue
        if event != previous:
            transitions.append({
                "time_s": round(time_s, 3),
                "type": EVENT_NAMES.get(event, f"unknown_{event}"),
                "priority": int(float(row.get("event_priority", 0))),
            })
            previous = event
    roll = finite_values(rows, "imu_roll_rad")
    pitch = finite_values(rows, "imu_pitch_rad")
    return {
        "experiment": path.name,
        "rows": len(rows),
        "controller_status": int(metadata.get("controller_status", -1)),
        "safety_status": int(metadata.get("safety_status", -1)),
        "quality_status": int(metadata.get("quality_status", -1)),
        "completion_status": int(metadata.get("completion_status", -1)),
        "max_abs_roll_deg": round(max((abs(x) for x in roll), default=0.0) * 180.0 / math.pi, 3),
        "max_abs_pitch_deg": round(max((abs(x) for x in pitch), default=0.0) * 180.0 / math.pi, 3),
        "event_transitions": transitions,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", nargs="+")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    results = [analyze(item) for item in args.experiment]
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
        return
    for result in results:
        print(
            f"{result['experiment']}: status="
            f"{result['controller_status']}/{result['safety_status']}/"
            f"{result['quality_status']}, "
            f"max_roll={result['max_abs_roll_deg']:.2f}deg, "
            f"max_pitch={result['max_abs_pitch_deg']:.2f}deg, "
            f"events={','.join(item['type'] for item in result['event_transitions'])}"
        )


if __name__ == "__main__":
    main()
