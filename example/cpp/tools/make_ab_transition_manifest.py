#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--long-manifest", type=Path, required=True)
    parser.add_argument("--representative-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--video-root",
        type=Path,
        default=None,
        help="optional local root for relative video paths; omit for portable output",
    )
    args = parser.parse_args()
    long_records = json.loads(args.long_manifest.read_text(encoding="utf-8"))
    representative_records = json.loads(args.representative_manifest.read_text(encoding="utf-8"))
    selected_indices = [2, 18, 34, 40, 43]
    selected = []
    by_index = {int(item["pair_index"]): item for item in long_records}
    for index in selected_indices:
        source = by_index[index]
        source_event = str(source["source_event"])
        target_event = str(source["target_event"])
        video = source["video"]
        if args.video_root is not None and not Path(video).is_absolute():
            video = str(args.video_root / video)
        selected.append({
            "id": f"pair_{index:03d}_{source_event}_to_{target_event}",
            "title": f"AB {index:03d} | {source_event.replace('_', ' ').upper()} -> {target_event.replace('_', ' ').upper()}",
            "subtitle": "scheduled reference handoff | A 4.0 s + B 4.0 s + recovery",
            "video": video,
            "run_directory": source["run_directory"],
            "clip_start_s": 3.5,
            "clip_duration_s": float(source["duration_s"]),
            "event_spans": source["event_spans"],
            "source_event": source_event,
            "target_event": target_event,
            "scene_file": source.get("scene_file"),
        })
    physical = next(item for item in representative_records if item["id"] == "obstacle_to_turn_handoff")
    selected.insert(0, {
        "id": "physical_obstacle_left_to_turn_left",
        "title": "AB PHYSICAL | OBSTACLE LEFT -> TURN LEFT",
        "subtitle": "real collision box | A 4.0 s + B 4.0 s + recovery",
        "video": physical["video"],
        "run_directory": physical["run_directory"],
        "clip_start_s": physical["clip_start_s"],
        "clip_duration_s": physical["clip_duration_s"],
        "event_spans": physical["event_spans"],
        "source_event": "obstacle_left",
        "target_event": "turn_left",
        "scene_file": "unitree_robots/go2/scene_reactive_obstacle.xml",
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(selected, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"selected AB representative clips: {len(selected)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
