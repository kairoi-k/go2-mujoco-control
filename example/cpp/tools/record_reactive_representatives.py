#!/usr/bin/env python3
"""Record long-window, human-facing reactive-event acceptance demos."""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path


EVENT_NAMES = {0: "none", 1: "emergency_stop", 2: "obstacle_left", 3: "obstacle_right", 4: "turn_left", 5: "turn_right", 6: "slip", 7: "low_friction", 8: "impact"}

SPECS = [
    {"id": "impact_strong_recovery", "title": "IMPACT | physical velocity impulse 0.80 m/s", "subtitle": "real MuJoCo push + automatic impact detection and recovery", "controller_duration": 16.0, "raw_duration": 17.0, "clip_start": 3.5, "clip_duration": 13.0, "fallback": [(3.0, 4.1, "PHYSICAL PUSH / IMPACT", "red")], "args": ["--reactive-events", "--push-time", "8.0", "--push-vel-x", "0.8", "--push-duration", "0.2"]},
    {"id": "slip_reference", "title": "SLIP | reference protection", "subtitle": "scripted velocity-mismatch response; no floor parameter change", "controller_duration": 16.0, "raw_duration": 17.0, "clip_start": 3.5, "clip_duration": 13.0, "event_script": "slip_reference.txt", "fallback": [(2.5, 6.5, "SLIP RESPONSE", "orange")], "args": []},
    {"id": "low_friction_physical", "title": "LOW FRICTION | physical floor change", "subtitle": "MuJoCo floor friction mu=0.02 for 4.0 s", "controller_duration": 16.0, "raw_duration": 17.0, "clip_start": 3.5, "clip_duration": 13.0, "fallback": [(4.0, 8.0, "FLOOR MU = 0.02", "blue")], "args": ["--reactive-events", "--friction-time", "8.0", "--friction-mu", "0.02", "--friction-duration", "4.0"]},
    {"id": "turn_left_long", "title": "TURN LEFT | yaw-only command", "subtitle": "continuous reference: yaw rate only, forward speed retained", "controller_duration": 16.0, "raw_duration": 17.0, "clip_start": 3.5, "clip_duration": 13.0, "event_script": "turn_left_long.txt", "fallback": [(2.5, 6.5, "TURN LEFT", "cyan")], "args": []},
    {"id": "obstacle_left_physical", "title": "OBSTACLE LEFT | physical box avoidance", "subtitle": "real collision geom; lane change + bounded yaw, zero contact required", "controller_duration": 18.0, "raw_duration": 19.0, "clip_start": 3.5, "clip_duration": 15.0, "event_script": "obstacle_left_physical.txt", "scene_file": "unitree_robots/go2/scene_reactive_obstacle.xml", "fallback": [(2.5, 10.5, "OBSTACLE LEFT / LANE CHANGE", "green")], "args": []},
    {"id": "obstacle_to_turn_handoff", "title": "HANDOFF | obstacle-left -> turn-left", "subtitle": "4.0 s obstacle response, 4.0 s turn response, then recovery", "controller_duration": 18.0, "raw_duration": 19.0, "clip_start": 3.5, "clip_duration": 15.0, "event_script": "obstacle_to_turn_handoff.txt", "scene_file": "unitree_robots/go2/scene_reactive_obstacle.xml", "fallback": [(2.5, 6.5, "OBSTACLE LEFT", "green"), (6.5, 10.5, "TURN LEFT", "cyan")], "args": []},
]


def esc(value: str) -> str:
    return value.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def color_for(name: str) -> str:
    if name in {"impact", "emergency_stop"}:
        return "red"
    if name.startswith("obstacle"):
        return "green"
    if name.startswith("turn"):
        return "cyan"
    return "orange"


def event_spans(run_dir: Path, clip_start: float, fallback: list[tuple[float, float, str, str]]) -> list[tuple[float, float, str, str]]:
    path = run_dir / "data.csv"
    if not path.exists():
        return fallback
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    spans: list[tuple[float, float, str, str]] = []
    previous = "none"
    started = 0.0
    for row in rows + [{"cmd_time_s": rows[-1].get("cmd_time_s", str(started)) if rows else str(started), "event_type": "0"}]:
        try:
            now = float(row.get("cmd_time_s", "nan"))
            current = EVENT_NAMES.get(int(float(row.get("event_type", "0"))), "unknown")
        except (TypeError, ValueError):
            continue
        if current == previous:
            continue
        if previous != "none" and now > started:
            spans.append((max(0.0, started - clip_start), max(0.05, now - clip_start), previous.replace("_", " ").upper(), color_for(previous)))
        previous, started = current, now
    return spans or fallback


def render(ffmpeg: Path, raw: Path, final: Path, spec: dict[str, object], spans: list[tuple[float, float, str, str]]) -> None:
    font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    filters = [
        "drawbox=x=0:y=0:w=iw:h=92:color=black@0.70:t=fill",
        f"drawtext=fontfile={font}:text='{esc(str(spec['title']))}':x=22:y=13:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.35:boxborderw=6",
        f"drawtext=fontfile={font}:text='{esc(str(spec['subtitle']))}':x=22:y=53:fontsize=17:fontcolor=white:box=1:boxcolor=black@0.35:boxborderw=5",
        "drawbox=x=0:y=ih-34:w=iw:h=34:color=black@0.55:t=fill",
    ]
    colors = {"red": "red@0.70", "orange": "orange@0.70", "blue": "blue@0.70", "cyan": "cyan@0.70", "green": "green@0.70"}
    for start, end, label, color in spans:
        start, end = max(0.0, start), max(start + 0.05, end)
        enable = f"between(t,{start:.3f},{end:.3f})"
        filters.append(f"drawbox=x=0:y=ih-34:w=iw:h=34:color={colors.get(color, 'orange@0.70')}:t=fill:enable='{enable}'")
        filters.append(f"drawtext=fontfile={font}:text='{esc(label)}':x=w/2-125:y=h-27:fontsize=18:fontcolor=white:enable='{enable}'")
    command = [str(ffmpeg), "-y", "-hide_banner", "-loglevel", "error", "-ss", str(spec["clip_start"]), "-i", str(raw), "-vf", ",".join(filters), "-t", str(spec["clip_duration"]), "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(final)]
    if subprocess.run(command).returncode != 0 or not final.exists() or final.stat().st_size < 100_000:
        raise RuntimeError(f"render failed: {final}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--scratch-root", type=Path)
    parser.add_argument("--only", nargs="*", choices=[str(s["id"]) for s in SPECS])
    parser.add_argument("--domain-base", type=int, default=210)
    parser.add_argument("--wall-timeout", type=float, default=35.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    record = repo / "example/cpp/scripts/record_reactive_acceptance.sh"
    ffmpeg = Path.home() / ".local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg"
    output_root = args.output_root.resolve()
    scratch_root = (args.scratch_root or output_root / ".scratch").resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "videos").mkdir(parents=True, exist_ok=True)
    scratch_root.mkdir(parents=True, exist_ok=True)
    wanted = set(args.only or [str(s["id"]) for s in SPECS])
    manifest_path = output_root / "representative_manifest.json"
    records: list[dict[str, object]] = []
    if manifest_path.exists():
        try:
            previous = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            previous = []
        if isinstance(previous, list):
            records = [item for item in previous if str(item.get("id")) not in wanted]
    for ordinal, spec in enumerate(SPECS):
        if str(spec["id"]) not in wanted:
            continue
        spec_id = str(spec["id"])
        script = None
        if spec.get("event_script"):
            script = repo / "example/cpp/experiments/go2_reactive_representatives_2026-08-20/event_scripts" / str(spec["event_script"])
        raw = scratch_root / f"{spec_id}.raw.mp4"
        final = output_root / "videos" / f"{spec_id}.mp4"
        run_name = f"_runs/reactive_representative_{spec_id}"
        run_dir = repo / "example/cpp/experiments" / run_name
        item: dict[str, object] = {"id": spec_id, "title": spec["title"], "subtitle": spec["subtitle"], "video": str(final), "run_directory": str(run_dir), "event_script": str(script) if script else None, "clip_start_s": spec["clip_start"], "clip_duration_s": spec["clip_duration"], "status": "pending"}
        if final.exists() and final.stat().st_size >= 100_000:
            spans = event_spans(run_dir, float(spec["clip_start"]), list(spec["fallback"]))
            item.update({"status": "pass", "size_bytes": final.stat().st_size, "resumed": True, "event_spans": [{"start_s": s, "end_s": e, "label": label, "color": color} for s, e, label, color in spans]})
            records.append(item)
            continue
        if args.dry_run:
            records.append(item)
            continue
        raw.unlink(missing_ok=True)
        final.unlink(missing_ok=True)
        env = os.environ.copy()
        env.update({"TROT_RECORD_DURATION_S": str(spec["raw_duration"]), "TROT_RECORD_FPS": "20", "TROT_RECORDING_GRACE_S": "1"})
        command = ["bash", str(record), str(args.wall_timeout), run_name, str(raw), "--wbc-full", "--step-length", "0.091", "--period", "0.60", "--duty", "0.75", "--foot-lift", "0.020", "--kernel", "raibert-trot", "--raibert-velocity-gain", "0.05", "--raibert-max-adjustment", "0.010", "--tau-limit", "35", "--controller-duration", str(spec["controller_duration"]), "--domain-id", str(args.domain_base + ordinal), "--camera-follow"]
        if script:
            command += ["--event-script", str(script)]
        if spec.get("scene_file"):
            command += ["--scene-file", str(spec["scene_file"])]
        command += [str(value) for value in spec["args"]]
        print(f"recording {spec_id}", flush=True)
        started_at = time.monotonic()
        result = subprocess.run(command, cwd=repo, env=env)
        if result.returncode != 0:
            item.update({"status": "record_failed", "returncode": result.returncode})
            records.append(item)
            manifest_path.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
            raise SystemExit(f"recording failed for {spec_id}")
        spans = event_spans(run_dir, float(spec["clip_start"]), list(spec["fallback"]))
        render(ffmpeg, raw, final, spec, spans)
        raw.unlink(missing_ok=True)
        item.update({"status": "pass", "size_bytes": final.stat().st_size, "elapsed_s": round(time.monotonic() - started_at, 2), "event_spans": [{"start_s": s, "end_s": e, "label": label, "color": color} for s, e, label, color in spans]})
        records.append(item)
        manifest_path.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest_path.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    passed = sum(item.get("status") == "pass" for item in records)
    print(f"representative suite complete: {passed}/{len(records)}", flush=True)
    return 0 if args.dry_run or passed == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
