#!/usr/bin/env python3
"""Record the exhaustive directed transition matrix with human-readable windows."""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import time
from pathlib import Path

from record_reactive_representatives import event_spans, render


OBSTACLE_SCENE = "unitree_robots/go2/scene_reactive_obstacle.xml"
DEFAULT_MAGNITUDES = {"obstacle_left": 0.48, "obstacle_right": 0.48, "turn_left": 0.55, "turn_right": 0.55, "slip": 0.0, "low_friction": 0.0, "impact": 0.0, "emergency_stop": 0.0}


def parse_magnitude(path: Path, event_name: str) -> float:
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            tokens = line.split("#", 1)[0].split()
            if len(tokens) >= 4 and tokens[2] == event_name:
                try:
                    return float(tokens[3])
                except ValueError:
                    break
    return DEFAULT_MAGNITUDES.get(event_name, 0.0)


def make_script(path: Path, source: str, target: str, source_mag: float, target_mag: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "# Generated long-window protocol: times are relative to walking controller clock.\n"
        f"2.5 4.0 {source} {source_mag:.6f}\n"
        f"6.5 4.0 {target} {target_mag:.6f}\n",
        encoding="utf-8",
    )


def obstacle_metrics(run_dir: Path, uses_obstacle: bool) -> tuple[float | None, float | None, bool | None]:
    if not uses_obstacle:
        return None, None, None
    path = run_dir / "contact_ground_truth.csv"
    if not path.exists():
        return None, None, False
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    force = max(float(row.get("reactive_obstacle_contact_force_N", "0")) for row in rows)
    count = max(float(row.get("reactive_obstacle_contact_count", "0")) for row in rows)
    return force, count, force <= 1.0e-6 and count <= 0.0


def media_duration(ffprobe: Path, path: Path) -> float:
    result = subprocess.run(
        [str(ffprobe), "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"ffprobe failed: {path}")
    try:
        return float(result.stdout.strip())
    except ValueError as exc:
        raise RuntimeError(f"invalid duration: {path}") from exc


def write_index(path: Path, records: list[dict[str, object]]) -> None:
    fields = ["pair_index", "run_id", "source_event", "target_event", "status", "video", "duration_s", "obstacle_contact_force_max_N", "obstacle_contact_count_max", "obstacle_contact_pass"]
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in records:
            writer.writerow({field: item.get(field, "") for field in fields})


def build_sequential(ffmpeg: Path, output_root: Path, records: list[dict[str, object]]) -> None:
    final = output_root / "reactive_transition_matrix_49_long_sequential.mp4"
    list_path = output_root / ".concat_list.txt"
    entries = []
    for item in records:
        video = Path(str(item["video"]))
        if not video.is_absolute():
            video = output_root / video
        entries.append(f"file '{video.resolve()}'\n")
    list_path.write_text("".join(entries), encoding="utf-8")
    command = [str(ffmpeg), "-y", "-hide_banner", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(list_path), "-c", "copy", "-movflags", "+faststart", str(final)]
    result = subprocess.run(command)
    list_path.unlink(missing_ok=True)
    if result.returncode != 0 or not final.exists() or final.stat().st_size < 1_000_000:
        raise RuntimeError("sequential montage failed")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--protocol-root", type=Path, required=True)
    parser.add_argument("--scratch-root", type=Path)
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--count", type=int, default=49)
    parser.add_argument("--domain-base", type=int, default=120)
    parser.add_argument("--raw-duration", type=float, default=19.0)
    parser.add_argument("--clip-start", type=float, default=3.5)
    parser.add_argument("--clip-duration", type=float, default=15.0)
    parser.add_argument("--controller-duration", type=float, default=18.0)
    parser.add_argument("--wall-timeout", type=float, default=40.0)
    parser.add_argument("--physical-obstacle", action="store_true", help="Use the collision-box scene; the default matrix isolates reference handoffs on the nominal floor.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[3]
    record_script = repo / "example/cpp/scripts/record_reactive_acceptance.sh"
    ffmpeg = Path.home() / ".local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg"
    ffprobe = ffmpeg.with_name("ffprobe")
    manifests = sorted((args.matrix_root / "manifests").glob("*.json"))
    selected: list[dict[str, object]] = []
    for path in manifests:
        item = json.loads(path.read_text(encoding="utf-8"))
        index = int(item["pair_index"])
        if args.start_index <= index < args.start_index + args.count:
            selected.append(item)
    selected.sort(key=lambda item: int(item["pair_index"]))
    if len(selected) != args.count:
        raise SystemExit(f"expected {args.count} source manifests, found {len(selected)}")
    args.output_root.mkdir(parents=True, exist_ok=True)
    (args.output_root / "videos").mkdir(parents=True, exist_ok=True)
    scratch = args.scratch_root or (args.output_root / ".scratch")
    scratch.mkdir(parents=True, exist_ok=True)
    protocol = args.protocol_root.resolve()
    event_dir = protocol / "event_scripts"
    event_dir.mkdir(parents=True, exist_ok=True)
    selected_indices = {int(item["pair_index"]) for item in selected}
    manifest_path = args.output_root / "video_manifest.json"
    records: list[dict[str, object]] = []
    if manifest_path.exists():
        try:
            previous = json.loads(manifest_path.read_text(encoding="utf-8"))
            if isinstance(previous, list):
                records = [
                    item for item in previous
                    if isinstance(item, dict)
                    and int(item.get("pair_index", -1)) not in selected_indices
                    and item.get("status") == "pass"
                ]
        except (OSError, ValueError, TypeError):
            records = []

    for ordinal, source_item in enumerate(selected):
        index = int(source_item["pair_index"])
        run_id = str(source_item["run_id"])
        source = str(source_item["source_event"])
        target = str(source_item["target_event"])
        original_script = Path(str(source_item["event_script"]))
        if not original_script.is_absolute():
            original_script = repo / original_script
        source_mag = parse_magnitude(original_script, source)
        target_mag = parse_magnitude(original_script, target)
        event_script = event_dir / f"{index:03d}_{run_id}.txt"
        make_script(event_script, source, target, source_mag, target_mag)
        uses_obstacle = args.physical_obstacle and ("obstacle" in source or "obstacle" in target)
        raw = scratch / f"{index:03d}_{run_id}.raw.mp4"
        final = args.output_root / "videos" / f"{index:03d}_{run_id}.mp4"
        run_name = f"_runs/reactive_transition_long_{index:03d}_{run_id}"
        run_dir = repo / "example/cpp/experiments" / run_name
        fallback = [(2.5, 6.5, f"A: {source.replace('_', ' ').upper()}", "green" if "obstacle" in source else "orange"), (6.5, 10.5, f"B: {target.replace('_', ' ').upper()}", "green" if "obstacle" in target else "cyan")]
        spec = {"title": f"REACTIVE TRANSITION {index:03d} | {source.replace('_', ' ').upper()} -> {target.replace('_', ' ').upper()}", "subtitle": "A and B: 4.0 s each | continuous reference handoff | recovery", "clip_start": args.clip_start, "clip_duration": args.clip_duration}
        item: dict[str, object] = {"protocol_name": "go2_reactive_transition_matrix_long", "protocol_version": "1.0", "pair_index": index, "run_id": run_id, "source_event": source, "target_event": target, "event_script": str(event_script.relative_to(protocol)), "run_directory": str(run_dir), "video": str(final.relative_to(args.output_root)), "scene_file": OBSTACLE_SCENE if uses_obstacle else "unitree_robots/go2/scene_leg_lift_demo.xml", "status": "pending"}
        if final.exists() and final.stat().st_size >= 100_000 and run_dir.joinpath("data.csv").exists():
            spans = event_spans(run_dir, args.clip_start, fallback)
            force, count, passed = obstacle_metrics(run_dir, uses_obstacle)
            duration = media_duration(ffprobe, final)
            item.update({"status": "pass", "size_bytes": final.stat().st_size, "duration_s": round(duration, 3), "resumed": True, "event_spans": [{"start_s": s, "end_s": e, "label": label, "color": color} for s, e, label, color in spans], "obstacle_contact_force_max_N": force, "obstacle_contact_count_max": count, "obstacle_contact_pass": passed})
            records.append(item)
            continue
        if args.dry_run:
            records.append(item)
            continue
        raw.unlink(missing_ok=True)
        final.unlink(missing_ok=True)
        env = os.environ.copy()
        env.update({"TROT_RECORD_DURATION_S": str(args.raw_duration), "TROT_RECORD_FPS": "20", "TROT_RECORDING_GRACE_S": "1"})
        command = ["bash", str(record_script), str(args.wall_timeout), run_name, str(raw), "--wbc-full", "--step-length", "0.091", "--period", "0.60", "--duty", "0.75", "--foot-lift", "0.020", "--kernel", "raibert-trot", "--raibert-velocity-gain", "0.05", "--raibert-max-adjustment", "0.010", "--tau-limit", "35", "--controller-duration", str(args.controller_duration), "--event-script", str(event_script), "--domain-id", str(args.domain_base + ordinal), "--camera-follow"]
        if uses_obstacle:
            command += ["--scene-file", OBSTACLE_SCENE]
        print(f"[{index:03d}/049] {source} -> {target}", flush=True)
        started = time.monotonic()
        result = subprocess.run(command, cwd=repo, env=env)
        if result.returncode != 0:
            manifest_path.write_text(json.dumps(sorted(records, key=lambda entry: int(entry["pair_index"])), ensure_ascii=False, indent=2), encoding="utf-8")
            raise SystemExit(f"recording failed for {run_id}")
        spans = event_spans(run_dir, args.clip_start, fallback)
        render(ffmpeg, raw, final, spec, spans)
        raw.unlink(missing_ok=True)
        force, count, passed = obstacle_metrics(run_dir, uses_obstacle)
        duration = media_duration(ffprobe, final)
        if abs(duration - args.clip_duration) > 0.05:
            raise RuntimeError(f"unexpected clip duration {duration:.3f}s: {final}")
        item.update({"status": "pass", "size_bytes": final.stat().st_size, "duration_s": round(duration, 3), "elapsed_s": round(time.monotonic() - started, 2), "event_spans": [{"start_s": s, "end_s": e, "label": label, "color": color} for s, e, label, color in spans], "obstacle_contact_force_max_N": force, "obstacle_contact_count_max": count, "obstacle_contact_pass": passed})
        records.append(item)
        manifest_path.write_text(json.dumps(sorted(records, key=lambda entry: int(entry["pair_index"])), ensure_ascii=False, indent=2), encoding="utf-8")

    records.sort(key=lambda item: int(item["pair_index"]))
    manifest_path.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    write_index(args.output_root / "video_index.csv", records)
    if not args.dry_run and args.count == 49 and len(records) == 49:
        build_sequential(ffmpeg, args.output_root, records)
    print(f"long matrix video batch complete: {len(records)}/{args.count}", flush=True)
    return 0 if args.dry_run or all(item.get("status") == "pass" for item in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
