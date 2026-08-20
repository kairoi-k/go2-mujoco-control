#!/usr/bin/env python3
"""Record and package GUI demos for the directed reactive transition matrix."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path


def escape_drawtext(value: str) -> str:
    return value.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--scratch-root", type=Path)
    parser.add_argument("--start-index", type=int, default=1)
    parser.add_argument("--count", type=int, default=49)
    parser.add_argument("--domain-base", type=int, default=120)
    parser.add_argument("--raw-duration", type=float, default=11.0)
    parser.add_argument("--clip-start", type=float, default=3.5)
    parser.add_argument("--clip-duration", type=float, default=7.0)
    parser.add_argument("--wall-timeout", type=float, default=25.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo = Path(__file__).resolve().parents[3]
    record = repo / "example/cpp/scripts/record_reactive_acceptance.sh"
    ffmpeg = Path.home() / ".local/share/unitree_mujoco_capture_tools/tools/ffmpeg-static/bin/ffmpeg"
    manifests = sorted((args.matrix_root / "manifests").glob("*.json"))
    selected = []
    for path in manifests:
        item = json.loads(path.read_text(encoding="utf-8"))
        index = int(item["pair_index"])
        if args.start_index <= index < args.start_index + args.count:
            selected.append((index, path, item))
    selected.sort(key=lambda item: item[0])
    if len(selected) != args.count:
        raise SystemExit(f"expected {args.count} manifests, found {len(selected)}")
    args.output_root.mkdir(parents=True, exist_ok=True)
    manifest_out = args.output_root / "video_manifest.json"
    scratch_root = args.scratch_root or args.output_root
    scratch_root.mkdir(parents=True, exist_ok=True)
    selected_indices = {item[0] for item in selected}
    records: list[dict[str, object]] = []
    if manifest_out.exists():
        try:
            previous = json.loads(manifest_out.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            previous = []
        if isinstance(previous, list):
            records = [
                item for item in previous
                if int(item.get("pair_index", -1)) not in selected_indices
            ]

    for ordinal, (index, _manifest_path, item) in enumerate(selected):
        run_id = str(item["run_id"])
        source = str(item["source_event"])
        target = str(item["target_event"])
        event_script = Path(str(item["event_script"]))
        if not event_script.is_absolute():
            event_script = repo / event_script
        raw = scratch_root / "raw" / f"{index:03d}_{run_id}.mp4"
        final = args.output_root / "videos" / f"{index:03d}_{run_id}.mp4"
        run_name = f"_runs/reactive_transition_video_{index:03d}_{run_id}"
        domain = args.domain_base + ordinal
        record_item: dict[str, object] = {
            "pair_index": index,
            "run_id": run_id,
            "source_event": source,
            "target_event": target,
            "event_script": str(event_script),
            "raw_video": str(raw),
            "video": str(final),
            "domain_id": domain,
            "clip_start_s": args.clip_start,
            "clip_duration_s": args.clip_duration,
            "status": "pending",
        }
        if final.exists() and final.stat().st_size >= 100_000:
            record_item.update({
                "status": "pass",
                "size_bytes": final.stat().st_size,
                "resumed": True,
            })
            records.append(record_item)
            continue
        records.append(record_item)
        if args.dry_run:
            continue
        raw.parent.mkdir(parents=True, exist_ok=True)
        final.parent.mkdir(parents=True, exist_ok=True)
        raw.unlink(missing_ok=True)
        final.unlink(missing_ok=True)
        env = os.environ.copy()
        env.update({
            "TROT_RECORD_DURATION_S": str(args.raw_duration),
            "TROT_RECORD_FPS": "20",
            "TROT_RECORDING_GRACE_S": "1",
        })
        command = [
            "bash", str(record), str(args.wall_timeout), run_name, str(raw),
            "--wbc-full", "--step-length", "0.091", "--period", "0.60",
            "--duty", "0.75", "--foot-lift", "0.020", "--kernel", "raibert-trot",
            "--raibert-velocity-gain", "0.05", "--raibert-max-adjustment", "0.010",
            "--tau-limit", "35", "--controller-duration", "10",
            "--event-script", str(event_script), "--domain-id", str(domain),
            "--camera-follow",
        ]
        print(f"[{index:03d}/049] {source} -> {target}", flush=True)
        started = time.monotonic()
        result = subprocess.run(command, cwd=repo, env=env)
        if result.returncode != 0:
            record_item.update({"status": "record_failed", "returncode": result.returncode})
            manifest_out.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
            raise SystemExit(f"recording failed for {run_id}: {result.returncode}")

        title = f"REACTIVE TRANSITION {index:03d} | {source} -> {target}"
        label_a = f"A: {source.replace('_', ' ').upper()}  1.5-3.5 s"
        label_b = f"B: {target.replace('_', ' ').upper()}  3.5-5.5 s"
        font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        filter_text = (
            "drawbox=x=0:y=0:w=iw:h=88:color=black@0.65:t=fill,"
            f"drawtext=fontfile={font}:text='{escape_drawtext(title)}':x=22:y=14:fontsize=24:"
            "fontcolor=white:box=1:boxcolor=black@0.35:boxborderw=6,"
            f"drawtext=fontfile={font}:text='{escape_drawtext(label_a)}':x=22:y=51:fontsize=18:"
            "fontcolor=yellow:box=1:boxcolor=black@0.35:boxborderw=5,"
            f"drawtext=fontfile={font}:text='{escape_drawtext(label_b)}':x=390:y=51:fontsize=18:"
            "fontcolor=cyan:box=1:boxcolor=black@0.35:boxborderw=5,"
            "drawbox=x=0:y=ih-32:w=iw:h=32:color=red@0.55:t=fill:enable='between(t,1.5,3.5)',"
            "drawbox=x=0:y=ih-32:w=iw:h=32:color=orange@0.55:t=fill:enable='between(t,3.5,5.499)',"
            f"drawtext=fontfile={font}:text='EVENT A ACTIVE':x=w/2-90:y=h-25:"
            "fontsize=18:fontcolor=white:enable='between(t,1.5,3.5)',"
            f"drawtext=fontfile={font}:text='EVENT B ACTIVE':x=w/2-90:y=h-25:"
            "fontsize=18:fontcolor=white:enable='between(t,3.5,5.5)',"
            f"drawtext=fontfile={font}:text='WBC STANCE HOLD / RECOVERY':x=w/2-170:y=h-25:"
            "fontsize=18:fontcolor=white:enable='gt(t,5.5)'"
        )
        ffmpeg_command = [
            str(ffmpeg), "-y", "-hide_banner", "-loglevel", "error",
            "-ss", str(args.clip_start), "-i", str(raw), "-vf", filter_text,
            "-t", str(args.clip_duration), "-an", "-c:v", "libx264",
            "-preset", "medium", "-crf", "18", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(final),
        ]
        render = subprocess.run(ffmpeg_command, cwd=repo)
        if render.returncode != 0 or not final.exists() or final.stat().st_size < 100_000:
            record_item.update({"status": "render_failed", "returncode": render.returncode})
            manifest_out.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
            raise SystemExit(f"rendering failed for {run_id}")
        raw.unlink(missing_ok=True)
        record_item.update({
            "status": "pass",
            "size_bytes": final.stat().st_size,
            "elapsed_s": round(time.monotonic() - started, 2),
        })
        manifest_out.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")

    manifest_out.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    passed = sum(item.get("status") == "pass" for item in records)
    print(f"video batch complete: {passed}/{len(records)}", flush=True)
    return 0 if args.dry_run or passed == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())

