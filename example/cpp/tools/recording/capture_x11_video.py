#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time
from pathlib import Path

from capture_x11_gif import X11Capture


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--ffmpeg", type=Path, required=True)
    parser.add_argument("--title", default="MuJoCo")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--stop-file", type=Path)
    parser.add_argument("--post-roll", type=float, default=2.0)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--crop-x", type=int, default=0)
    parser.add_argument("--crop-y", type=int, default=0)
    parser.add_argument("--crop-width", type=int, default=1280)
    parser.add_argument("--crop-height", type=int, default=720)
    parser.add_argument("--output-width", type=int, default=1920)
    parser.add_argument("--output-height", type=int, default=1080)
    parser.add_argument("--preset", default="medium")
    parser.add_argument("--crf", default="18")
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(args.ffmpeg),
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{args.crop_width}x{args.crop_height}",
        "-framerate",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-vf",
        (
            f"scale={args.output_width}:{args.output_height}:"
            "flags=lanczos"
        ),
        "-c:v",
        "libx264",
        "-preset",
        args.preset,
        "-crf",
        args.crf,
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(args.output),
    ]

    capture = X11Capture()
    encoder = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    frame_count = 0
    try:
        window = capture.find_window(args.title)
        period = 1.0 / args.fps
        next_frame = time.monotonic()
        deadline = (
            next_frame + args.duration
            if args.duration is not None
            else None
        )
        stop_seen_at = None
        crop_box = (
            args.crop_x,
            args.crop_y,
            args.crop_x + args.crop_width,
            args.crop_y + args.crop_height,
        )
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                break
            if args.stop_file and args.stop_file.exists():
                if stop_seen_at is None:
                    stop_seen_at = now
                elif now - stop_seen_at >= args.post_roll:
                    break
            frame = capture.frame(window).crop(crop_box)
            encoder.stdin.write(frame.tobytes())
            frame_count += 1
            next_frame += period
            time.sleep(max(0.0, next_frame - time.monotonic()))
    except BrokenPipeError:
        pass
    finally:
        capture.close()
        if encoder.stdin:
            encoder.stdin.close()
        stderr = encoder.stderr.read().decode(errors="replace")
        return_code = encoder.wait()

    if return_code:
        print(stderr, file=sys.stderr)
        raise SystemExit(return_code)
    print(f"Saved {frame_count} frames to {args.output}")


if __name__ == "__main__":
    main()
