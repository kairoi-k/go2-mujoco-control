#!/usr/bin/env python3
"""Burn a synchronized event/response overlay into a reactive demo video."""

from __future__ import annotations

import argparse
import sys
import math
import subprocess
from pathlib import Path

from analyze_reactive_acceptance import analyze


FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def escape_text(text: str) -> str:
    return (
        text.replace("\\", "\\\\")
        .replace(":", "\\:")
        .replace(",", "\\,")
        .replace("'", "\\'")
    )


def text_filter(
    text: str,
    x: str,
    y: str,
    size: int = 24,
    color: str = "white",
    enable: str | None = None,
) -> str:
    value = (
        f"fontfile={FONT}:text='{escape_text(text)}':"
        f"x={x}:y={y}:fontsize={size}:fontcolor={color}:"
        "box=1:boxcolor=black@0.55:boxborderw=8"
    )
    if enable:
        value += f":enable='{enable}'"
    return "drawtext=" + value


def num(value: object, default: float = math.nan) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment")
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--time-offset", type=float, default=0.0)
    parser.add_argument("--title", required=True)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--external-start", type=float)
    parser.add_argument("--external-end", type=float)
    parser.add_argument("--external-label")
    args = parser.parse_args()

    result = analyze(args.experiment)
    scheduled = result.get("scheduled_event")
    metrics = result.get("metrics", {})
    start = num(scheduled.get("start_s")) if scheduled else (args.external_start if args.external_start is not None else math.nan)
    end = num(scheduled.get("end_s")) if scheduled else (args.external_end if args.external_end is not None else math.nan)
    start -= args.time_offset
    end -= args.time_offset
    event_name = (
        str(scheduled["type"]).upper()
        if scheduled else (args.external_label or "NOMINAL").upper()
    )
    phase_text = (
        f"EVENT ACTIVE: {event_name}  [{start:.2f}-{end:.2f}s]"
        if math.isfinite(start) and math.isfinite(end)
        else f"EVENT: {event_name}"
    )
    strict = "PASS" if result.get("strict_pass") else ("N/A" if not scheduled else "CHECK")
    yaw = num(metrics.get("yaw_change_rad"))
    dv = num(metrics.get("braking_drop_mps"))
    ref_yaw = num(metrics.get("reference_yaw_rate_radps"))
    dy = num(metrics.get("lateral_shift_m"))
    contact_max = num(metrics.get("obstacle_contact_max_force_N"))
    if event_name.startswith("OBSTACLE") and math.isfinite(dy):
        response = (
            f"response: Δy={dy:+.3f} m | yaw_delta={yaw:+.3f} rad | "
            f"ref_yaw={ref_yaw:+.3f} rad/s"
            + (f" | obstacle_contact_max={contact_max:.1f} N"
               if math.isfinite(contact_max) else "")
        )
    elif math.isfinite(yaw) and math.isfinite(dv):
        response = (
            f"response: yaw_delta={yaw:+.3f} rad | "
            f"vx_drop={dv:.3f} m/s | ref_yaw={ref_yaw:+.3f} rad/s"
        )
    else:
        response = ("response: nominal walk; no event injected"
                    if not scheduled else
                    "response: physical disturbance; see synchronized CSV report")
    filters = [
        "drawbox=x=0:y=0:w=iw:h=86:color=black@0.65:t=fill",
        text_filter(
            f"REACTIVE ACCEPTANCE | {args.title.upper()}",
            "24",
            "12",
            26,
        ),
        text_filter(phase_text, "24", "48", 20, "yellow"),
        text_filter(f"DATA GATE: {strict} | {response}", "24", "h-48", 18),
    ]
    if math.isfinite(start) and math.isfinite(end):
        filters += [
            f"drawbox=x=0:y=ih-30:w=iw:h=30:color=red@0.55:t=fill:"
            f"enable='between(t,{start:.3f},{end:.3f})'",
            text_filter(
                "EVENT ACTIVE",
                "w/2-100",
                "h-26",
                18,
                "white",
                f"between(t,{start:.3f},{end:.3f})",
            ),
            text_filter(
                "RECOVERY / REFERENCE RAMP",
                "w/2-170",
                "h-26",
                18,
                "cyan",
                f"gte(t,{end:.3f})",
            ),
        ]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    trace_path = output.with_suffix(".trace.png")
    subprocess.run([sys.executable, str(Path(__file__).with_name("plot_reactive_trace.py")), args.experiment, str(trace_path), "--time-offset", str(args.time_offset)], check=True)
    filter_complex = f"[0:v]{','.join(filters)}[base];[1:v]format=rgba[trace];[base][trace]overlay=900:90:format=auto[v]"
    subprocess.run(
        [
            args.ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            args.input,
            "-loop",
            "1",
            "-i",
            str(trace_path),
            "-filter_complex",
            filter_complex,
            "-map",
            "[v]",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            "-t",
            f"{args.duration:.3f}",
            "-shortest",
            "-an",
            str(output),
        ],
        check=True,
    )
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
