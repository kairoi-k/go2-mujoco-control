"""Deterministic, task-shaped benchmark for the Atlas local diagnosis server.

This script only calls an OpenAI-compatible loopback endpoint. The server
process and its model/runtime are deliberately managed outside the benchmark
so each candidate can be started with identical llama.cpp flags.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import time
from pathlib import Path
from typing import Any
from urllib import error as urlerror
from urllib import request as urlrequest


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = json.loads((ROOT / "schemas" / "local_diagnosis.schema.json").read_text(encoding="utf-8"))
ENUM_CLASSES = {
    "PASS_DEV",
    "FAIL_TIMING",
    "FAIL_CONTACT",
    "FAIL_WBC",
    "FAIL_PLANNER",
    "FAIL_SAFE_STOP",
    "RUNNER_FAILURE",
    "UNKNOWN",
}


def _case(case_id: str, expected: str, verdict: str, status: str, metrics: dict[str, Any], notes: str) -> dict[str, Any]:
    return {
        "case_id": case_id,
        "expected": expected,
        "bundle": {
            "experiment": {
                "experiment_id": f"bench-{case_id}",
                "question": "Classify the observed Go2 MuJoCo development result.",
                "profile": "b0-development",
                "duration_s": 40.0,
                "seed": 42,
                "parameters": {"scenario": "accel_1_to_3", "domain_id": 190},
            },
            "result": {
                "status": status,
                "verdict": verdict,
                "runner": "atlas",
                "duration_s": 40.0,
                "metrics": metrics,
                "notes": [notes],
            },
            "deterministic_diagnosis": {
                "failure_class": expected,
                "confidence": 1.0 if expected != "UNKNOWN" else 0.0,
                "summary": "Deterministic baseline from the bounded result classifier.",
                "requires_codex": expected == "UNKNOWN",
            },
        },
    }


CASES = [
    _case("pass", "PASS_DEV", "PASS_DEV", "completed", {"safe_stop": False, "wall_clock_rate_hz": 498.7}, "All lifecycle and quality statuses are zero."),
    _case("timing", "FAIL_TIMING", "FAIL_TIMING", "failed", {"safe_stop": False, "first_failure_s": 12.4, "wall_clock_rate_hz": 301.2, "extra": {"quality_status": 1}}, "The quality guard rejected a late control update."),
    _case("contact", "FAIL_CONTACT", "FAIL_CONTACT", "failed", {"safe_stop": False, "first_failure_s": 9.8, "extra": {"contact_consistency": 0.21}}, "Contact phase and measured support disagree while timing remains healthy."),
    _case("wbc", "FAIL_WBC", "FAIL_WBC", "failed", {"safe_stop": False, "first_failure_s": 18.1, "wbc_saturation_fraction": 0.94}, "The whole-body controller reaches sustained torque saturation."),
    _case("planner", "FAIL_PLANNER", "FAIL_PLANNER", "failed", {"safe_stop": False, "first_failure_s": 6.2, "extra": {"terrain_analysis_status": 1}}, "Terrain analysis produced no usable plan."),
    _case("safety", "FAIL_SAFE_STOP", "FAIL_SAFE_STOP", "failed", {"safe_stop": True, "first_failure_s": 3.0}, "The hard posture safety guard stopped the run."),
    _case("runner", "RUNNER_FAILURE", "RUNNER_FAILURE", "failed", {"safe_stop": False, "extra": {"controller_status": 127}}, "The runner exited before a readable manifest was produced."),
    _case("conflict", "UNKNOWN", "UNKNOWN", "completed", {"safe_stop": False, "first_failure_s": 7.0, "wall_clock_rate_hz": 499.0, "extra": {"quality_status": 0, "safety_status": 0, "terrain_analysis_status": 0}}, "The artifact claims both a pass and a contradictory unexplained anomaly."),
]


def _post(base_url: str, model: str, prompt: str, timeout_s: float) -> tuple[str, dict[str, Any]]:
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": "Output only schema-valid JSON. Do not add prose."},
            {"role": "user", "content": prompt},
        ],
        "temperature": 0.1,
        "top_p": 0.9,
        "seed": 42,
        "max_tokens": 1536,
        "stream": False,
        "response_format": {
            "type": "json_schema",
            "json_schema": {"name": "go2_local_diagnosis", "strict": True, "schema": SCHEMA},
        },
    }
    request = urlrequest.Request(
        f"{base_url.rstrip('/')}/v1/chat/completions",
        data=json.dumps(body, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlrequest.urlopen(request, timeout=timeout_s) as response:
        payload = json.loads(response.read(100_000).decode("utf-8"))
    message = payload["choices"][0]["message"]
    content = message.get("content", "")
    if not isinstance(content, str):
        content = json.dumps(content, ensure_ascii=False)
    return content, payload


def _parse_object(content: str) -> dict[str, Any]:
    value = json.loads(content.strip().strip("`"))
    if not isinstance(value, dict):
        raise ValueError("response is not an object")
    return value


def _valid_shape(value: dict[str, Any]) -> bool:
    required = set(SCHEMA["required"])
    return (
        set(value) == required
        and value.get("failure_class") in ENUM_CLASSES
        and isinstance(value.get("summary"), str)
        and isinstance(value.get("evidence"), list)
        and isinstance(value.get("needs_human_review"), bool)
        and isinstance(value.get("confidence"), (int, float))
        and 0 <= float(value["confidence"]) <= 1
    )


def _load_real_case(path: Path) -> dict[str, Any] | None:
    payload = json.loads(path.read_text(encoding="utf-8"))
    result = payload.get("result", payload) if isinstance(payload, dict) else None
    if not isinstance(result, dict):
        return None
    verdict = str(result.get("verdict", "UNKNOWN"))
    expected = verdict if verdict in ENUM_CLASSES else "UNKNOWN"
    return _case("real_artifact", expected, verdict, str(result.get("status", "failed")), result.get("metrics", {}), "Real result.v1 artifact supplied by the Atlas workflow.")


def run(base_url: str, model: str, timeout_s: float, real_case: Path | None) -> dict[str, Any]:
    cases = list(CASES)
    if real_case is not None:
        case = _load_real_case(real_case)
        if case is not None:
            cases.append(case)
    rows: list[dict[str, Any]] = []
    for case in cases:
        prompt = (
            "Diagnose this bounded JSON bundle. Use only observed evidence. "
            "If it is insufficient or contradictory, return UNKNOWN and escalate. "
            "Never propose changing controller algorithms, thresholds, or physics.\n"
            + json.dumps(case["bundle"], ensure_ascii=False, sort_keys=True)
        )
        started = time.perf_counter()
        content = ""
        response: dict[str, Any] = {}
        row: dict[str, Any] = {"case_id": case["case_id"], "expected": case["expected"]}
        try:
            content, response = _post(base_url, model, prompt, timeout_s)
            latency_ms = int(round((time.perf_counter() - started) * 1000))
            parsed = _parse_object(content)
            valid = _valid_shape(parsed)
            row.update(
                {
                    "ok": True,
                    "latency_ms": latency_ms,
                    "json_valid": True,
                    "schema_shape_valid": valid,
                    "class": parsed.get("failure_class"),
                    "class_match": parsed.get("failure_class") == case["expected"],
                    "confidence": parsed.get("confidence"),
                    "response_sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
                    "usage": response.get("usage", {}),
                    "response": parsed,
                }
            )
        except (OSError, ValueError, TypeError, KeyError, IndexError, json.JSONDecodeError, urlerror.URLError) as exc:
            row.update(
                {
                    "ok": False,
                    "latency_ms": int(round((time.perf_counter() - started) * 1000)),
                    "error": type(exc).__name__ + ": " + str(exc)[:400],
                }
            )
            if content:
                row["raw_content"] = content[:2000]
            if response:
                row["response_keys"] = sorted(response)
        rows.append(row)
    latencies = [int(row["latency_ms"]) for row in rows if row.get("ok")]
    successes = [row for row in rows if row.get("ok")]
    matches = [row for row in successes if row.get("class_match")]
    summary = {
        "cases": len(rows),
        "request_successes": len(successes),
        "json_successes": sum(bool(row.get("schema_shape_valid")) for row in successes),
        "class_matches": len(matches),
        "class_match_rate": len(matches) / len(rows) if rows else 0.0,
        "latency_ms_mean": round(statistics.mean(latencies), 1) if latencies else None,
        "latency_ms_p50": statistics.median(latencies) if latencies else None,
        "latency_ms_max": max(latencies) if latencies else None,
    }
    return {"schema_version": "local_llm_benchmark.v1", "base_url": base_url, "model": model, "summary": summary, "cases": rows}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8090")
    parser.add_argument("--model", default="Qwen3-Coder-30B-A3B-Instruct-Q2_K_L")
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--real-result", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    value = run(args.base_url, args.model, args.timeout_s, args.real_result)
    encoded = json.dumps(value, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
        print(args.output.resolve())
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()
