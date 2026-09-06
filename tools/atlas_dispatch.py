#!/usr/bin/env python3
"""Run the small, explicit allow-list of tasks accepted by Atlas."""

from __future__ import annotations

import argparse
import ast
import json
import re
import subprocess
from pathlib import Path
from typing import Any


MAX_BODY_BYTES = 16_384
MAX_TEXT_CHARS = 4_000
SCHEMA_VERSION = 1
ALLOWED_TASKS = ("repo-smoke", "workspace-status")


class TaskError(ValueError):
    """A request does not match the dispatcher contract."""


def _load_event(event_path: Path) -> dict[str, Any]:
    try:
        event = json.loads(event_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TaskError(f"cannot read GitHub event: {exc}") from exc
    if not isinstance(event, dict):
        raise TaskError("GitHub event must be a JSON object")
    return event


def _extract_command(event: dict[str, Any]) -> tuple[dict[str, Any], int | None]:
    issue = event.get("issue")
    if not isinstance(issue, dict):
        raise TaskError("event does not contain an issue")
    issue_number = issue.get("number")
    if not isinstance(issue_number, int) or issue_number <= 0:
        raise TaskError("event issue number is invalid")

    body = issue.get("body") or ""
    if not isinstance(body, str):
        raise TaskError("issue body must be text")
    if len(body.encode("utf-8")) > MAX_BODY_BYTES:
        raise TaskError("issue body is too large")

    blocks = re.findall(r"~~~(?:json)?\s*(.*?)\s*~~~", body, flags=re.IGNORECASE | re.DOTALL)
    if not blocks:
        blocks = re.findall(r"\x60\x60\x60(?:json)?\s*(.*?)\s*\x60\x60\x60", body, flags=re.IGNORECASE | re.DOTALL)
    candidate = blocks[0].strip() if len(blocks) == 1 else body.strip()
    if not candidate:
        raise TaskError("issue body must contain one JSON task object")
    try:
        command = json.loads(candidate)
    except json.JSONDecodeError as exc:
        raise TaskError(f"task JSON is invalid: {exc.msg}") from exc
    if not isinstance(command, dict):
        raise TaskError("task JSON must be an object")
    unknown = sorted(set(command) - {"task", "parameters"})
    if unknown:
        raise TaskError(f"unsupported task fields: {', '.join(unknown)}")

    task = command.get("task")
    if not isinstance(task, str) or task not in ALLOWED_TASKS:
        allowed = ", ".join(ALLOWED_TASKS)
        raise TaskError(f"task must be one of: {allowed}")
    parameters = command.get("parameters", {})
    if not isinstance(parameters, dict) or parameters:
        raise TaskError("parameters must be an empty object for the current task set")
    return {"task": task, "parameters": parameters}, issue_number


def _run(argv: list[str], repo_root: Path, timeout: int = 120) -> str:
    completed = subprocess.run(
        argv,
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    output = (completed.stdout + completed.stderr).strip()
    if completed.returncode:
        detail = output[-MAX_TEXT_CHARS:] if output else "no command output"
        raise TaskError(f"{argv[0]} failed with exit code {completed.returncode}: {detail}")
    return output[-MAX_TEXT_CHARS:]


def _repo_smoke(repo_root: Path) -> dict[str, Any]:
    checked = []
    for relative in ("tools/atlas_dispatch.py", "tools/atlas_issue_state.py"):
        path = repo_root / relative
        if not path.is_file():
            raise TaskError(f"required file is missing: {relative}")
        ast.parse(path.read_text(encoding="utf-8"), filename=relative)
        checked.append(relative)
    _run(["git", "diff", "--check"], repo_root)
    return {"checks": checked + ["git diff --check"]}


def _workspace_status(repo_root: Path) -> dict[str, Any]:
    sha = _run(["git", "rev-parse", "HEAD"], repo_root)
    status = _run(["git", "status", "--short", "--branch"], repo_root)
    return {"source_sha": sha, "git_status": status}


def _run_task(task: str, repo_root: Path) -> dict[str, Any]:
    if task == "repo-smoke":
        return _repo_smoke(repo_root)
    if task == "workspace-status":
        return _workspace_status(repo_root)
    raise TaskError(f"unhandled task: {task}")


def _write_outputs(
    output_dir: Path,
    *,
    issue_number: int | None,
    task: str | None,
    status: str,
    source_sha: str | None,
    details: dict[str, Any],
    error: str | None,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "issue_number": issue_number,
        "task": task,
        "source_sha": source_sha,
        "details": details,
        "error": error,
    }
    (output_dir / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Atlas task result",
        "",
        f"- Status: {status}",
        f"- Issue: #{issue_number}" if issue_number is not None else "- Issue: unknown",
        f"- Task: {task}" if task else "- Task: unknown",
        f"- Source SHA: {source_sha}" if source_sha else "- Source SHA: unknown",
    ]
    if details:
        lines.extend(["", "## Details", "", "~~~json", json.dumps(details, indent=2, sort_keys=True), "~~~"])
    if error:
        lines.extend(["", "## Error", "", error[:MAX_TEXT_CHARS]])
    (output_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--event", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    issue_number: int | None = None
    task: str | None = None
    source_sha: str | None = None
    status = "failure"
    details: dict[str, Any] = {}
    error: str | None = None
    repo_root = Path(__file__).resolve().parents[1]

    try:
        event = _load_event(args.event)
        command, issue_number = _extract_command(event)
        task = command["task"]
        source_sha = _run(["git", "rev-parse", "HEAD"], repo_root)
        details = _run_task(task, repo_root)
        status = "success"
    except (OSError, subprocess.SubprocessError, TaskError) as exc:
        error = str(exc)
    except Exception as exc:  # Keep a result artifact even for unexpected failures.
        error = f"unexpected dispatcher error: {type(exc).__name__}: {exc}"

    _write_outputs(
        args.output_dir,
        issue_number=issue_number,
        task=task,
        status=status,
        source_sha=source_sha,
        details=details,
        error=error,
    )
    print((args.output_dir / "summary.md").read_text(encoding="utf-8"), end="")
    return 0 if status == "success" else 2


if __name__ == "__main__":
    raise SystemExit(main())
