#!/usr/bin/env python3
"""Update the issue state for a trusted Atlas workflow run."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen


API_ROOT = "https://api.github.com"
API_VERSION = "2022-11-28"
MAX_COMMENT_CHARS = 6_000
STATES = ("running", "complete", "failed")


class GitHubError(RuntimeError):
    """A GitHub API operation failed."""


def _api(
    method: str,
    path: str,
    token: str,
    payload: dict[str, Any] | None = None,
) -> Any:
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    request = Request(
        API_ROOT + path,
        data=data,
        method=method,
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": API_VERSION,
            "Content-Type": "application/json",
        },
    )
    try:
        with urlopen(request, timeout=30) as response:
            raw = response.read().decode("utf-8")
    except (HTTPError, URLError, TimeoutError) as exc:
        detail = ""
        if isinstance(exc, HTTPError):
            detail = exc.read().decode("utf-8", errors="replace")[:500]
        raise GitHubError(f"GitHub API {method} {path} failed: {detail or exc}") from exc
    return json.loads(raw) if raw else None


def _event(path: Path) -> tuple[str, int]:
    try:
        event = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GitHubError(f"cannot read GitHub event: {exc}") from exc
    repository = event.get("repository", {})
    issue = event.get("issue", {})
    full_name = repository.get("full_name")
    number = issue.get("number")
    if not isinstance(full_name, str) or not full_name:
        raise GitHubError("event repository is missing")
    if not isinstance(number, int) or number <= 0:
        raise GitHubError("event issue number is invalid")
    return full_name, number


def _issue_path(full_name: str, number: int, suffix: str = "") -> str:
    return f"/repos/{full_name}/issues/{number}{suffix}"


def _label_path(full_name: str, number: int, label: str) -> str:
    return _issue_path(full_name, number, f"/labels/{quote(label, safe='')}")


def _set_state(
    *,
    full_name: str,
    number: int,
    token: str,
    state: str,
    summary_file: Path | None,
) -> None:
    _api("POST", _issue_path(full_name, number, "/labels"), token, {"labels": [f"atlas-{state}"]})
    for old_state in ("running", "complete", "failed"):
        if old_state != state:
            try:
                _api("DELETE", _label_path(full_name, number, f"atlas-{old_state}"), token)
            except GitHubError as exc:
                if "404" not in str(exc):
                    raise

    run_url = os.environ.get("GITHUB_SERVER_URL", "https://github.com")
    run_id = os.environ.get("GITHUB_RUN_ID", "")
    workflow_url = f"{run_url}/{full_name}/actions/runs/{run_id}" if run_id else run_url
    if summary_file and summary_file.is_file():
        summary = summary_file.read_text(encoding="utf-8")[:MAX_COMMENT_CHARS]
    else:
        summary = "No result summary was produced."

    if state == "running":
        body = "Atlas accepted this allow-listed task and started it.\n\n" + workflow_url
    elif state == "complete":
        body = "Atlas completed the allow-listed task.\n\n" + summary + "\nWorkflow: " + workflow_url
    else:
        body = "Atlas could not complete the allow-listed task.\n\n" + summary + "\nWorkflow: " + workflow_url
    _api("POST", _issue_path(full_name, number, "/comments"), token, {"body": body[:MAX_COMMENT_CHARS]})

    if state == "complete":
        _api(
            "PATCH",
            _issue_path(full_name, number),
            token,
            {"state": "closed", "state_reason": "completed"},
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--event", type=Path, required=True)
    parser.add_argument("--state", choices=STATES, required=True)
    parser.add_argument("--summary-file", type=Path)
    args = parser.parse_args()

    token = os.environ.get("GITHUB_TOKEN")
    if not token:
        raise SystemExit("GITHUB_TOKEN is required")
    full_name, number = _event(args.event)
    _set_state(
        full_name=full_name,
        number=number,
        token=token,
        state=args.state,
        summary_file=args.summary_file,
    )
    print(f"Atlas issue #{number}: {args.state}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
