#!/usr/bin/env python3
"""Portable repository hygiene checks for hosted CI and local preflight."""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[3]
REQUIRED = (
    "README.md",
    "docs/RESEARCH_INDEX.md",
    "docs/RESEARCH_HISTORY.md",
    "docs/REPOSITORY_GOVERNANCE.md",
    "example/cpp/experiments/CATALOG.md",
    "example/cpp/scripts/README.md",
    "example/cpp/tools/analysis/INDEX.md",
)
TRACKED_BAD = re.compile(r"(^|/)(build|_runs|__pycache__)(/|$)|\\.pyc$|\\.pyo$")
SECRET = re.compile(
    r"BEGIN (?:RSA|OPENSSH|EC|DSA|PRIVATE) KEY|(?:ghp|github_pat|AKIA)[A-Za-z0-9_\-]{12,}"
)
HOST_PATH = re.compile(r"(?:/mnt/[a-z]/Users/|[A-Za-z]:\\Users\\|/home/[A-Za-z0-9_.-]+/)")
MEDIA = re.compile(r"\.(?:mp4|mov|avi|mkv|gif|png|jpg|jpeg|webm)$", re.I)


def run(*args: str) -> str:
    return subprocess.check_output(args, cwd=ROOT, text=True)


def main() -> int:
    failures: list[str] = []
    tracked = [path for path in run("git", "ls-files").splitlines() if path]
    for required in REQUIRED:
        if not (ROOT / required).is_file():
            failures.append(f"missing required file: {required}")
    for path in tracked:
        if TRACKED_BAD.search(path):
            failures.append(f"generated output tracked: {path}")
        full = ROOT / path
        if full.is_file() and full.stat().st_size <= 5_000_000:
            try:
                text = full.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            if SECRET.search(text):
                failures.append(f"credential-like material: {path}")
            maintained = path.startswith(("example/cpp/scripts/", "example/cpp/configs/", "example/cpp/tools/", "simulate/", ".github/"))
            if maintained and HOST_PATH.search(text):
                failures.append(f"host-specific absolute path: {path}")
    try:
        changed = run("git", "diff", "--name-status", "origin/main...HEAD").splitlines()
    except subprocess.CalledProcessError:
        changed = []
    for line in changed:
        status, _, path = line.partition("\t")
        if status.startswith("A") and MEDIA.search(path):
            size = (ROOT / path).stat().st_size if (ROOT / path).exists() else 0
            if size > 1_000_000:
                failures.append(f"new large media needs explicit allowlist/justification: {path}")
    if failures:
        print("repo_hygiene=FAIL")
        print("\n".join(f"failure={item}" for item in failures))
        return 1
    print(f"repo_hygiene=PASS tracked_files={len(tracked)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
