#!/usr/bin/env python3
"""Inventory and quarantine one raw run without providing a delete path."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path


class GuardError(RuntimeError):
    pass


CRITICAL_FILES = (
    "data.csv",
    "contact_ground_truth.csv",
    "controller.log",
    "simulator.log",
    "run_manifest.json",
)
STAMP_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z$")
RUNS_RELATIVE_PATH = Path("example/cpp/experiments/_runs")


def sha256_file(path: Path) -> tuple[str, int]:
    before = path.stat()
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise GuardError(f"file changed while hashing: {path}")
    return digest.hexdigest(), after.st_size


def validate_source(target: Path, runs_root: Path) -> Path:
    root = runs_root.resolve(strict=True)
    if target.is_symlink():
        raise GuardError("source symlinks are forbidden")
    source = target.resolve(strict=True)
    if source == root:
        raise GuardError("the _runs root can never be quarantined")
    if source.parent != root:
        raise GuardError("source must be one direct child of _runs")
    if not source.is_dir():
        raise GuardError("source must be a run directory")
    return source


def validate_repo_layout(repo_root: Path, runs_root: Path) -> None:
    repo = repo_root.resolve(strict=True)
    expected_runs = repo / RUNS_RELATIVE_PATH
    if repo_root.is_symlink() or runs_root.is_symlink():
        raise GuardError("repository and _runs roots must not be symlinks")
    if runs_root.absolute() != expected_runs:
        raise GuardError("runs root must be the canonical repository _runs path")


def inventory(source: Path) -> tuple[list[dict[str, object]], int]:
    files: list[dict[str, object]] = []
    total_bytes = 0
    for path in sorted(source.rglob("*")):
        if path.is_symlink():
            raise GuardError(f"symlink inside run is forbidden: {path}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise GuardError(f"special file inside run is forbidden: {path}")
        digest, size = sha256_file(path)
        files.append(
            {
                "path": path.relative_to(source).as_posix(),
                "bytes": size,
                "sha256": digest,
            }
        )
        total_bytes += size
    return files, total_bytes


def verify_inventory_unchanged(source: Path, files: list[dict[str, object]]) -> None:
    observed, _ = inventory(source)
    if observed != files:
        raise GuardError("run contents changed after inventory")


def validate_quarantine_root(
    quarantine_root: Path, runs_root: Path, repo_root: Path
) -> None:
    repo = repo_root.absolute()
    workspace = repo.parent
    runs = runs_root.absolute()
    quarantine = quarantine_root.absolute()
    try:
        runs.relative_to(repo)
        quarantine.relative_to(workspace)
    except ValueError as error:
        raise GuardError("runs and quarantine roots escaped the workspace") from error
    expected = workspace / "archive" / "quarantine" / "raw-runs"
    if quarantine != expected:
        raise GuardError("quarantine must use the workspace archive path")
    for anchor, guarded_path in ((repo, runs), (workspace, quarantine)):
        candidate = anchor
        if candidate.exists() and candidate.is_symlink():
            raise GuardError(f"symlink path component is forbidden: {candidate}")
        for part in guarded_path.relative_to(anchor).parts:
            candidate = candidate / part
            if candidate.exists() and candidate.is_symlink():
                raise GuardError(f"symlink path component is forbidden: {candidate}")


def git_references(repo_root: Path, run_name: str) -> list[str]:
    result = subprocess.run(
        ["git", "grep", "-n", "--fixed-strings", run_name],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode not in (0, 1):
        raise GuardError(result.stderr.strip() or "git reference scan failed")
    return [line for line in result.stdout.splitlines() if line]


def write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    if path.is_symlink():
        raise GuardError(f"manifest path must not be a symlink: {path}")
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=path.name + ".", suffix=".tmp"
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        if path.is_symlink():
            raise GuardError(f"manifest path became a symlink: {path}")
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def protect_tree_read_only(root: Path) -> None:
    directories: list[Path] = []
    for path in root.rglob("*"):
        if path.is_dir():
            directories.append(path)
        elif path.is_file():
            path.chmod(path.stat().st_mode & ~0o222)
    for directory in sorted(directories, key=lambda item: len(item.parts), reverse=True):
        directory.chmod(directory.stat().st_mode & ~0o222)
    root.chmod(root.stat().st_mode & ~0o222)


@contextmanager
def hold_experiment_lock(lock_path: Path):
    descriptor = os.open(
        lock_path,
        os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0),
        0o600,
    )
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise GuardError("an experiment currently holds the shared lock") from error
        yield
    finally:
        os.close(descriptor)


def run_guard(
    target: Path,
    runs_root: Path,
    quarantine_root: Path,
    repo_root: Path,
    apply: bool,
    stamp: str | None = None,
    lock_path: Path = Path("/tmp/go2_mujoco_experiment.lock"),
) -> dict[str, object]:
    if apply:
        with hold_experiment_lock(lock_path):
            return _run_guard_locked(
                target, runs_root, quarantine_root, repo_root, True, stamp
            )
    return _run_guard_locked(
        target, runs_root, quarantine_root, repo_root, False, stamp
    )


def _run_guard_locked(
    target: Path,
    runs_root: Path,
    quarantine_root: Path,
    repo_root: Path,
    apply: bool,
    stamp: str | None,
) -> dict[str, object]:
    validate_repo_layout(repo_root, runs_root)
    source = validate_source(target, runs_root)
    stamp = stamp or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    if not STAMP_RE.fullmatch(stamp):
        raise GuardError("invalid UTC stamp")
    validate_quarantine_root(quarantine_root, runs_root, repo_root)
    destination = quarantine_root / f"{stamp}__{source.name}"
    if destination.exists():
        raise GuardError(f"quarantine destination already exists: {destination}")

    files, total_bytes = inventory(source)
    critical_files = {name: (source / name).is_file() for name in CRITICAL_FILES}
    references = git_references(repo_root, source.name)
    payload: dict[str, object] = {
        "schema_version": 1,
        "state": "dry-run",
        "created_at_utc": stamp,
        "source": str(source),
        "destination": str(destination),
        "file_count": len(files),
        "total_bytes": total_bytes,
        "critical_files_present": critical_files,
        "git_references": references,
        "files": files,
    }
    if not apply:
        return payload

    if not all(critical_files.values()):
        raise GuardError("incomplete runs remain in place for manual review")
    if references:
        raise GuardError("tracked references must be resolved before quarantine")
    verify_inventory_unchanged(source, files)

    quarantine_root.mkdir(parents=True, exist_ok=True)
    manifest_root = quarantine_root / "_manifests"
    manifest_root.mkdir(exist_ok=True)
    if manifest_root.is_symlink():
        raise GuardError("manifest root must not be a symlink")
    if os.stat(runs_root).st_dev != os.stat(quarantine_root).st_dev:
        raise GuardError("quarantine must be on the same filesystem for atomic move")

    manifest_path = manifest_root / f"{stamp}__{source.name}.sha256.json"
    if manifest_path.exists():
        raise GuardError(f"manifest already exists: {manifest_path}")
    payload["state"] = "planned"
    payload["manifest"] = str(manifest_path)
    write_json_atomic(manifest_path, payload)
    source.rename(destination)
    fsync_directory(source.parent)
    fsync_directory(destination.parent)
    protect_tree_read_only(destination)
    moved_files, moved_total_bytes = inventory(destination)
    if moved_files != files or moved_total_bytes != total_bytes:
        payload["state"] = "quarantine-verification-failed"
        write_json_atomic(manifest_path, payload)
        raise GuardError("post-move SHA-256 verification failed; data remains quarantined")
    payload["state"] = "quarantined"
    write_json_atomic(manifest_path, payload)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Hash and quarantine one direct _runs child; never deletes data."
    )
    parser.add_argument("target", type=Path)
    parser.add_argument(
        "--apply", action="store_true", help="perform the atomic move after inventory"
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    experiments = repo_root / "example" / "cpp" / "experiments"
    try:
        payload = run_guard(
            args.target,
            experiments / "_runs",
            repo_root.parent / "archive" / "quarantine" / "raw-runs",
            repo_root,
            args.apply,
        )
    except (GuardError, OSError) as error:
        print(f"raw-run guard refused: {error}", file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
