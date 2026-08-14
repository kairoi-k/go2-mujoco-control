"""Reject generated/private artifacts and broken local documentation links."""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import unquote

FORBIDDEN_PARTS = {
    ".cursor",
    ".idea",
    ".vscode",
    "__pycache__",
    "CMakeFiles",
}
FORBIDDEN_NAMES = {
    ".DS_Store",
    "CMakeCache.txt",
    "cmake_install.cmake",
    "CTestTestfile.cmake",
}
FORBIDDEN_PREFIXES = ("._",)
# `.obj` is intentionally allowed: Unitree robot/scene meshes are source assets.
FORBIDDEN_SUFFIXES = (
    ".o",
    ".pyc",
    ".pyo",
    ".pt",
    ".pth",
    ".ckpt",
)
TEXT_SUFFIXES = {
    ".c",
    ".cc",
    ".cpp",
    ".h",
    ".hpp",
    ".md",
    ".py",
    ".sh",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
}
MACHINE_LOCAL_MARKERS = ("/home/che/", "C:\\Users\\", "/tmp/kine2go")
SELF_PATH = "tools/check_repo_hygiene.py"
UPSTREAM_DOC_PREFIX = "docs/upstream/"
MARKDOWN_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")


def tracked_files(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    )
    return [item.decode() for item in result.stdout.split(b"\0") if item]


def check_markdown_links(root: Path, rel: str, text: str, problems: list[str]) -> None:
    source = root / rel
    for raw_target in MARKDOWN_LINK_RE.findall(text):
        target = raw_target.strip().split(maxsplit=1)[0].strip("<>")
        if not target or target.startswith(("http://", "https://", "mailto:", "#")):
            continue
        target = unquote(target.split("#", 1)[0])
        if not target:
            continue
        candidate = (root / target.lstrip("/")) if target.startswith("/") else (source.parent / target)
        if not candidate.exists():
            problems.append(f"broken local markdown link in {rel}: {raw_target}")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    problems: list[str] = []

    for rel in tracked_files(root):
        path = root / rel
        parts = set(Path(rel).parts)
        name = path.name

        if parts & FORBIDDEN_PARTS:
            problems.append(f"forbidden generated/private path: {rel}")
        if name in FORBIDDEN_NAMES:
            problems.append(f"forbidden generated/private file: {rel}")
        if name.startswith(FORBIDDEN_PREFIXES):
            problems.append(f"forbidden OS metadata: {rel}")
        if name.endswith(FORBIDDEN_SUFFIXES):
            problems.append(f"forbidden generated checkpoint/cache: {rel}")

        if path.is_file() and path.suffix in TEXT_SUFFIXES.union({".log"}):
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            if rel != SELF_PATH:
                for marker in MACHINE_LOCAL_MARKERS:
                    if marker in text:
                        problems.append(f"machine-local absolute path in {rel}: {marker}")
            # Upstream READMEs are kept verbatim and may reference assets that were not
            # copied into this research fork; validate links only for maintained docs.
            if path.suffix == ".md" and not rel.startswith(UPSTREAM_DOC_PREFIX):
                check_markdown_links(root, rel, text, problems)

    if problems:
        print("Repository hygiene check failed:")
        for problem in sorted(set(problems)):
            print(f"  - {problem}")
        return 1

    print("Repository hygiene check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
