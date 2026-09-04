"""Reject generated/private artifacts and broken local documentation links."""

from __future__ import annotations

import json
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
FORBIDDEN_TRACKED_PREFIXES = ("example/cpp/experiments/_runs/",)
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
MACHINE_LOCAL_MARKERS = ("/home/che/", "/mnt/c/", "C:\\Users\\", "/tmp/kine2go")
SELF_PATH = "tools/check_repo_hygiene.py"
UPSTREAM_DOC_PREFIX = "docs/upstream/"
EVIDENCE_PREFIXES = (
    "docs/research/evidence/",
    "docs/validation/",
    "example/cpp/experiments/",
)
EVIDENCE_ROOT = Path("docs/research/evidence")
EVIDENCE_MANIFEST_NAMES = (
    "MANIFEST.json",
    "FORMAL_MANIFEST.json",
    "PREREGISTERED_MANIFEST.json",
)
MACHINE_PORTABLE_PREFIXES = (
    ".github/",
    "example/cpp/configs/",
    "example/cpp/scripts/",
    "example/cpp/tools/",
    "patches/",
    "scripts/",
    "simulate/",
    "simulate_python/",
    "tools/",
)
REQUIRED_FILES = (
    "AGENTS.md",
    "CURRENT.md",
    "README.md",
    "docs/REPOSITORY_GOVERNANCE.md",
    "docs/RESEARCH_HISTORY.md",
    "docs/research/PHASE2_ACCEPTANCE.md",
    "docs/research/PHASE2_HOLDOUT_MANIFEST.json",
    "docs/research/evidence/README.md",
    "example/cpp/experiments/CATALOG.md",
)
REQUIRED_DOC_MARKERS = {
    "CURRENT.md": (
        "Stage C",
        "TerrainBelief",
        "TerrainExecutionState",
        "B0, B1, B2, and B3 are acceptance milestones",
    ),
    "AGENTS.md": ("CURRENT.md", "Stage C", "TerrainExecutionState"),
    "README.md": ("CURRENT.md", "Canonical milestone ledger"),
    "CONTRIBUTING.md": ("CURRENT.md", "AGENTS.md"),
    "docs/README.md": (
        "CURRENT.md",
        "PHASE2_ACCEPTANCE.md",
        "PHASE2_HOLDOUT_MANIFEST.json",
        "research/evidence/README.md",
    ),
    "docs/RESEARCH_HISTORY.md": (
        "Canonical milestone ledger",
        "CURRENT.md",
        "Non-acceptance register",
    ),
    "docs/ARCHITECTURE.md": (
        "CURRENT.md",
        "not a Phase 2 route",
        "TerrainBelief",
        "TerrainExecutionState",
    ),
    "docs/REPRODUCIBILITY.md": (
        "CURRENT.md",
        "/tmp/go2_mujoco_experiment.lock",
        "PHASE2_HOLDOUT_MANIFEST.json",
    ),
    "docs/CODE_GUIDE.md": ("CURRENT.md", "PHASE2_ACCEPTANCE.md"),
    "example/cpp/README.md": ("CURRENT.md", "run_phase2_b0_pair.sh"),
    "example/cpp/MODULES.md": ("CURRENT.md", "not be used as a Phase 2"),
    "example/cpp/configs/README.md": ("CURRENT.md", "not Phase 2 profiles"),
    "example/cpp/scripts/README.md": (
        "CURRENT.md",
        "run_phase2_b0_pair.sh",
        "run_phase2_b0_fixed_pair.sh",
    ),
    "example/cpp/tools/analysis/INDEX.md": (
        "CURRENT.md",
        "analyze_phase2_b0.py",
        "analyze_phase2_terrain.py",
    ),
}
PHASE2_SOURCE_PREFIXES = (
    "example/cpp/gait/",
    "example/cpp/terrain/",
    "example/cpp/trot/",
)
FORBIDDEN_PHASE2_SOURCE_TOKENS = (
    "GaitExecutionRequest",
    "GaitPattern::kCrawl",
    "TerrainCrawl",
    "stage_c_execution",
    "terrain_crawl",
)
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


def check_milestone_ledger(root: Path, problems: list[str]) -> None:
    history = root / "docs/RESEARCH_HISTORY.md"
    if not history.is_file():
        return
    text = history.read_text(encoding="utf-8")
    result = subprocess.run(
        ["git", "tag", "--list", "milestone/*"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    )
    for tag in result.stdout.splitlines():
        occurrences = text.count(f"`{tag}`")
        if occurrences == 0:
            problems.append(
                f"milestone tag missing from docs/RESEARCH_HISTORY.md: {tag}"
            )
        elif occurrences > 1:
            problems.append(
                f"milestone tag appears more than once in docs/RESEARCH_HISTORY.md: {tag}"
            )


def check_evidence_bundles(root: Path, problems: list[str]) -> None:
    evidence_root = root / EVIDENCE_ROOT
    if not evidence_root.is_dir():
        problems.append(f"missing evidence root: {EVIDENCE_ROOT.as_posix()}")
        return
    for bundle in sorted(evidence_root.iterdir()):
        if not bundle.is_dir():
            continue
        rel = bundle.relative_to(root).as_posix()
        if not any((bundle / name).is_file() for name in EVIDENCE_MANIFEST_NAMES):
            problems.append(f"evidence bundle missing manifest: {rel}")
        if not any((bundle / name).is_file() for name in ("README.md", "SUMMARY.md")):
            problems.append(f"evidence bundle missing README.md or SUMMARY.md: {rel}")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    problems: list[str] = []

    for rel in REQUIRED_FILES:
        if not (root / rel).is_file():
            problems.append(f"missing required file: {rel}")

    for rel, markers in REQUIRED_DOC_MARKERS.items():
        path = root / rel
        if not path.is_file():
            problems.append(f"missing navigation document: {rel}")
            continue
        text = path.read_text(encoding="utf-8")
        for marker in markers:
            if marker not in text:
                problems.append(f"missing required guidance in {rel}: {marker}")

    check_milestone_ledger(root, problems)
    check_evidence_bundles(root, problems)

    for rel in tracked_files(root):
        path = root / rel
        parts = set(Path(rel).parts)
        name = path.name

        if path.is_file() and path.suffix == ".json":
            try:
                json.loads(path.read_text(encoding="utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                problems.append(f"invalid JSON in {rel}: {exc}")

        if rel.startswith(FORBIDDEN_TRACKED_PREFIXES):
            problems.append(f"tracked runtime evidence path: {rel}")

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
            portable_text = rel.startswith(MACHINE_PORTABLE_PREFIXES) and not \
                rel.startswith(EVIDENCE_PREFIXES)
            if rel != SELF_PATH and portable_text:
                for marker in MACHINE_LOCAL_MARKERS:
                    if marker in text:
                        problems.append(f"machine-local absolute path in {rel}: {marker}")
            if rel.startswith(PHASE2_SOURCE_PREFIXES):
                for token in FORBIDDEN_PHASE2_SOURCE_TOKENS:
                    if token in text:
                        problems.append(
                            f"retired Phase 2 source token in {rel}: {token}"
                        )
            # Upstream READMEs are kept verbatim and may reference assets that were not
            # copied into this research fork; validate links only for maintained docs.
            if path.suffix == ".md" and \
                    not rel.startswith((UPSTREAM_DOC_PREFIX,) + EVIDENCE_PREFIXES):
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
