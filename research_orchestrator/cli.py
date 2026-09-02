"""Small client for creating and running bounded research workflows."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from temporalio.client import Client

from .schemas.models import (
    ExecutionMode,
    ExperimentSpec,
    PolicyId,
    ProbeProfile,
    SourceRevision,
    Verdict,
    WorkflowResult,
)
from .activities.atlas import DEV_SCENARIO_DURATIONS
from .workflows.research import ResearchWorkflow


def _git(repo_root: Path, *args: str, check: bool = True) -> str:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if check and completed.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)} failed with status {completed.returncode}")
    return completed.stdout.strip()


def make_fixture_spec(repo_root: Path, args: argparse.Namespace) -> ExperimentSpec:
    source = SourceRevision(
        git_sha=_git(repo_root, "rev-parse", "HEAD"),
        git_ref=_git(repo_root, "symbolic-ref", "--short", "-q", "HEAD", check=False) or "detached",
        dirty=bool(_git(repo_root, "status", "--porcelain")),
    )
    return ExperimentSpec(
        experiment_id=args.experiment_id,
        question="Verify Base Temporal wiring without executing controller or MuJoCo code.",
        policy_id=PolicyId.B0,
        profile=ProbeProfile.B0_DEVELOPMENT,
        execution_mode=ExecutionMode.FIXTURE,
        duration_s=1.0,
        wall_timeout_s=30.0,
        seed=0,
        source=source,
        control_plane=source,
        parameters={"fixture_verdict": args.fixture_verdict},
        allow_local_llm=args.allow_local_llm,
        local_llm_timeout_s=args.local_llm_timeout_s,
        allow_codex=args.allow_codex,
        requested_at=datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
    )


def make_atlas_spec(repo_root: Path, args: argparse.Namespace) -> ExperimentSpec:
    if re.fullmatch(r"[0-9a-f]{40}", args.source_sha.lower()) is None:
        raise ValueError("--source-sha must be a 40-character lowercase hexadecimal Git SHA")
    control_plane = SourceRevision(
        git_sha=_git(repo_root, "rev-parse", "HEAD"),
        git_ref=_git(repo_root, "symbolic-ref", "--short", "-q", "HEAD", check=False) or "detached",
        dirty=bool(_git(repo_root, "status", "--porcelain")),
    )
    duration_s = (
        args.duration_s
        if args.duration_s is not None
        else DEV_SCENARIO_DURATIONS[args.scenario]
    )
    wall_timeout_s = (
        args.wall_timeout_s
        if args.wall_timeout_s is not None
        else duration_s + 60.0
    )
    profile = ProbeProfile(args.profile)
    policy = PolicyId.B0 if profile.value.startswith("b0-") else PolicyId.B1
    return ExperimentSpec(
        experiment_id=args.experiment_id,
        question=args.question,
        policy_id=policy,
        profile=profile,
        execution_mode=ExecutionMode.ATLAS,
        duration_s=duration_s,
        wall_timeout_s=wall_timeout_s,
        seed=args.seed,
        source=SourceRevision(
            git_sha=args.source_sha.lower(),
            git_ref=args.source_ref,
            dirty=False,
        ),
        control_plane=control_plane,
        parameters={
            "scenario": args.scenario,
            "domain_id": args.domain_id,
            **({"force_local_llm": True} if args.force_local_llm else {}),
        },
        allow_local_llm=args.allow_local_llm,
        local_llm_timeout_s=args.local_llm_timeout_s,
        allow_codex=args.allow_codex,
        requested_at=datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
    )


async def run_workflow(
    spec: ExperimentSpec, address: str, namespace: str, task_queue: str, workflow_id: str
) -> WorkflowResult:
    client = await Client.connect(address, namespace=namespace)
    raw = await client.execute_workflow(
        ResearchWorkflow.run,
        spec.model_dump(mode="json"),
        id=workflow_id,
        task_queue=task_queue,
    )
    return WorkflowResult.model_validate(raw)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    fixture = subparsers.add_parser("make-fixture", help="write a source-pinned synthetic experiment spec")
    fixture.add_argument("--repo", type=Path, default=Path.cwd())
    fixture.add_argument("--output", type=Path, required=True)
    fixture.add_argument("--experiment-id", default="base-orchestration-smoke")
    fixture.add_argument(
        "--fixture-verdict",
        choices=[item.value for item in Verdict],
        default=Verdict.PASS_DEV.value,
    )
    fixture.add_argument("--allow-codex", action="store_true")
    fixture.add_argument("--allow-local-llm", action="store_true")
    fixture.add_argument("--local-llm-timeout-s", type=int, default=360)

    atlas = subparsers.add_parser("make-atlas-spec", help="write a source-pinned Atlas development spec")
    atlas.add_argument("--repo", type=Path, default=Path.cwd(), help="Base control-plane checkout")
    atlas.add_argument("--output", type=Path, required=True)
    atlas.add_argument("--source-sha", required=True, help="exact Git SHA in the Atlas source checkout")
    atlas.add_argument("--source-ref", default="phase2-b1-b3")
    atlas.add_argument("--experiment-id", default="atlas-development-smoke")
    atlas.add_argument(
        "--question",
        default="Run the fixed sensor-only development probe on the pinned Atlas source.",
    )
    atlas.add_argument("--profile", choices=[item.value for item in ProbeProfile], default="b0-development")
    atlas.add_argument("--scenario", choices=sorted(DEV_SCENARIO_DURATIONS), default="accel_1_to_3")
    atlas.add_argument("--duration-s", type=float)
    atlas.add_argument("--wall-timeout-s", type=float)
    atlas.add_argument("--domain-id", type=int, default=190)
    atlas.add_argument("--seed", type=int, default=0)
    atlas.add_argument("--allow-codex", action="store_true")
    atlas.add_argument("--allow-local-llm", action="store_true")
    atlas.add_argument("--force-local-llm", action="store_true")
    atlas.add_argument("--local-llm-timeout-s", type=int, default=360)

    run = subparsers.add_parser("run", help="execute one spec against the local Temporal server")
    run.add_argument("--spec", type=Path, required=True)
    run.add_argument("--address", default=os.environ.get("TEMPORAL_ADDRESS", "localhost:7233"))
    run.add_argument("--namespace", default="default")
    run.add_argument("--task-queue", default="agent")
    run.add_argument("--workflow-id")
    return parser


def main() -> None:
    args = _build_parser().parse_args()
    if args.command == "make-fixture":
        spec = make_fixture_spec(args.repo.resolve(), args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(spec.model_dump(mode="json"), indent=2) + "\n", encoding="utf-8")
        print(args.output.resolve())
        return

    if args.command == "make-atlas-spec":
        spec = make_atlas_spec(args.repo.resolve(), args)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(spec.model_dump(mode="json"), indent=2) + "\n", encoding="utf-8")
        print(args.output.resolve())
        return

    payload: dict[str, Any] = json.loads(args.spec.read_text(encoding="utf-8"))
    spec = ExperimentSpec.model_validate(payload)
    workflow_id = args.workflow_id or f"research-{spec.experiment_id}"
    result = asyncio.run(run_workflow(spec, args.address, args.namespace, args.task_queue, workflow_id))
    print(json.dumps(result.model_dump(mode="json"), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
