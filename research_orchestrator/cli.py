"""Small client for creating and running bounded research workflows."""

from __future__ import annotations

import argparse
import asyncio
import json
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
    return ExperimentSpec(
        experiment_id=args.experiment_id,
        question="Verify Base Temporal wiring without executing controller or MuJoCo code.",
        policy_id=PolicyId.B0,
        profile=ProbeProfile.B0_DEVELOPMENT,
        execution_mode=ExecutionMode.FIXTURE,
        duration_s=1.0,
        wall_timeout_s=30.0,
        seed=0,
        source=SourceRevision(
            git_sha=_git(repo_root, "rev-parse", "HEAD"),
            git_ref=_git(repo_root, "symbolic-ref", "--short", "-q", "HEAD", check=False) or "detached",
            dirty=bool(_git(repo_root, "status", "--porcelain")),
        ),
        parameters={"fixture_verdict": args.fixture_verdict},
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

    run = subparsers.add_parser("run", help="execute one spec against the local Temporal server")
    run.add_argument("--spec", type=Path, required=True)
    run.add_argument("--address", default="localhost:7233")
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

    payload: dict[str, Any] = json.loads(args.spec.read_text(encoding="utf-8"))
    spec = ExperimentSpec.model_validate(payload)
    workflow_id = args.workflow_id or f"research-{spec.experiment_id}"
    result = asyncio.run(run_workflow(spec, args.address, args.namespace, args.task_queue, workflow_id))
    print(json.dumps(result.model_dump(mode="json"), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
