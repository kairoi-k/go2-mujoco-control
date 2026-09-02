"""Base worker: workflow plus validation/diagnosis activities."""

from __future__ import annotations

import asyncio
import os

from temporalio.client import Client
from temporalio.worker import Worker

from ..activities.agent import (
    classify_result,
    decide_next_action,
    diagnose_with_codex,
    run_fixture_probe,
    validate_experiment,
)
from ..config import WorkerSettings
from ..workflows.research import ResearchWorkflow


async def serve() -> None:
    settings = WorkerSettings.from_env()
    os.environ["RESEARCH_REPO_ROOT"] = str(settings.repo_root)
    client = await Client.connect(settings.temporal_address, namespace=settings.temporal_namespace)
    worker = Worker(
        client,
        task_queue=settings.agent_task_queue,
        workflows=[ResearchWorkflow],
        activities=[
            validate_experiment,
            run_fixture_probe,
            classify_result,
            diagnose_with_codex,
            decide_next_action,
        ],
    )
    print(
        "agent worker ready "
        f"address={settings.temporal_address} namespace={settings.temporal_namespace} "
        f"task_queue={settings.agent_task_queue} repo={settings.repo_root}",
        flush=True,
    )
    await worker.run()


def main() -> None:
    asyncio.run(serve())


if __name__ == "__main__":
    main()
