"""Atlas worker entry point for the fail-closed Activity contract."""

from __future__ import annotations

import argparse
import asyncio
import os
import platform
from pathlib import Path

from temporalio.client import Client
from temporalio.worker import Worker

from ..activities.atlas import (
    ATLAS_ACTIVITY_NAMES,
    build_source,
    extract_failure_window,
    run_b0_holdout,
    run_b0_member,
    run_b1_probe,
    run_dev_probe,
    run_unit_tests,
)
from ..config import WorkerSettings


def _check_environment() -> list[str]:
    settings = WorkerSettings.from_env()
    issues: list[str] = []
    if platform.system() != "Linux":
        issues.append("Atlas worker must run inside the verified Linux/WSL environment")
    workspace = os.environ.get("ATLAS_WORKSPACE")
    if not workspace:
        issues.append("ATLAS_WORKSPACE is required and must point to the Atlas checkout")
    elif not Path(workspace).expanduser().resolve().is_dir():
        issues.append("ATLAS_WORKSPACE does not resolve to a directory")
    if os.environ.get("ATLAS_ADAPTER_READY") != "1":
        issues.append("ATLAS_ADAPTER_READY=1 is required; the current activity bodies are contract-only")
    print(f"required_task_queue={settings.atlas_task_queue}")
    print("registered_activities=" + ",".join(ATLAS_ACTIVITY_NAMES))
    return issues


async def serve() -> None:
    issues = _check_environment()
    if issues:
        raise SystemExit("Atlas worker refused to start:\n- " + "\n- ".join(issues))
    settings = WorkerSettings.from_env()
    client = await Client.connect(settings.temporal_address, namespace=settings.temporal_namespace)
    worker = Worker(
        client,
        task_queue=settings.atlas_task_queue,
        max_concurrent_activities=1,
        activities=[
            build_source,
            run_unit_tests,
            run_dev_probe,
            run_b0_member,
            run_b0_holdout,
            run_b1_probe,
            extract_failure_window,
        ],
    )
    print(
        "atlas worker ready "
        f"address={settings.temporal_address} namespace={settings.temporal_namespace} "
        f"task_queue={settings.atlas_task_queue}",
        flush=True,
    )
    await worker.run()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="print the contract without starting a worker")
    args = parser.parse_args()
    if args.check:
        issues = _check_environment()
        if issues:
            print("check_status=deferred")
            for issue in issues:
                print(f"check_issue={issue}")
        else:
            print("check_status=ready")
        return
    asyncio.run(serve())


if __name__ == "__main__":
    main()
