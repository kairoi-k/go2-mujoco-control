"""Deterministic Temporal workflow for one bounded research decision."""

from __future__ import annotations

from datetime import timedelta
from typing import Any

from temporalio import workflow
from temporalio.common import RetryPolicy


def _once() -> RetryPolicy:
    # Physical runs and paid/model calls must not be duplicated by a default retry.
    return RetryPolicy(maximum_attempts=1)


@workflow.defn
class ResearchWorkflow:
    """Run one experiment, compress its evidence, and stop at a safe decision boundary."""

    @workflow.run
    async def run(self, spec_payload: dict[str, Any]) -> dict[str, Any]:
        spec = await workflow.execute_activity(
            "validate_experiment",
            spec_payload,
            start_to_close_timeout=timedelta(seconds=30),
            retry_policy=_once(),
        )
        if not isinstance(spec, dict):
            raise ValueError("validate_experiment returned a non-object payload")

        if spec.get("execution_mode") == "fixture":
            result = await workflow.execute_activity(
                "run_fixture_probe",
                spec,
                start_to_close_timeout=timedelta(seconds=30),
                retry_policy=_once(),
            )
        elif spec.get("execution_mode") == "atlas":
            wall_timeout_s = float(spec.get("wall_timeout_s", 60.0))
            result = await workflow.execute_activity(
                "run_dev_probe",
                spec,
                task_queue=str(spec.get("atlas_task_queue", "atlas")),
                start_to_close_timeout=timedelta(seconds=wall_timeout_s),
                retry_policy=_once(),
            )
        else:
            raise ValueError("execution_mode must be 'fixture' or 'atlas'")

        diagnosis = await workflow.execute_activity(
            "classify_result",
            result,
            start_to_close_timeout=timedelta(seconds=30),
            retry_policy=_once(),
        )
        if not isinstance(diagnosis, dict):
            raise ValueError("classify_result returned a non-object payload")

        if diagnosis.get("requires_codex") is True and spec.get("allow_codex") is True:
            diagnosis = await workflow.execute_activity(
                "diagnose_with_codex",
                args=[result, diagnosis, spec],
                start_to_close_timeout=timedelta(
                    seconds=float(spec.get("codex_timeout_s", 90)) + 15
                ),
                retry_policy=_once(),
            )

        next_action = await workflow.execute_activity(
            "decide_next_action",
            args=[spec, diagnosis],
            start_to_close_timeout=timedelta(seconds=30),
            retry_policy=_once(),
        )
        return {
            "schema_version": "research_run.v1",
            "experiment": spec,
            "result": result,
            "diagnosis": diagnosis,
            "next_action": next_action,
        }
