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

        preflight: list[dict[str, Any]] = []
        if spec.get("execution_mode") == "fixture":
            result = await workflow.execute_activity(
                "run_fixture_probe",
                spec,
                start_to_close_timeout=timedelta(seconds=30),
                retry_policy=_once(),
            )
        elif spec.get("execution_mode") == "atlas":
            if spec.get("profile") != "b0-development":
                raise ValueError(
                    "formal B0 holdout and B1 activities require a separate human-approved workflow"
                )
            wall_timeout_s = float(spec.get("wall_timeout_s", 60.0))
            atlas_queue = str(spec.get("atlas_task_queue", "atlas"))
            for activity_name, timeout_s in (
                ("build_source", 900.0),
                ("run_unit_tests", 900.0),
            ):
                receipt = await workflow.execute_activity(
                    activity_name,
                    spec,
                    task_queue=atlas_queue,
                    start_to_close_timeout=timedelta(seconds=timeout_s),
                    retry_policy=_once(),
                )
                if not isinstance(receipt, dict):
                    raise ValueError(f"{activity_name} returned a non-object receipt")
                preflight.append(receipt)
                if receipt.get("status") != "completed":
                    raise ValueError(f"{activity_name} did not complete successfully")
            result = await workflow.execute_activity(
                "run_dev_probe",
                spec,
                task_queue=atlas_queue,
                start_to_close_timeout=timedelta(seconds=wall_timeout_s + 60.0),
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
            "preflight": preflight,
            "result": result,
            "diagnosis": diagnosis,
            "next_action": next_action,
        }
