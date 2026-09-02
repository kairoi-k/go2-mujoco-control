"""Fail-closed Atlas Activity contract.

The names and payload boundary are ready on Base, but the Linux/WSL adapter is
deliberately not implemented here. Until it is implemented and verified on
Atlas, every registered physical activity refuses work without executing a
command. This prevents a contract-only worker from looking like a valid
MuJoCo result producer.
"""

from __future__ import annotations

from typing import Any, NoReturn

from temporalio import activity
from temporalio.exceptions import ApplicationError

from ..schemas.models import ExperimentSpec


ATLAS_ACTIVITY_NAMES = (
    "build_source",
    "run_unit_tests",
    "run_dev_probe",
    "run_b0_member",
    "run_b0_holdout",
    "run_b1_probe",
    "extract_failure_window",
)


def _validate_payload(payload: Any) -> None:
    if not isinstance(payload, dict):
        raise ApplicationError(
            "Atlas Activity payload must be a JSON object",
            type="INVALID_ATLAS_PAYLOAD",
            non_retryable=True,
        )
    candidate = payload.get("spec", payload)
    try:
        ExperimentSpec.model_validate(candidate)
    except Exception as exc:  # Pydantic errors are intentionally not retried.
        raise ApplicationError(
            "Atlas Activity payload does not contain a valid experiment.v1 spec",
            type="INVALID_ATLAS_PAYLOAD",
            non_retryable=True,
        ) from exc


def _not_ready(name: str, payload: Any) -> NoReturn:
    _validate_payload(payload)
    raise ApplicationError(
        f"{name} is contract-only until the verified Atlas adapter is installed; no command was executed",
        type="ATLAS_ADAPTER_NOT_READY",
        non_retryable=True,
    )


@activity.defn(name="build_source")
async def build_source(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("build_source", payload)


@activity.defn(name="run_unit_tests")
async def run_unit_tests(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("run_unit_tests", payload)


@activity.defn(name="run_dev_probe")
async def run_dev_probe(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("run_dev_probe", payload)


@activity.defn(name="run_b0_member")
async def run_b0_member(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("run_b0_member", payload)


@activity.defn(name="run_b0_holdout")
async def run_b0_holdout(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("run_b0_holdout", payload)


@activity.defn(name="run_b1_probe")
async def run_b1_probe(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("run_b1_probe", payload)


@activity.defn(name="extract_failure_window")
async def extract_failure_window(payload: dict[str, Any]) -> dict[str, Any]:
    _not_ready("extract_failure_window", payload)
