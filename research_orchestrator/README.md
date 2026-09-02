# Local research orchestration (WIP)

This package is the Base-side control plane for the `go2-mujoco-control`
research repository. It uses a local Temporal development server and a Python
SDK worker. The first runnable path is intentionally synthetic:

```text
experiment.v1
  -> ResearchWorkflow
  -> fixture result.v1
  -> deterministic diagnosis.v1
  -> next_action.v1 (human checkpoint)
```

The fixture path does not start MuJoCo, run a controller, alter C++ files, or
claim a B0/B1 result. An Atlas execution spec is accepted only through the
fixed `atlas` task queue, and the current Atlas worker is fail-closed until its
Linux/WSL adapter is implemented and verified.

## Base setup

The Base machine already has the Temporal CLI, Codex CLI, and GitHub CLI. From
the repository root:

```bash
uv sync --extra dev
mkdir -p .temporal
temporal server start-dev --db-filename .temporal/dev.sqlite
```

In a second terminal, start the Base worker:

```bash
RESEARCH_REPO_ROOT="$PWD" \
  .venv/bin/python -m research_orchestrator.workers.agent_worker
```

In a third terminal, create and execute the no-cost fixture workflow:

```bash
.venv/bin/python -m research_orchestrator.cli make-fixture \
  --repo "$PWD" \
  --output /tmp/go2-base-fixture.json
.venv/bin/python -m research_orchestrator.cli run \
  --spec /tmp/go2-base-fixture.json
```

The generated spec records the exact Git SHA, ref, and dirty state. The final
JSON contains the validated experiment, result, diagnosis, and next action.

Codex is an escalation path, not the default loop. To explicitly test it with
an unknown fixture, add `--fixture-verdict UNKNOWN --allow-codex`; this invokes
the logged-in Codex CLI with a bounded prompt, read-only sandbox, ephemeral
session, and output schema. By default the worker removes API-key variables so
the Base path uses the local ChatGPT login rather than silently switching to a
metered API. The worker defaults to medium reasoning and the model/timeout are
configurable through `CODEX_MODEL`, `CODEX_REASONING_EFFORT`, and
`CODEX_TIMEOUT_S`. No Codex activity is allowed to edit or push the repository.

## Atlas contract

Inspect the contract without starting a worker:

```bash
.venv/bin/python -m research_orchestrator.workers.atlas_worker --check
```

The reserved Activity set is:

```text
build_source
run_unit_tests
run_dev_probe
run_b0_member
run_b0_holdout
run_b1_probe
extract_failure_window
```

The future adapter must bind these names to an allowlisted operation map for
the existing checkout scripts and analyzers. Inputs are structured specs, not
`shell`, executable paths, or arbitrary argument arrays. Each operation must
record the source SHA, effective profile, semantic environment, artifact
hashes, and analyzer status. Physical runs use one attempt by default because
Temporal retrying a run after an acknowledgement boundary can duplicate the
experiment.

`policies/b0.yaml` is a WIP sensor-only contract and `policies/b1.yaml` is a
draft. Neither grants an acceptance claim, changes a controller algorithm, or
defines missing B1 thresholds. Formal holdouts remain explicit human-approved
checkpoints.
