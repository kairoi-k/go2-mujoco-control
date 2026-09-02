# Local research orchestration (WIP)

This package is the local control plane for the `go2-mujoco-control` research
repository. Base hosts Temporal and the agent worker; Atlas/Ubuntu-22.04 WSL
hosts the only worker allowed to start MuJoCo/controller processes.

```text
Base / Mac mini
  Temporal dev server (Tailscale address, local SQLite)
  agent task queue: validation, deterministic diagnosis, optional read-only Codex
        |
        | atlas task queue: fixed physical Activities
        v
Atlas / WSL
  exact source SHA, fixed CMake/CTest, reviewed runner, hashed artifacts
```

The physical source revision and the Base control-plane revision are recorded
separately in `experiment.v1`. This is required because the long-lived Atlas
research checkout may be on a reviewed Phase 2 branch while the orchestration
code evolves independently.

## Base

```bash
uv sync --extra dev
research_orchestrator/scripts/start_temporal_dev.sh
research_orchestrator/scripts/start_agent_worker.sh
```

The Temporal server binds to the current Base Tailscale IPv4 address only and
stores state in `.temporal/dev.sqlite`. The worker uses the same address. The
Codex escalation path runs the logged-in local CLI with an explicit model,
bounded timeout, read-only sandbox, ephemeral session, and strict output
schema. API-key environment variables are removed by default.

For a no-cost control-plane smoke test:

```bash
.venv/bin/python -m research_orchestrator.cli make-fixture \
  --repo "$PWD" --output /tmp/go2-base-fixture.json
.venv/bin/python -m research_orchestrator.cli run \
  --spec /tmp/go2-base-fixture.json --address "${TEMPORAL_ADDRESS:-100.90.49.95:7233}"
```

## Atlas development workflow

The current verified Atlas source checkout is `/home/che/dev/go2-workspace/current`
on branch `phase2-b1-b3`; its source SHA must be read live before making a
spec. The separate control-plane checkout is
`/home/che/dev/go2-workspace/research-orchestrator`.

On Base, create a spec using the exact Atlas SHA:

```bash
.venv/bin/python -m research_orchestrator.cli make-atlas-spec \
  --repo "$PWD" \
  --source-sha <40-character-atlas-sha> \
  --source-ref phase2-b1-b3 \
  --scenario accel_1_to_3 \
  --output /tmp/go2-atlas-development.json
```

On Atlas, install/verify the worker:

```bash
cd /home/che/dev/go2-workspace/research-orchestrator
ATLAS_WORKSPACE=/home/che/dev/go2-workspace/current \
ATLAS_ARTIFACT_ROOT=/home/che/dev/go2-workspace/atlas-artifacts \
ATLAS_MUJOCO_ROOT=/home/che/.mujoco/mujoco-3.3.6 \
TEMPORAL_ADDRESS=mac-mini.tail4a075c.ts.net:7233 \
ATLAS_ADAPTER_READY=1 \
uv run python -m research_orchestrator.workers.atlas_worker --check
```

Start the worker with the same variables and omit `--check`. The worker is
serialized to one physical Activity at a time. The first actual workflow runs
`build_source`, `run_unit_tests`, and then the fixed sensor-only
`run_dev_probe`; it returns `result.v1`, preflight receipts, and artifact
references back to Base.

The fixed development profiles are `steps`, `accel_1_to_3`, `brake_3_to_0`,
`ramp`, and `varying`. The adapter constructs their paths and arguments; a
spec cannot provide shell text, executable paths, arbitrary arguments, or
environment assignments. Formal B0 holdout, B1, and acceptance claims remain
behind an explicit human-approved workflow and are currently rejected by the
worker.

## Safety and provenance

- Atlas requires a clean checkout at the exact `source.git_sha`.
- Build and test commands are fixed CMake/CTest invocations; physical runs use
  the existing `example/cpp/scripts/run_trot.sh` through a fixed argv map.
- Every run preserves bounded logs, CSV/JSON evidence, the existing manifest,
  source SHA, runner hash, and artifact SHA-256 values outside the checkout.
- Temporal retries for physical/model calls are disabled (`maximum_attempts=1`).
- No Activity edits controller sources, changes gains/physics/thresholds, or
  treats a development pass as B0/B1 acceptance.
