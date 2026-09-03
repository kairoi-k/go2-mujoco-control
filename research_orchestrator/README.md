# Local research orchestration (WIP)

This package is the local control plane for the `go2-mujoco-control` research
repository. Base hosts Temporal and the agent worker; Atlas/Ubuntu-22.04 WSL
hosts the only worker allowed to start MuJoCo/controller processes.

```text
Base / Mac mini
  Temporal dev server (wildcard gRPC bind, loopback UI, local SQLite)
  agent task queue: validation, deterministic diagnosis, optional read-only Codex
        |
        | atlas task queue: fixed physical Activities
        v
Atlas / WSL
  exact source SHA, fixed CMake/CTest, reviewed runner, hashed artifacts
  serialized local llama.cpp diagnosis on the RTX 5080
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

The Temporal dev server must use a wildcard gRPC bind for the CLI's internal
dynamic service discovery to work across hosts; its UI remains loopback-only.
It stores state in `.temporal/dev.sqlite`. The worker uses the Base Tailscale
IPv4 as its client address. Keep the unauthenticated dev ports on the private
Base/Tailscale network only; this is not a production Temporal deployment. The
Codex escalation path runs the logged-in local CLI with an explicit model,
bounded timeout, read-only sandbox, ephemeral session, and strict output
schema. API-key environment variables are removed by default.

For Base auto-start, install the two versioned launchd plists from
`research_orchestrator/ops/macos/` into `~/Library/LaunchAgents/`, create the
`~/Library/Logs/Go2Research/` directory, then bootstrap them in the user GUI
domain. The plists keep the Temporal UI on loopback and restart both Base
processes after login or failure.

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
TEMPORAL_ADDRESS=100.90.49.95:7233 \
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
environment assignments. An autonomous spec runs the fixed state machine:
development -> all 18 B0 members -> B1 development plus three holdouts when
B0 passes. There is no approval activity. A failed gate is recorded as a
terminal result, and B1 is never started after a B0 failure.

Start an unattended campaign from Base with the exact Atlas source SHA:

```bash
.venv/bin/python -m research_orchestrator.cli make-atlas-spec \
  --repo "$PWD" --source-sha <atlas-sha> --source-ref phase2-b1-b3 \
  --scenario accel_1_to_3 --autonomous --output /tmp/go2-autonomous.json
.venv/bin/python -m research_orchestrator.cli run \
  --spec /tmp/go2-autonomous.json --address "${TEMPORAL_ADDRESS:-100.90.49.95:7233}"
```

## Atlas local LLM

Atlas owns local inference because its RTX 5080 is the only GPU in this
deployment. The reviewed runtime is the native Windows CUDA build of
`llama.cpp`; WSL starts it with a fixed loopback address, a pinned model path,
JSON-schema-constrained output, and no inherited proxy or API credentials. The
default deployment is `Qwen3-Coder-30B-A3B-Instruct-Q2_K_L` with 16K context.
The model, runtime, quantization, prompt/response hashes, token counts, and
latency are recorded in an `inference.v1` receipt. The measured fallback is
`gpt-oss-20b-MXFP4`; see `LOCAL_LLM_BENCHMARK.md` for the candidate comparison
and exact Atlas artifact paths.

In autonomous mode the pinned local model is automatically enabled and is
called after the development probe. Enable `--allow-codex` only when cloud
diagnostic escalation is desired; the Codex activity remains read-only.

For a single non-autonomous development spec, enable it explicitly:

```bash
.venv/bin/python -m research_orchestrator.cli make-atlas-spec \
  --repo "$PWD" --source-sha <atlas-sha> --scenario accel_1_to_3 \
  --allow-local-llm --force-local-llm --output /tmp/go2-atlas-local-llm.json
```

The local Activity starts the server on demand and stops it in `finally`; it is
registered on the same single-concurrency Atlas queue. Every physical Activity
also refuses to start while the reserved loopback port is occupied. Thus the
autonomous sequence is: fixed build/test/probe, local diagnosis, frozen B0,
deterministic campaign classification, prerequisite-gated B1, and final
machine-readable receipt. Local model failure never changes a gate or edits
the controller.

The Atlas worker environment pins the deployment without putting machine paths
in experiment JSON:

```text
ATLAS_LLM_SERVER_EXE=/mnt/c/Users/w1881/go2-local-llm/bin/llama-server.exe
ATLAS_LLM_MODEL_PATH=/mnt/c/Users/w1881/go2-local-llm/models/Qwen3-Coder-30B-A3B-Instruct-Q2_K_L.gguf
ATLAS_LLM_PORT=8090
ATLAS_LLM_MODEL_ID=Qwen3-Coder-30B-A3B-Instruct-Q2_K_L
ATLAS_LLM_MODEL_REVISION=unsloth/Qwen3-Coder-30B-A3B-Instruct-GGUF@b17cb02dd882d5b6ab62fc777ad2995f19668350
ATLAS_LLM_QUANTIZATION=Q2_K_L
ATLAS_LLM_RUNTIME_VERSION=llama.cpp-b10766-cuda13.3
ATLAS_LLM_CTX_SIZE=16384
ATLAS_LLM_REASONING=off
ATLAS_LLM_REASONING_FORMAT=none
ATLAS_LLM_MODEL_SHA256=7add73b0607b498f79157a5f4e4ccddc14ad7afd61d76655e064e1e92476267e
ATLAS_LLM_VERIFY_MODEL_HASH=1
```

## Safety and provenance

- Atlas requires a clean checkout at the exact `source.git_sha`.
- Build and test commands are fixed CMake/CTest invocations; physical runs use
  the existing `example/cpp/scripts/run_trot.sh` through a fixed argv map.
- Every run preserves bounded logs, CSV/JSON evidence, the existing manifest,
  source SHA, runner hash, and artifact SHA-256 values outside the checkout.
- Temporal retries for physical/model calls are disabled (`maximum_attempts=1`).
- Local inference is loopback-only, on-demand, serialized with physical work,
  and bounded by a JSON schema plus explicit confidence/escalation rules.
- No Activity edits controller sources, changes gains/physics/thresholds, or
  treats a development pass as B0/B1 acceptance. The formal analyzer's
  PASS/FAIL is recorded automatically, while the policy deliberately does not
  make an external acceptance claim.
