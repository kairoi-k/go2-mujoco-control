# Atlas integration plan (WIP)

This document is the handoff boundary between the Base control plane and the
Atlas/WSL execution worker. It is a plan, not evidence that an Atlas run has
passed. On 2026-09-03 the Base-side check of `ssh 5080` failed because the
configured Tailscale hostname did not resolve, so the Atlas WSL distribution,
checkout, Python environment, and runtime state remain unverified.

## Fixed topology

```text
Base / Temporal dev server
  agent task queue
    ResearchWorkflow
    validate_experiment
    fixture probe
    deterministic classifier
    optional read-only Codex diagnosis
          |
          | run_dev_probe -> fixed atlas task queue
          v
Atlas / Ubuntu-22.04 WSL
  allowlisted adapter
    build / tests / probe / evidence extraction
          |
          v
  compact result.v1 + hashed artifact references
```

The workflow routes only the physical `run_dev_probe` Activity to the `atlas`
queue. The Atlas worker does not host a workflow and does not call Codex. This
keeps model calls and real-time-sensitive simulation separate.

## Required recovery checks

Run these only after `ssh 5080` is live again; the expected checkout path comes
from the existing Atlas convention and must be confirmed rather than assumed:

```text
ssh 5080 hostname
ssh 5080 wsl.exe --list --quiet
ssh 5080 wsl.exe -d Ubuntu-22.04 -- git -C /home/che/dev/go2-mujoco-control status --short --branch
ssh 5080 wsl.exe -d Ubuntu-22.04 -- git -C /home/che/dev/go2-mujoco-control rev-parse HEAD
ssh 5080 wsl.exe -d Ubuntu-22.04 -- python3 --version
```

The Atlas checkout must be clean or isolated in a fresh worktree at the exact
`experiment.v1.source.git_sha`. Do not copy Base absolute paths, auth files, or
controller changes into it. Install the same locked Python dependencies in an
Atlas-local virtual environment and point `TEMPORAL_ADDRESS` at the Base
server only when the network path is deliberately available.

## Activity boundary

The registered names are intentionally fixed:

```text
build_source
run_unit_tests
run_dev_probe
run_b0_member
run_b0_holdout
run_b1_probe
extract_failure_window
```

Each request is a JSON object validated against `experiment.v1` or a narrowly
defined derived contract. It may carry an experiment id, exact source SHA,
policy/profile, seed, duration, and scalar parameters. It must not carry shell
text, executable paths, environment assignments, or arbitrary argument arrays.

The adapter should implement a static operation map roughly as follows, after
the live Atlas checkout and existing scripts are verified:

| Activity | Operation boundary | Safety/provenance requirement |
|---|---|---|
| `build_source` | isolated worktree at the requested SHA, then the documented CMake build | record SHA, compiler/dependency snapshot, binary hashes |
| `run_unit_tests` | configured CTest target set | return test names and status; no acceptance claim |
| `run_dev_probe` | one bounded development probe using an existing reviewed runner | record effective args, CPU affinity, logs, analyzer identity |
| `run_b0_member` | one sensor-only B0 member | prove no terrain actuation and preserve raw failure evidence |
| `run_b0_holdout` | formal holdout checkpoint | human approval, one attempt, no automatic retry |
| `run_b1_probe` | draft B1 probe only | remain WIP until semantics and thresholds are reviewed |
| `extract_failure_window` | deterministic extraction around first anomaly | return bounded evidence window and artifact hashes |

The current Base implementation registers these names but fails closed with
`ATLAS_ADAPTER_NOT_READY`; it cannot produce a fake MuJoCo result. The exact
runner/profile mapping is deliberately not frozen until Atlas is reachable and
the existing scripts are tested there.

## Bring-up order

1. Restore and verify `ssh 5080` and the WSL checkout.
2. Install `temporalio` from `uv.lock` and run `atlas_worker --check`.
3. Implement the static adapter with repository-root and artifact-root
   allowlists; use subprocess argument arrays only inside the adapter.
4. Run build and unit-test Activities against the exact source SHA.
5. Run one short development probe and validate `result.v1` plus manifest/hash
   output back on Base.
6. Add B0 member execution only after CPU affinity, wall-clock timing, and
   sensor-only non-actuation are evidenced.
7. Keep B0 holdout and all B1 work behind explicit human approval.

No step in this plan changes C++ control laws, gains, physics, or acceptance
thresholds.
