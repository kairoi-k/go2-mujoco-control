# Atlas integration boundary

The Atlas side is now a real, bounded Activity adapter. The control-plane
checkout and the physical research checkout are intentionally separate:

```text
Base:   /Users/wangche/Code/go2-mujoco-control
        Temporal + agent worker + Codex diagnosis

Atlas:  /home/che/dev/go2-workspace/research-orchestrator
        Atlas worker code, pinned to the WIP control-plane commit

Atlas:  /home/che/dev/go2-workspace/current
        physical Go2 source, currently phase2-b1-b3 at its exact source SHA
```

This separation keeps the existing Atlas research worktree untouched while
making the physical source revision explicit in every experiment. The worker
requires `ATLAS_WORKSPACE`, `ATLAS_ARTIFACT_ROOT`, `ATLAS_MUJOCO_ROOT`, and
`ATLAS_ADAPTER_READY=1`. It connects to the Base Temporal dev server through
the Base Tailscale address.

## Activity contract

The registered names are fixed:

```text
build_source
run_unit_tests
run_dev_probe
run_b0_member
run_b0_holdout
run_b1_probe
extract_failure_window
```

`build_source` configures and compiles the existing `simulate` and
`example/cpp` CMake projects in their standard ignored build directories. It
also checks that the resulting controller exposes the reviewed
`--terrain-sensor-only` entry point. `run_unit_tests` runs the controller and
simulator CTest suites. Both return an `activity.v1` receipt with preserved
logs.

`run_dev_probe` accepts only `b0-development`, one of the fixed Phase 1
velocity profiles, a matching profile duration, and a DDS domain in the
development range 190..199. It constructs the full runner argv internally,
uses `TROT_CPU_AUTOPIN=1` and the existing B0 development environment, starts
the reviewed `run_trot.sh`, kills the whole process group on timeout/cancel,
and copies bounded evidence to `ATLAS_ARTIFACT_ROOT/<experiment_id>/`.

The returned `result.v1` separates execution status from the physical verdict:
a completed run may still be `FAIL_TIMING` or `FAIL_SAFE_STOP`. Missing
manifest, timeout, source drift, or lifecycle failure is retained as runner
failure. A failure window is created only when a timestamp is actually
present; the adapter never invents one.

`run_b0_member`, `run_b0_holdout`, and `run_b1_probe` are registered but fail
closed with `ATLAS_FORMAL_APPROVAL_REQUIRED`. `extract_failure_window` only
accepts a validated result containing an observed failure timestamp.

## Bring-up and verification

The verified Atlas host facts are:

- Tailscale reaches Atlas at `100.106.114.65`; Base reaches Temporal through
  `mac-mini.tail4a075c.ts.net:7233`.
- Ubuntu-22.04 WSL is available and uses Python 3.12 through `uv`.
- MuJoCo 3.3.6 and Unitree SDK2 are installed.
- The pinned source built both simulator/controller targets and passed 26
  controller CTest cases; the simulator CTest suite is part of the worker
  preflight.
- The original `/home/che/dev/go2-workspace/current` checkout remains clean.

To run the worker check:

```bash
cd /home/che/dev/go2-workspace/research-orchestrator
ATLAS_WORKSPACE=/home/che/dev/go2-workspace/current \
ATLAS_ARTIFACT_ROOT=/home/che/dev/go2-workspace/atlas-artifacts \
ATLAS_MUJOCO_ROOT=/home/che/.mujoco/mujoco-3.3.6 \
TEMPORAL_ADDRESS=mac-mini.tail4a075c.ts.net:7233 \
ATLAS_ADAPTER_READY=1 \
uv run python -m research_orchestrator.workers.atlas_worker --check
```

No formal holdout is started by the generic workflow. A future approval
workflow must add a human decision, frozen membership, separate domains, and
an explicit evidence/rollback policy before binding those names to physical
commands.
