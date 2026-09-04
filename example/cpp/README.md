# Go2 C++ control stack

This directory contains the model-based locomotion controller, tests, runners,
and retained evidence. Before any Phase 2 action, read
[`CURRENT.md`](../../CURRENT.md); it is the sole route/status authority.

## Build and test

From the repository root, after MuJoCo and Unitree SDK2 are configured:

```bash
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j2
ctest --test-dir example/cpp/build --output-on-failure
```

The main target is `real_trot_go2`. Some tests require
`simulate/mujoco/lib/libmujoco.so`. See
[`docs/REPRODUCIBILITY.md`](../../docs/REPRODUCIBILITY.md) for environment,
experiment-lock, and acceptance rules.

## Phase 2 entrypoints

Only the following runners belong to the current Phase 2 development workflow:

| Runner | Role |
|---|---|
| `scripts/run_phase2_b0_pair.sh` | profile-based B0 development pair |
| `scripts/run_phase2_b0_fixed_pair.sh` | fixed B0 development pair |
| `scripts/run_phase2_b0_lockstep_pair.sh` | determinism diagnostic only |

Use the exact arguments and DDS domains specified by `CURRENT.md` and
`docs/research/PHASE2_HOLDOUT_MANIFEST.json`. Hold
`/tmp/go2_mujoco_experiment.lock` for timed simulations. No generic runner,
historical script, or experiment note may substitute for these entrypoints.

## Source map

| Area | Location | Responsibility |
|---|---|---|
| Entry and lifecycle | `trot/real_trot_go2.cpp`, `trot/trot_cli.*`, `trot/trot_experiment_lifecycle.cpp` | CLI, DDS, startup/shutdown |
| Execution | `trot/trot_task.*`, `trot/trot_experiment_control.cpp` | 500 Hz sequencing and commands |
| Gait | `trot/trot_experiment_gait.cpp`, `gait/` | running-trot phase, footholds, velocity targets |
| Terrain | `terrain/` | sensor-derived model, feasibility, planner interfaces |
| MPC/WBC | `trot/trot_experiment_wbc.cpp`, `wbc/` | SRBD MPC and 18-DoF ID-WBC |
| Contact/kinematics | `contact/`, `kinematics/` | contact truth/filtering, wrench mapping, FK/IK |
| Timing/diagnostics | `trot/lockstep_*`, `trot/trot_experiment_diagnostics.cpp` | timing checks, limits, logs |
| Verification | `tests/`, `tools/` | registered tests and analyzers |
| Operations/evidence | `scripts/`, `experiments/` | runners and retained records |

For deeper navigation use [`MODULES.md`](MODULES.md),
[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md), and
[`docs/CODE_GUIDE.md`](../../docs/CODE_GUIDE.md).

## Historical utilities

`scripts/go2sim`, sustained-running wrappers, and the standalone
`leg_lift/` executable remain useful for their documented Phase 1 or historical
scope. The leg-lift/multi-step sequence is not a Phase 2 route or design source.
Do not infer current status from a script name or retained artifact.

Research-relevant runs must record revision, configuration, semantic
environment, analyzer, result, and evidence path. Raw `experiments/_runs/`
content is ignored immutable local evidence. Claims and history live in
[`docs/RESEARCH_INDEX.md`](../../docs/RESEARCH_INDEX.md) and
[`docs/RESEARCH_HISTORY.md`](../../docs/RESEARCH_HISTORY.md).
