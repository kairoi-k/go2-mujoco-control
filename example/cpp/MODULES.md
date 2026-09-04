# C++ module index

This is a stable source map, not a progress tracker. Phase 2 route and status
come only from [`CURRENT.md`](../../CURRENT.md). For task-oriented navigation,
use [`docs/CODE_GUIDE.md`](../../docs/CODE_GUIDE.md).

## Runtime path

Read the active controller in this order:

1. `trot/trot_cli.*` — configuration and public flags.
2. `trot/trot_experiment_lifecycle.cpp` — DDS and lifecycle.
3. `trot/trot_task.*` and `trot/trot_experiment_control.cpp` — sequencing and
   the 500 Hz command loop.
4. `trot/trot_experiment_gait.cpp` and `gait/` — running-trot phase,
   footholds, and the Phase 1 velocity path.
5. `terrain/` — sensor-derived terrain model, feasibility, and plan interfaces.
6. `trot/trot_experiment_wbc.cpp`, `wbc/`, `contact/`, and `kinematics/`
   — SRBD MPC, ID-WBC, contact, and robot math.
7. `trot/trot_experiment_diagnostics.cpp` and `trot/lockstep_*` — status,
   logging, and timing diagnostics.

## Executables and verification

| Item | Location |
|---|---|
| Main controller | `trot/real_trot_go2.cpp` |
| Stand/hold/track utilities | `apps/` |
| Historical leg-lift executable | `leg_lift/real_leg_lift_go2.cpp` |
| Tests | `tests/` |
| Runners | `scripts/` |
| Analyzers | `tools/`, `tools/analysis/` |
| Retained artifacts | `experiments/`, indexed by `experiments/CATALOG.md` |

The `leg_lift/` action sequence and its configuration files are retained only
for historical experiments. They must not be used as a Phase 2 implementation,
fallback, or design source. There is no `scripts/batch/` entrypoint.

Generated include graphs, line counts, and refactor-progress snapshots are
intentionally omitted because the source and CMake graph are authoritative.
