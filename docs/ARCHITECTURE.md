# Architecture

This document maps the code; it does not define research status or the next
task. For Phase 2, read [`CURRENT.md`](../CURRENT.md) first.

## Runtime data flow

The MuJoCo process and 500 Hz C++ controller communicate through the Unitree
SDK2 DDS interface.

```text
MuJoCo -> LowState / lidar -> state snapshot and filtering
                                  |
requested velocity -> Phase 1 shaper -> running-trot gait and footholds
                                           |
                                        SRBD MPC
                                           |
                                      18-DoF ID-WBC
                                           |
                               limits, safety, diagnostics
                                           |
                                      LowCmd -> MuJoCo
```

The sensor-only terrain path derives a terrain model, feasibility results, and
a plan from lidar. On the current line it may be logged and analyzed, but it
does not alter gait, MPC, WBC, contact policy, or velocity. There is no
production terrain-actuation path.

## Active modules

| Area | Primary files | Responsibility |
|---|---|---|
| CLI and lifecycle | `example/cpp/trot/trot_cli.*`, `trot_experiment_lifecycle.cpp` | configuration, DDS, startup, shutdown |
| Control loop | `example/cpp/trot/trot_experiment_control.cpp` | state snapshot, phase execution, command publication |
| Gait | `example/cpp/trot/trot_experiment_gait.cpp`, `example/cpp/gait/*` | running-trot phase, footholds, velocity targets |
| Terrain interfaces | `example/cpp/terrain/*` | sensor-derived model, feasibility, immutable plan interface |
| MPC and WBC | `example/cpp/trot/trot_experiment_wbc.cpp`, `example/cpp/wbc/*` | SRBD preview and inverse-dynamics control |
| Kinematics | `example/cpp/kinematics/*` | FK, IK, Jacobians, rigid-body model |
| Contact | `example/cpp/contact/*` | measured-contact filtering, wrench allocation, torque mapping |
| Timing | `example/cpp/trot/lockstep_*` | lockstep clock/writer diagnostics; not a realtime acceptance claim |
| Diagnostics | `example/cpp/trot/trot_experiment_diagnostics.cpp`, `trot_types.h` | limits, status, structured logs |
| Tests and analysis | `example/cpp/tests/`, `example/cpp/tools/` | unit/integration checks and protocol analyzers |

## Phase 2 invariants

The Phase 1 shaper remains the only velocity authority. Planned contact and
force-supported measured contact remain separate. A future terrain execution
path must publish one immutable, time-indexed snapshot shared by gait, SRBD-MPC,
and ID-WBC; no consumer may invent its own timing, contact, or recovery state.

The retained `example/cpp/leg_lift/` executable and multi-step configurations
are historical experiments. They are not a Phase 2 route or design source.
Removed crawl/three-contact code and Git history are not fallback
implementations.

## Repository boundaries

`simulate/` and `unitree_robots/go2/` remain close to the upstream Unitree
simulator. Research-specific model control lives in `example/cpp/`. Isaac Lab
RL and Kine2Go imitation live only in their companion repositories and are not
runtime dependencies here.

See [`CODE_GUIDE.md`](CODE_GUIDE.md) for source navigation,
[`REPRODUCIBILITY.md`](REPRODUCIBILITY.md) for execution rules, and
[`../UPSTREAM_AND_CONTRIBUTIONS.md`](../UPSTREAM_AND_CONTRIBUTIONS.md) for
provenance.
