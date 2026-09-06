# Stage C C0-01/C0-02 foundation evidence

Date: 2026-09-06

This record belongs to `feat/stage-c-joint-planner`. The fetched remote head
before implementation was `330e2cb98b0b5e79b71b9b59152d900847857ce5`.
The implementation commit is `0e09535c43d4fd26be3de387f02752a43115fa0f`.

## Scope and evidence boundary

The repository fixture is `example/cpp/tests/test_stage_c_foundations.cpp`,
registered as `test_stage_c_foundations`. It exercises actual production
helpers and the new default-off Stage C foundation headers. It does not run
the controller, B0/B1, Atlas, terrain actuation, or a continuous COM/body/
force solver.

The H8/H9 raw `_runs` directories were not present in either the fetched
worktree or the inspected canonical worktree during this turn. No historical
H5 foothold coordinates or missing telemetry were reconstructed. All new
fixtures are explicitly synthetic and complete enough only for the stated
contract witness.

## C0-01 witnesses

| fixture | result | established boundary |
|---|---|---|
| T01 | PASS | The normalized adapter ignores capture mode and preserves the same force-backed measured world anchor for shadow and actuation inputs. Missing measured anchor returns `ObservationUnavailable`; planned/applied contact is not promoted. The legacy producer remains asymmetric: `trot_experiment_control.cpp` still gates FK/anchor initialization on `terrain_actuation && have_high_state` (lines 156 and 178-183). |
| T02 | PASS | `SafeFootholdRegionHalfExtent()` is the production expression used by `BuildSafeFootholdRegions()`. Exact `0.05/0.025` yields zero; float32 round-off is below `1e-8 m`. No epsilon was added; C0 must use point candidates. |
| T03 | PASS | Valid map metadata is separated from coverage: all-unknown in-grid cells classify as `unknown_inside`, out-of-grid as `outside_grid`, and complete observed coverage as `known`. No height is imputed. |
| T04 | PASS | Heading-map XY uses yaw-only rotation; body FK uses full roll/pitch/yaw rotation. The fixture produces the expected >3 cm x difference at 5 degrees pitch. |
| T05 | PASS | Same-leg touchdowns retain distinct `(schedule_epoch, leg, sequence)`, time, target, and contact interval; duplicate event identities are rejected. |
| T06 | PASS | Coverage is checked through the consumer interval end. A 20 ms planning grid and 30 ms consumer demand a rejection when the last required endpoint is outside the prediction horizon; no last-knot clamp occurs. |
| T07 | PASS | The frozen 15 mm initial support condition is treated as a fixed-state conflict. The fixture does not move initial COM or relabel the conflict as a future-candidate failure. |
| T13 | PASS | A transfer sample with an aerial interval and `min_contacts=0` is retained as `kTransferRequiresTwoContactsButAerialInterval`; no analyzer flag or contact is rewritten. |
| T14 | PASS | Supplied anchor span, solver-reference span, and predicted-state span are separate fields. The frozen `wbc_mpc_reference_x_*` metric is modeled as solver-reference span, not predicted COM span. Current legacy logging assigns the same `mpc_in.reference[3]` to first and last (lines 620-623), so its observed span is not a horizon trajectory span. |
| T15 | PASS | A proposal may change only the uncommitted suffix. A committed touchdown target/time change is rejected by the event-table compatibility check. |

## C0-02 foundation

`example/cpp/terrain/stage_c/types.h` defines typed time/frame points,
body/COM/foot observations, separate measured/planned/applied contact
provenance, map coverage state, command authority, event identities and
intervals, candidate sets, rollout/certificate placeholders, and pending vs
accepted execution bundle types. `joint_planner.h` defines the common planner
request, failure taxonomy, deterministic exhaustive implementation, and
unbounded small-fixture oracle. It evaluates complete multi-event combinations
in lexicographic order and distinguishes empty candidates, incomplete search,
budget exhaustion, numerical failure, observation failure, initial-condition
conflict, coverage failure, and commitment conflict.

The comparison seam is `PlannerComparisonFixture`: legacy raw, normalized
legacy, and future C0 entry points carry the same post-adapter input, event
table, and candidate-set types. No old planner optimization or control-path
integration was changed.

## Reproduction

```bash
cmake -S example/cpp -B example/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build example/cpp/build --target test_stage_c_foundations -j2
ctest --test-dir example/cpp/build -R stage_c_foundations --output-on-failure
```

Focused result: 1/1 PASS. This is a unit/contract result only; it is not B0,
B1, real-time, command-invariance, terrain-crossing, or release evidence.

The requested full build was also attempted. It stops in the pre-existing
`real_trot_go2` target because this checkout has no
`simulate/mujoco/include/mujoco/mujoco.h` (the MuJoCo directory is absent).
After that environmental blocker, all 30 available registered tests, including
the new fixture, built and passed with
`ctest --test-dir example/cpp/build -E test_lockstep_motion_clock_integration
--output-on-failure`. The 31st integration test requires the same unavailable
MuJoCo dependency and was not falsely counted as passed.
