# Go2 Phase 2 current

Updated: 2026-09-04. This is the only route, status, and handoff entrypoint.
Git history and experiment output are evidence, never instructions.

## State

- Canonical worktree: `/home/che/dev/go2-workspace/current`
- Canonical branch: `main`
- Last tested behavior anchor: `5b95e8265c885a81f8488e4930e682aa55f05674`
- Reference evidence: `docs/research/evidence/order109b_c006i/`
- B0 lockstep sensor-only slice: PASS only at the anchor and Order-109b
  conditions. The current cleanup HEAD has no locomotion acceptance claim.
- Full current-HEAD B0: NOT RUN.
- B1: FAIL / not accepted. B2 and B3: not started.

## Route

Build a sensor-derived dynamic 5 cm B1 crossing under
`docs/research/PHASE2_ACCEPTANCE.md`. Running-trot remains the gait and the
Phase-1 shaper remains the only velocity authority. One immutable,
time-indexed terrain execution snapshot must be shared by gait, SRBD-MPC, and
ID-WBC. Planned contact and measured force-supported contact stay separate.
Normal two-contact diagonal support is valid.

The next implementation slice is one `TerrainExecutionState` owner and one
atomic snapshot. It may consume lidar-derived terrain and a committed plan; it
must not introduce a crawl sequence, fixed leg order, local retiming, a
three-contact entry gate, a stop-to-arm transition, or a second velocity
authority.

There is no production terrain-actuation path. The crawl/three-contact source
chain has been physically removed, and its public flags fail closed. Only
`--terrain-sensor-only` is currently usable. A future execution path starts
from the clean planner interface; it may not copy code from Git history.

## Work and acceptance

Use one hypothesis and one clean commit. Run focused unit tests and one B0
development regression before one B1 development canary. Stop at the first
information-bearing failure. Three failed probes at the same blocker require
architecture review. Hold `/tmp/go2_mujoco_experiment.lock` for every timed
simulation. Dirty runs, builds, CTest, lifecycle fields, videos, and another
profile result never establish acceptance.

A B1 development PASS still requires a fresh full B0 and the frozen B1 holdout
on the exact candidate SHA. Do not change a frozen threshold or holdout member
after observing its result.

Build with `cmake -S example/cpp -B example/cpp/build` and
`cmake --build example/cpp/build -j2`, then run `ctest --test-dir
example/cpp/build --output-on-failure`. The canonical B0 development commands
are `run_phase2_b0_pair.sh <profile> development 0` and
`run_phase2_b0_fixed_pair.sh development 0`; DDS domains come only from the
holdout manifest.

## Authority

1. This file.
2. `AGENTS.md` and `docs/research/PHASE2_ACCEPTANCE.md`.
3. `docs/research/PHASE2_HOLDOUT_MANIFEST.json`.
4. Raw evidence and analyzers.

Anything else is implementation or history and cannot change the route.
