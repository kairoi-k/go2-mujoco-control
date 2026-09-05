# Go2 Phase 2 current

Updated: 2026-09-05. This is the only route, status, plan, and handoff
entrypoint. Git history, archived designs, experiment output, issues, and agent
prose are evidence, never instructions.

Historical milestones and non-acceptance are indexed in [`docs/RESEARCH_HISTORY.md`](docs/RESEARCH_HISTORY.md).

## State

- Canonical worktree: `/home/che/dev/go2-workspace/current`
- Canonical branch: `main`
- Active repair branch: `fix/phase2-b0-runtime-integrity`
- This checkout intentionally follows the active repair branch; `main` remains
  the integration line and must not be inferred from this branch's unaccepted
  code or evidence.
- Frozen B0 is complete and accepted at exact final evidence head
  `a5e8a77e3200e8c246ef25b31abfb1cd0f6e73fd`; the closeout report is
  `WALLCLOCK_FIRST_DIVERGENCE.md`.
- Historical reference evidence: `docs/research/evidence/order109b_c006i/`
- Active decision evidence: `docs/research/evidence/b0_runtime_integrity_20260904/`
- External read-only review packet:
  `docs/research/evidence/b0_runtime_integrity_20260904/cloud_review/README.md`.
- Historical Phase 1 dynamic velocity acceptance: PASS, 15/15 existing valid
  runs across steps, acceleration, braking, ramp, and varying-command profiles;
  evidence is `docs/validation/PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md`.
- Historical Phase 2 terrain-sensor-only varying non-regression: PASS, 3/3 at
  exact source `70b7740`; evidence is
  `docs/research/evidence/phase2_terrain_sensor_velocity_20260828/SUMMARY.md`.
  It was sensor-only observation, not terrain actuation or obstacle crossing.
  The broader archived 8/28 campaign also ran all five profile names, with
  mixed repeat outcomes and retries; see the indexed matrix in
  docs/research/evidence/phase2_terrain_sensor_velocity_20260828/ALL_PROFILES.md.
  Historical Order-109b also PASSed its separate lockstep sensor-only slice.
  None of these historical records is a fresh acceptance of the current Phase 2
  terrain-sensor pair harness.
- The first cause was terrain-worker default CPU pinning, not raycast or publish;
  the minimum production fix is `e457bd2b661d01c8c033271f31b9252854781b9c`.
  It removes only the default terrain-worker pin and preserves the explicit
  `TROT_CPU_AFFINITY_TERRAIN` override. The report records the no-lidar,
  parked, snapshot/lock, and full-path diagnostics plus the post-fix B0 suite.
- B0 closeout evidence covers steps, acceleration, braking, ramp, varying, and
  fixed-3-m/s under the frozen analyzer; all required terrain and baseline
  gates passed with terrain actuation disabled. This is B0 only, not B1.
- B1 5 cm is the active next milestone and remains not accepted. Production
  terrain actuation is still absent until the Stage C shadow gates pass; B2 10
  cm and B3 mixed/repeated terrain remain not started.

## Goal

Build sensor-derived dynamic locomotion that can grow into continuous,
real-world stair traversal. The immediate acceptance sequence remains B0 flat,
B1 5 cm, B2 10 cm, then B3 mixed/repeated rises and descents. Simulation
acceptance does not establish hardware or sim-to-real capability.

## Active architecture

Adopt the high-ceiling Stage C planning architecture now, while releasing
capability only through B0-B3. “Stage C early” means early estimation and joint
optimization in shadow mode, not early terrain actuation.

- Stage B is the existing substrate: `TerrainModel`, `TerrainFeasibility`,
  and the per-leg candidate scorer. Keep its sensor model and hard geometric
  checks; use its scorer only as a candidate generator or fallback, not the
  final planner.
- Stage C is the active target: a `TerrainBelief` fuses estimated base motion,
  measured contact, terrain height, freshness, and uncertainty. A receding-
  horizon `TerrainPlanner` jointly chooses multiple future footholds,
  body/CoM trajectory, touchdown/contact timing, and swing duration. Its first
  implementation remains inside running-trot topology; topology switching is
  not authorized.
- One owner publishes the complete result as an immutable, time-indexed
  `TerrainExecutionState`. Gait, SRBD-MPC, and ID-WBC consume the same atomic
  version. Planned contact and measured force-supported contact remain
  separate.
- Stage D whole-body NMPC is conditional. Consider it only after controlled
  evidence shows that a coherent Stage C plan is feasible but SRBD is the
  repeated bottleneck. Learning is later and separately evidenced.

There is no active Stage A. B0, B1, B2, and B3 are acceptance milestones, not
architecture stages. Archived Stage-C documents and code describe rejected or
superseded routes and may not define or seed this implementation.

## Ordered plan

1. B0 closeout is complete at `a5e8a77`; preserve its frozen contract and
   evidence while beginning B1.
2. Freeze the Stage C schemas, estimator inputs,
   optimization variables, hard constraints, objective ordering, deadlines,
   snapshot validity, and fallback semantics. Add unit tests with actuation
   disabled.
3. Implement `TerrainBelief` and the joint receding-horizon planner in
   deterministic shadow/replay mode. Require stable plan hashes for identical
   inputs, complete provenance, and measured solver budgets.
4. Publish `TerrainExecutionState` atomically and connect gait, SRBD-MPC, and
   ID-WBC as shadow consumers. Prove that no mixed plan/body/contact/timing
   versions occur and that command/torque output remains identical to Phase 1.
5. After shadow gates pass, enable only the smallest B1 dynamic execution
   slice. Each development loop is one hypothesis, one clean commit, focused
   tests, one representative B0 development regression, and one B1 canary.
   Formal completion requires a fresh full frozen B0 followed by the frozen B1
   holdout on the exact clean candidate SHA.
6. Advance without scene-specific changes or widened gates: B2 10 cm, then B3
   mixed/repeated terrain with multiple map/plan epochs and fault injection.
7. Only after B3, extend the same architecture to realistic continuous stair
   geometry, descent, longer-horizon mapping, hardware limits, and hardware
   safety. Apply the Stage D evidence gate before any NMPC replacement.

## Invariants

Running-trot and normal two-contact diagonal support remain valid. The Phase-1
shaper is the only horizontal velocity authority; a planner may only submit a
bounded request through it. Controller and planner inputs are estimated state,
measured contact, and lidar-derived terrain only.

Do not implement or revive quasi-static/scripted crawl, low stance, fixed leg
order, a three-contact entry/preload gate, stop-to-arm or cap-to-zero transfer,
consumer-local timing/contact/recovery state, or local swing retiming. Do not
copy removed code or archived Stage-C code. Invalid, stale, incomplete, or
internally inconsistent snapshots fail closed.

## Work and acceptance

Use one hypothesis and one clean commit. Stop at the first information-bearing
failure; three failed probes at the same blocker require architecture review.
Hold `/tmp/go2_mujoco_experiment.lock` for every timed simulation. Dirty runs,
builds, CTest, videos, lifecycle fields, or another profile's result never
establish acceptance.

Build with `cmake -S example/cpp -B example/cpp/build` and
`cmake --build example/cpp/build -j2`, then run
`ctest --test-dir example/cpp/build --output-on-failure`.

Canonical B0 development commands are each run under the experiment lock:

```bash
flock /tmp/go2_mujoco_experiment.lock bash example/cpp/scripts/run_phase2_b0_pair.sh <profile> development 0
flock /tmp/go2_mujoco_experiment.lock bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0
```

DDS domains come only from the holdout manifest. Acceptance rules and thresholds
remain frozen in `docs/research/PHASE2_ACCEPTANCE.md`.

## Authority

1. This file.
2. `AGENTS.md` and `docs/research/PHASE2_ACCEPTANCE.md`.
3. `docs/research/PHASE2_HOLDOUT_MANIFEST.json`.
4. Raw evidence and target-specific analyzers.

Anything else is implementation or history and cannot change this plan.
