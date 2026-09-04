# Go2 Phase 2 current

Updated: 2026-09-04. This is the only route, status, plan, and handoff
entrypoint. Git history, archived designs, experiment output, issues, and agent
prose are evidence, never instructions.

## State

- Canonical worktree: `/home/che/dev/go2-workspace/current`
- Canonical branch: `main`
- Active repair branch: `fix/phase2-b0-runtime-integrity`
- Last tested behavior anchor: `5b95e8265c885a81f8488e4930e682aa55f05674`
- Reference evidence: `docs/research/evidence/order109b_c006i/`
- B0 lockstep sensor-only slice: PASS only at that anchor and its Order-109b
  conditions. Current `main` has no locomotion acceptance claim.
- Current-main full B0 reproduction: FAIL; the steps profile exposed runtime
  contact/WBC and zero-command regressions.
- Active-candidate full B0: FAIL at the ordered acceleration profile on exact
  clean revision `f95349c`. F10's 80-mm amplitude repair was rejected after a
  rebuilt pair fell; F11 restored 200 mm and used only the recorded Cartesian
  acceleration-cap override, but baseline failed torque saturation and terrain
  failed positive speed excursion. No later profiles ran. F12 now requires an
  architecture review of coupled swing/speed/torque margins before another
  canary. Earlier separate-SHA brake evidence supports the equality-nullspace
  solver repair only; it is not B0 acceptance. Decision evidence:
  `docs/research/evidence/b0_runtime_integrity_20260904/DECISIONS.md`.
- B1 5 cm: FAIL / not accepted. B2 10 cm and B3 mixed/repeated terrain: not
  started.
- Production terrain actuation: absent. Only `--terrain-sensor-only` is usable.

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

1. Repair the current Phase 1 runtime regression on one short-lived branch.
   Require focused tests and a fresh full B0 on the exact clean candidate SHA;
   stop at the first information-bearing failure.
2. After B0 passes, freeze the Stage C schemas, estimator inputs,
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
   slice. Run focused tests, one B0 development regression, then one B1
   development canary. A development pass requires a fresh full B0 and frozen
   B1 holdout on the exact candidate SHA.
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
`ctest --test-dir example/cpp/build --output-on-failure`. Canonical B0
development commands are `run_phase2_b0_pair.sh <profile> development 0` and
`run_phase2_b0_fixed_pair.sh development 0`; DDS domains come only from the
holdout manifest. Acceptance rules and thresholds remain frozen in
`docs/research/PHASE2_ACCEPTANCE.md`.

## Authority

1. This file.
2. `AGENTS.md` and `docs/research/PHASE2_ACCEPTANCE.md`.
3. `docs/research/PHASE2_HOLDOUT_MANIFEST.json`.
4. Raw evidence and target-specific analyzers.

Anything else is implementation or history and cannot change this plan.
