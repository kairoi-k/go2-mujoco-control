# Go2 Phase 2 current

Updated: 2026-09-06. This is the only route, status, plan, and handoff
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
- H1 time-indexed plan-consumer development result is not accepted. Clean SHA
  `91656557596d2e01950db17354bfeae38d079e3c` aligned gait, MPC, WBC, and
  diagnostics to the immutable plan's absolute-time knot while preserving
  planner-only terrain touchdown execution. The correct-scene no-debug canary
  prepared 70 planner targets and ran to 98.09 s, but retained 28,126 required
  plan-rejection rows, 38,431 execution rows without a WBC plan, 33 collision
  rows, and never cleared the step. Evidence is
  `example/cpp/experiments/_runs/phase2_b1_development_canary_h1_timealigned_20260906_033000`;
  debug attribution is retained at
  `example/cpp/experiments/_runs/phase2_b1_debug_h1_timealigned_reject_20260906_040000`.
- H2 running-trot aerial-knot development result is not accepted. Clean SHA
  `ff48dac98928ef5488a5c2bde3f8bc4b7c588ca1` allows a legitimate zero-contact
  running-trot knot without changing the frozen contract or gait topology. The
  correct-scene no-debug canary retained 2,462 required-plan rejection rows,
  all `support_infeasible=5`, 140 execution rows without a WBC plan, zero
  collision rows, and stopped at an IK failure before clearing the step. The
  evidence is
  `example/cpp/experiments/_runs/phase2_b1_development_canary_h2_aerial_20260906_060000`;
  this is a development result only and B1 remains unaccepted.
- H3 support-rejection witness diagnostics are retained at clean SHA
  `c251f00a3b35e39eabac838e09783cfc9cddf8ce`. The passive planner witness
  shows the dominant rejection at knot 0 with measured support mask 9
  (FR+RL, 2,673 rows) and support margin down to -0.06246 m; mask 6
  (FL+RR) accounts for 644 rows. The debug canary retained 1,577 required
  rejection rows, 748 execution rows without WBC plan, and 169 collision
  rows. Its nested-path evidence is
  `example/cpp/experiments/_runs/example/cpp/experiments/_runs/phase2_b1_debug_h3_support_20260906_013500`;
  debug timing makes it attribution-only, not B1 evidence.
- H4 measured-support-anchor development result is not accepted. Clean SHA
  `ae1f4730aaaddbaf00adc27e1dc29f7e41533a0f` connected the gait's
  schedule-based support anchor directly to planner support validation, but
  the correct-scene no-debug canary retained 10,468 required-plan rejection
  rows, 4,956 execution rows without a WBC plan, 22 collision rows, and
  stopped after 33.1 s on cycle-quality rejection. Its support witness was
  dominated by masks 9/6 with margin down to -0.10501 m. Evidence is
  `example/cpp/experiments/_runs/phase2_b1_development_canary_h4_support_anchor_20260906_020000`;
  this hypothesis is retired; B1 remains unaccepted.
- H5's force-backed measured-contact anchor path is not yet eligible for B1:
  the clean exact SHA `5bc8c73cb4cc96370de0e2bfc34535785e59870b` failed the
  representative B0 varying regression. Terrain analyzer strict PASS became
  false with `id_wbc_ok_fraction=0.99980678`; the retained run otherwise
  passed the main quantitative checks. Evidence is
  `example/cpp/experiments/_runs/phase2_b0_development_varying_r0_20260906_020741_terrain`;
  this is a Phase-1/B0 protection failure; boundary fix
  `324bd5d6f36d856f4bd5f9f75390855fb9abb251` restored B0 varying PASS.
- H5 force-backed B1 canary is not accepted at clean SHA
  `324bd5d6f36d856f4bd5f9f75390855fb9abb251`. Planner replacement was
  proven (`clean_source=true`, `target_prepared=54`, 198 execution plan
  IDs, `b3_plan_replacement=true`), but the canary retained 541
  required-plan rejection rows and 220 execution rows without WBC plan;
  it stopped at 18.83 s on cycle-quality rejection. The first support
  witness was knot 0 mask 9 at margin 0.014376 m, below the frozen 0.015 m.
  Evidence is `example/cpp/experiments/_runs/phase2_b1_development_canary_h5_force_anchor_20260906_023000`; B1 remains unaccepted.
- H6 planner-to-MPC absolute-time consistency is implemented at clean SHA
  1f6adc1723b744265e74af17318dad4ee6f2ede9. The new absolute-time lookup
  prevents last-knot clamping, maps MPC samples by terrain_now_s + k*mpc_dt,
  and reports uncovered horizon samples. Targeted tests cover delayed
  consumption, 20/30-ms mismatch, boundaries, and overrun; focused CTest is
  4/4 PASS and real_trot_go2 builds. No support-margin, contact, COM,
  parameter, threshold, or acceptance-semantic change was made. The complete
  fixed-3-m/s B0 development pair passed at
  example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260906_123514_baseline
  and
  example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260906_123514_terrain.
  The varying pair was not counted because baseline domain 220 aborted before
  DDS startup twice; both failure directories remain. The same-SHA B1 canary
  example/cpp/experiments/_runs/phase2_b1_development_canary_timeindex_20260906_124300
  failed frozen B1 with 1,434 required rejections, 811 execution rows without
  a WBC plan, 84 collision rows, coherence 0.3358, and cycle-quality stop.
  The new horizon field recorded 2,104 in-range and 8,958 out-of-range rows.
  B1 remains unaccepted.
- H7 offline/shadow planner-execution consistency repair is recorded at source SHA
  338fe1ceee0c7b5c094c79081fe9493334db56dc; the old-version witness at
  ff937daf1bd768225066406940a6c46cec30a256 failed because legacy horizon
  overrun fabricated a valid current foot. Contract tests at
  8c02c2a708218af56261da65986df905e7f8d9fa cover absolute-time horizon
  bounds, delayed consumption, 20/30 ms
  mismatch, touchdown commitment lifetime, model-COM provenance, and separate
  measured, planned, and applied contacts. Focused CTest is 5/5 PASS and
  real_trot_go2 builds. The new path is env-gated by
  TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW and defaults off; it changes no
  commands and no B1 run was made.
- Read-only flat-reference support analysis is retained at
  example/cpp/experiments/_runs/phase2_h7_offline_support_margin_20260906_338fe1c;
  with the unchanged 15 mm criterion, 12,005/18,615 base-point rows passed
  and 12,803/18,615 model-COM rows passed. The exact four-contact subset passed
  3,072/3,072 for both. Two-contact rows are dynamic diagnostics, not proof of
  stable-state safety; H5 selected foothold coordinates remain missing.
- H8 offline replay plus Atlas shadow validation is complete at clean source
  d550fb36aaee877b31f451c72fe6beec3d8b5fd2. The replay checker is
  example/cpp/tools/analysis/replay_terrain_execution_consistency.py; old H5
  CSVs are explicitly rejected for missing model-COM/shadow provenance.
  Focused consistency CTest is PASS and real_trot_go2 builds. The shadow path
  is independent of terrain_actuation, defaults off, and records same-tick
  model COM, absolute horizon coverage, per-leg touchdown events, source
  plan/epoch, target time/xyz, and commitment inheritance. Shadow-only plans
  historical d550 evidence used 16 knots/0.30 s; e6253fd uses 24 real knots/0.46 s to cover observed delay; no terminal-knot copy or expired-plan extension is allowed. Verification requires: shadow-off
  is unchecked (never pass); shadow-on normal rows have a real plan, same-tick
  model COM, full absolute MPC coverage, event/target-consistent commitments,
  and a valid snapshot; missing plan, COM, coverage, expiry, or target mismatch
  is rejected.
- Final Atlas flat shadow-on evidence is
  example/cpp/experiments/_runs/phase2_h8_final_flat_shadow_on_d550fb3:
  6,653 rows, 2,103 checked, 1,820 complete snapshots, 283 explicit rejects,
  6,653/6,653 same-tick COM rows. The safe 5 cm approach evidence is
  example/cpp/experiments/_runs/phase2_h8_final_step5cm_shadow_on_d550fb3:
  4,443 rows, 2,296 checked, 1,890 complete snapshots, 406 explicit rejects,
  4,443/4,443 same-tick COM rows. Both are diagnostic only; terrain actuation
  stayed off and neither is B1 acceptance.
- The lockstep off/on command comparison is not counted as invariant proof:
  repeated off runs already showed large wall-clock/start-state variation, and
  the paired runs began at different first state ticks. This is a remaining
  provenance limitation, not evidence that shadow outputs were consumed.
  The representative fixed-3-m/s B0 pair at the same exact source passed B0
  acceptance at
  example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r1_20260906_153740_baseline
  and
  example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r1_20260906_153740_terrain.
  B1 remains unaccepted; no threshold, contract, or acceptance-semantic change
  was made.

- H8 rejection decomposition and classification follow-up is at clean source
  e6253fd0940724042d682fccc35b7cbaf1d89774. The new readiness classifier
  separates invalid plan, expired plan, and horizon coverage; focused
  consistency CTest and full 33/33 CTest pass, and real_trot_go2 builds.
  Fresh terrain-sensor-only shadow runs are
  [flat](example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_flat_lockstep)
  (9,323 rows, 967 checked/valid, code4=0) and
  [5 cm](example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_step5cm_lockstep)
  (9,418 rows, 990 checked/valid, code4=0); same-tick model-COM mismatches
  are zero. The non-lockstep startup attempt `phase2_h8_shadow_fixed_e6253fd_flat` has
  completion_status=1 and is retained but excluded from metrics. In the stable
  locomotion interval, flat is 6,171 rows with
  441/441 checked valid, 0 checked invalid, and 5,730 unchecked; 5 cm is
  6,266 with 389/389, 0, and 5,877. Remaining stable fail-closed reasons
  are plan-invalid/no-plan 5,629/5,776, horizon 86/87, and target/event
  mismatch 15/14. The mismatch leg masks are distributed rather than one
  fixed leg (flat 15:6, 6:3, 7/9/14/1:2 each; 5 cm 15:4, 6/9:3/3,
  14/2/13/4/8/1:1/1/1/1/2/1). The historical d550 code4 rows were
  flat 283 (stage0=118, stage2=165) and 5 cm 406 (stage0=124, stage2=282);
  inherited-mask was zero in all, with commitment-valid/in-flight leg
  counts. Legacy code4 time bins (0.5 s) were flat 2.0/2.5/4.5/5.0 =
  113/5/16/149 and 5 cm 2.0/2.5/4.5/5.0/5.5 = 119/5/70/111/101.
  Phase-decile counts were flat 126/24/26/23/18/23/12/12/0/13/6 and
  5 cm 139/15/19/30/49/37/34/11/24/30/18; plan=epoch sets were flat
  6:118,48:16,51:39,55:110 and 5 cm 6:124,48:42,51:15,53:13,55:111,66:101.
  counts flat FR/FL/RR/RL=38/55/55/38 and 5 cm=27/114/114/27. This removes
  the checked-snapshot rejection, but the stable shadow gate remains
  incomplete because real plans are absent or horizon-uncovered for most
  rows; it is not an acceptance PASS.
- The e6253fd fixed-3-m/s B0 pair passed with exact clean manifests at
  [baseline](example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r2_20260906_171729_baseline)
  and
  [terrain](example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r2_20260906_171729_terrain).
  Shadow-off/on command invariance remains unproven: no same-input
  LowState/HighState deterministic replay currently exists. Minimum
  infrastructure is a recorder/replayer feeding identical state ticks into
  command generation and capturing per-tick LowCmd, torque, and joint
  targets; no control mathematics was changed for this proof. No terrain
  actuation or B1 canary/holdout was run.

- The B1 planner-execution consistency audit is diagnostic and blocked, not
  an acceptance attempt. No new Atlas run was created. At the first retained
  H5 support rejection (state_tick_s=2.122), planner plan 8 was rejected
  with failure 5, knot 0, planned mask 9, and margin 0.014376197 m, while
  execution still consumed usable plan 7. The same row reports WBC measured
  mask 15 versus terrain planned mask 6; the paired contact record at 2.122 s
  has nonzero force on all four feet. Plan 7 becomes execution id 0 at
  2.170 s, confirming the rejection-to-expiry chain. The source audit also
  confirms that support validation uses base_position_world while MPC uses
  dyn.com_world; planned contact drives the polygon mask while measured
  contact only seeds touchdown detection; candidate legs are selected
  independently with one final support check; planner knots are 20 ms while
  MPC uses 30 ms at a 0.24 s gait period but previously indexed terrain_k0+k; H6 now maps by absolute time and reports coverage; and
  gait latches a per-leg target plan id that MPC does not cross-check against
  its latest plan. The H5 CSV lacks the selected foothold coordinates needed
  to decompose 0.014376197 m into line-distance versus endpoint margin, so it
  is not a dynamic-stability measurement. Focused CTest remained 4/4 PASS;
  B1 stays unaccepted pending architecture-level consistency repair.

- H9 H8 stable no-plan lifecycle attribution is complete from the clean
  analysis head 39c6fbf12858088d7f496cf05b72c1820f23c10e, using the two
  terrain-sensor-only shadow manifests at clean source
  e6253fd0940724042d682fccc35b7cbaf1d89774:
  [flat](example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_flat_lockstep)
  and
  [5 cm](example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_step5cm_lockstep).
  Both manifests are clean, complete, lockstep shadow runs with terrain
  actuation off; no new run or control-code change was made.
- In stable locomotion, all 5,629 flat and 5,776 5-cm rows classified as
  shadow failure reason 1 were true no-plan rows: the consumer plan pointer
  was null, and the recorded plan state/valid-until fields were missing
  (zero), never a valid plan rejected by the shadow checker. Every such row
  had terrain_map_valid=1, terrain_model_com_valid=1, terrain_map_source=lidar,
  terrain_plan_status=rejected, terrain_planner_deadline_misses=0, and
  terrain_plan_contact_rejections=0. The terrain planner update counter
  continued through stable motion (181 flat and 183 5-cm update events; event
  gaps 0.002--0.102 s), so no-opportunity, missing TerrainModel/COM input,
  and deadline are not supported as the direct cause.
- The direct consumer lifecycle is expiry: flat has a 27-row initial gap
  after the sixth publication, then the last valid plan 56 was generated at
  4.986 s with valid_until=5.446 s; no-plan starts at 5.448 s while the
  publication counter remains 17 and consumption remains 1,194. The 5-cm
  run has a 29-row initial gap, then plan 47 at 4.650 s expires at 5.110 s;
  no-plan starts at 5.112 s while publication remains 15 and consumption
  remains 1,237. No separate arbitration-drop field exists; the observable
  store path publishes only valid plans and retains the latest one until
  expiry.
- The upstream cause is planner infeasibility, not publication scheduling:
  the latest planner snapshot is rejected for every stable no-plan row.
  Failure 5 (support-infeasible) accounts for flat 4,105/5,629 (72.93%) and
  5-cm 4,209/5,776 (72.87%); failure 4 (no-safe-foothold) accounts for
  1,524/5,629 (27.07%) and 1,567/5,776 (27.13%). Stable planner events
  independently show flat 128 support/42 no-safe/11 valid and 5-cm
  130/44/9. Late no-safe rows have zero known cells and zero feasible regions
  (flat 1,449; 5-cm 1,493), but the map/model readiness fields remain valid.
- Candidate-per-leg summaries and support failure knot/contact-mask are
  missing from these H8 CSVs; the row-level cause is therefore covered
  100%, while the exact per-leg candidate witness is not fabricated. Source
  confirms the old planner selects each touchdown independently and performs
  one final whole-plan support check. This is the first information-bearing
  Stage-C boundary: stop repairing the old planner and hand off a joint
  foothold plus COM/body-trajectory planner, with absolute-time coverage,
  measured/planned/applied contact provenance, and a shared execution
  commitment snapshot as interface constraints. No threshold, contract,
  support/COM/contact semantic, analyzer, terrain actuation, or B1
  canary/holdout change was made.

- Stage-C C0-01/C0-02 preparation is implemented on
  `feat/stage-c-joint-planner` at clean source
  `0e09535c43d4fd26be3de387f02752a43115fa0f`. The real repository fixture
  `test_stage_c_foundations` covers T01-T07 and T13-T15: capture-mode/anchor
  equality, 5 cm region degeneration, map-coverage causes, full-frame
  transform, multi-touchdown identity, interval-end coverage, fixed initial
  support conflict, transfer/aerial contract conflict, reference-span
  semantics, and committed-prefix preservation. Focused CTest is 1/1 PASS.
  No H8/H9 raw CSV was available in the inspected worktrees, so no missing
  foothold coordinates were inferred. The legacy snapshot producer still
  gates force-backed anchor initialization on terrain actuation; the new typed
  adapter is a default-off preparation seam and is not claimed as an
  integration repair. That C0-02 foundation stopped at deterministic discrete joint
  search/oracle, typed event/candidate/failure/rollout/bundle interfaces, and
  test framework; no continuous COM/body/contact-force solver, terrain
  actuation, B1, or threshold/analyzer change was made. Evidence is
  `docs/research/evidence/stage_c_c0_foundations/README.md`. The full build
  is blocked by the absent `simulate/mujoco/include/mujoco/mujoco.h`; all 30
  available registered CTest targets pass, while the MuJoCo-dependent
  integration target is not counted.


- Stage C C0-02 continuous dynamics core is implemented offline on
  `feat/stage-c-joint-planner`, based on fetched `57dbd790de23107c870c5135e0fb18a8e57198b9`.
  The containing commit and source hashes in
  `docs/research/evidence/stage_c_continuous_core/manifest.json` identify the
  tested source. It provides fixed-event COM/velocity/angular-momentum and
  contact-force trajectories, exact constant-force centroidal integration,
  deterministic SCP using existing DenseQP, independent original-model
  verification, and sound separating infeasibility witnesses. Focused tests
  pass 4/4, the core has 93 checks and external synthetic oracles pass 9/9.
  This is a reduced dynamics certificate: no body pose, full geometry, torque,
  execution or B0/B1 acceptance. The 15 mm geometric diagnostic and T13 aerial
  conflict remain separate and unchanged. Evidence, formulation, latency,
  audit corrections and limitations are in the packet README. It is ready
  to begin C0-03 offline comparison, not to connect controller consumers.
  Missing MuJoCo still blocks the full build/integration target. No new
  terrain actuation, B0/B1 or threshold/analyzer/old-planner change occurred.

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
   holdout on the exact clean candidate SHA. H1 is the first time-alignment
   probe; its failure evidence remains retained and does not authorize the
   formal sequence.
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
