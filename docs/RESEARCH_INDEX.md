# Research index

This repository records **model-based Go2 control in MuJoCo**. Isaac Lab velocity RL and Kine2Go imitation are maintained in separate companion repositories.

This index records accepted Phase 1 and explicitly scoped Phase 2 claims. It
does not define the current Phase 2 route; use [`CURRENT.md`](../CURRENT.md).

## Milestone: Phase 1 arbitrary velocity (2026-08-25)

Accepted at exact SHA `6e34f99` (`milestone/phase1-arbitrary-velocity-2026-08-25`).
Five frozen profiles—steps, acceleration, braking, ramp, and varying
commands—had three valid runs each: 15/15 passed both the legacy and
quantitative gates. The varying profile includes 0.6, 1.4, 2.3, and 2.8 m/s.
See [the quantitative acceptance](validation/PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md)
and [the closeout](validation/PHASE1_RUNTIME_VELOCITY_CLOSEOUT_2026-08-25.md).
This is the pre-Phase 2 dynamic baseline; current Phase 2 status is in
[`CURRENT.md`](../CURRENT.md).

## Milestone: Phase 2 terrain-sensor-only varying non-regression (2026-08-28)

Accepted as historical non-regression evidence at exact SHA `70b7740`
(`milestone/phase2-terrain-sensor-varying-2026-08-28`) on
`phase2-b1-b3`. With `terrain_lidar=true` and `--terrain-sensor-only`, three
independent varying runs passed both the frozen quantitative and strict gates.
This is sensor-only terrain observation: it is not terrain actuation, obstacle
crossing, or current full-B0 acceptance. See [the evidence summary](research/evidence/phase2_terrain_sensor_velocity_20260828/SUMMARY.md).
Later terrain ownership and runtime changes mean this historical passing path
must not be assumed identical to the current head. Its bundle identity and raw
manifest hashes are recorded in [the evidence manifest](research/evidence/phase2_terrain_sensor_velocity_20260828/MANIFEST.json).

The raw 8/28 campaign was broader than that compact accepted subset: it also
contains steps, acceleration, braking, and ramp runs. The per-profile PASS/FAIL
and retry matrix is indexed in [the campaign record](research/evidence/phase2_terrain_sensor_velocity_20260828/ALL_PROFILES.md);
mixed repeats are not promoted to a single all-green B0 claim.

## Historical lockstep sensor-only slice (2026-09-01)

Order-109b passed a separate fixed-3 m/s lockstep sensor-only slice at exact
source `5b95e826`. This is a historical timing/equivalence slice, not the
current full wall-clock B0 result, terrain actuation, or a B1 authorization. See
[the scoped evidence summary](research/evidence/order109b_c006i/SUMMARY.md).

## Milestone: Frozen B0 wall-clock closeout (2026-09-05)

Accepted at exact final evidence SHA
`a5e8a77e3200e8c246ef25b31abfb1cd0f6e73fd`. The complete frozen B0
development campaign passed steps, acceleration, braking, ramp, varying, and
fixed-3-m/s terrain/baseline gates with terrain actuation disabled. The retained
wall-clock causal investigation identified the terrain worker's default CPU pin
as the first cause; the minimum production fix is exact SHA
`e457bd2b661d01c8c033271f31b9252854781b9c`. See
[`WALLCLOCK_FIRST_DIVERGENCE.md`](../WALLCLOCK_FIRST_DIVERGENCE.md) for raw run
directories, manifests, analyzers, and the non-B1 boundary. B1 5 cm remains
unaccepted.

## Milestone: B1 H1 absolute-time plan-consumer development probe (2026-09-06)

Not accepted at clean SHA
`91656557596d2e01950db17354bfeae38d079e3c` (implementation
`1e42d4c`). The correct-scene no-debug running-trot canary prepared 70
planner-selected targets and ran to 98.09 s, but retained 28,126 required-plan
rejection rows, 38,431 execution rows without a WBC plan, 33 collision rows,
and never cleared the 5 cm step. The retained debug run identified
`unknown[path]` candidate rejection. This evidence proves planner targets
entered the execution chain; it does not establish B1 acceptance.

## B1 H2 aerial-knot running-trot development probe (2026-09-06)

Not accepted at clean SHA
`ff48dac98928ef5488a5c2bde3f8bc4b7c588ca1`. The planner now permits the
legitimate zero-contact aerial knots present in the frozen running-trot
schedule, while retaining the frozen support margin for every nonzero-contact
knot. The correct-scene no-debug canary retained 2,462 required-plan rejection
rows, all `support_infeasible=5`, 140 execution rows without a WBC plan, zero
collision rows, and stopped on IK before clearing the 5 cm step. Evidence:
[`phase2_b1_development_canary_h2_aerial_20260906_060000`](../example/cpp/experiments/_runs/phase2_b1_development_canary_h2_aerial_20260906_060000).
This does not establish B1 acceptance.

## B1 H3 support-rejection witness diagnostic (2026-09-06)

Not accepted at clean SHA
`c251f00a3b35e39eabac838e09783cfc9cddf8ce`. Passive planner witness fields
showed the dominant required-plan rejection at knot 0 with measured support mask
9 (FR+RL, 2,673 rows), with mask 6 (FL+RR) in 644 rows and support margin
down to -0.06246 m. The debug canary retained 1,577 required rejection rows,
748 execution rows without WBC plan, and 169 collisions. Evidence:
[`phase2_b1_debug_h3_support_20260906_013500`](../example/cpp/experiments/_runs/example/cpp/experiments/_runs/phase2_b1_debug_h3_support_20260906_013500).
Because debug timing is perturbed, this is attribution-only and does not
establish B1 acceptance.
## B1 H4 schedule-anchor planner probe (2026-09-06)

Not accepted at clean SHA
`ae1f4730aaaddbaf00adc27e1dc29f7e41533a0f`. The correct-scene no-debug
canary retained 10,468 required-plan rejection rows, 4,956 execution rows
without a WBC plan, 22 ground-truth collision rows, and stopped after 33.1 s
on cycle-quality rejection. Support witness masks 9/6 dominated, with margin
down to -0.10501 m. Evidence:
[`phase2_b1_development_canary_h4_support_anchor_20260906_020000`](../example/cpp/experiments/_runs/phase2_b1_development_canary_h4_support_anchor_20260906_020000).
This schedule-based anchor hypothesis is retired and does not establish B1
acceptance.
## H5 force-backed support-anchor B0 regression (2026-09-06)

Not accepted at clean SHA
`5bc8c73cb4cc96370de0e2bfc34535785e59870b`. The representative B0 varying
terrain member failed strict quantitative acceptance with
`id_wbc_ok_fraction=0.99980678`; the retained evidence is
[`phase2_b0_development_varying_r0_20260906_020741_terrain`](../example/cpp/experiments/_runs/phase2_b0_development_varying_r0_20260906_020741_terrain).
No B1 canary was run because the B0/Phase-1 boundary must be restored first.
## B1 H5 force-backed support-anchor canary (2026-09-06)

Not accepted at clean SHA
`324bd5d6f36d856f4bd5f9f75390855fb9abb251`. Planner replacement was proven
(`target_prepared=54`, 198 execution plan IDs, `b3_plan_replacement=true`);
the correct-scene no-debug canary retained 541 required-plan rejection rows,
220 execution rows without WBC plan, zero collisions, and stopped at 18.83 s
on cycle-quality rejection. First support witness was knot 0 mask 9 at
0.014376 m, below the frozen 0.015 m. Evidence:
[`phase2_b1_development_canary_h5_force_anchor_20260906_023000`](../example/cpp/experiments/_runs/phase2_b1_development_canary_h5_force_anchor_20260906_023000).

## B1 H6 planner-to-MPC absolute-time horizon consistency (2026-09-06)

Not accepted at clean SHA
1f6adc1723b744265e74af17318dad4ee6f2ede9. The fix adds non-clamping
absolute-time planner-knot lookup, maps every MPC horizon sample by
terrain_now_s + k*mpc_dt, and emits explicit out-of-coverage telemetry.
Targeted focused CTest passed 4/4, and the production controller built. The
complete fixed-3-m/s B0 development pair passed in the retained baseline and
terrain run directories. The same-SHA B1 development canary retained 1,434
required-plan rejection rows, 811 execution rows without a WBC plan, 84
collisions, WBC coherence 0.3358, and a cycle-quality stop. No support-margin,
contact, COM, parameter, threshold, or acceptance-semantic change was made;
B1 remains unaccepted.

## B1 H7 planner-execution consistency offline/shadow repair (2026-09-06)

Diagnostic only; not accepted at source SHA 338fe1ceee0c7b5c094c79081fe9493334db56dc.
The old-version targeted witness at ff937da failed because horizon overrun
fabricated a valid current foot; contract tests begin at 8c02c2a.
The integration at 338fe1ce now covers absolute-time horizon bounds, delayed
consumption, 20/30 ms mismatch, touchdown commitment lifetime, model-COM
provenance, and separate measured/planned/applied contacts. Focused CTest is
5/5 PASS and real_trot_go2 builds. The new path is guarded by
TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW and defaults off; no B1 run was made.

Read-only flat evidence is phase2_h7_offline_support_margin_20260906_338fe1c
under example/cpp/experiments/_runs. With the unchanged 15 mm criterion,
12,005/18,615 base-point rows and 12,803/18,615 model-COM rows passed; the
exact four-contact subset passed 3,072/3,072 for both. Two-contact running
rows are dynamic diagnostics, not proof of stable-state safety. H5 selected
foothold coordinates are missing; remaining gaps are the complete worker COM
snapshot, full Stage-C joint planning, and Atlas shadow validation; B1 remains
unaccepted.

**Stand-walk-lie sequencing + `--wbc-full` Raibert trot**, 2026-08-18 (`2b82dae`).

| Item | Record |
|---|---|
| Interface | 500 Hz `LowCmd` / `LowState` over Unitree SDK2 DDS |
| Task | `go2sim task`: stand-up 3 s → settle 0.5 s → `--wbc-full` trot → 2 s blend to stand → lie-down 3 s (`--tau-limit 35`) |
| Transitions | Smoothstep interpolation of 12 joint targets; gait amplitude ramps over 0.8 s from the stand pose |
| Reliable cruise | `--wbc-full` 64-cycle n=5: **0.130 ± 0.011 m/s** (0.116–0.147); stand-walk-lie 8 s n=3: **0.139 ± 0.004 m/s**. Target `0.091/0.60 = 0.151667`. Evidence: [`example/cpp/experiments/go2_wbc_full_mainline_repeat_2026-08-18`](../example/cpp/experiments/go2_wbc_full_mainline_repeat_2026-08-18/README.md) |
| Beyond that | `go2sim fast` (~0.18 m/s, `--tau-limit 19`) and 0.21 m/s are `--wbc-primary` history; 0.21 was rejected. `go2sim walk` (`--wbc-primary`) failed the 2026-08-15 same-gate session (`q_error=0.296`) and is not claimed on this tree |
| WBC | `--wbc-full`: 18-DoF ID-WBC + SRBD MPC. `go2sim task` and `go2sim full` both turn it on. A 2026-08-15 single run hit 0.149 m/s (ratio 0.985); that is historical, not the n=5 number. See `docs/WBC_MPC.md` |
| Clip | `docs/media/stand_walk_lie_wbcfull.gif` / `.mp4`. The older `stand_walk_lie.gif` is the `--wbc-primary` homepage clip and was kept |

## Validated high-speed simulation profile

**Source:** `66dc3e810dcf8766e4e2fd838e14fb772805c76d` (`gait/sustained-sprint-running-2026-08-21`), exact-head revalidated on 2026-08-24 in an isolated clean checkout.

The separate `running-trot` wall-clock profile passed the strict analyzer in all three independent 40 s repeats. The analyzer requires a continuous 20 s window at 2.90–3.80 m/s, bounded attitude, running-gait contact/clearance structure, zero lifecycle/safety/quality/dynamics/contact status failures, and a completed stop hold below 0.10 m/s.

| repeat | median speed | good window | roll/pitch P95 | aerial fraction | minimum pair sync | stop-tail P95 |
|---|---:|---:|---:|---:|---:|---:|
| `codex_reval_3mps_r1` | 3.235254 m/s | 61.352 s | 2.818/2.325° | 0.290171 | 0.812456 | 0.003860 m/s |
| `codex_reval_3mps_r2` | 3.225791 m/s | 61.342 s | 2.971/2.500° | 0.284504 | 0.814020 | 0.003713 m/s |
| `codex_reval_3mps_r3` | 3.225737 m/s | 61.630 s | 3.081/2.562° | 0.289935 | 0.799793 | 0.003905 m/s |

This is a model-based MuJoCo simulation result for running-trot. It is distinct from the diagonal-trot sprint wording and does not establish natural-animal gait, hardware performance, or sim-to-real transfer. The compact provenance record is [`docs/validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`](validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md); raw `_runs/` remain ignored.

**Supported claim:** the sequenced `--wbc-full` task, the historical slow trot baseline, and the separately validated 3 m/s-class running-trot profile in this simulator stack.

**Outside the C++ claim:** natural animal gait, sim-to-real, command-conditioned imitation, A→B world goals, or a reproduced `--wbc-primary` homepage walk.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Companion research tracks

Isaac Lab / RSL-RL velocity curricula, their environment snapshot, clips, and
checkpoint record live in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).
Motion imitation, AMP, and seam JSON live in
[`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## B1 planner-execution consistency audit (2026-09-06)

Diagnostic and blocked at clean source a0cba291e0334f986b4a7512c26e8591377e13d0;
no new Atlas run was created. The retained H5 first rejection at
state_tick_s=2.122 rejected planner plan 8 at knot 0 with planned contact
mask 9 and margin 0.014376197 m, while usable execution plan 7 remained
active. At 2.170 s the execution plan became empty. The same row reports WBC
measured mask 15 versus planned mask 6; the paired contact record reports
nonzero force on all four feet. Source review independently confirms
base-origin versus dynamic-COM, planned-versus-measured contact, per-leg
candidate selection followed by one support check, 20 ms planner versus
30 ms MPC horizon indexing at a 0.24 s gait period, and a latched per-leg
target plan id not cross-checked by MPC. The H5 CSV does not contain the
selected foothold coordinates needed to split the support margin into
line-distance and endpoint components. Focused CTest is 4/4 PASS, but B1
remains unaccepted and no threshold or acceptance semantic change is
authorized.

## H8 offline replay and Atlas shadow validation (2026-09-06)

Diagnostic complete at clean SHA d550fb36aaee877b31f451c72fe6beec3d8b5fd2;
B1 remains unaccepted. The offline replay checker explicitly rejects legacy
H5/H6 CSVs missing same-tick model-COM and shadow provenance. Final flat
terrain-sensor-only shadow evidence is
[phase2_h8_final_flat_shadow_on_d550fb3](../example/cpp/experiments/_runs/phase2_h8_final_flat_shadow_on_d550fb3):
6,653 rows, 2,103 checked, 1,820 complete snapshots, 283 explicit rejects,
and zero same-tick COM mismatches. The safe 5 cm approach fragment is
[phase2_h8_final_step5cm_shadow_on_d550fb3](../example/cpp/experiments/_runs/phase2_h8_final_step5cm_shadow_on_d550fb3):
4,443 rows, 2,296 checked, 1,890 complete snapshots, 406 explicit rejects,
and zero COM timestamp mismatches. Shadow plans use diagnostic-only extra
absolute-time coverage; terrain actuation stayed off. Off/on command
invariance was not proven because repeated lockstep runs had different first
ticks and baseline output variation. The exact-SHA fixed-3-m/s B0 pair passed;
no B1 holdout or acceptance-semantic change was made.


## H8 shadow rejection decomposition follow-up (2026-09-06)

At clean source `e6253fd0940724042d682fccc35b7cbaf1d89774`, readiness
classification separates invalid plan, expiry, and horizon coverage; focused
and full 33/33 CTest plus `real_trot_go2` passed. Fresh terrain-sensor-only
shadow runs [flat](../example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_flat_lockstep)
and [5 cm](../example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_step5cm_lockstep)
have code4=0 and zero same-tick COM mismatches. Stable intervals still have
unchecked 5,730/5,877 rows and target/event mismatches 15/14, so the shadow
gate is incomplete, not accepted. The historical d550 code4=283/406
decomposition by time, phase, plan/epoch, leg, and commitment state is in
`CURRENT.md`. The exact-source fixed-3-m/s B0 pair passed; same-input
LowState/HighState command-invariance replay infrastructure is not present.
No terrain actuation or B1 canary/holdout was run.

## Stage-C C0-01/C0-02 foundation preparation (2026-09-06)

Diagnostic/unit evidence only at clean source
`0e09535c43d4fd26be3de387f02752a43115fa0f` on
`feat/stage-c-joint-planner`; B0/B1 remain unaccepted. The real repository
fixture `test_stage_c_foundations` passes 1/1 and covers the requested T01-T07
and T13-T15 counterexamples. The new default-off Stage C headers define typed
input, separate contact provenance, explicit touchdown events and candidates,
failure taxonomy, rollout/bundle interfaces, a common comparison seam, and a
deterministic exhaustive discrete planner plus small-fixture oracle. The old
planner was not optimized, no continuous COM/body/force solver or terrain
actuation was added, and frozen thresholds/analyzer semantics were unchanged.
Available CTest is 30/30 PASS; the full build/integration test is blocked by
the absent MuJoCo checkout dependency. See [`C0 foundation evidence`](research/evidence/stage_c_c0_foundations/README.md).

## H9 H8 stable no-plan lifecycle attribution (2026-09-06)

Diagnostic attribution is complete at analysis head
39c6fbf12858088d7f496cf05b72c1820f23c10e, using clean-source
e6253fd0940724042d682fccc35b7cbaf1d89774 manifests
[flat](../example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_flat_lockstep)
and
[5 cm](../example/cpp/experiments/_runs/phase2_h8_shadow_fixed_e6253fd_step5cm_lockstep).
The stable shadow no-plan rows are 5,629/5,629 and 5,776/5,776 direct
consumer misses: the plan pointer is null after store expiry, not a checked
plan rejected by the shadow gate. All rows have ready terrain map/model COM,
rejected latest planner status, zero deadline misses, and continuing planner
updates. The final valid plan expires at 5.446 s (flat, plan 56) and 5.110 s
(5 cm, plan 47), after which publication and consumption plateau at 17/1,194
and 15/1,237. The latest planner rejection is support-infeasible in 4,105
flat and 4,209 5-cm rows, and no-safe-foothold in 1,524 and 1,567 rows:
72.9%/27.1% in both scenes. Per-leg candidate and support-knot/mask fields
are missing. The evidence supports stopping the old per-leg planner and
starting Stage-C joint foothold plus COM/body-trajectory design; no threshold,
contract, semantic, actuation, or B1 change was made.

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated
builds are not in git; RL assets belong to the RL companion repository.
