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

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated
builds are not in git; RL assets belong to the RL companion repository.
