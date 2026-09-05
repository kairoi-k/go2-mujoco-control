# Research history

Milestone-level research progress.


## Canonical milestone ledger

This is the single chronological ledger for meaningful Go2 research milestones, route decisions, accepted historical results, and explicit non-acceptance. Each row names the exact source identity, the evidence entry, and the boundary of the claim.

`CURRENT.md` is the only authority for the present route, status, and plan. `RESEARCH_INDEX.md` is the compact index of accepted claims. This file is the chronological provenance ledger; it does not replace either authority.

State vocabulary: `accepted` means the named contract passed at the exact source; `historical` means the result is retained but is not the current route; `active` means work is in progress; `not accepted` means evidence or scope is insufficient for a capability claim; `retired` means the route is explicitly closed.

Every accepted result, historical non-regression result, `milestone/*` tag, and retired route has exactly one row in this ledger. A raw PASS with no row is an indexing defect, not a complete milestone.

| Date | Milestone / decision | State | Exact source | Evidence and boundary |
|---|---|---|---|---|
| 2026-08-18 | Modular `--wbc-full` stand/walk/lie baseline | historical result | `wbc-full-stable-2026-08-19` -> `2b82daeb50c522a7b0134119b1a87ef60393ba28` | [`RESEARCH_INDEX.md`](RESEARCH_INDEX.md); MuJoCo low-level sequence, not a speed or natural-gait acceptance. |
| 2026-08-19 to 2026-08-21 | RL and motion-imitation tracks split to companion repositories | historical scope decision | This file, supporting sections 3-4; [`go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl), [`kine2go-research`](https://github.com/kairoi-k/kine2go-research) | Keeps learned velocity and imitation claims outside the C++ Phase 1/2 acceptance line. |
| 2026-08-20 to 2026-08-21 | Environment adaptation and sensing-priority line | historical decision | `environment-adaptation-baseline-2026-08-20` -> `99d6f5b2771b1729ec53d875e0ddc018741d01a8`; `auto-environment-sensing-priority-final-v2-2026-08-21` -> `2636590e6bd960d0501775719b69f0978b58931b` | Sensing/priority behavior only; not terrain traversal. |
| 2026-08-20 | Unified reactive transition matrix | historical coverage | `5135ccafb96a0b838e03c115bebee9d62395eee5` | [`REACTIVE_ENVIRONMENT_ADAPTATION.md`](REACTIVE_ENVIRONMENT_ADAPTATION.md); 49 scripted transitions on one plant, not autonomous perception. |
| 2026-08-21 | Automatic height-map obstacle/impact sensing and priority preemption | accepted historical | `auto-environment-sensing-final-2026-08-21` -> `fd666467228a9fd8dab1a34f8f550cb214976ab1`; `auto-environment-sensing-priority-final-2026-08-21` -> `2cdb27bbe118a0579ff9827e2c960ba33364dbc7`; `auto-environment-sensing-priority-final-v2-2026-08-21` -> `2636590e6bd960d0501775719b69f0978b58931b` | [`AUTO_ENVIRONMENT_DELIVERY_2026-08-21.md`](AUTO_ENVIRONMENT_DELIVERY_2026-08-21.md) and [`AUTO_PRIORITY_PREEMPTION_ACCEPTANCE_2026-08-21.md`](AUTO_PRIORITY_PREEMPTION_ACCEPTANCE_2026-08-21.md); MuJoCo sensor-driven safety ordering, not hardware or terrain locomotion. |
| 2026-08-21 | Physical low-friction patch sensor acceptance | accepted historical | `auto-environment-sensing-physical-patch-final2-2026-08-21` -> `97b6b0a0abd36202171cff2f98ec1df69731860c` | [`AUTO_ENVIRONMENT_DELIVERY_2026-08-21.md`](AUTO_ENVIRONMENT_DELIVERY_2026-08-21.md); collidable `mu=0.0001` patch passed twice, while friction-only/no-observable-slip remains a negative boundary. |
| 2026-08-21 | 1 m/s `--wbc-full` speed baseline | historical result | `archive/branches/speed/1mps-2026-08-21` -> `d55335bedd95541b0bec3c21add53920586b38ad` | [`SPEED_1MPS_ACCEPTANCE_2026-08-21.md`](SPEED_1MPS_ACCEPTANCE_2026-08-21.md); simulation claim. |
| 2026-08-21 | 1 m/s natural-trot and low-duty running-trot branches | historical result | `archive/branches/gait/natural-trot-1mps-2026-08-21` -> `d41143faba7e7064a7064adc6470adec9b30a529` | [`NATURAL_GAIT_1MPS_ACCEPTANCE_2026-08-21.md`](NATURAL_GAIT_1MPS_ACCEPTANCE_2026-08-21.md) and [`RUNNING_GAIT_1MPS_ACCEPTANCE_2026-08-21.md`](RUNNING_GAIT_1MPS_ACCEPTANCE_2026-08-21.md); not the current default gait. |
| 2026-08-21 | Bounded 3 m/s WBC sprint | accepted historical | `baseline/3mps-bounded-sprint-2026-08-21` -> `d41143faba7e7064a7064adc6470adec9b30a529` | [`SPRINT_3MPS_WBC_FULL_ACCEPTANCE_2026-08-21.md`](SPRINT_3MPS_WBC_FULL_ACCEPTANCE_2026-08-21.md); three independent passes with a short high-speed window and controlled stop, not sustained 3 m/s. |
| 2026-08-22 to 2026-08-24 | Sustained 3 m/s running-trot | accepted historical | `milestone/sustained-running-3mps-2026-08-22` -> `66dc3e810dcf8766e4e2fd838e14fb772805c76d` | [`SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`](validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md); three exact-head passes, MuJoCo running-trot only. |
| 2026-08-23 | Phase 2 P0/P1 lidar sensing and foothold observation | accepted historical sensor milestone | P0 `ce8de923d71dc8d14fe1c6f8d7f1d1765a18b9e9`; P1 `9f7423f39ee4d40a5870b46943b5b07634d1b249` | Three scenes x three runs observed flat, 10 cm barrier, and stair heights; observe-only, no terrain actuation or crossing acceptance. |
| 2026-08-25 | Phase 1 arbitrary-velocity contract | accepted historical | `milestone/phase1-arbitrary-velocity-2026-08-25` -> `6e34f99f39731982dd1c1646f9d9673ecf50737a` | [`PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md`](validation/PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md) and [closeout](validation/PHASE1_RUNTIME_VELOCITY_CLOSEOUT_2026-08-25.md); five profiles x three runs = 15/15. |
| 2026-08-25 | Phase 2 B0 acceptance contract frozen | historical contract | `f140fac358187699b437b4eb8e6291b5363f5419` | Current [`PHASE2_ACCEPTANCE.md`](research/PHASE2_ACCEPTANCE.md) is the consolidated contract; this freeze set the sensor-only/no-actuation prerequisite and the B0-before-B1 order. |
| 2026-08-28 | Phase 2 terrain-sensor-only profile campaign | accepted historical subset; broader matrix indexed | `milestone/phase2-terrain-sensor-varying-2026-08-28` -> `70b7740c77dccd9b6610f772100f2df6d4d792e2` on `phase2-b1-b3` | [`SUMMARY.md`](research/evidence/phase2_terrain_sensor_velocity_20260828/SUMMARY.md) records the 3/3 varying PASS; [`ALL_PROFILES.md`](research/evidence/phase2_terrain_sensor_velocity_20260828/ALL_PROFILES.md) indexes raw steps/accel/brake/ramp/varying coverage, including mixed repeats and retries. No current full-B0 or terrain-actuation claim. |
| 2026-09-01 | Order-109b fixed-speed lockstep B0 slice | accepted historical slice | Source `5b95e8265c885a81f8488e4930e682aa55f05674` | [`evidence/order109b_c006i/SUMMARY.md`](research/evidence/order109b_c006i/SUMMARY.md); fixed 3 m/s sensor-only slice, separate from variable-speed and full current B0 acceptance. |
| 2026-09-04 | Phase 2 clean convergence designated as the canonical integration base | historical base | `archive/branches/phase2-current-pre-main-20260904` -> `c58d1612406d2028a3e9f84c0c7c9bc50a1cc1d3` | Integration/topology decision only; it does not promote any unaccepted terrain capability. |
| 2026-09-04 | B0 runtime-integrity paired investigation (F10-F14) | historical diagnostic; superseded | `fix/phase2-b0-runtime-integrity` | The retained wall-clock diagnostics isolated terrain-worker default CPU pinning as the first cause; the detailed causal boundary remains in [`WALLCLOCK_FIRST_DIVERGENCE.md`](../WALLCLOCK_FIRST_DIVERGENCE.md). |
| 2026-09-05 | Frozen B0 wall-clock closeout | accepted | `a5e8a77e3200e8c246ef25b31abfb1cd0f6e73fd` (minimum fix `e457bd2b661d01c8c033271f31b9252854781b9c`) | [`WALLCLOCK_FIRST_DIVERGENCE.md`](../WALLCLOCK_FIRST_DIVERGENCE.md) records the six terrain profiles plus fixed-3-m/s evidence: frozen B0 PASS with terrain actuation disabled. B1 5 cm is not claimed. |
| 2026-09-06 | B1 H1 absolute-time plan-consumer probe | not accepted | Clean source `91656557596d2e01950db17354bfeae38d079e3c` (implementation `1e42d4cccce39f1f61f48954c52a3fadf512c469`) | Correct-scene no-debug canary `phase2_b1_development_canary_h1_timealigned_20260906_033000` proved 70 planner targets entered execution, but required-plan rejections remained 28,126, execution rows without WBC plan 38,431, collision rows 33, and no step clearance. Debug attribution `phase2_b1_debug_h1_timealigned_reject_20260906_040000` retained `unknown[path]` as a candidate rejection cause. No B1 claim. |
| 2026-09-06 | B1 H2 running-trot aerial-knot probe | not accepted | Clean source `ff48dac98928ef5488a5c2bde3f8bc4b7c588ca1` | `terrain_planner.h` now skips only zero-contact running-trot knots in support-polygon validation; one-contact knots and all frozen thresholds remain unchanged. Correct-scene no-debug canary `phase2_b1_development_canary_h2_aerial_20260906_060000` retained 2,462 required-plan rejection rows, all `support_infeasible=5`, 140 execution rows without a WBC plan, zero collisions, and an IK stop before step clearance. B1 is not claimed. |

| 2026-09-06 | B1 H3 support-rejection witness diagnostics | diagnostic only; not accepted | Clean source `c251f00a3b35e39eabac838e09783cfc9cddf8ce` | Passive planner telemetry attributed the dominant support rejection to knot 0, measured contact mask 9 (FR+RL, 2,673 rows; margin minimum -0.06246 m), with mask 6 (FL+RR) in 644 rows. The retained debug canary `example/cpp/experiments/_runs/example/cpp/experiments/_runs/phase2_b1_debug_h3_support_20260906_013500` had 1,577 required-plan rejection rows, 748 execution rows without WBC plan, and 169 collisions; debug timing makes it attribution-only. |
| 2026-09-06 | B1 H4 schedule-anchor planner validation | not accepted | Clean source `ae1f4730aaaddbaf00adc27e1dc29f7e41533a0f` | Correct-scene no-debug canary `phase2_b1_development_canary_h4_support_anchor_20260906_020000` retained 10,468 required-plan rejections, 4,956 execution rows without WBC plan, 22 collision rows, and stopped at cycle-quality rejection after 33.1 s. Support witness masks 9/6 reached margin -0.10501 m; the schedule-based gait anchor was not a valid planner support anchor. |
| 2026-09-06 | H5 force-backed support-anchor B0 regression | not accepted | Clean source `5bc8c73cb4cc96370de0e2bfc34535785e59870b` | Representative B0 varying terrain member failed strict quantitative acceptance with `id_wbc_ok_fraction=0.99980678`; the retained run is `example/cpp/experiments/_runs/phase2_b0_development_varying_r0_20260906_020741_terrain`. No B1 canary was run; the fix must preserve the accepted B0/Phase-1 boundary. |

## Non-acceptance register


These rows are part of the same ledger. They make failed, superseded, or design-only work discoverable without allowing it to masquerade as a capability milestone.

| Date | Route | State | Exact identity | Boundary / reason |
|---|---|---|---|---|
| 2026-08-23 | `TerrainApproachFsm` quasi-static/scripted crawl | retired | `0068b144746d0c6dff484583b8db3158aa59bd7a` | Retired because quasi-static/scripted crawl is outside the current contract; never promote its PASS as locomotion evidence. |
| 2026-08-26 | Stage-B terrain interfaces (`lidar -> planner -> MPC/WBC`) | not accepted | `archive/branches/research/phase2-stage-b-implementation-20260826` -> `8362346a62d8225afeb4f6012f64b535b6e00894` | Wiring and tests do not establish B0/B1 runtime acceptance. |
| 2026-08-26 | Deterministic functional/realtime split | not accepted | `archive/branches/research/phase2-determinism-20260826` -> `e155f0f91419ecbbf4e8a6b78b15000beb23512` | Functional bytes matched, but realtime quality failed from early tick divergence; diagnostic tooling only. |
| 2026-08-27 | B1/B2 5 cm terrain handoff and fixed-time swing work | not accepted | `archive/branches/research/phase2-b1-5cm-20260826` -> `6b28e91894e1db83e5ce7851cf39ca9cfd7e2203` | Upper-surface touchdown and support transfer were not verified; no B1/B2 or B0 recertification. |
| 2026-08-31 | Early Stage-C estimate/optimization co-planning design | design only | `ee8dba8e98223b4db5fc620775a1cce4a1ef8a91` | A planning direction, not runtime evidence or a replacement acceptance contract. |
| 2026-09-01 | Order117 standalone diagnostic observer | design only; not accepted | `1621ecd1e600808fc14f74a87683e9ddb4cea494` | Read-only DDS capture design for LowState, SportModeState, lidar, and environment height-map topics; interface partial, runtime probe unauthorized, and no controller/publish/actuation path. Not part of the current route. |
| 2026-09-02 to 2026-09-04 | Bootstrap/misrouted Stage-C branches | archived; not accepted | `f3b6e96654499d80a3dc2b1ba9912d057a2eabac` | Preserved for audit, excluded from the canonical route and capability claims. |
| 2026-09-04 | B0 steps development pass (F5) | diagnostic only; not accepted | `archive/b0-steps-pass-20260904` -> `9f97f6457ad86481054058476405bd9fa3f4142a` | Both members passed the Phase 1 quantitative analyzer with ID-WBC validity 1.0, but this was not the complete B0 suite and did not authorize B1. |
| 2026-09-06 | B1 H2 aerial-knot running-trot canary | not accepted | `ff48dac98928ef5488a5c2bde3f8bc4b7c588ca1` | The frozen running-trot schedule's zero-contact aerial knots are now excluded from support-polygon rejection; all nonzero-contact support checks remain governed by the frozen 1.5 cm margin. The correct-scene canary retained 2,462 required rejection rows, all failure code 5, 140 execution rows without WBC plan, zero collisions, and stopped on IK before clearing the 5 cm step. Evidence is retained under `example/cpp/experiments/_runs/phase2_b1_development_canary_h2_aerial_20260906_060000`; no B1 claim. |
| 2026-09-06 | B1 H1 time-aligned running-trot canary | not accepted | `91656557596d2e01950db17354bfeae38d079e3c`; implementation `1e42d4c` | The planner-selected endpoint was latched and applied (`target_prepared=70`), so Phase-1 endpoint substitution was not the cause. The canary nevertheless failed the frozen B1 gates: no clear/stable crossing, 28,126 required rejections, 38,431 execution rows without WBC plan, and 33 ground-truth collision rows. Raw no-debug and debug attribution directories are retained under `example/cpp/experiments/_runs/`. |

| 2026-09-06 | H5 B0 protection failure | not accepted | `5bc8c73cb4cc96370de0e2bfc34535785e59870b` | Force-backed measured-contact anchor plumbing caused the representative B0 varying terrain member to fail strict quantitative acceptance (`id_wbc_ok_fraction=0.99980678`). Evidence is retained at `example/cpp/experiments/_runs/phase2_b0_development_varying_r0_20260906_020741_terrain`; no B1 claim. |
| 2026-09-06 | B1 H4 support-anchor canary | not accepted | `ae1f4730aaaddbaf00adc27e1dc29f7e41533a0f` | The canary used the schedule-based gait anchor for planner support validation, retained 10,468 required rejection rows, 4,956 execution rows without WBC plan, 22 collisions, and failed cycle quality at 33.1 s. Evidence: `example/cpp/experiments/_runs/phase2_b1_development_canary_h4_support_anchor_20260906_020000`; hypothesis retired. |
| 2026-09-06 | B1 H3 support-witness debug canary | diagnostic only; not accepted | `c251f00a3b35e39eabac838e09783cfc9cddf8ce` | Support-witness fields made the planner rejection boundary auditable: knot 0 with mask 9 (FR+RL) dominated at 2,673 rows, mask 6 (FL+RR) added 644, and support margin reached -0.06246 m. The debug run is retained at `example/cpp/experiments/_runs/example/cpp/experiments/_runs/phase2_b1_debug_h3_support_20260906_013500`; timing perturbation prevents acceptance use. |


## Supporting narratives
The sections below preserve detailed context; the ledger above is the canonical index.

## 1. Low-level control and instrumentation

**Question.** Can the Unitree Go2 simulator/runtime stack support repeatable low-level action experiments with enough instrumentation to distinguish commanded motion from realized motion?

**Work.** The project established the `LowCmd → MuJoCo → LowState` loop, joint/IMU/contact logging, forward and inverse kinematics, world-frame foot-clearance measurements, and parameterized stand / weight-shift / leg-lift sequences.

**Result.** Repeatable stand-up, settle, trot, return-to-stand, and lie-down run as one LowCmd state machine. Smoothstep interpolation and a stand-pose settle are what stitch the segments. The current sequenced plant is `--wbc-full` at about 0.12–0.15 m/s; 0.18 m/s was a `--wbc-primary` torque-gate edge; 0.21 m/s was rejected. This is Go2 motion sequencing in MuJoCo, not a speed or natural-gait result.

**Evidence.** `example/cpp/` and the retained artifacts under `example/cpp/experiments/`.

## 2. Continuous trot and dynamics-informed control

**Question.** How far can a hand-designed model-based locomotion stack be pushed while retaining interpretable control structure and measurable failure modes?

**Work.** The controller was extended with diagonal-trot phase generation, smooth swing trajectories, Raibert landing adjustment, world/support feedback, constrained contact-force allocation, runtime gating, incremental dynamics-informed feedforward, and later an 18-DoF `--wbc-full` ID-WBC + SRBD MPC path.

**Result.** The indexed cruise on this tree is `--wbc-full`: 64-cycle n=5 at 0.130 ± 0.011 m/s. Under the older `--wbc-primary` gates, 0.15 m/s was the reliable cruise, 0.18 m/s was marginal, and 0.21 m/s was out of range; `go2sim walk` is not claimed as currently reproduced.

**Evidence.** `example/cpp/` (now modular under `trot/`, `wbc/`, …), and the retained experiment artifacts.

## 3. RL exploration

**Question.** Could learned locomotion provide a useful alternative to continued hand tuning for higher-speed and more dynamic behavior?

**Work.** The repository explored a small MuJoCo PPO implementation and later Isaac Lab / RSL-RL training. The experiments exposed evaluation and training-design pitfalls, including policies that could score well on coarse velocity metrics while producing undesirable or unstable motion.

**Result.** Isaac Lab velocity curricula reached commanded speeds up to ±3.5 m/s
with a short-stride gait (`model_54950`). That is a useful speed result and not
a natural-gait result. The package and evidence now live in the companion
repository [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).

**Evidence.** The companion repository's README, environment snapshot, clips,
and checkpoint record.

## 4. Motion imitation moved to a companion repository

Kine2Go / Genesis imitation, the seam JSON record, and the conditional-AMP negative baseline are in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## 5. Exact-head high-speed running-trot validation

**Question.** Does the sustained high-speed reference remain reproducible at the exact integration head without weakening its acceptance semantics?

**Work.** Commit `66dc3e810dcf8766e4e2fd838e14fb772805c76d` was checked in an isolated clean checkout. The simulator and C++ example stack built, all 25 registered C++ tests passed, and the documented `run_sustained_running.sh --headless` entry was run three independent times with the unchanged strict analyzer.

**Result.** All three repeats passed. Median speeds were 3.2257–3.2353 m/s, continuous accepted windows were 61.342–61.630 s, roll/pitch P95 stayed below 3.081/2.563°, and stop-tail speed P95 was 0.003713–0.003905 m/s. This is a simulation-only `running-trot` result; it does not upgrade the historical slow-trot number into a hardware or natural-gait claim.

**Evidence.** [`docs/validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`](validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md) and the retained protocol/acceptance document [`docs/SUSTAINED_RUNNING_3MPS_ACCEPTANCE_2026-08-22.md`](SUSTAINED_RUNNING_3MPS_ACCEPTANCE_2026-08-22.md). Raw `_runs/` are ignored and were not committed.
