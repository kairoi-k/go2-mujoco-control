# Phase2 Stage B Handoff — 2026-08-26

Status: implementation handoff only. No further B0 debugging, canary, holdout, or terrain implementation is authorized in this handoff.

## 1. Project and architecture background

Repository: `go2-mujoco-control`, canonical WSL checkout `/home/che/dev/go2-mujoco-control`. This handoff branch is `research/phase2-stage-b-20260825`; accepted main at the time of handoff is `origin/main=71d0e9ba7ca1097e840fe878aa30207f6f63600d`. All old Phase2 branches, worktrees, stashes, and run evidence remain reference-only.

Phase1 is the accepted baseline. Its runtime path is `v_cmd` → acceleration/jerk-limited command shaper → continuous gait parameterization → Raibert foothold logic → SRBD-MPC → ID-WBC. Phase1 tracking, gait, hard safety limits, and quantitative constraints are not to be retuned or widened for terrain work.

The Stage B target is state estimation plus local terrain representation → terrain feasibility and safe foothold regions → online foothold/body/contact planning → terrain-aware SRBD-MPC → ID-WBC. The intended boundary is an online planner interface that can later evolve into a hybrid contact planner, whole-body NMPC, and learning augmentation. B0 has exercised the observation and interface path in sensor-only mode; it has not validated dynamic terrain crossing.

## 2. B0 capability delivered

The current branch contains the B0 interface and observability batch, including:

- lidar/elevation-map input and a `TerrainModel` representation;
- `TerrainFeasibility` safe-region construction and region consumption by the planner-side path;
- epoch, validity, and latest-valid-plan metadata for an atomic `TerrainMotionPlan` boundary;
- separation of planned contact from measured contact, plus future foothold/contact horizon fields for the terrain-aware SRBD input boundary and ID-WBC diagnostics;
- a sensor-only execution gate and terrain telemetry that show planner updates, solver time, deadlines, map validity, plan publication/consumption, and actuation;
- regression scripts, manifests, analyzers, and evidence documents for the B0 contract.

In B0, the terrain planner did not publish a plan, the control path did not consume a terrain plan, and terrain actuation remained zero. These results validate interface isolation and instrumentation only. They are not evidence of feasible footholds, swing clearance, support transfer, or a successful 5 cm crossing. Build and CTest evidence for the preceding clean freezes was successful, with CTest 27/27.

## 3. Formal B0 evidence

### Epoch 21

Epoch 21 was a formal FAIL: 15/18 terrain members passed. The failures were:

1. brake 3→0 r1: `phase1_quantitative.undershoot=-0.235920114 m/s` against the frozen `-0.20 m/s` bound;
2. varying r1: `steady_state_error=0.460990856 m/s` against `0.45 m/s`;
3. fixed 3 m/s r1: hard-posture/runtime analyzer failure, including a 45.160950 ms maximum wall-clock motion step and a 44 ms state-tick gap.

All failed members had zero terrain plan publication, consumption, actuation, safe-stop request, and planner deadline miss. The complete run directories, CSV files, manifests, logs, and analyzer outputs are retained. No successful retry replaces a failed member.

### Epoch 22

Epoch 22 was the next clean formal freeze and also a FAIL: 17/18 terrain sensor-only members passed. The only failure was:

`example/cpp/experiments/_runs/phase2_b0_holdout_brake_3_to_0_r1_20260826_114517_terrain`

The failed gate was `phase1_quantitative.undershoot=false`: measured excursion `-0.211308188 m/s` versus the frozen `-0.20 m/s` lower bound. The paired baseline measured `-0.124889172 m/s`. Controller, safety, quality, completion, dynamics, and analyzer status fields were zero; terrain planner updates were 666, deadline misses were zero, maximum reported solver time was 1833.657 us, and plan publication, consumption, and actuation were zero. This run had maximum wall-clock motion delta 0.004214218 s and maximum state-tick gap 0.006 s, so it did not reproduce the epoch-21 45 ms stall.

The epoch-22 failed run and all other epoch-22 evidence remain under `example/cpp/experiments/_runs/`. B1 is blocked by the mandatory B0-PASS-before-B1 rule.

### Latest try-lock development canary

The final development canary was steps-only and was run before the try-lock patch was reverted. It is preserved as non-certification evidence in `docs/research/PHASE2_B0_LIDAR_TRYLOCK_CANARY.md` with both original run directories:

- baseline: `phase2_b0_development_steps_r0_20260826_124240_baseline`;
- terrain: `phase2_b0_development_steps_r0_20260826_124240_terrain`.

The baseline passed. The terrain member failed only the frozen steps undershoot gate: `-0.289389868 m/s` versus `-0.25 m/s`; its paired baseline was `-0.146613853 m/s`. Controller, safety, quality, completion, and dynamics statuses were zero. Planner updates were 1476, deadline misses were zero, map valid fraction was 0.999979819, and terrain plan publication, consumption, and actuation were zero. Maximum terrain wall-clock motion delta was 0.004260126 s and maximum state-tick gap was 0.008 s. No accel or brake canary followed this failure, and no new holdout was started.

## 4. Investigation paths already checked

The audit checked the simulator lidar observer and its copied MuJoCo model/data, the lidar observation-rate bound, the DDS height-map publisher/subscriber callback, terrain-map mutex protection, terrain-control generation and snapshot mutexes, the terrain worker scheduling path, and the shared simulator mutex used while copying state. It also checked the PhysicsLoop critical section and the controller-side 500 Hz writer path that triggers the low-rate terrain snapshot.

Run metadata and controller logs were checked for CPU affinity and pinning. The intended separation was simulator physics/bridge/lidar on their recorded CPUs, controller/writer on their recorded CPUs, and the terrain worker on its isolated CPU. No new affinity change is part of this handoff.

The terrain worker was checked for sensor-only behavior: it can update the model/planner and diagnostics, but B0 does not publish a terrain plan or feed terrain footholds/contact data into MPC/WBC. The WBC path consumes a terrain plan only when the actuation gate is enabled. This explains why the B0 evidence shows no terrain plan actuation, but it does not prove that observer execution is dynamically orthogonal to Phase1.

The try-lock change was a diagnostic experiment intended to avoid blocking the PhysicsLoop during lidar state copying. It produced the preserved steps FAIL above and did not collect lock-miss telemetry. It has been reverted and is not included in the final code handoff.

## 5. Current determinism blocker

The blocker is not a missing B1 planner feature; it is unresolved determinism and observer-isolation behavior. Sensor-only terrain-enabled flat runs still show run-to-run quantitative divergence in inherited Phase1 gates, including epoch 21, epoch 22, and the latest steps canary. The failures occur while terrain plan publication, consumption, actuation, and planner deadline misses remain zero. The presence or absence of the epoch-21 wall-clock stall also varies between runs.

The try-lock experiment did not prove that observer and dynamics are orthogonal. It only showed that this particular non-blocking observer change was insufficient to remove the terrain-enabled quantitative divergence. No root cause should be claimed from these results. Existing failed evidence must remain visible and cannot be covered by retry.

## 6. B1 status and actual gaps

B1 5 cm dynamic crossing has not started and has no development or holdout PASS. The current work provides interface scaffolding and diagnostics, not a validated crossing controller. Before B1 acceptance, the project still needs a deterministic clean-run contract for flat terrain, a proven observer/control isolation boundary, and an actual live path for safe foothold, body/CoM, contact schedule, swing clearance, support transfer, future footholds, and terrain-aware SRBD inputs.

The missing B1 evidence includes approach, first touchdown, support transfer, and exit diagnostics; foothold reachability and edge margin; swing swept-volume clearance; roll/pitch and base-height safety; contact loss/slip; torque and saturation; SRBD validity; ID-WBC validity; planner success and deadline behavior; and fresh development/holdout runs for the 5 cm step. No threshold or existing hard safety envelope may be relaxed.

## 7. Recommended split for follow-up

### Determinism line

Treat observer/dynamics orthogonality and run-to-run reproducibility as an independent workstream. Preserve all epoch and canary evidence, establish a controlled clean flat comparison, and isolate one causal path at a time across simulator lidar, DDS callback/map copying, terrain worker scheduling, and snapshot publication. Any code change that touches the real-time or terrain-to-control path requires the documented canary/certification policy. Do not change Phase1 thresholds, hard limits, or physics constraints. A new clean B0 certification is required only at the next meaningful frozen control-path change, not for documentation or non-real-time diagnostics.

### B1 line

Keep B1 implementation separate and dormant until the determinism line satisfies the B0 gate. Then use one deterministic development run to close the smallest real blocker in this order: perception correctness → feasible foothold region → swing clearance → support/body plan → future contact/foothold SRBD input → dynamic crossing. Freeze B1 acceptance only after stable development evidence, then run fresh separated holdout evidence. Terrain may request speed changes only through Phase1 `v_cmd` arbitration; normal crossing must not be a scene-specific FSM or scripted leg sequence.

## 8. Non-negotiable constraints and handoff state

No Phase1 hard safety limit, gait constraint, quantitative threshold, or physical constraint was widened. No old stash was applied, no old Phase2 history was rewritten, and no evidence directory was deleted or overwritten. The unverified try-lock patch was reverted before the final handoff commit. The final commit adds only the preserved canary evidence and this handoff document; it does not claim B0 PASS or B1 progress.

Final validation for this handoff was one simulator build, one controller build, and one CTest invocation. Observed evidence is simulator build PASS, controller build PASS, and CTest 27/27 PASS; these commands were a state check only and were not a new terrain experiment.

## 9. Primary files for the next agent

- `docs/research/PHASE2_TERRAIN_PLAN.md` — approved architecture and acceptance plan;
- `docs/research/PHASE2_B0_HOLDOUT_MANIFEST.json` — frozen B0 contract/manifest;
- `docs/research/PHASE2_B0_EPOCH21_RESULT.md` — preserved epoch-21 FAIL;
- `docs/research/PHASE2_B0_EPOCH22_RESULT.md` — preserved epoch-22 FAIL;
- `docs/research/PHASE2_B0_LIDAR_TRYLOCK_CANARY.md` — preserved try-lock canary FAIL;
- `example/cpp/experiments/_runs/` — original run data, manifests, logs, and analyzer outputs.

Definition at handoff: Stage B is not complete. The repository is ready for review with complete failure evidence and a clear split between determinism work and future B1 work.
