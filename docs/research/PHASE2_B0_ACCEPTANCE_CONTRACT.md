# Phase 2 B0 Acceptance Contract

Status: FROZEN before Phase 2 implementation tuning. Version: 'b0-contract-v1.2'.

This contract is a no-actuation flat-ground regression for the first Stage B
milestone. It is not a terrain locomotion result and it does not authorize any
change to the accepted Phase 1 controller contract.

## Frozen provenance

* accepted source base at freeze: origin/main=71d0e9ba7ca1097e840fe878aa30207f6f63600d;
* Phase 2 plan: docs/research/PHASE2_TERRAIN_PLAN.md;
* controller input: the existing Phase 1 runtime v_cmd arbitration and
  shaper;
* terrain mode: sensor/map and planner telemetry enabled, actuation disabled;
* terrain controller input: sensor-derived map only; no MuJoCo oracle map,
  scene XML, obstacle geom, obstacle coordinate, or ground-truth contact;
* accepted result: every required legacy and quantitative Phase 1 gate passes,
  and the terrain path is proven observationally orthogonal.

The contract hash, source HEAD, build hashes, effective CLI, environment
snapshot, analyzer hash, profile hash, and scene hash are recorded in every
run_manifest.json. A missing or dirty provenance field is a contract failure.

Version 1.2 corrects one pre-holdout measurement error in v1.1. Independent
wall-clock MuJoCo launches are not sample-identical experiments: a startup
state-tick offset of only a few milliseconds can put the nonlinear feedback
plant on a different trajectory. Therefore row-aligned max-difference gates
on feedback-dependent applied speed, gait geometry, WBC target, and requested
acceleration are diagnostic fields, not acceptance gates. They remain in the
report so an unexplained systematic shift is visible. Orthogonality is instead
accepted through explicit no-consumer/no-arbitration telemetry, identical
non-terrain effective configuration, and the exact inherited Phase 1 gates on
the terrain member itself. The paired baseline must have complete provenance
and zero lifecycle status failures; its quantitative metrics remain diagnostic,
because an independent wall-clock launch is not a valid second quantitative
sample of the same nonlinear trajectory.

The first implementation epoch's holdout evidence is retained as diagnostic
evidence. It exposed an avoidable terrain-diagnostics mutex/shared-pointer
read on the no-terrain Phase 1 hot path; commit
45bf4904d7e38029fa1d91e3f646d36866fc27af removes that work when terrain is
disabled. The frozen gates, profile membership, repeats, analyzer semantics,
and safety limits are unchanged. The current acceptance epoch therefore
starts at that commit and must rerun every listed holdout member; results from
the earlier epoch cannot be silently reused or mixed into the verdict.

Epoch 2 then showed a marginal inherited torque-saturation failure only in the
terrain member while its paired baseline passed. Commit
ce16735a10781fe9592a9df6ab4fbb4731aa3557 demotes the sensor-only observer
worker to best-effort Linux scheduling so it cannot preempt the accepted
500 Hz command writer. This is another implementation-only correction: the
same frozen gates and holdout membership define epoch 3, which must again be
run in full.

Epoch 3 still showed elevated state-tick gaps in both paired members. The
simulator's terrain height-map raycasting had been enabled unconditionally,
including for the no-terrain baseline. Commit
4d5fee9f96f8b839e8716f5c7c2ac6ed7a399703 gates that publisher behind the
explicit terrain-sensing simulator flag, restoring the accepted Phase 1
simulator path for baseline runs. The unchanged contract is restarted as
epoch 4; earlier epochs remain diagnostic only.

The epoch-4 development pair then showed that enabling the lidar publisher in
the bridge still perturbed simulator timing: the publisher held the simulator
snapshot path while raycasting. Commit
b9214777a7938c89ad39da20199cc26bfada9323 moves lidar acquisition to a
best-effort thread that copies `mjData` before raycasting. The epoch-5
development pair passed both members with the unchanged gates; epoch 5 is the
next frozen holdout implementation epoch.

The first epoch-5 holdout accel pair exposed a runner inconsistency: the
terrain member passed every B0 and inherited Phase 1 gate, while the paired
baseline independently missed only its 1-to-3 settling sample. The analyzer
already treated baseline quantitative output as diagnostic, but the pair
runner incorrectly made it fatal. The runner now enforces baseline lifecycle
and provenance while keeping the terrain quantitative gate authoritative.
This is a contract-measurement correction, not a threshold or safety-envelope
change; epoch 6 must rerun the complete frozen membership.

Epoch 6 then produced a terrain-member inherited Phase 1 failure in the
accel_1_to_3 repeat-2 run: the frozen overshoot limit is 0.50 m/s and the
observed excursion was 0.512597306 m/s. All B0 interface gates, lifecycle
statuses, and the paired baseline lifecycle passed. Diagnosis found that the
terrain-enabled LowCmdWrite path still copied the DDS HeightMap and assembled
planner work about every 50 ms, despite the planner itself being asynchronous.
Commit 6244c81eedbfdb9f06aa8f482e3df667e229d000 moves map/work capture to the
best-effort terrain worker and leaves only a bounded control snapshot on the
500 Hz path. This is an implementation isolation fix; thresholds, holdout
membership, and safety limits are unchanged. Epoch 7 must rerun the complete
frozen membership, and epoch-6 evidence remains diagnostic only.

## Development and holdout split

The development set is for plumbing/debugging only. It may use one repeat of
each profile, deterministic DDS domain, and injected stale/unknown/deadline
faults. Development runs never establish acceptance.

The holdout set is frozen before the first tuning run:

* the five Phase 1 profiles: steps, accel_1_to_3, brake_3_to_0, ramp,
  and varying;
* three fresh valid repeats per profile, with DDS domains 200, 201, and 202;
* one fixed 3 m/s sustained-running regression repeat per domain 203, 204,
  and 205;
* all listed domains are within the repository's enforced CycloneDDS range
  [0, 232]; the earlier 260-265 proposal was invalid and was corrected before
  any development or holdout run;
* no terrain planner source/config change between repeats;
* the analyzer and this contract are hashed before the first holdout run.

If the runner later supports initial-x or gait-phase perturbations, the
holdout manifest must include values not used for development. Until then,
the absence of those perturbations is reported as an explicit limitation, not
silently treated as anti-memorization evidence.

## B0 actuation and interface gates

For every active row:

1. terrain_mode=sensor_only and terrain_actuation=0;
2. terrain may publish TerrainModel and TerrainFeasibility telemetry, but
   no TerrainMotionPlan may alter footholds, body/CoM reference, contact
   schedule, gait topology/period/duty/phase/lift, or SRBD/ID-WBC inputs;
3. the effective non-terrain configuration must match the paired baseline;
   requested-profile reproduction, shaper, gait, WBC, MPC, and ID-WBC fields
   must satisfy the inherited Phase 1 contract in the terrain run itself;
   terrain-specific consumer/arbitration counters must remain zero;
4. planner/map updates, stale data, unknown cells, rejected plans, and solver
   deadlines must not block the control loop or clear a valid Phase 1 command;
5. controller/planner processes must not read simulator ground truth.

Any terrain-to-runtime coupling, hidden event activation, direct gait setter,
motion-reference overwrite, contact-plan mutation, or unexplained numerical
divergence is B0 FAIL even if the legacy analyzer happens to pass.

For a paired no-terrain run using the same profile and controller arguments,
the analyzer compares normalized effective argv, run provenance, lifecycle
status, and all terrain consumer/arbitration counters. It also reports the
v1.1 row-aligned numerical differences as a diagnostic with no pass/fail
meaning, because those fields depend on independent wall-clock feedback
trajectories. A missing paired baseline, mismatched non-terrain argv, dirty
source, or missing diagnostics is FAIL.

## Inherited Phase 1 quantitative gates

The selected profile uses the exact row below; no common replacement is
allowed. 'tracking' is shaped-command-to-measured velocity error.

| profile | tracking P95 | steady max | positive excursion | negative excursion lower bound | settling max |
|---|---:|---:|---:|---:|---:|
| steps | 0.40 m/s | 0.40 m/s | 0.50 m/s | -0.25 m/s | 8.2 s |
| accel_1_to_3 | 0.42 m/s | 0.40 m/s | 0.50 m/s | -0.25 m/s | 10.0 s |
| brake_3_to_0 | 1.50 m/s | 0.55 m/s | 0.05 m/s | -0.20 m/s | 1.5 s |
| ramp | 0.42 m/s | 0.18 m/s | 0.60 m/s | -0.20 m/s | 2.0 s |
| varying | 0.42 m/s | 0.45 m/s | 0.50 m/s | -0.40 m/s | 8.2 s |

Common gates are: requested profile reproduction <=1e-6 m/s; shaped-to-
measured P95 <=0.45 m/s; shaper acceleration <=1.25 m/s2; jerk <=4.20 m/s3;
acceleration sample change <=0.02 m/s3; roll P95 <=4 degrees; pitch P95 <=4
degrees; max absolute roll and pitch <=15 degrees; contact loss fraction
<=0.25; single-contact fraction <=0.45; touchdown x <=0.18 m and y <=0.07 m;
torque saturation fraction <=0.003 at unchanged --tau-limit 45; slip
evidence fraction ==0; solver, SRBD, ID-WBC, and footstep-plan validity ==1.0;
solver-budget fraction >=0.80; minimum base height >=0.28 m; and stop-tail
P95 <=0.05 m/s for profiles that stop. All legacy status fields remain zero:
controller, safety, quality, completion, dynamics, and analysis.

The fixed 3 m/s analyzer retains its separate frozen contract: cruise roll and
pitch P95 <=5 degrees, base-height percentiles 0.33-0.40 m, foot clearance
>=0.08 m, aerial fraction 0.15-0.40, pair synchronization >=0.75, two-contact
fraction >=0.30, three-contact fraction <=0.10, all-feet-low fraction <=0.05,
and stop-tail/final speed <=0.10 m/s, plus all lifecycle status fields zero.

## Evidence and verdict

Each run directory must contain data.csv, run_manifest.json, controller and
simulator logs, the applicable Phase 1 analyzer JSON, and a B0 analyzer JSON.
The B0 analyzer reports:

* all inherited gates and their exact thresholds;
* plan/map/source/epoch/age/latency fields;
* terrain actuation, event, velocity-arbitration, plan-consumer, gait, contact,
  MPC, and WBC isolation fields;
* planner success/rejection/deadline/stale/fallback counts;
* clean/dirty status and exact source/config/analyzer/contract hashes.

B0=PASS only if every terrain holdout member passes the applicable Phase 1
contract and all B0 actuation/interface gates pass, the paired baseline has
matching provenance and zero lifecycle failures, and all required evidence is
present and reproducible. Baseline quantitative metrics are reported as
diagnostics, not a second acceptance sample. Any failure remains in its run
directory and is diagnosed before code changes. B1 cannot start until this
contract is passed by all frozen B0 holdout repeats.

Thresholds, holdout membership, and analyzer semantics are immutable after the
first holdout run. A result cannot be used to loosen, redefine, or selectively
remove a gate.
