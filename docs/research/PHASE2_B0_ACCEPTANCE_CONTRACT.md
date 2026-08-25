# Phase 2 B0 Acceptance Contract

Status: FROZEN before Phase 2 implementation tuning. Version: 'b0-contract-v1.1'.

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
3. the effective shaped/applied v_cmd, event state, kernel target, WBC
   target, MPC input, and ID-WBC output must match the no-terrain baseline
   within the pre-registered numerical comparison tolerance;
4. planner/map updates, stale data, unknown cells, rejected plans, and solver
   deadlines must not block the control loop or clear a valid Phase 1 command;
5. controller/planner processes must not read simulator ground truth.

Any terrain-to-runtime coupling, hidden event activation, direct gait setter,
motion-reference overwrite, contact-plan mutation, or unexplained numerical
divergence is B0 FAIL even if the legacy analyzer happens to pass.

For a paired no-terrain run using the same profile and controller arguments,
the pre-registered comparison tolerances are: requested v_cmd 1e-6 m/s;
shaped/applied v_cmd 0.010 m/s; period, duty, step length, and foot lift
1e-5 in native units; event active/type 0.5; event target v_cmd 0.020 m/s;
WBC velocity target 0.020 m/s; and requested acceleration 0.20 m/s2. The
comparison is row-aligned after the active locomotion start and is reported
alongside the exact terrain-mode flags. A missing paired baseline is FAIL.

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
* terrain actuation, event, velocity-arbitration, gait, contact, MPC, and WBC
  comparison fields;
* planner success/rejection/deadline/stale/fallback counts;
* clean/dirty status and exact source/config/analyzer/contract hashes.

B0=PASS only if every holdout repeat passes the applicable Phase 1 contract,
all B0 actuation/interface gates pass, and all required evidence is present and
reproducible. Any failure remains in its run directory and is diagnosed before
code changes. B1 cannot start until this contract is passed by all frozen B0
holdout repeats.

Thresholds, holdout membership, and analyzer semantics are immutable after the
first holdout run. A result cannot be used to loosen, redefine, or selectively
remove a gate.
