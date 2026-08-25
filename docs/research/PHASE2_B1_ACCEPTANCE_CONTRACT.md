# Phase 2 B1 Acceptance Contract

Status: FROZEN before the first B1 tuning run. Version: `b1-contract-v1.0`.

B1 is the first sensor-derived 5 cm single-step execution milestone. It does
not authorize changing the accepted Phase 1 controller, gait topology, v_cmd
shaper, hard safety limits, or physical limits.

## Provenance and scope

* source base: `origin/main=71d0e9ba7ca1097e840fe878aa30207f6f63600d`;
* prerequisite: B0 `b0-contract-v1.2`, epoch 11, PASS;
* controller: `--wbc-full --gait-pattern running-trot --kernel raibert-trot`;
* terrain input: lidar-derived HeightMap, frame `base_link`, never oracle;
* planner input: state/contact estimate plus TerrainModel only;
* controller/planner must not read scene XML, geom names/positions, true
  contact/collision, an oracle map, a step index, or obstacle coordinates;
* the harness may read MuJoCo ground truth only after the run for scoring;
* Phase 1 v_cmd arbitration remains the only terrain speed-request path;
* normal crossing is from safe regions and predicted contacts; FSM is only
  plan-lifecycle/safety supervision.

The three holdout cases in `PHASE2_B1_HOLDOUT_MANIFEST.json` are frozen
before tuning. Development runs are separate and never count toward acceptance.
Each run must record exact HEAD, clean-source status, config, scene/profile,
contract/analyzer/harness hashes, seed, and effective arguments.

## Inherited Phase 1 gates

The `steps` profile keeps the exact accepted gates: tracking P95 <=0.40 m/s,
steady max <=0.40 m/s, positive excursion <=0.50 m/s, negative excursion
>=-0.25 m/s, settling <=8.2 s; roll/pitch P95 <=4 degrees and absolute max
<=15 degrees; contact loss <=0.25; single-contact <=0.45; touchdown x <=0.18 m
and y <=0.07 m; torque saturation <=0.003 at unchanged 45 Nm; slip evidence
==0; solver, SRBD, ID-WBC, and footstep-plan validity ==1.0; solver budget
>=0.80; base height >=0.28 m; stop-tail P95 <=0.05 m/s; and all lifecycle
statuses zero. The active lower hard limit always wins. No B1 gate widens it.

## Terrain gates frozen before tuning

The lidar resolution is 0.05 m and foot-patch radius 0.025 m. These values come
from the current feasibility configuration and geometry, not observed results.

| metric | gate |
|---|---:|
| committed plan hard-feasible fraction | 1.0 |
| raw edge margin | >=0.040 m |
| uncertainty-inflated edge margin | >=0.000 m |
| slope | <=20 degrees |
| roughness | <=0.025 m |
| local surface step | <=0.040 m |
| IK reachability margin | >=0.010 m |
| interior swept clearance after uncertainty | >=0.000 m |
| touchdown z error against sensed patch | <=0.075 m |
| predicted support margin | >=0.015 m |
| uncertainty-inflated support margin | >=0.000 m |
| planner latency | p95 <=2 ms; hard <=5 ms; deadline misses ==0 |
| stale-plan grace | at most one declared touchdown, then v_cmd brake/hold |
| successful-run safe-stop and ground-truth collision | 0 |

The z value is one map cell plus the patch radius. If sensor/frame/uncertainty
semantics change, stop and create a new contract; never move a threshold after
seeing holdout results. Terrain gates may only become stricter with rationale.

## Required evidence and verdict

Every run contains data.csv, run_manifest.json, controller/simulator logs,
contact_ground_truth.csv, the applicable Phase 1 quantitative JSON, and
b1_analyzer.json. The analyzer reports inherited velocity/posture/base-height/
contact/slip/torque/model gates plus candidate count, safe-region membership,
reachability, edge/slope/roughness/uncertainty, swept clearance, touchdown
x/y/z, predicted support margin, map/source/age, plan id/epoch, publish/consume
counts, SRBD/ID-WBC validity, latency/deadlines, collision, and safe-stop.
Ground-truth fields are explicitly harness-only.

PASS requires all inherited, terrain, lifecycle, interface, and provenance
gates. A required-plan rejection, brake, or safe-stop is diagnostic FAIL.
Every failure is retained. Any code/config fix creates a new implementation
epoch and requires fresh B0 regression before B1 tuning/holdout.
