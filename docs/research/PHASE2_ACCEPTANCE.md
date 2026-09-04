# Phase 2 acceptance

This is the sole active acceptance contract. It consolidates without widening
the frozen `b0-contract-v1.2`, `b1-contract-v1.0`, and `phase2-b123-v1`
criteria. Historical contract text remains in Git and cannot authorize work.

## Provenance and scope

Every verdict requires exact HEAD, clean source, effective argv, scene/profile,
seed/domain, and hashes for source, binary, simulator, config, contract,
analyzer, and input data. Controller and planner input is state estimation plus
lidar-derived terrain only. They may not read scene XML, geom identity,
obstacle coordinates, step index, simulator contact/collision, or an oracle
map. Ground truth is harness-only post-run scoring.

The Phase-1 shaper is the only horizontal velocity authority. Running-trot and
its normal two-contact diagonal interval remain valid. Terrain execution must
use one time-indexed snapshot across gait, SRBD-MPC, and ID-WBC; knot-zero MPC
contact must match applied WBC support. Planned and measured contact are
separate. Quasi-static crawl, fixed leg order, a three-contact gate/preload,
low stance, stop-to-arm, cap-to-zero transfer, and local swing retiming are
contract failures.

## B0 sensor-only gate

B0 enables lidar/map/planner telemetry with terrain actuation disabled. Terrain
must not alter footholds, body/CoM reference, contact schedule, gait timing or
topology, velocity command, MPC, or WBC. Plan publication/consumption,
terrain arbitration, hidden events, safety stops, and deadline misses are zero.
The paired no-terrain run has matching non-terrain configuration and zero
lifecycle failures. Every terrain member itself passes its Phase-1 profile.

Frozen profiles and per-profile gates are:

| profile | tracking P95 | steady max | positive excursion | negative bound | settling |
|---|---:|---:|---:|---:|---:|
| steps | 0.40 | 0.40 | 0.50 | -0.25 | 8.2 s |
| accel_1_to_3 | 0.42 | 0.40 | 0.50 | -0.25 | 10.0 s |
| brake_3_to_0 | 1.50 | 0.55 | 0.05 | -0.20 | 1.5 s |
| ramp | 0.42 | 0.18 | 0.60 | -0.20 | 2.0 s |
| varying | 0.42 | 0.45 | 0.50 | -0.40 | 8.2 s |

Values are m/s except settling. Common gates: requested-profile reproduction
`<=1e-6 m/s`; shaped-to-measured P95 `<=0.45 m/s`; acceleration
`<=1.25 m/s2`; jerk `<=4.20 m/s3`; acceleration sample change
`<=0.02 m/s3`; roll/pitch P95 `<=4 deg`; absolute roll/pitch `<=15 deg`;
contact loss `<=0.25`; single-contact `<=0.45`; touchdown x/y
`<=0.18/0.07 m`; torque saturation `<=0.003` at 45 Nm; slip `==0`; solver,
SRBD, ID-WBC, and footstep-plan validity `==1.0`; solver budget `>=0.80`;
base height `>=0.28 m`; stopping-profile tail P95 `<=0.05 m/s`; all lifecycle
statuses zero.

The fixed 3 m/s slice additionally requires roll/pitch P95 `<=5 deg`, base
height p01-p99 `0.33-0.40 m`, foot clearance `>=0.08 m`, aerial fraction
`0.15-0.40`, diagonal pair synchronization `>=0.75`, two-contact fraction
`>=0.30`, three-contact fraction `<=0.10`, all-feet-low `<=0.05`, and final
and stop-tail speed `<=0.10 m/s`.

## B1 dynamic 5 cm gate

B0 domains and B1 scenes are frozen in `PHASE2_HOLDOUT_MANIFEST.json`. A fresh
full B0 PASS on the exact candidate
SHA is prerequisite. Development runs never count as holdout acceptance.

Frozen terrain gates: committed-plan hard-feasible fraction `1.0`; raw edge
margin `>=0.040 m`; uncertainty-inflated edge margin `>=0`; slope `<=20 deg`;
roughness `<=0.025 m`; local surface step `<=0.040 m`; IK margin
`>=0.010 m`; uncertainty-adjusted swept clearance `>=0`; touchdown z error
`<=0.075 m`; predicted support margin `>=0.015 m`; uncertainty-inflated
support margin `>=0`; planner latency p95 `<=2 ms`, hard `<=5 ms`, misses
`==0`. A required-plan rejection, stale plan, brake/hold, collision, safety
stop, second velocity reference, or incomplete crossing is FAIL.

PASS requires every leg to achieve force-supported touchdown on each crossed
surface, zero obstacle collision, body and all feet beyond the terrain, and at
least `0.45 s` stable post-crossing evidence. MPC first/last horizontal
velocity references match shaped/applied Phase-1 v_cmd within `0.020 m/s`;
planner-derived horizontal reference span must be `<=0.001 m`.

## B2 and B3

B2 repeats the same contract on the 10 cm fixture. B3 covers mixed/repeated
rises and descents with multiple map/plan epochs. Neither milestone may add
scene-specific behavior or widen any B1 gate.

## Evidence and verdict

Each run contains `data.csv`, `run_manifest.json`, controller/simulator logs,
contact ground truth, the applicable Phase-1 quantitative result, and the
milestone analyzer result. Missing provenance or evidence is FAIL. Failed runs
are retained. Any code/config change creates a new candidate and requires fresh
B0 before B1 holdout. A threshold, member, or analyzer meaning cannot change
after results are observed.
