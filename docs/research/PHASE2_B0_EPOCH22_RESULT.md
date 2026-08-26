# Phase2 B0 epoch-22 result

Status: **FAIL**. This is the single formal B0 holdout batch for the clean
freeze at `4a5a28a109ecb593c5b45c9b242a88824cd5e430`, with implementation
commit `2fefd71ce59e54fbd4a2a3fd0cf9a34a1d9c7ca9` and
`b0-v1.2-epoch-22` manifest. Build and CTest were clean: 27/27 passed.

The batch contains 18 terrain sensor-only members: five profiles at three
repeats plus three fixed-3 m/s repeats. Seventeen passed. Steps, all
accel_1_to_3, brake repeats 2--3, all ramp, all varying, and all fixed-3 m/s
members passed.

The only failure is:

- Run:
  `example/cpp/experiments/_runs/phase2_b0_holdout_brake_3_to_0_r1_20260826_114517_terrain`
- Analyzer: `phase1_quantitative.undershoot=false`
- Measured transition excursion: `-0.211308188 m/s`
- Frozen lower bound: `-0.20 m/s`
- Paired baseline excursion: `-0.124889172 m/s`
- Controller, safety, quality, completion, dynamics, and analyzer statuses:
  all zero.
- Terrain planner updates: 666; deadline misses: 0; maximum reported solver
  elapsed time: 1833.657 us; no plan publish, consume, or terrain actuation.

The failed terrain CSV had maximum wall-clock motion delta 0.004214218 s and
state-tick gap 0.006 s, so this member does not reproduce epoch-21's 45 ms
stall. The evidence proves the failure is not an elevated terrain plan or
direct planner actuation, but it does not yet prove complete observational
orthogonality of the lidar/DDS path from the accepted Phase 1 dynamics.

No retry was used to cover the failed member. The failed run and analyzer are
retained. B1 is blocked by the mandatory rule "B0 PASS before B1"; no
threshold, safety envelope, controller, or physics constraint was relaxed.

The next minimum action is a targeted observer-isolation investigation
(DDS/map callback and terrain worker scheduling) followed by a new frozen
implementation only if a real control-path or real-time coupling is proven.
Do not start B1 or rerun this failed member on the current freeze.
