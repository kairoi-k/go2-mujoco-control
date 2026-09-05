# Wall-clock B0 first divergence

Updated: 2026-09-05

## Verdict

The production wall-clock path has a reproducible runtime interference when
the terrain-sensor-only path is enabled. The first observed difference is the
LowState snapshot consumed by the controller's first logged `LowCmdWrite`:
the terrain run starts from a different `state_tick`, before any terrain plan
is consumed. This is a scheduling/snapshot provenance difference, not evidence
of a gait, WBC, MPC, or planner actuation effect.

The lidar raycast and HeightMap publish are not the necessary first cause.
With the terrain path enabled and `TROT_SIM_LIDAR_NOOP=1`, the lidar thread
still exists and takes the simulator lock, but performs no raycast and
publishes zero maps; the first consumed tick still differs from baseline.
The remaining exact boundary is the terrain-enabled DDS/controller worker and
simulator lidar-thread/lock scheduling around startup and latest-state
consumption. The evidence does not isolate those two scheduler sources from
each other; confidence is high for runtime interference and medium for the
exact subcomponent.

## Contract

Both accepted pairs used the same SHA within the pair, the same running-trot
parameters, domain 190, explicit CPU placement, serial execution, and
`lockstep=false`. The only normal-pair difference was terrain sensor-only:
baseline did not enable terrain; terrain enabled `--terrain-sensor-only`,
which also enables simulator terrain lidar. The controller's added telemetry
is read-only and does not gate control decisions.

## Evidence

| pair | baseline | terrain/validation | SHA | first consumed tick |
|---|---|---|---|---|
| normal A/B | `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_baseline` | `..._r1_terrain` | `922cc89` | A `1786`, B `1734` |
| lidar no-op | `..._r2_baseline` | `..._r2_terrain_noop` (`TROT_SIM_LIDAR_NOOP=1`) | `cb920b4` | A `1730`, B `1728` |

The normal terrain run produced 1,094 controller-side lidar arrivals and
1,058 simulator publishes. Its simulator telemetry p95 values were
`lock_wait=0.0669 ms`, `lock_hold=0.000411 ms`, lidar operation `8.028 ms`,
and DDS publish `0.0460 ms`. The no-op run produced 1,576 lidar-thread
telemetry events, zero publishes, p95 `lock_wait=0.0654 ms`,
`lock_hold=0.000441 ms`, and operation `0.000020 ms`, yet its first consumed
tick still differed from baseline.

In the normal terrain CSV, the terrain map became valid, the planner updated
1,224 times, and `terrain_plan_consumed`, `terrain_gait_target_overrides`, and
`terrain_mpc_plan_consumed` stayed zero. In the no-op CSV the planner updated
1,219 times, but the same three actuation-consumption fields stayed zero.

`HighState.stamp()` was zero in both pairs; controller-side HighState arrival
wall time and age are recorded, but the source message does not provide a
usable simulation timestamp. The first instrumentation pair was discarded
because its newly added CSV header was misordered; it is not used above.

## Raw evidence

- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_baseline/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_terrain/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_terrain/wallclock_runtime_telemetry.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_baseline/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_terrain_noop/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_terrain_noop/wallclock_runtime_telemetry.csv`

No gait/WBC/planner/B0 threshold or parameter tuning was performed, and no
production behavior change was made. The next architectural fix should make
the wall-clock state/command handoff time-indexed or otherwise record/replay
the exact state schedule; this report does not claim B0 acceptance.
