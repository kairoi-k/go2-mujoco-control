# B0 sensor-rate canary

This record covers the B-class sensing-isolation change later committed as
`2fefd71ce59e54fbd4a2a3fd0cf9a34a1d9c7ca9`. The canary processes were
launched before that commit from a dirty worktree containing exactly the same
seven-line diff; they are diagnostic evidence, not formal holdout evidence.

The change bounds the simulated lidar ray/map production path at 20 Hz:
`TerrainLidarLoop` and map publication use a 50 ms period. It does not alter
controller commands, footholds, contact schedule, MPC, WBC, safety limits, or
any acceptance threshold. It stays within the Phase 2 initial 20--50 Hz
terrain-map budget.

Development canary evidence:

| Scenario | Terrain run | Baseline run | Result |
|---|---|---|---|
| steps | `phase2_b0_development_steps_r0_20260826_111430_terrain` | `phase2_b0_development_steps_r0_20260826_111430_baseline` | terrain B0 PASS; Phase1 PASS |
| accel_1_to_3 | `phase2_b0_development_accel_1_to_3_r0_20260826_111842_terrain` | `phase2_b0_development_accel_1_to_3_r0_20260826_111842_baseline` | terrain B0 PASS; Phase1 PASS |
| brake_3_to_0 | `phase2_b0_development_brake_3_to_0_r0_20260826_112049_terrain` | `phase2_b0_development_brake_3_to_0_r0_20260826_112049_baseline` | terrain B0 PASS; baseline Phase1 failed |

The brake baseline failure is retained evidence, not covered by the terrain
success: `shaper_accel_continuity=0.02016404` exceeded the frozen 0.02
limit. The terrain member passed that gate. All three terrain members had
zero planner deadline misses, no plan publication/consumption/actuation, and
valid lidar provenance/map telemetry.

This canary is sufficient to proceed to a clean implementation freeze and one
formal B0 epoch-22 holdout. It does not authorize B1; B0 remains blocked until
that holdout passes.
