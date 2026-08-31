# Order-101 C-006 formal B0 summary

Date: 2026-09-01. Source under test: `7861bf98cd32f454b3da6783a09b5571f4cfe037` (clean). The pre-registered sample was three serial fixed pairs using the unchanged `run_phase2_b0_fixed_pair.sh development 0` entrypoint, DDS Base=4000 preload, baseline domain 222 and terrain domain 223. Terrain used sensor-only mode with planner and `TROT_TERRAIN_SHADOW_DIAGNOSTICS=1`; Stage-C execution was not requested and all publish/consumer/actuation counters were required to remain zero.

## Stop-gated outcome

| Pair | baseline lifecycle | baseline fixed analyzer | terrain B0 | terrain fixed analyzer | result |
|---|---|---|---|---|---|
| 1 (`045117`) | all zero | PASS | PASS | PASS | PASS |
| 2 (`045423`) | **FAIL: safety=1, completion=1** | **FAIL** | PASS | PASS | **STOP** |
| 3 | not run | not run | not run | not run | not run |

Pair 2 baseline stopped at the existing hard posture limit (`roll=178.557 deg`) after 60 cycle-health records and did not record controlled-stop completion. Its terrain member completed normally. This is a run-local baseline lifecycle/safety failure; it is not evidence of terrain actuation. The available evidence cannot distinguish nonlinear wall-clock jitter from an infrastructure/startup disturbance without a rerun, which is prohibited by the stop rule.

Formal result: **FAIL / incomplete; required 3/3 was not established**. Verification stopped immediately on the first authoritative failure. No thresholds, analyzer semantics, configuration, source, or behavior code were changed, and no B1 simulation was run.

## Authoritative evidence

Both completed terrain members had `controller_status=0`, `safety_status=0`, `quality_status=0`, `analysis_status=0`, `ground_truth_status=0`, `dynamics_status=0`, and `completion_status=0`; fixed 3 m/s analyzer `validation=PASS`; and terrain B0 `acceptance_status=PASS` except that the pair-2 overall B0 result correctly failed its paired-baseline-lifecycle check. Pair 1 terrain had 39,009 rows and pair 2 had 39,000 rows. Planner updates were 2,865 and 2,724, respectively; planner deadline misses were zero. Shadow records were 2,840 and 2,708, with maximum latency 4,429.69 us and 4,358.228 us (deadline 5,000 us); `shadow_output_consumed` was zero.

On both terrain members, `terrain_plan_published`, `terrain_plan_consumed`, `terrain_gait_target_overrides`, `terrain_mpc_plan_consumed`, `terrain_has_stage_c_timing`, and `wbc_terrain_planned_contact_mask` were zero for every row. Raw, fused, and robust measured-contact masks were separately logged. All four measured-FK legs were valid on every row, with source exactly `state_q+base_pose_fk`.

## Diagnostic-only paired differences

The B0 analyzer reports independent wall-clock paired fields separately and they are not acceptance gates under frozen contract v1.2. Pair 1 max differences included gait period 0.020 s, duty 0.099610, requested acceleration 4.291845 m/s2, and WBC velocity target 1.324851 m/s. Pair 2 values were 0.200 s, 0.500000, 11.894068 m/s2, and 3.000000 m/s. These remain diagnostic and were not rewritten as PASS.

The machine-readable manifest, pre-registration, and Wilson calculation are in this directory. Wilson is diagnostic only because the pre-registered n=3 sample was stopped at n=2: 1/2 PASS, 95% Wilson interval [0.094531, 0.905469].
