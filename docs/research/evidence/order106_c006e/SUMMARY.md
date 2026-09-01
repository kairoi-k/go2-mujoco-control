# Order-106 C-006e exact-SHA causal lockstep verification — SUMMARY

**Status: NOT ACCEPTED (STOPPED ON AUTHORITATIVE GATE FAILURE — CANARY)**

Date: 2026-09-01. Mode: REVIEW then VERIFY. Tested source SHA: **97c766a7340494625967b0583d9a495ef5130e86** (exact clean committed SHA; `git_dirty=false`, `origin/phase2-b1-b3 == 97c766a`, no ahead/behind). Reviewer approval of 97c766a was given before this verification (Order-106 Phase B). Pre-registered manifest frozen externally at 2026-09-01T09:03:00+0800 before any run; no source/script/analyzer/contract/config file was modified after freeze. Every run manifest records `git_head=97c766a…`, `git_dirty=false`, `simulator_sha256=4ba25e94…`, `controller_sha256=93185864…`, `scene_sha256=12286418…` — all matching the frozen manifest.

## Canary (causal lockstep vs authoritative Order-101 wall-clock PASS pair-1 `045117`)

Run: `SIM_LOCKSTEP=1 bash example/cpp/scripts/run_phase2_b0_lockstep_pair.sh development 0` (domains baseline=222, terrain=223; Stage-C execution off, terrain shadow diagnostics on, LD_PRELOAD dds_base4000 preload, duration 40 s / wall timeout 75 s).

### Baseline (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_090304_baseline`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped at the very end, final_angle_deg=179.9884).
- Lockstep trace: intervals=20225, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched on every lockstep row, per-tick ack_state_seq==sim_tick_ms, ack_cmd_seq=0 (side-channel removed as designed).
- Fixed 3 m/s analyzer: FAIL — `cycle_health_count=443`, `speed_median=3.348 m/s`, `good_speed_window_s=31.652`, `stop_start_s=67.284`, `final_angle_deg=179.9884`. Cruise was healthy (3.35 m/s median) until the end-of-run flip.
- Ground-truth/dynamics analyzers: PASS (force-balance p95 15.68 N ≤ 20 N).
- Comparator (diagnostic): PASS — startup dz p95 0.0066 m; lockstep segment dz p95 0.0344 m, dpitch p95 3.923 deg, droll p95 4.246 deg. Baseline tracked the reference through the run; the flip is end-of-run.

### Terrain (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_090304_terrain`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped early).
- Lockstep trace: intervals=4104, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched on every lockstep row, per-tick ack_state_seq==sim_tick_ms, ack_cmd_seq=0.
- Fixed 3 m/s analyzer: FAIL — `stop_start_s=14.250`, `cycle_health_count=64<100`, `good_window_s=0.830<20`, `speed_median=2.3937`, `roll_p95=179.8181`, `pitch_p95=62.2431`.
- B0 analyzer: FAIL — `fixed_3mps_analyzer=false`, `paired_baseline_lifecycle=false` (safety_status=1).
- Ground-truth/dynamics analyzers: PASS (force-balance p95 12.11 N ≤ 20 N).
- Comparator: FAIL — lockstep segment dz p95 0.3009 m, dpitch p95 28.029 deg, droll p95 179.556 deg (flipped).
- Publish/consumer/actuation counters: all 0 on every terrain row (`terrain_plan_published`, `terrain_plan_consumed`, `terrain_gait_target_overrides`, `terrain_mpc_plan_consumed`, `terrain_has_stage_c_timing`, `wbc_terrain_planned_contact_mask`, `terrain_execution_adapter_updates`).

## Lockstep mechanism finding (positive)

The Order-105 fail-closed race is **fixed**: no `SIM_LOCKSTEP_FAIL_CLOSED` in either simulator.log; both members ran the full protocol with 0 violations, exact dt=2 ms, every exchange completed on `kAckMatched`, and per-tick ack state_seq matched the published tick. The canary now fails the **controller-side authoritative gates** (posture flip) instead of the simulator-side mechanism.

## Stop rule applied

The canary baseline lifecycle/analyzer failure is the first authoritative gate failure. Per the frozen manifest stop rule: verification stopped immediately; **no** holdout pairs were run, **no** rerun, **no** replacement, **no** threshold/config/code/analyzer/contract edit. B1 and thresholds were not touched. Required 3/3 was not attempted.

## Dev suites (evidence of coverage gap, not a substitute gate)

- `ctest --test-dir simulate/build`: 2/2 PASS (`test_lockstep`, `test_lockstep_sim`).
- `ctest --test-dir example/cpp/build`: 28/28 PASS.

## Permanent prior failures (listed separately; unchanged by this order)

- Order-101 C-006 pair-2 wall-clock baseline posture safety stop `roll=178.557 deg` — `docs/research/evidence/order101_c006/`.
- Order-102 C-006b pair-1 wall-clock baseline `stop_start=11.500 s / cycle_health_count=44 / speed_median=1.170566` — `docs/research/evidence/order102_c006b/`.
- Order-104 C-006d holdout r3 lockstep baseline `stop_start=11.652 s / cycle_health_count=45 / speed_median=1.250163` — `docs/ESCALATION/order104_c006d/`.
- Order-105 C-006e canary simulator fail-closed stale-ack race `violations=32` — `docs/research/evidence/order105_c006e/`.

These remain separate and permanent; this order's failure is a new, distinct failure class (controller posture flip under lockstep at 97c766a), most closely related to the Order-101 pair-2 posture-stop family, observed on both canary members.

## Verdict

Canary **FAIL** — all subsequent gates (B0, holdout 3/3, publish/consumer/actuation as a pass gate) were not reached and holdout pairs were not attempted. C-006 acceptance is **NOT established** at 97c766a. The lockstep mechanism race from Order-105 is fixed (0 violations, no fail-closed), but the controller flips on both members under this protocol at exact SHA. No code behavior or acceptance threshold was changed. Rollback remains Stage-C flags off; previously verified SHAs and this tested SHA (97c766a) are recorded. Do not advance C-007/B1 from this order.
