# Order-107 C-006g exact-SHA causal lockstep verification — SUMMARY

**Status: PIVOT — PROTOCOL INFRA COMPLETE, CONTROLLER AUTHORITATIVE GATE FAILURE (CANARY)**

Date: 2026-09-01. Mode: REVIEW then VERIFY. Tested source SHA: **e65e155b0a5fb5789337b76b1b7201735e626223** (exact clean committed SHA; `git_dirty=false`, `origin/phase2-b1-b3 == e65e155`, no ahead/behind). Reviewer approval of e65e155 was given before this verification (Order-107 Phase B). Pre-registered manifest frozen externally at 2026-09-01T09:58:00+0800 before any run; no source/script/analyzer/contract/config file was modified after freeze. Every run manifest records `git_head=e65e155…`, `git_dirty=false`, `simulator_sha256=52df3645…`, `controller_sha256=062c397b…`, `scenario_sha256=12286418…` — all matching the frozen manifest.

## Canary (causal lockstep vs authoritative Order-101 wall-clock PASS pair-1 `045117`)

Run: `SIM_LOCKSTEP=1 bash example/cpp/scripts/run_phase2_b0_lockstep_pair.sh development 0` (domains baseline=222, terrain=223; Stage-C execution off, terrain shadow diagnostics on, LD_PRELOAD dds_base4000 preload, duration 40 s / wall timeout 75 s).

### Baseline (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_095853_baseline`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped, final_angle_deg=171.448).
- Lockstep trace: intervals=9707, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched(3) on every lockstep row (9706/9706), per-tick ack_state_seq==sim_tick_ms (0 mismatches), ack_cmd_seq==cmd_seq_at_ready exact arrival (0 zero, 0 > ready).
- Fixed 3 m/s analyzer: FAIL — `cycle_health_count=211`, `speed_median=3.311 m/s`, `good_speed_window_s=11.716`, `stop_start_s=34.928`, `final_angle_deg=171.448`. Cruise was healthy (3.31 m/s median) until the end-of-run flip.
- Ground-truth/dynamics analyzers: PASS.
- Comparator (diagnostic): startup dz p95 0.0078 m, dpitch 0.186 deg, droll 0.003 deg; lockstep segment dz p95 0.2474 m, dpitch p95 4.146 deg, droll p95 156.652 deg (flipped) — FAIL, downstream of the controller flip, not the protocol.

### Terrain (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_095853_terrain`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped, final_angle_deg=178.619).
- Lockstep trace: intervals=4789, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched(3) on every lockstep row (4788/4788), per-tick ack_state_seq==sim_tick_ms (0 mismatches), ack_cmd_seq==cmd_seq_at_ready exact arrival (0 zero, 0 > ready).
- Fixed 3 m/s analyzer: FAIL — `cycle_health_count=82<100`, `good_window_s=2.260<20`, `speed_median=2.776`, `stop_start_s=16.870`, `final_angle_deg=178.619`.
- B0 analyzer: FAIL — `fixed_3mps_analyzer=false`, `paired_baseline_lifecycle=false` (safety_status=1). All contract/analyzer/lidar-only/orthogonality checks true; terrain_map_valid_fraction=0.9998, planner_updates=395, deadline_misses=0.
- Ground-truth/dynamics analyzers: PASS.
- Comparator: FAIL (diagnostic; flipped).
- Publish/consumer/actuation counters: all 0 on every terrain row (Stage-C execution off as designed).

## Lockstep protocol finding (positive — Order-107 fix verified)

The Order-106 reviewer-P2 fail-open causality is **fixed**: ack now carries the exact pair `{state_seq, lockstep_command_seq}`; on every exchange the trigger is `kAckMatched`, `ack_cmd_seq` resolves to the EXACT LowCmd arrival (`== cmd_seq_at_ready`, never ahead, never 0), and `ack_state_seq==sim_tick_ms` on every row. Both members ran the full protocol with 0 violations, no `SIM_LOCKSTEP_FAIL_CLOSED`, exact dt=2 ms. The canary now fails the **controller-side authoritative gates** (posture flip) instead of the simulator-side mechanism.

## Order-107 pivot rule applied

Per the frozen Order-107 pivot rule (`WORKER_ORDERS.md` Order 107): the full `{state_seq, command_seq}` canary has **zero protocol violations** yet an **authoritative controller gate failed** on both members — declared **verification infrastructure complete** and **stopped protocol work**. **No holdout pairs were run, no rerun, no replacement, no threshold/config/code/analyzer/contract edit.** B1 and thresholds were not touched. Required 3/3 was not attempted.

## Dev suites (evidence of coverage gap, not a substitute gate)

- `ctest --test-dir simulate/build`: 2/2 PASS (`test_lockstep`, `test_lockstep_sim`).
- `ctest --test-dir example/cpp/build`: 28/28 PASS.

## Permanent prior failures (listed separately; unchanged by this order)

- Order-101 C-006 pair-2 wall-clock baseline posture safety stop `roll=178.557 deg` — `docs/research/evidence/order101_c006/`.
- Order-102 C-006b pair-1 wall-clock baseline `stop_start=11.500 s / cycle_health_count=44 / speed_median=1.170566` — `docs/research/evidence/order102_c006b/`.
- Order-104 C-006d holdout r3 lockstep baseline `stop_start=11.652 s / cycle_health_count=45 / speed_median=1.250163` — `docs/ESCALATION/order104_c006d/`.
- Order-105 C-006e canary simulator fail-closed stale-ack race `violations=32` — `docs/research/evidence/order105_c006e/`.
- Order-106 C-006e canary ack-state-only fail-open (reviewer P2) fixed here; controller posture flip at 97c766a — `docs/research/evidence/order106_c006e/`.

These remain separate and permanent; this order's failure is a new, distinct observation: protocol-clean (exact pair, zero violations) controller posture flip under lockstep at e65e155, same posture-stop family as Order-101 pair-2 / Order-106, reproduced independently on both members.

## Verdict

Canary **PIVOT** — protocol verification infrastructure **complete** (exact `{state_seq, command_seq}` binding, zero violations, deterministic); controller authoritative gates **FAIL** (posture flip on both members at exact SHA e65e155). B0 / holdout 3/3 / publish-consumer-actuation gates were not reached by design (pivot rule). C-006 acceptance is **NOT established** at e65e155; per the pivot rule, the next order must repair the inherited Phase-1 controller robustness under deterministic scheduling **without weakening B0**. No code behavior or acceptance threshold was changed. Rollback remains Stage-C flags off; previously verified SHAs and this tested SHA (e65e155) are recorded. Do not advance C-007/B1 from this order.
