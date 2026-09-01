# Order-108 C-006h exact-SHA lockstep equivalence verification — SUMMARY

**Status: PIVOT — PROTOCOL/TRACE CLEAN, AUTHORITATIVE CONTROLLER GATE FAILURE (CANARY)**

Date: 2026-09-01. Mode: REVIEW then VERIFY. Tested source SHA: **3273bd5797c743edf53d842e0f7945d48bcb4d24** (exact clean committed SHA; `git_dirty=false`, `origin/phase2-b1-b3 == 3273bd5`, no ahead/behind). Reviewer approval of 3273bd5 was given before this verification (Order-108 Phase B). Pre-registered manifest frozen externally at 2026-09-01T11:34:32+0800 before any run; no source/script/analyzer/contract/config file was modified after freeze. Every run manifest records `git_head=3273bd5…`, `git_dirty=false`, `simulator_sha256=52df3645…`, `controller_sha256=62bbbb3a…`, `scenario_sha256=12286418…` — all matching the frozen manifest.

## Canary (causal lockstep equivalence vs authoritative Order-101 wall-clock PASS pair-1 `045117`)

Run: `SIM_LOCKSTEP=1 bash example/cpp/scripts/run_phase2_b0_lockstep_pair.sh development 0` (domains baseline=222, terrain=223; Stage-C execution off, terrain shadow diagnostics on, LD_PRELOAD dds_base4000 preload, duration 40 s / wall timeout 75 s).

### Baseline (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_113510_baseline`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped at end, final_angle_deg=179.790).
- Lockstep trace: intervals=27611, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched(3) on every lockstep row (27610/27610), per-tick ack_state_seq==sim_tick_ms (0 mismatches), ack_cmd_seq==cmd_seq_at_ready exact arrival (0 zero, 0 > ready), no SIM_LOCKSTEP_FAIL_CLOSED, no TROT_LOCKSTEP_WRITER_FAIL_CLOSED.
- Command-seq cadence: exactly one command_seq increment per physics tick on 26682/27610 rows; residual delta 2–3 on 928 rows all confined to the first ~1196 rows (ticks 1846–4234 ms — pre-engagement wall-clock free-run plus DDS delivery backlog drain); from tick 4236 ms onward delta=1 exactly. **One controller update per tick achieved from the engagement handoff onward.**
- Controller time vs sim time: **NOT 1:1** — cmd_time runs 1.414× sim time in the lockstep segment (2.759 ms cmd advance vs 1.951 ms state advance per row) because `wall_clock_motion=1` overrides `motion_dt` with per-cycle wall dt. The Order-107 ~2× skew is reduced but not eliminated.
- Fixed 3 m/s analyzer: FAIL — `cycle_health_count=500`, `speed_median=3.4397 m/s`, `good_speed_window_s=44.608` (9.616..54.224), `stop_start_s=75.550`, `final_angle_deg=179.790`. The robot cruised healthily (body_angle_p95 roll 4.25°/pitch 2.71°, aerial_fraction 0.309) and flipped in the last ~5 gait cycles (health governor was already braking v_cmd 3.0→0.44 over cycles 490–500).
- Ground-truth/dynamics analyzers: PASS (p95 force-balance residual 13.75 N).
- Comparator (diagnostic): startup segment tracked exactly (dz p95 0.0000 m, droll 0.000°); lockstep segment dz p95 0.0810 m (>0.06), dpitch p95 3.535°, droll p95 5.031° (end-of-run flip drives max droll 181.7°) — FAIL, downstream of the controller flip, not the protocol.

### Terrain (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_113510_terrain`)

- Lifecycle: safety_status=1, completion_status=1 (hard posture limit; robot flipped, final_angle_deg=179.867).
- Lockstep trace: intervals=21665, violations=0, fail_closed=0, dt_ms=2, trigger=kAckMatched(3) on every lockstep row (21664/21664), per-tick ack_state_seq==sim_tick_ms (0 mismatches), ack_cmd_seq==cmd_seq_at_ready exact arrival (0 zero, 0 > ready).
- Command-seq cadence: delta=1 on 20602/21664 rows; delta 2–3 confined to the first 1074 rows; delta=1 exactly thereafter. Controller time 1.435× sim time (2.773 ms vs 1.932 ms per row).
- Fixed 3 m/s analyzer: FAIL — `cycle_health_count=389`, `speed_median=3.4379 m/s`, `good_speed_window_s=33.788` (9.444..43.232), `stop_start_s=59.740`, `final_angle_deg=179.867`.
- B0 analyzer: FAIL — `acceptance_status=FAIL`, `fixed_3mps_analyzer=false`, `paired_baseline_lifecycle=false` (safety_status=1). All contract/analyzer/lidar-only/orthogonality checks true; terrain_map_valid_fraction=0.9999, planner_updates=1478, deadline_misses=0.
- Ground-truth/dynamics analyzers: PASS (p95 13.70 N). Comparator (diagnostic): startup exact; lockstep dz p95 0.1107 m, dpitch p95 4.029°, droll p95 6.306° — FAIL.
- Publish/consumer/actuation counters: all 0 on every terrain row (Stage-C execution off as designed).

## Lockstep protocol finding (positive — Order-108 writer gate verified)

The Order-107 ~2× controller cadence is repaired at the per-tick scheduling level: from the engagement handoff onward exactly one controller update (one command_seq increment, one LowCmd publish, one exact {state_seq, command_seq} ack) occurs per physics tick; the protocol remains exact-pair clean (0 violations, no fail-closed, per-tick ack_state_seq==sim_tick_ms, ack_cmd_seq resolves to the exact arrival) on both members. The canary now runs dramatically healthier than Order-107 (baseline healthy cruise 44.6 s vs 11.7 s; flip moved from ~17 s sim to ~55 s sim; terrain from ~10 s to ~43 s) but still ends in the same controller posture-flip family. **The controller internal clock is still not sim-synchronous**: `wall_clock_motion=1` keeps `motion_dt` on per-cycle wall time (~2.76 ms per 2 ms sim tick), so cmd_time runs ~1.41–1.44× sim time.

## Order-108 pivot rule applied

Per the frozen Order-108 pivot rule (`WORKER_ORDERS.md` Order 108): the 1:1 per-tick writer schedule is proven (trace clean — zero protocol violations, exact-pair ack on every exchange, no fail-closed, delta=1 from handoff onward) yet the authoritative controller gate failed on both members — declared **Phase-1 control-law/contact robustness classification** and **stopped infrastructure work**. **No holdout pairs were run, no rerun, no replacement, no threshold/config/code/analyzer/contract edit.** B1 and thresholds were not touched. Required 3/3 was not attempted.

## Dev suites (evidence of coverage gap, not a substitute gate)

- `ctest --test-dir simulate/build`: 2/2 PASS (`test_lockstep`, `test_lockstep_sim`).
- `ctest --test-dir example/cpp/build`: 29/29 PASS (incl. `test_lockstep_writer_gate`).

## Permanent prior failures (listed separately; unchanged by this order)

- Order-101 C-006 pair-2 wall-clock baseline posture safety stop `roll=178.557 deg` — `docs/research/evidence/order101_c006/`.
- Order-102 C-006b pair-1 wall-clock baseline `stop_start=11.500 s / cycle_health_count=44 / speed_median=1.170566` — `docs/research/evidence/order102_c006b/`.
- Order-104 C-006d holdout r3 lockstep baseline `stop_start=11.652 s / cycle_health_count=45 / speed_median=1.250163` — `docs/ESCALATION/order104_c006d/`.
- Order-105 C-006e canary simulator fail-closed stale-ack race `violations=32` — `docs/research/evidence/order105_c006e/`.
- Order-106 C-006e canary ack-state-only fail-open (reviewer P2) fixed at 97c766a; controller posture flip at 97c766a — `docs/research/evidence/order106_c006e/`.
- Order-107 C-006g protocol-clean (exact-pair, 0 violations) controller posture flip at e65e155 (~2× controller clock) — `docs/research/evidence/order107_c006g/`.

These remain separate and permanent; this order's failure is a new, distinct observation: **one-update-per-tick protocol-clean writer gate with residual ~1.42× controller clock skew and an end-of-run posture flip under lockstep at 3273bd5**, same posture-stop family as Order-101 pair-2 / Order-106 / Order-107, reproduced independently on both members.

## Verdict

Canary **PIVOT** — writer cadence repair **verified** (one controller update per physics tick from handoff onward, exact {state_seq, command_seq} binding, zero violations, no fail-closed); **controller-time 1:1 gate NOT met** (cmd_time 1.414×/1.435× sim time); authoritative controller gates **FAIL** (posture flip on both members at exact SHA 3273bd5). B0 / holdout 3/3 / publish-consumer-actuation gates were not reached by design (stop rule). C-006 acceptance is **NOT established** at 3273bd5; per the pivot rule, the next order may change **controller behavior** (Phase-1 control-law/contact robustness, including the wall-clock motion-clock override residual) while preserving every B0 threshold/profile, and no further verification-infrastructure changes are allowed. No code behavior or acceptance threshold was changed. Rollback remains Stage-C flags off; previously verified SHAs and this tested SHA (3273bd5) are recorded. Do not advance C-007/B1 from this order.
