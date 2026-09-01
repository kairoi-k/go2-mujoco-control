# Order-105 C-006e exact-SHA causal lockstep verification — SUMMARY

**Status: NOT ACCEPTED (STOPPED ON AUTHORITATIVE GATE FAILURE — CANARY)**

Date: 2026-09-01. Mode: REVIEW then VERIFY. Tested source SHA: **1b29974eb58919686186641041d8e9bf728c123c** (exact clean committed SHA; `git_dirty=false`, `origin/phase2-b1-b3 == 1b29974`, no ahead/behind). Reviewer approval of 1b29974 was given before this verification (Order-105 Phase B). Pre-registered manifest frozen externally at 2026-09-01T08:08:39+0800 before any run; no source/script/analyzer/contract/config file was modified during or after verification. Every run manifest records `git_head=1b29974…`, `git_dirty=false`, `simulator_sha256=61857a5f…`, `controller_sha256=d05a5884…`, `scene_sha256=12286418…` — all matching the frozen manifest.

## Canary (causal lockstep vs authoritative Order-101 wall-clock PASS pair-1 `045117`)

Run: `SIM_LOCKSTEP=1 bash example/cpp/scripts/run_phase2_b0_lockstep_pair.sh development 0` (domains baseline=222, terrain=223; Stage-C execution off, terrain shadow diagnostics on, LD_PRELOAD dds_base4000 preload, duration 40 s / wall timeout 75 s).

### Baseline (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_080900_baseline`)

- Lifecycle: not reached (fail-closed before first step). Trace: barrier tick 2300, `#summary intervals=1 violations=32 fail_closed=1 dt_ms=2`.
- `simulator.log`: `SIM_LOCKSTEP_FAIL_CLOSED reason=stale ack for older state state_seq=2300 command_seq=3 violations=32`.

### Terrain (run `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_080900_terrain`)

- Trace: barrier tick 1922; first lockstep exchange clean (tick 1924, ack_state_seq=1924, ack_cmd_seq=4, violations=0); then `#summary intervals=2 violations=32 fail_closed=1 dt_ms=2`.
- `simulator.log`: `SIM_LOCKSTEP_FAIL_CLOSED reason=stale ack for older state state_seq=1924 command_seq=5 violations=32`.

## Root-cause diagnosis (simulator-scope defect in the approved commit)

The controller ack adapter publishes `ack{state_seq, command_seq}` after every LowCmd write on the controller's own cadence. The simulator validates each arriving ack against `published_state_seq_` (the tick of the state it most recently published) and ignores only acks whose `command_seq <= barrier_seq_` (startup-phase acks). The ack for the just-published state is written by the controller before it has received the next state, and its DDS delivery is delayed until after the simulator has already advanced to the next lockstep tick (`OnStatePublished`/`WaitForExchange` of the following interval). That in-flight ack then satisfies `command_seq > barrier_seq_` but references the previous tick, so `OnAckReceived` classifies it as `kViolationAckStale` and fails closed. Both members reproduced this independently (baseline at the first exchange, terrain at the second exchange after one clean exchange), i.e. it is a deterministic structural race, not a flake. Exact-SHA/clean environment was confirmed on both members, so this is not environmental. The unit tests (simulate ctest 2/2) script one-command-per-state acks without real DDS latency and do not cover this path; the dev suites pass while the real system fails.

## Stop rule applied

The canary baseline fail-closed is the first authoritative gate failure. Per the frozen manifest stop rule: verification stopped immediately; **no** holdout pairs were run, **no** rerun, **no** replacement, **no** threshold/config/code/analyzer/contract edit. B1 and thresholds were not touched. Required 3/3 was not attempted.

## Dev suites (evidence of coverage gap, not a substitute gate)

- `ctest --test-dir simulate/build`: 2/2 PASS (`test_lockstep`, `test_lockstep_sim`).
- `ctest --test-dir example/cpp/build`: 28/28 PASS.

## Publish / consumer / actuation

Not evaluable: both canary members fail-closed before producing terrain data; no terrain row was generated, so no publish/consumer/actuation counters could be checked. This gate was not reached.

## Permanent prior failures (listed separately; unchanged by this order)

- Order-101 C-006 pair-2 wall-clock baseline posture safety stop `roll=178.557 deg` — `docs/research/evidence/order101_c006/`.
- Order-102 C-006b pair-1 wall-clock baseline `stop_start=11.500 s / cycle_health_count=44 / speed_median=1.170566` — `docs/research/evidence/order102_c006b/`.
- Order-104 C-006d holdout r3 lockstep baseline `stop_start=11.652 s / cycle_health_count=45 / speed_median=1.250163` — `docs/ESCALATION/order104_c006d/`.

These remain separate and permanent; this order's failure is a new, distinct failure class (causal ack handshake first-exchange race).

## Verdict

Canary **FAIL** — all subsequent gates (fixed 3 m/s analyzer, B0, per-tick trace matching, publish/consumer/actuation) were not reached and holdout pairs were not attempted. C-006 acceptance is **NOT established** at 1b29974. No code behavior or acceptance threshold was changed. Rollback remains Stage-C flags off; the previously verified SHA and this tested SHA (1b29974) are both recorded. Do not advance C-007/B1 from this order.
