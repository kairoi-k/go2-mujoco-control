# Order-103 C-006c sim-time lockstep verification infrastructure

Date: 2026-09-01. Working tree HEAD `55118a9bbf8970c198daf08770f0c6d1d41255db`
(this commit; the Order-103 delta is uncommitted until this evidence is
recorded). Scope: **verification infrastructure only** — the simulator and
harness. Controller, gait/MPC/WBC, command profile, binaries/config, B0
contract, analyzers and thresholds are unchanged (controller binary SHA
`46033a1ca918ab5e143aa64124b93a2637f4fa7c4060cdcc6c5e508cde363f39` is
identical to the authoritative Order-101 pair-1 run; `git status --porcelain`
shows only `simulate/*` and `example/cpp/scripts/run_trot.sh` plus the new
harness/evidence files).

## What was built

`simulate/src/lockstep.h` — a DDS/MuJoCo-free `Coordinator` state machine that
makes the sim/controller exchange deterministic:

- **Startup is byte-identical to the wall-clock runner.** Physics free-runs
  and the bridge publishes at 1000 Hz exactly as before. The controller's
  existing lifecycle boundary (natural-settle + world-reference capture,
  observable as the first lowcmd arrival) completes the ready barrier on both
  sides. Startup safety/lifecycle failures remain authoritative harness
  failures and are never skipped.
- **Explicit handoff.** On the first controller command the barrier completes;
  the physics thread then advances exactly one `mj_step` (T0 -> T0+dt) and the
  coordinator records the barrier row at T0 with the command sequence. The
  first lockstep state publish is T0+dt, so the trace and ground-truth tick
  sequences are continuous across the handoff (no duplicate / missing tick).
  A startup watchdog fails closed if no command arrives within
  `SIM_LOCKSTEP_BARRIER_TIMEOUT_S`.
- **Frozen intervals.** After the handoff the physics thread advances exactly
  one `mj_step` per completed exchange: the bridge publishes the state, waits
  for the next controller command (the step+publish completes inside one 500 Hz
  controller write period, so the arrival is guaranteed computed from the
  just-published state; loopback DDS delivery < 1 ms), applies it, and only
  then grants the step. Controller clock and sim clock stay 1:1.
- **Fail closed.** Barrier/exchange/step timeouts and tick-sequence anomalies
  write `SIM_LOCKSTEP_FAIL_CLOSED reason=...` and `_Exit(1)`.
- **Trace** `lockstep_trace.csv`: `sim_tick_ms,step_index,phase,
  cmd_seq_at_publish,cmd_seq_at_ready,exchange_wait_us,publish_wall_us,
  exchange_trigger,violations` + a `#summary` line. Recorded per interval.
- Flag defaults OFF (`--lockstep`); the wall-clock runner is unchanged when
  off (verified below). `run_trot.sh` forwards `SIM_LOCKSTEP=1` /
  `SIM_LOCKSTEP_TRACE` and checks the trace (exact dt, zero violations) as a
  run gate.

Tests: `simulate/src/tests/test_lockstep.cpp` (unit) and
`simulate/src/tests/run_lockstep_sim_tests.sh` (integration) wired into
`simulate/CMakeLists.txt` via `enable_testing()`. All 2/2 ctest pass.

## Canary (equivalence vs authoritative wall-clock PASS)

Replayed the Order-101 pair-1 authoritative config (duration 40, wall timeout
75, domains 222/223, terrain sensor-only + shadow diagnostics, `LD_PRELOAD`
dds_base4000 preload) with `SIM_LOCKSTEP=1`:
`_runs/phase2_b0_lockstep_development_fixed_3mps_r0_20260901_062231`.

| member | lifecycle | fixed 3 m/s analyzer | B0 | lockstep trace |
|---|---|---|---|---|
| baseline | all 0 | PASS (median 3.244 m/s, good-window 61.14 s, 504 cycles) | n/a | intervals=38836, violations=0, dt=2 ms |
| terrain | all 0 | PASS (median 3.229 m/s) | **PASS** | intervals=38865, violations=0, dt=2 ms |

Segment comparison vs the authoritative Order-101 pair-1 run
(`compare_lockstep_canary.py`, p95 gate dz<=0.06 m, angle<=6 deg):

- **startup segment** (t in [0, handoff]): identical — max |dz| <= 0.0000 m,
  max |droll/dpitch| <= 0.001 deg (handoff at 1.714 s baseline / 1.754 s
  terrain).
- **lockstep segment** (handoff..end): baseline dz p95 0.009 m, roll p95
  3.10 deg, pitch p95 2.68 deg; terrain dz p95 0.010 m, roll p95 3.72 deg,
  pitch p95 3.03 deg. Worst-case deviations are transient (stop braking
  roll ~7 deg at t~75.8 s, one touchdown pitch ~6 deg at t~13.9 s), reported
  as diagnostic.

## Pre-registered hashes (canary, before the 3 serial pairs)

- git_head `55118a9bbf8970c198daf08770f0c6d1d41255db`, git_dirty=true
  (delta recorded here)
- simulator_sha256 `deac717eb0666c946152bfc2e9f0bb389e433b3b3e39c2eebebeb5308d9b3684`
- controller_sha256 `46033a1ca918ab5e143aa64124b93a2637f4fa7c4060cdcc6c5e508cde363f39`
  (unchanged vs Order-101)
- scene_sha256 `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`
- canary trace hashes: baseline `fc1c7813...` (full `sha256sum` in run dir),
  terrain `9c866bfe...`; ground truth `7876da81...` / `34ee60f0...`

## 3 serial lockstep fixed pairs (holdout, Stage-C execution off, shadow on)

All ran serially with the frozen config above. **3/3 authoritative PASS.**

| pair | baseline lifecycle | baseline analyzer | terrain lifecycle | terrain analyzer | terrain B0 | trace (violations) |
|---|---|---|---|---|---|---|
| holdout r1 (`063119`) | all 0 | PASS (3.247 m/s) | all 0 | PASS (3.234 m/s) | PASS | 38787/38906, 0 |
| holdout r2 (`063424`) | all 0 | PASS (3.245 m/s) | all 0 | PASS (3.257 m/s) | PASS | 38711/38846, 0 |
| holdout r3 (`063727`) | all 0 | PASS (3.249 m/s) | all 0 | PASS (3.242 m/s) | PASS | 38817/38898, 0 |

## Commands

- `ctest` (simulate/build): 2/2 pass — `test_lockstep` (unit), `test_lockstep_sim`
  (fail-closed watchdog + flag-off equivalence).
- Flag-off equivalence: no `--lockstep` -> normal "Unitree DDS bridge ready",
  no LOCKSTEP marker, no trace file, physics stepping, SIGTERM stop; and the
  wall-clock 20 s smoke run reproduced the authoritative run's timeline.
- Fail-closed: no controller with `SIM_LOCKSTEP_BARRIER_TIMEOUT_S=3` ->
  `SIM_LOCKSTEP_FAIL_CLOSED reason=ready barrier timeout`, exit 1,
  `#summary intervals=0 violations=1 fail_closed=1`.
- Canary + 3 holdout pairs via `example/cpp/scripts/run_phase2_b0_lockstep_pair.sh`.

## Residual risks

- Startup segment uses the existing wall-clock path, so its command/state
  jitter remains (bounded; measured identical to the reference within
  0.001 deg / 1 mm over the 1.7 s startup).
- The 1:1 exchange freshness relies on loopback DDS delivery < ~1.5 ms
  (nominal < 0.1 ms). A pathological delivery stall would show as a
  controller-visible tick-gap or fail-closed timeout and is recorded in the
  trace; none occurred in 8 runs (4 pairs).
- WSL wall-clock robustness is not claimed by lockstep runs; the exchange
  removes command/state races but not scheduler stalls of the sim itself.
