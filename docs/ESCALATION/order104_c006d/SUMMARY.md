# Order-104 C-006d exact-committed-SHA lockstep evidence — SUMMARY

Date: 2026-09-01. Mode: REVIEW then VERIFY. Tested source SHA: **2b3bc5d634e998c77fc14486665a4df6d052f58c** (exact clean committed SHA, `git_dirty=false`, `origin/phase2-b1-b3 == 2b3bc5d`, no ahead/behind). Reviewer approval of 2b3bc5d was given before this verification.

All runs are **lockstep** (`SIM_LOCKSTEP=1`, `run_phase2_b0_lockstep_pair.sh`), Stage-C execution off and terrain shadow diagnostics on, identical frozen B0 configuration to the authoritative Order-101 wall-clock fixed pair (duration 40, wall timeout 75, LD_PRELOAD dds_base4000 preload). Pre-run manifest was frozen at 2026-09-01T06:59:00+0800 (sha256 `ec98fd902c54547b7e6af5a3119005a01b40a09e3699c6b25b8ead615c70c238`) before any run; no source/script/analyzer/contract/config file was modified during verification. Every run manifest records `git_head=2b3bc5d…`, `git_dirty=false`, `simulator_sha256=deac717e…`, `controller_sha256=46033a1c…` (identical to the authoritative Order-101 pair-1 controller binary), `scene_sha256=12286418…`.

## Canary (equivalence vs authoritative Order-101 wall-clock PASS pair-1 `045117`)

Run: `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_070014` (domains 222/223).

| member | lifecycle | fixed 3 m/s analyzer | B0 | trace |
|---|---|---|---|---|
| baseline | all 0 | PASS (median 3.235 m/s, 504 cycles, good-window 60.50 s) | n/a | intervals=38931, violations=0, dt=2 ms |
| terrain | all 0 | PASS (median 3.242 m/s, 500 cycles) | **PASS** | intervals=38909, violations=0, dt=2 ms |

Segment comparison vs Order-101 pair-1 wall-clock run (`compare_lockstep_canary.py`, frozen tolerances dz p95<=0.06 m, angle p95<=6 deg):

- baseline: startup (t<2.216 s) dz p95 0.0037 m / roll p95 0.003 deg / pitch p95 0.189 deg; lockstep segment dz p95 0.0132 m, roll p95 3.259 deg, pitch p95 2.838 deg → **PASS**
- terrain: startup (t<1.742 s) max |dz|<=0.0000 m, max angle <=0.000 deg; lockstep segment dz p95 0.0091 m, roll p95 3.111 deg, pitch p95 2.740 deg → **PASS**

All terrain publish/consumer/actuation counters zero (`terrain_plan_published/consumed`, `terrain_gait_target_overrides`, `terrain_mpc_plan_consumed`, `terrain_has_stage_c_timing`, `wbc_terrain_planned_contact_mask` = 0 on every row).

## 3 pre-registered serial lockstep fixed pairs (holdout; Stage-C execution off, terrain shadow on)

Domains preregistered in the frozen manifest: r1 183/203, r2 184/204, r3 185/205 (baseline/terrain). All runs retained; no replacement, no rerun.

| pair | baseline lifecycle | baseline analyzer | terrain lifecycle | terrain analyzer | terrain B0 | trace (violations, dt) |
|---|---|---|---|---|---|---|
| holdout r1 (`070337`) | all 0 | PASS (3.234 m/s, 503) | all 0 | PASS (3.244 m/s, 499) | PASS | 38782/38888, 0, 2 ms |
| holdout r2 (`070644`) | all 0 | PASS (3.242 m/s, 504) | all 0 | PASS (3.247 m/s, 497) | PASS | 38727/38883, 0, 2 ms |
| holdout r3 (`070951`) | all 0 | **FAIL** | all 0 | PASS (3.233 m/s, 495) | PASS | 6712/38962, 0, 2 ms |

Holdout r3 baseline FAIL — authoritative fixed 3 m/s analyzer `validation=FAIL`: `cycle_health_count=45<100`, `good_window_s=0.000<20.000`, `speed_median=1.2502 m/s`, `aerial_fraction=0.1439`, `stop_start_s=11.652`. Lifecycle statuses were all zero (no safety/quality trip in the harness); the controller's own health governor degraded the reference and executed the high-speed auto-brake at cycle ~45, then completed the WBC four-contact hold. The failure signature (stop ~11.5-11.7 s, cycle count ~44-45, speed median ~1.17-1.25 m/s) matches the inherited baseline failure class already recorded permanently in Order-102 pair-1 (stop_start 11.500 s, cycle_health_count 44, speed_median 1.1706). Lockstep mechanism on r3 baseline was sound: trace exact dt=2 ms, 0 violations, fail_closed=0, exchange_wait p50 1631 us / p99 2973 us / max 43 ms (comparable to r1 37 ms / r2 14 ms). The failure is controller-side, not a lockstep-exchange anomaly.

## Stop rule applied

First authoritative gate failure (r3 baseline fixed analyzer) → verification **stopped immediately**. No threshold/config/code/analyzer/contract edit, no outcome-selected rerun, no replacement. Terrain members of all four pairs and all six baseline/terrain counters remain PASS/zero as recorded above.

## Result

- Exact-SHA lockstep equivalence canary: **PASS** (all frozen authority gates + segment comparison).
- Formal 3/3 holdout: **2/3 pairs PASS — required 3/3 not established → C-006 BLOCKED**.
- Wilson (diagnostic only, stopped sample): 2/3, 95% CI [0.207660, 0.938508].
- Order-101 (1/2) and Order-102 (0/1) **wall-clock** failures remain separate and permanent; lockstep acceptance proves deterministic functional non-regression only, and WSL wall-clock robustness is not claimed.

## Tests (full suite, at tested SHA)

- `example/cpp/build`: ctest 28/28 PASS (incl. gait/planner/MPC/WBC/QP tests).
- `simulate/build`: ctest 2/2 PASS (`test_lockstep`, `test_lockstep_sim`).

## Artifacts

- `docs/ESCALATION/order104_c006d/PRERUN_MANIFEST.json` (verbatim frozen pre-run manifest)
- `docs/ESCALATION/order104_c006d/CANARY_SUMMARY.md`
- `docs/ESCALATION/order104_c006d/PAIRS_SUMMARY.md`
- `docs/ESCALATION/order104_c006d/WILSON.json`
- `docs/ESCALATION/order104_c006d/FORMAL_MANIFEST.json`
- Run artifacts (gitignored): `example/cpp/experiments/_runs/phase2_b0_lockstep_development_fixed_3mps_r0_20260901_070014_*` and `phase2_b0_lockstep_holdout_fixed_3mps_r{1,2,3}_20260901_07*_*`
- `example/cpp/experiments/_runs/ESCALATION_LOG.md` (appended entry)
