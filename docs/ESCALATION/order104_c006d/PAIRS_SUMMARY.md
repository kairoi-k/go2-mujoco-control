# Order-104 C-006d holdout pairs evidence (3 pre-registered serial lockstep fixed pairs)

Tested SHA: 2b3bc5d634e998c77fc14486665a4df6d052f58c (clean). All pairs ran serially with the frozen pre-run manifest configuration (Stage-C execution off, terrain shadow diagnostics on, LD_PRELOAD dds_base4000 preload). Every member manifest records `git_dirty=false`, `simulator_sha256=deac717e…`, `controller_sha256=46033a1c…`, `scene_sha256=12286418…`.

Preregistered domains: r1 baseline 183 / terrain 203; r2 baseline 184 / terrain 204; r3 baseline 185 / terrain 205.

## Results table

| pair | baseline lifecycle | baseline analyzer | baseline trace | terrain lifecycle | terrain analyzer | terrain B0 | terrain trace |
|---|---|---|---|---|---|---|---|
| r1 (`070337`) | all 0 | PASS (3.234072 m/s, 503, 60.830 s) | 38782 rows, 0 viol, 2 ms | all 0 | PASS (3.244055 m/s, 499, 60.994 s) | PASS | 38888 rows, 0 viol, 2 ms |
| r2 (`070644`) | all 0 | PASS (3.241655 m/s, 504, 61.408 s) | 38727 rows, 0 viol, 2 ms | all 0 | PASS (3.247080 m/s, 497, 60.984 s) | PASS | 38883 rows, 0 viol, 2 ms |
| r3 (`070951`) | all 0 | **FAIL** | 6712 rows, 0 viol, 2 ms | all 0 | PASS (3.233304 m/s, 495, 60.840 s) | PASS | 38962 rows, 0 viol, 2 ms |

`viol` = lockstep trace `violations`; all traces `fail_closed=0 dt_ms=2`.

## r3 baseline failure detail (authoritative)

`sustained_running_analysis.txt`: `validation=FAIL`, `stage2_window_s=4.300..11.650`, `stop_start_s=11.652`, `cruise_window_s=6.300..9.650`, `cycle_health_count=45` (<100), `good_speed_window_s=0.000000 (nan..nan)` (<20), `speed_median_mps=1.250163`, `speed_p05_mps=0.886750`, `aerial_fraction=0.1439`.

Lifecycle statuses all 0 (no harness/safety trip). `controller.log` ends: health-governor cycle-quality rejections (hard limits active, tau_est 45.43 N·m FL_calf, q_error ~1.0-1.14 rad), then `High-speed stop: brake complete; entering WBC four-contact hold` / `WBC four-contact hold complete; finished in WBC stance`. The controller stopped itself at ~11.65 s (auto-brake path) instead of running the full 40 s.

Lockstep mechanism was normal on this run: `#summary intervals=6712 violations=0 fail_closed=0 dt_ms=2`; barrier at 1.67 s; exchange_wait_us p50 1631 / p99 2973 / max 43292; no tick anomaly. Exchange-wait statistics are comparable to the passing members (r1 max 37.6 ms, r2 max 14.2 ms), so this is not a lockstep-exchange stall.

Signature match: Order-102 pair-1 wall-clock baseline failure was `stop_start=11.500 s`, `cycle_health_count=44`, `speed_median=1.170566` — the r3 baseline failure (11.652 s / 45 / 1.2502) is the same inherited controller-side failure class, now observed under lockstep. As recorded in Order-102, this is an inherited stochastic Phase-1 controller failure; no code fix is justified in this order and none was made. WSL wall-clock robustness is not claimed by lockstep runs.

## No publish / consumer / actuation (all four terrain members)

`terrain_plan_published=0, terrain_plan_consumed=0, terrain_gait_target_overrides=0, terrain_mpc_plan_consumed=0, terrain_has_stage_c_timing=0, wbc_terrain_planned_contact_mask=0` on every row (checked for `070014_terrain`, `070337_terrain`, `070644_terrain`, `070951_terrain`).

## Stop rule

The r3 baseline fixed-analyzer failure is the first authoritative gate failure. Per the frozen manifest stop rule and Order-104, verification stopped immediately: no threshold/config/code/analyzer/contract edit, no outcome-selected rerun, no replacement. Result: 2/3 holdout pairs PASS; required 3/3 not established.
