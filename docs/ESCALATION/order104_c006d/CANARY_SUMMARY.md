# Order-104 C-006d canary evidence (exact-SHA lockstep equivalence)

- Tested SHA: 2b3bc5d634e998c77fc14486665a4df6d052f58c (clean).
- Lockstep canary run: `phase2_b0_lockstep_development_fixed_3mps_r0_20260901_070014` (baseline domain 222, terrain domain 223).
- Reference: authoritative Order-101 wall-clock PASS pair-1 `phase2_b0_development_fixed_3mps_r0_20260901_045117`.
- Comparator: `compare_lockstep_canary.py` with frozen defaults (dz p95 <= 0.06 m, angle p95 <= 6 deg). No tolerance changes.

## Lifecycle / analyzer / B0 gates (from run_manifest.json + sustained_running_analysis.txt + b0_analyzer.json)

| member | statuses | validation | speed_median | cycle_health | good_window_s | B0 |
|---|---|---|---|---|---|---|
| baseline | 0/0/0/0/0/0/0/0 | PASS | 3.235414 | 504 | 60.496000 (13.526..74.022) | n/a |
| terrain | 0/0/0/0/0/0/0/0 | PASS | 3.241897 | 500 | 61.002000 (13.030..74.032) | PASS |

Lockstep trace discipline (`#summary` lines):

- baseline: `intervals=38931 violations=0 fail_closed=0 dt_ms=2`
- terrain: `intervals=38909 violations=0 fail_closed=0 dt_ms=2`

## Segment comparison output (verbatim, `compare_lockstep_canary.py`)

### baseline
```
member=baseline handoff_tick_s=2.216 end_s=79.782
startup: samples=1108 dz(p50/p95/max)=0.0000/0.0037/0.0098 m dpitch=0.000/0.189/0.222 deg droll=0.000/0.003/0.008 deg
lockstep: samples=38784 dz(p50/p95/max)=0.0034/0.0132/0.0819 m dpitch=0.943/2.838/5.121 deg droll=1.043/3.259/6.289 deg
RESULT: PASS (tolerances dz<=0.06 m, angle<=6.0 deg)
```

### terrain
```
member=terrain handoff_tick_s=1.742 end_s=79.560
startup: samples=871 dz(p50/p95/max)=0.0000/0.0000/0.0000 m dpitch=0.000/0.000/0.000 deg droll=0.000/0.000/0.000 deg
lockstep: samples=38910 dz(p50/p95/max)=0.0029/0.0091/0.0211 m dpitch=0.797/2.740/5.095 deg droll=0.882/3.111/6.705 deg
RESULT: PASS (tolerances dz<=0.06 m, angle<=6.0 deg)
```

## No publish / consumer / actuation (every terrain row)

`terrain_plan_published=0, terrain_plan_consumed=0, terrain_gait_target_overrides=0, terrain_mpc_plan_consumed=0, terrain_has_stage_c_timing=0, wbc_terrain_planned_contact_mask=0`.

## Verdict

Canary **PASS**: all authoritative gates (lifecycle, fixed 3 m/s analyzer, B0) and the frozen-tolerance segment comparison passed for both members.
