# Go2 sustained 3 m/s sprint — wall-clock phase validation

## Purpose

Validate that the current `--wbc-full` diagonal-trot sprint can accelerate,
hold a 3 m/s-class speed for a sustained interval, and finish with a
controlled stop without attitude or contact failures.

## Reproduction

The release entry point is:

```bash
bash example/cpp/scripts/run_sustained_sprint.sh --headless
```

The script defaults to the validated wall-clock motion phase (`--wall-clock-motion`),
3 m/s target, 0.14 s period, 0.44 duty, 0.50 m step length, 0.20 m foot lift,
45 Nm torque limit, and automatic braking. Override `SUSTAINED_SPRINT_NAME`,
`SUSTAINED_SPRINT_DOMAIN_ID`, `SUSTAINED_SPRINT_DURATION_S`, or
`SUSTAINED_SPRINT_WALL_TIMEOUT_S` for a new run.

## Why wall-clock phase is part of the release profile

The earlier release accumulated motion phase from sampled state ticks. Missed
state samples could make the gait phase drift relative to the simulator clock,
which caused repeatability failures. The wall-clock option keeps phase tied to
elapsed controller time while preserving the same gait/WBC plant.

## Acceptance criteria

- at least 10 s in the 2.90–3.80 m/s good-speed window;
- peak speed at least 3.0 m/s;
- whole-run roll/pitch P95 below 5 degrees;
- safety, controller, quality, dynamics, and contact checks all zero;
- automatic controlled stop with final speed below 0.10 m/s.

## Repeated validation

Four independent 40 s controller runs passed the same strict analyzer:

| run | peak / median (m/s) | good window (s) | angle P95 window / whole (deg) | final speed (m/s) | stop |
|---|---:|---:|---:|---:|---|
| `hs_wallclock_r1` | 3.715 / 3.275 | 53.222 | 2.019 / 2.117 | 0.002 | pass |
| `hs_wallclock_r2` | 3.701 / 3.262 | 32.996 | 2.084 / 2.154 | 0.002 | pass |
| `hs_wallclock_r3` | 3.697 / 3.269 | 63.255 | 2.168 / 2.217 | 0.002 | pass |
| `hs_wallclock_r4` | 3.702 / 3.260 | 34.324 | 2.308 / 2.311 | 0.002 | pass |

The release-default smoke run `hs_default_wallclock_r1` also passed:
3.718 m/s peak, 3.266 m/s median, 35.648 s good window, 2.182 degree
whole-run angle P95, 0.002 m/s final speed, and all status checks zero.

## Demonstration artifact

The complete replay is delivered in OneDrive:
`收件箱/Go2_Sprint_3mps_WBC_Full_2026-08-22_v2/go2_sprint_stand_accel_sustain_stop_final.mp4`.
It covers stage 0 (stand-up), stage 1 (settle), stage 2 (acceleration and
sustained sprint), and stage 3 (automatic braking and stationary finish).
