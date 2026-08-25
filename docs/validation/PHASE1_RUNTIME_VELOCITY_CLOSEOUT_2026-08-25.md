# Phase 1 runtime arbitrary-velocity closeout

Date: 2026-08-25
Branch: feature/phase1-runtime-arbitrary-velocity-clean
HEAD: 07cbc7efdd17b9960897e277f401654c46410e48
Base implementation: 048eb1b5aae019fc6fad26f956c0aeeef8d9b816

## Scope

Phase 1 was evaluated as one online controller run per profile. The runtime path is
requested profile -> jerk/acceleration-limited shaper -> continuous gait schedule
-> SRBD-MPC velocity reference -> 18-DoF ID-WBC. The fixed 3 m/s protocol is
kept separate and uses analyze_sustained_running.py.

## Frozen benchmark evidence

The five profiles were each run three valid times at this HEAD. Every valid run
returned controller_status=0, safety_status=0, quality_status=0,
completion_status=0, and analysis_status=0. The new analyzer reports the
additional metrics below without changing the legacy status gate.

| scenario | valid runs | max profile-error P95 | max steady error | max excursion | max settling |
|---|---:|---:|---:|---:|---:|
| steps | 3 | 0.358 m/s | 0.348 m/s | +0.438 m/s | 7.852 s |
| accel_1_to_3 | 3 | 0.384 m/s | 0.367 m/s | +0.440 m/s | 9.362 s |
| brake_3_to_0 | 3 | 1.424 m/s | 0.501 m/s | -0.163 m/s | 1.302 s |
| ramp | 3 | 0.387 m/s | 0.136 m/s | +0.546 m/s | 1.788 s |
| varying | 3 | 0.387 m/s | 0.409 m/s | +0.449 m/s | 7.964 s |

All varying-command runs contain the non-integer targets 0.6, 1.4, 2.3,
and 2.8 m/s. The requested profile reproduction error was at most 5.1e-10 m/s.
Across valid runs, roll/pitch P95 were at most 2.37/3.18 degrees, full SRBD
and ID-WBC status fractions were 1.0, plan-valid fraction was 1.0, and
low-friction evidence was zero. The analyzer also records touchdown errors,
support-foot speed, contact structure, solver budget, torque estimates and
stop tails. One brake run has a motor-estimate torque peak of 246.5 Nm during
startup; the WBC shadow command peak was 45.43 Nm and saturation fraction was
0.14 percent. This is retained as a diagnostic outlier, not suppressed.

Fresh valid run directories:
- example/cpp/experiments/_runs/phase1_completion_20260825/
- example/cpp/experiments/_runs/phase1_repeat_20260825/
Each valid directory contains data.csv, run_metadata.txt, run_manifest.json,
and phase1_analysis.json. The two failed startup attempts have no data.csv and
remain as failure evidence: ramp_20260825_090200 and
varying_20260825_090553.

## Immutable 3 m/s baseline

The exact run_sustained_running.sh protocol was repeated three times on this
HEAD and analyze_sustained_running.py returned PASS each time:
phase1_baseline_reval_r1, phase1_baseline_reval_r2, and
phase1_baseline_reval_r3. Median speed was 3.228356, 3.238650, and 3.226834
m/s; stop-tail P95 was 0.004316, 0.003068, and 0.003414 m/s. The accepted
3 m/s analyzer and its thresholds were not changed.

## Build and reproducibility

In WSL, simulate and example/cpp both built after linking the existing
/home/che/.mujoco/mujoco-3.3.6 environment into the clean worktree.
CTest passed 26/26. Re-run the five profiles with
example/cpp/scripts/run_phase1_velocity_benchmark.sh and analyze with
example/cpp/scripts/analyze_phase1_velocity.py --profile <profile>.
Re-run the immutable baseline with example/cpp/scripts/run_sustained_running.sh
and analyze_sustained_running.py.

Phase 1 status: COMPLETE. This closeout does not start, modify, or claim
acceptance for terrain/Phase 2. The feature branch is PR-ready; main remains
untouched and no force-push or tag rewrite was performed.

