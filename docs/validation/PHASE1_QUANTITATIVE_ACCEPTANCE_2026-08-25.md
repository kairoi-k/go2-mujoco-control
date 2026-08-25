# Phase 1 quantitative acceptance follow-up

Date: 2026-08-25
Branch: feature/phase1-quantitative-acceptance
Controller base: origin/main at a1d4e294092a39c8e649eda6f285ea2cfe01b05d

## Scope and semantics

This is the quantitative follow-up to the merged Phase 1 runtime arbitrary-velocity implementation. It changes only the Phase 1 analyzer and evidence; it does not modify controller code, MuJoCo physics, scenes, legacy 3 m/s analyzer semantics, or terrain WIP.

The legacy status gate remains unchanged: controller_status, safety_status, quality_status, completion_status, and analysis_status must all be zero. With `--require-quantitative`, the analyzer also requires the frozen quantitative gate below. The result is PASS only when both gates pass.

No new simulation was used for this re-evaluation. The analyzer was run against the 15 existing valid dynamic runs: five frozen profiles, three valid runs per profile. The two failed startup attempts remain preserved as failure evidence.

## Frozen quantitative gate

The thresholds are encoded in `example/cpp/scripts/analyze_phase1_velocity.py` and are evaluated without changing the old status meanings.

| scenario | tracking P95 | steady-state max | positive excursion | negative excursion lower bound | settling max |
|---|---:|---:|---:|---:|---:|
| steps | 0.40 m/s | 0.40 m/s | 0.50 m/s | -0.25 m/s | 8.2 s |
| accel_1_to_3 | 0.42 m/s | 0.40 m/s | 0.50 m/s | -0.25 m/s | 10.0 s |
| brake_3_to_0 | 1.50 m/s | 0.55 m/s | 0.05 m/s | -0.20 m/s | 1.5 s |
| ramp | 0.42 m/s | 0.18 m/s | 0.60 m/s | -0.20 m/s | 2.0 s |
| varying | 0.42 m/s | 0.45 m/s | 0.50 m/s | -0.40 m/s | 8.2 s |

Common gates: requested profile reproduction <=1e-6 m/s; shaped-to-measured P95 <=0.45 m/s; shaper acceleration <=1.25 m/s2; jerk <=4.20 m/s3; acceleration sample-to-sample change <=0.02 m/s3; roll and pitch P95 <=4 degrees; contact loss <=0.25; single-contact fraction <=0.45; touchdown error <=0.18 m x and <=0.07 m y; torque saturation fraction <=0.003; slip evidence fraction ==0; solver, SRBD, ID-WBC, and footstep-plan validity ==1.0; solver-budget fraction >=0.80; minimum base height >=0.28 m; stop-tail P95 <=0.05 m/s except the non-stopping accel_1_to_3 profile.

requested-to-shaped error is reported separately because it is intentionally nonzero during acceleration/jerk limiting. Its maximum is 2.99998 m/s in the 3-to-0 brake profile; this is shaping behavior, not a measured-velocity tracking failure. shaped-to-measured error is the closed-loop tracking quantity and is gated explicitly.

## Re-evaluation result

All 15 valid runs returned quantitative PASS and legacy strict PASS: 5 scenarios x 3 runs, 0 failures.

| scenario | runs | max tracking P95 | max steady error | max excursion | max settling |
|---|---:|---:|---:|---:|---:|
| steps | 3 | 0.358 m/s | 0.348 m/s | +0.438 m/s | 7.852 s |
| accel_1_to_3 | 3 | 0.384 m/s | 0.367 m/s | +0.440 m/s | 9.362 s |
| brake_3_to_0 | 3 | 1.424 m/s | 0.501 m/s | -0.163 m/s | 1.302 s |
| ramp | 3 | 0.387 m/s | 0.136 m/s | +0.546 m/s | 1.788 s |
| varying | 3 | 0.387 m/s | 0.409 m/s | +0.449 m/s | 7.964 s |

Across the 15 runs, shaped-to-measured P95 was at most 0.410 m/s, shaper acceleration at most 1.2 m/s2, jerk at most 4.0 m/s3, low-friction/slip evidence was zero, and torque saturation fraction was at most 0.002507. The previously observed 246.5 Nm motor-estimate startup peak remains retained as a diagnostic outlier; it is not suppressed or used as a saturation gate. The WBC shadow peak in that run was 45.43 Nm.

The immutable fixed 3 m/s baseline remains separate. Its three original analyzer runs remain PASS with median speeds 3.228356, 3.238650, and 3.226834 m/s; its analyzer thresholds and semantics were not changed.

## Reproduction

For each existing run:

```bash
python3 example/cpp/scripts/analyze_phase1_velocity.py <run-dir> \
  --profile example/cpp/configs/phase1_velocity_<scenario>.csv \
  --require-quantitative \
  --json-out <run-dir>/phase1_quantitative_analysis.json
```

Evidence roots:

- `example/cpp/experiments/_runs/phase1_completion_20260825/`
- `example/cpp/experiments/_runs/phase1_repeat_20260825/`

The quantitative JSON files are ignored experiment evidence; the analyzer and this acceptance record are the reviewable source changes. Terrain remains isolated and is not part of this milestone.
