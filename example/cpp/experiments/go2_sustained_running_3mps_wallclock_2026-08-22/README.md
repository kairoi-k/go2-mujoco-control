# Go2 sustained 3 m/s running-trot validation

## Scope

This is the independent running gait, not the validated diagonal-trot sprint.
It keeps the same ID-WBC + SRBD-MPC plant and uses the `running-trot` phase
pattern: diagonal pairs remain coordinated, the opposite pair is offset by
0.46 phase, and the lower duty factor creates a measurable aerial interval.

## Reproduction

```bash
bash example/cpp/scripts/run_sustained_running.sh --headless
python3 example/cpp/tools/analysis/analyze_sustained_running.py \
  example/cpp/experiments/_runs/<run-name>
```

The release reference is wall-clock phase, period `0.14 s`, duty `0.44`, step
length `0.50 m`, foot lift `0.20 m`, target `3.0 m/s`, and automatic braking.
The runner records the exact command, binary hashes, environment, contact
ground truth, and controller status in `_runs/<run-name>`.

## Acceptance semantics

The strict analyzer requires all lifecycle/status checks to be zero; the
running-trot pattern and `--wall-clock-motion` to be present in the recorded
argv; at least 20 s continuously inside 2.90–3.80 m/s; median speed at least
2.90 m/s; roll/pitch P95 below 5 degrees; base height P01/P99 within
0.33–0.40 m; every foot's swing-height P95 at least 0.08 m; aerial fraction
0.15–0.40; diagonal-pair contact synchrony at least 0.75; at least 30% of
samples with two contacts and at most 10% with three or four; all-feet-low
fraction at most 5%; and a completed brake/四足 WBC hold with stop-tail speed
P95 below 0.10 m/s.

## Repeated evidence

| run | peak / median (m/s) | good window (s) | roll / pitch P95 (deg) | aerial | pair sync | stop-tail P95 (m/s) |
|---|---:|---:|---:|---:|---:|---:|
| `natural_run_wallclock_p14d44s50_release_r1` | 3.713 / 3.236 | 60.730 | 2.869 / 2.363 | 0.289 | 0.817 | 0.004 |
| `natural_run_wallclock_p14d44s50_r2b` | 3.742 / 3.233 | 61.266 | 2.840 / 2.460 | 0.289 | 0.808 | 0.003 |
| `natural_run_wallclock_p14d44s50_r3` | 3.732 / 3.238 | 61.476 | 2.901 / 2.468 | 0.291 | 0.810 | 0.004 |

All three runs passed the analyzer with all status fields zero, complete
braking, and final speed below `0.0021 m/s`. The first visual replay was
checked at stand-up, acceleration, sustained running, and stopped stages; it
shows long-stride aerial running rather than the previous low-duty small-step
appearance.
