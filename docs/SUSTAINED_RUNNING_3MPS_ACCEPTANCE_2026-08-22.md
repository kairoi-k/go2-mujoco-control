# Go2 3 m/s sustained running-trot acceptance (2026-08-22)

This branch adds a separate, independently repeatable running gait. It does
not replace the diagonal-trot sprint reference: both use the same
`--wbc-full` ID-WBC + SRBD-MPC plant, while this profile changes only the
locomotion reference to `running-trot`.

## Release entry point

```bash
bash example/cpp/scripts/run_sustained_running.sh --headless
python3 example/cpp/tools/analysis/analyze_sustained_running.py \
  example/cpp/experiments/_runs/<run-name>
```

The default profile is `period=0.14 s`, `duty=0.44`, `step=0.50 m`,
`foot_lift=0.20 m`, target `3.0 m/s`, wall-clock phase, and automatic
locomotion-plant braking followed by a four-contact WBC hold.

## Strict acceptance

The analyzer requires a continuous 20 s window in 2.90–3.80 m/s, median speed
at least 2.90 m/s, roll/pitch P95 below 5 degrees, base height P01/P99 in
0.33–0.40 m, swing-height P95 at least 0.08 m, aerial fraction 0.15–0.40,
diagonal-pair synchrony at least 0.75, two-contact samples at least 30%,
three/four-contact samples at most 10%, all-feet-low samples at most 5%, and a
completed stop hold whose speed P95 is below 0.10 m/s. Lifecycle, dynamics,
contact, quality, and safety status fields must all be zero.

## Repeated evidence

| run | peak / median (m/s) | good window (s) | roll / pitch P95 (deg) | aerial | pair-sync minimum | stop-tail P95 (m/s) |
|---|---:|---:|---:|---:|---:|---:|
| `natural_run_wallclock_p14d44s50_release_r1` | 3.713 / 3.236 | 60.730 | 2.869 / 2.363 | 0.289 | 0.817 | 0.004 |
| `natural_run_wallclock_p14d44s50_r2b` | 3.742 / 3.233 | 61.266 | 2.840 / 2.460 | 0.289 | 0.808 | 0.003 |
| `natural_run_wallclock_p14d44s50_r3` | 3.732 / 3.238 | 61.476 | 2.901 / 2.468 | 0.291 | 0.810 | 0.004 |

All three runs passed the gait-specific analyzer with zero status failures and
final speeds below 0.0021 m/s. Raw `_runs/` data are disposable and ignored;
the protocol and summary are retained here. The final visual replay is in
OneDrive at
`收件箱/Go2_Sustained_Running_3mps_WBC_Full_2026-08-22/go2_running_stand_accel_sustain_stop_final.mp4`;
SHA256 `4468D7375118ACE389F21CD6CA110D31E9E635546DD2DFCF32457198C2860B6F`.
