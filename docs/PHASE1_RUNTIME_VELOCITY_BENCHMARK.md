# Phase 1 runtime velocity benchmark

This fixture freezes the benchmark contract for runtime arbitrary forward-speed
commands. The fixed 3 m/s running-trot protocol remains a separate immutable
regression and continues to use its original analyzer.

Each profile is a single controller run. The command script is sampled online;
the controller must expose requested, shaped, applied, measured, acceleration,
jerk, gait parameters, and regime in diagnostics. The benchmark runner keeps
the validated running-trot plant parameters and does not enable terrain,
motion-event scripts, reactive events, or exploratory continuation.

Scenarios are:

- steps: 0 -> 1 -> 2 -> 3 -> 1 -> 0 m/s with 8 s plateaus.
- accel_1_to_3: 1 -> 3 m/s with sustained plateaus.
- brake_3_to_0: 3 -> 0 m/s with a sustained stop tail.
- ramp: continuous 0 -> 3 -> 0 m/s.
- varying: non-integer 0.6, 1.4, 2.3, 2.8 m/s commands.

analyze_phase1_velocity.py reports tracking error, applied-command limiting,
shaper acceleration/jerk, gait schedule, attitude, and run status. A strict
pass requires controller, safety, quality, completion, and analyzer statuses
to be zero. This does not change the legacy 3 m/s analyzer or its thresholds.
