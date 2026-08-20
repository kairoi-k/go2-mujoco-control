# Automatic environment sensing delivery

This note records the reproducible acceptance package for the Go2 WBC-full controller at commit `5158ff2`.

## Verified automatic sensing

- Baseline: height-map valid rate 1.000, maximum map age 0.020 s, no obstacle event, and zero obstacle contact.
- Physical obstacle: `scene_reactive_obstacle.xml`; `obstacle_left` is detected at controller time 5.040 s (MuJoCo state tick 6.692 s), with 0.040 s post-warmup latency, target `vy=0.45 m/s` and yaw `0.18 rad/s`, lateral shift 0.519 m, and zero contact count/force.
- Physical impact: an 0.8 m/s simulator push at state tick 8.002 s is detected at 8.006 s (4 ms), followed by `emergency_stop` at 8.506 s (0.5 s delay); velocity jump 0.792 m/s, maximum roll/pitch 0.062/0.162 rad, WBC residual `1.6963e-5`.
- Priority preemption: while the physical obstacle response is active, the same 0.8 m/s push is detected at 7.006 s, 4 ms after the push, and preempts `obstacle_left` before its eight-second response window expires; the final sequence is `none -> obstacle_left -> impact -> emergency_stop` with zero obstacle contact.

All final runs have controller, safety, quality, analysis, ground-truth, dynamics, and completion status `0`; strict analyzers pass.

## Unified-transition evidence

The 49 directed transitions in `go2_reactive_transition_matrix_final_2026-08-20` use one `MotionEventResponseLayer` and one gait/WBC/MPC plant. They are scripted transition coverage, not a claim of autonomous perception for every token. The priority-preemption run adds the sensor-driven safety ordering that the matrix alone cannot prove.

## Explicit capability boundary

The automatic path currently consumes the simulator height-map and state topics. A friction-only change to `mu=0.01` did not create measurable slip in the tested WBC gait, so no low-friction event is claimed; this is retained as a negative acceptance result rather than a fabricated success. Real-robot deployment still needs sensor-driver integration and threshold calibration.

## Reproduction and evidence

Use `example/cpp/tools/analyze_auto_environment.py`, `analyze_auto_impact.py`, and `analyze_auto_priority.py` to regenerate the strict reports from the raw experiment directories. Curated JSON/Markdown reports, source analyzers, controller documentation, and GUI MP4 demos are in the OneDrive delivery folder. Raw experiment CSV/log directories remain local evidence and are intentionally ignored by Git.
