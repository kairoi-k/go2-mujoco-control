# Automatic environment sensing delivery
This note records the current, reproducible acceptance package for the Go2 WBC-full controller.
## Verified automatic sensing
- Baseline: height-map valid rate 1.000, max age 0.020 s, no obstacle event, zero obstacle contact.
- Physical obstacle: scene `scene_reactive_obstacle.xml`; `obstacle_left` detected at 5.040 s, latency 0.040 s after warmup, lateral shift 0.449 m, contact count/force 0/0. A later reappearance is treated as a separate event only after the obstacle has been clear for at least 250 ms.
- Physical impact: simulator push 0.8 m/s at state tick 8.002 s; `impact` at 8.004 s (2 ms), `emergency_stop` at 8.504 s (0.5 s delay), velocity jump 0.800 m/s, max roll/pitch 0.1245/0.1863 rad, WBC residual 1.6964e-5.
- All three runs have controller, safety, quality, analysis, ground-truth, dynamics and completion status 0; strict analyzers pass.
## Unified-transition evidence
The 49 directed transitions in `go2_reactive_transition_matrix_final_2026-08-20` use one MotionEventResponseLayer and one gait/WBC/MPC plant. All 49 pass the same sequence, timing, continuity, priority and safety gates. This is scripted transition coverage, not a claim of autonomous perception for every token.
## Limitations stated explicitly
The automatic perception path currently consumes the simulator height-map and state topics. Low-friction is not claimed as an automatically detected sensor event unless measurable slip is present; its response and negative case are covered separately. Real-robot deployment still needs sensor-driver integration and threshold calibration.
## Reproduction
Use `tools/analyze_auto_environment.py` for baseline/obstacle runs and `tools/analyze_auto_impact.py` for physical-impact runs. Raw CSV/log directories are local evidence and are intentionally ignored by Git; the curated MP4s and reports are in the OneDrive delivery folder.
## OneDrive contents
`videos/auto_obstacle_left.mp4` is the GUI obstacle-avoidance demo.
`videos/auto_physical_impact_emergency_stop.mp4` is the GUI physical-impact/emergency-stop demo.
`reports/` contains strict JSON/Markdown reports plus the 49-transition metrics.
`code/` contains the relevant analyzer scripts and controller documentation.
