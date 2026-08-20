# Automatic environment sensing delivery

This note records the reproducible source-aware acceptance package for the Go2 WBC-full controller at code commit `239f940` (documentation may be committed separately).

## Verified automatic sensing

- Baseline: height-map valid rate 1.000, maximum map age 0.020 s, no obstacle event, and zero obstacle contact.
- Physical obstacle: `scene_reactive_obstacle.xml`; `obstacle_left` is detected at controller time 5.048 s (MuJoCo state tick 6.322 s), with 0.046 s post-warmup latency, target `vy=0.45 m/s` and yaw `0.18 rad/s`, lateral shift 0.460 m, and zero contact count/force; source is `sensor`.
- Physical impact: an 0.8 m/s simulator push at state tick 8.002 s is detected at 8.004 s (2 ms), followed by `emergency_stop` at 8.504 s (0.5 s delay); velocity jump 0.801 m/s, maximum roll/pitch 0.075/0.162 rad, WBC residual `1.6964e-5`; impact source is `sensor`.
- Priority preemption: while the physical obstacle response is active, the same 0.8 m/s push is detected at state tick 7.004 s (2 ms after the push) and preempts `obstacle_left`; the final sequence is `none -> obstacle_left -> impact -> emergency_stop` with zero obstacle contact, sources `sensor -> sensor -> scheduled`.

All final runs have controller, safety, quality, analysis, ground-truth, dynamics, and completion status `0`; strict analyzers pass.

The CSV `event_source` field makes the evidence auditable: automatic obstacle and impact transitions are required to be `sensor`; the follow-up emergency stop is `scheduled` and the absorbing hold is `safety_latch`.

## Physical low-friction acceptance

The former global-friction-only trial is intentionally retained as a negative boundary: changing the whole floor coefficient without producing measurable support-foot motion did not justify a sensor event. The positive acceptance is now a physical patch scene, `unitree_robots/go2/scene_low_friction_patch.xml`: the robot walks from a normal plane onto a collidable patch with `mu=0.0001`, while no `--friction-time` script is used. Support-foot world kinematics feed a leaky evidence window; duplicate DDS state ticks do not erase elapsed evidence, and low-friction detection has hysteresis, re-arm suppression after other events, and a minimum current support-foot speed gate. The acceptance uses a deliberately faster probe gait to make the friction loss physically observable rather than claiming that every nominal gait must slip.

The repeated strict runs `go2_auto_environment_low_friction_patch_sensor_fast_v14_2026-08-21` and `...fast_v15...` both pass: `none -> low_friction(sensor) -> none`, detection occurs after entering the patch, evidence peaks at `0.2420` and `0.1571`, target `vx` is reduced, and posture/solver/quality statuses remain zero. Re-run either with `example/cpp/tools/analyze_auto_low_friction.py`; the analyzer rejects scripted friction changes, missing physical patch geometry, wrong event source, out-of-patch detections, insufficient support evidence, and unsafe numerical/posture results.

## Unified-transition evidence

The 49 directed transitions in `go2_reactive_transition_matrix_final_2026-08-20` use one `MotionEventResponseLayer` and one gait/WBC/MPC plant. They are scripted transition coverage, not a claim of autonomous perception for every token. The priority-preemption run adds the sensor-driven safety ordering that the matrix alone cannot prove.

## Explicit capability boundary

The automatic path currently consumes the simulator height-map and state topics. The detector now also computes world-frame support-foot kinematics from the high-state/leg model: sustained motion of at least two contacting support feet can produce `slip` or `low_friction`, and the path is unit-tested. A friction-only change to `mu=0.01` still did not create a strict sensor event in the tested WBC gait, so no low-friction event is claimed; this is retained as a negative acceptance result rather than a fabricated success. Real-robot deployment still needs sensor-driver integration and threshold calibration.

## Reproduction and evidence

Use `example/cpp/tools/analyze_auto_environment.py`, `analyze_auto_impact.py`, `analyze_auto_priority.py`, and `analyze_auto_low_friction.py` to regenerate the strict reports from the raw experiment directories. Curated JSON/Markdown reports, source analyzers, controller documentation, and GUI MP4 demos are in the OneDrive delivery folder. Raw experiment CSV/log directories remain local evidence and are intentionally ignored by Git.
