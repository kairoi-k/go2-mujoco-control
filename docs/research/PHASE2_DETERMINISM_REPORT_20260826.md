# Phase 2 B0 Determinism and Realtime Verification Report

Date: 2026-08-26. Scope is B0 flat regression and verification infrastructure only. This branch does not advance B1/B2/B3, terrain crossing, terrain foothold/contact policy, gains, shaper limits, safety thresholds, or main.

## Result

The observed divergence is an architecture/timing-boundary problem, not a terrain locomotion policy result and not seeded MuJoCo randomness. The realtime path has no simulation-tick barrier: physics progress, DDS delivery, controller wakeup, and wall-clock motion time are scheduler-dependent.

The deterministic functional runner is single-threaded, tick-driven, and has no DDS, wall-clock sleeps, callbacks, or worker scheduling. It executes one controller update followed by one mj_step per tick. In accel_1_to_3, brake_3_to_0, and varying, for both phase1 and terrain_sensor_only, 5/5 data files are byte-identical. Every group has one unique SHA-256 and no first divergence; maximum parsed population variance is 8.08e-28 (aggregation roundoff).

The realtime path remains non-deterministic, now with a reproducible diagnosis rather than a statistical PASS/FAIL guess. In all six 5-run groups, the first scheduling divergence is at controller_tick=1 in state_tick_s. The first trajectory-visible divergence is velocity_command_measured_mps at the same controller tick. The first consumed simulation tick across the six groups is 1164..1700 for run r1.

## Provenance and isolation

- base and verified origin/research/phase2-stage-b-20260825: d77a16b9ed2c914de96899e599f254032148c873
- origin/main and merge-base at startup: 71d0e9ba7ca1097e840fe878aa30207f6f63600d
- branch: research/phase2-determinism-20260826
- worktree: /home/che/dev/go2-mujoco-control-determinism
- initial status: clean
- handoff SHA-256: 252df564d50a55a83f22b6461155d46057d4f9dec7a44e5cc1803a1a7771498d
- required lock: /tmp/go2_mujoco_experiment.lock
- isolated builds: example/cpp/build-determinism and simulate/build-determinism
- seed: 1; MuJoCo dt: 0.002 s
- all benchmark workloads and analysis were run under the exclusive lock
- raw evidence: example/cpp/experiments/_determinism_runs/

## Root cause and evidence

1. simulate/src/main.cc, PhysicsLoop, sleeps/yields and computes elapsedCPU/elapsedSim from wall-clock timing. Catch-up/resynchronization can execute a different number of mj_step calls before a controller sample. Thus the same initial state does not imply the same simulation tick at a controller wakeup.

2. simulate/src/unitree_sdk2_bridge.h, RobotBridge::run, publishes LowState asynchronously from the current MuJoCo state. DDS callbacks update a latest-state buffer; the 500 Hz writer consumes whichever callback has arrived when it wakes. LowState tick is derived from mj_data_->time, while the added callback sequence is delivery order, not a publisher timestamp. There is no controller-tick-to-simulation-tick barrier.

3. example/cpp/trot/trot_experiment_lifecycle.cpp uses a sleep_until 500 Hz writer. example/cpp/trot/trot_experiment_control.cpp, MotionClockStep, uses steady_clock wall_dt when --wall-clock-motion is active. That dt advances running_time_ and profile/event/shaper/gait timing, so scheduling jitter changes controller state even with identical command/config/seed.

4. TerrainLidarLoop and the planner worker add wall-clock threads, mutex/queue/condition-variable activity, and observer contention. terrain_sensor_only consumes no terrain plan/contact/foothold actuation; its remaining effect is scheduling/state freshness, not terrain policy.

The actual realtime chain is: PhysicsLoop mj_step -> bridge state copy/publication -> DDS callback -> latest state snapshot -> shaper/gait/Raibert/SRBD-MPC/ID-WBC -> LowCmd DDS -> bridge applies latest command to mjData.ctrl -> next mj_step. Physics, DDS, controller, lidar, planner, and logging are wall-clock/OS-scheduler influenced; only the new runner is fully simulation-tick driven.

## Instrumentation

Realtime data.csv records controller_tick, explicit simulation_tick/simulation_time_s, physics_sequence (published simulation-tick proxy), callback/input/output/LowCmd sequences, controller input simulation tick and ages, wall jitter, requested/shaped/applied command, gait, contacts, SRBD, WBC, torques/saturation, qpos/qvel, and terrain lidar/map/planner epochs.

bridge_trace.csv records bridge_tick, simulation_tick, simulation_time_s, LowCmd content hash/sequence, content age, and wall_time_ns. Its command_age_s is simulation-time age since command content changed, not a DDS message timestamp age. Performance analysis separately reports wall-clock Hz, dt/jitter, state/lidar age, tick lag, and bridge intervals.

The determinism analyzer aligns by controller_tick first because a realtime row may observe repeated physics ticks. Each divergence event reports alignment_key, controller_tick, simulation_tick, physics_sequence, controller_input_sim_tick, field, and absolute difference, plus first overall/trajectory/scheduling divergence.

## Functional evidence

| profile/mode | repeats | common ticks | unique SHA | result |
|---|---:|---:|---:|---|
| accel_1_to_3/phase1 | 5 | 21750 | 1/5 | deterministic |
| accel_1_to_3/terrain_sensor_only | 5 | 21750 | 1/5 | deterministic |
| brake_3_to_0/phase1 | 5 | 23750 | 1/5 | deterministic |
| brake_3_to_0/terrain_sensor_only | 5 | 23750 | 1/5 | deterministic |
| varying/phase1 | 5 | 44750 | 1/5 | deterministic |
| varying/terrain_sensor_only | 5 | 44750 | 1/5 | deterministic |

The functional runner includes the flat terrain sensor/model/planner path but no terrain control consumption. Realtime runner behavior was not removed.

## Realtime evidence

| profile/mode | common ticks | final base-x range m | final population variance m2 |
|---|---:|---:|---:|
| accel_1_to_3/phase1 | 10471 | 15.287775..20.782415 | 3.93512089 |
| accel_1_to_3/terrain_sensor_only | 10412 | 14.687336..20.414595 | 4.15835173 |
| brake_3_to_0/phase1 | 5800 | 5.547335..8.009850 | 0.767999882 |
| brake_3_to_0/terrain_sensor_only | 5721 | 4.747951..8.062448 | 1.32070146 |
| varying/phase1 | 17202 | 28.866043..34.298878 | 4.03836052 |
| varying/terrain_sensor_only | 17403 | 29.743153..35.274824 | 4.05450071 |

All 30 realtime raw runs completed data/contact/dynamics generation; all returned status=1 because the existing controller cycle-quality gate rejected them. This is preserved evidence, not a determinism verdict. Representative metadata has controller/safety/analysis/ground-truth/dynamics/completion status 0 and quality status 1. No retry overwrote failures.

Realtime performance across the six groups was approximately controller 499.895..500.084 Hz and bridge 998.630..1000.003 Hz. Maximum state age was 0.004848 s in phase1 and 0.014775 s in terrain_sensor_only; terrain lidar age was 0.100553..0.115645 s when enabled. Deadline miss is diagnostic only, not an acceptance gate.

## Changes and reproducibility

Added phase2_deterministic_functional_runner.cpp, analyze_phase2_determinism.py, analyze_phase2_realtime.py, and run_phase2_determinism.sh. Added explicit control/state/terrain telemetry, bridge tracing, global-lock support in run_trot.sh, and CTest smoke coverage. TerrainPlannerConfig::measure_realtime_timing remains true for realtime and is false only in the deterministic runner. LowCmd trace hashing is mutex-protected so telemetry cannot race command updates. No locomotion policy or main was changed.

Reproduce from the worktree:

    cd /home/che/dev/go2-mujoco-control-determinism
    flock /tmp/go2_mujoco_experiment.lock -c "cmake -S example/cpp -B example/cpp/build-determinism -DCMAKE_BUILD_TYPE=Release && cmake --build example/cpp/build-determinism -j2"
    flock /tmp/go2_mujoco_experiment.lock -c "cmake -S simulate -B simulate/build-determinism -DCMAKE_BUILD_TYPE=Release && cmake --build simulate/build-determinism -j2"
    bash example/cpp/scripts/run_phase2_determinism.sh all 5 phase2_determinism_YYYYMMDD
    flock /tmp/go2_mujoco_experiment.lock -c "ctest --test-dir example/cpp/build-determinism --output-on-failure"

The script enforces the exact base HEAD, exactly 5 repeats, isolated output roots, and lock-protected high-load runs. It refuses an existing output root.

## Remaining blocker and recommendation

The remaining blocker is the realtime architecture's missing simulation-tick synchronization contract. A fixed trajectory cannot be required while controller consumption is selected by wall-clock wakeup and DDS callback timing.

Keep realtime for Hz, latency, jitter, transport, CPU, and scheduler diagnostics. Use the deterministic runner for algorithm/dynamics functional gates and earliest-divergence regression. If a deterministic realtime-like gate is required, introduce a recorded/replayed time-indexed state/control/sensor schedule or a simulation-tick barrier with one owner of state freshness and LowCmd consumption. Time-index planner outputs before including them in that contract. No B1 integration is recommended from this branch.
