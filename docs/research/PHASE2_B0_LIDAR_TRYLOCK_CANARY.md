# Phase2 B0 development canary: non-blocking lidar observation

Status: FAIL; this is development evidence, not a certification result.

## Provenance

* branch: research/phase2-stage-b-20260825;
* base HEAD: c8cb6f1d0f94d86cfd51be2f611dff1a494765ec;
* implementation was dirty because the lidar try-lock change was not yet
  committed;
* build: simulate and controller succeeded;
* CTest: 27/27 passed;
* no B0 holdout was started after this canary.

## Change under test

TerrainLidarLoop now uses std::try_to_lock for the shared simulator mutex.
If the PhysicsLoop owns the mutex, the lidar sample is skipped rather than
blocking the simulator. No controller output, gait parameter, MPC/WBC input,
safety limit, or acceptance threshold was changed.

## Paired development run

* baseline:
  example/cpp/experiments/_runs/phase2_b0_development_steps_r0_20260826_124240_baseline
* terrain:
  example/cpp/experiments/_runs/phase2_b0_development_steps_r0_20260826_124240_terrain

The baseline passed its Phase1 quantitative analyzer. The terrain member failed
only phase1_quantitative.undershoot:

* measured transition excursion: -0.289389868 m/s;
* frozen steps lower bound: -0.25 m/s;
* paired baseline excursion: -0.146613853 m/s;
* controller, safety, quality, completion, and dynamics statuses: zero;
* terrain planner updates: 1476;
* terrain planner deadline misses: zero;
* terrain map valid fraction: 0.999979819;
* terrain plan publish, consume, and actuation: zero;
* maximum terrain wall-clock motion delta: 0.004260126 s;
* maximum state-tick gap: 0.008 s.

The run therefore reproduces the flat observer-enabled quantitative failure
without the epoch-21 45 ms stall. The try-lock change alone does not establish
Phase1/terrain observational orthogonality. No lock-miss telemetry was
collected in this run; headless simulator shutdown uses _exit, so such
instrumentation would not be emitted by the current runner.

Do not run the remaining canaries or a full B0 from this dirty implementation.
Keep both run directories. The next investigation must isolate controller-side
terrain DDS/map callbacks and low-rate terrain snapshot/worker scheduling
without changing Phase1 control semantics. B1 remains blocked.
