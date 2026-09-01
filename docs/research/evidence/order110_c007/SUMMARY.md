# Order-110 C-007 5 cm development probe

Status: **DEVELOPMENT FAIL / STOP**, not holdout and not acceptance. The run was pre-registered at source `77b52d647d3ebd9471d3b2ee100cfeb8c9298f6e`, clean, with fixed scene/config/seed/binaries/scripts/analyzers/contracts. It acquired `/tmp/go2_mujoco_experiment.lock` and the runner DDS-domain lock, with a 60 s wall timeout and 30 s controller duration.

Execution used `--terrain-planner --stage-c-execution` with the v2 window only. Normal `TerrainCrawlSequencer` policy was not the Stage-C owner; it remained fallback-only. V3-C was observer/shadow-only and was never consumed; no executed two-contact plan was permitted.

The sensor-derived planner repeatedly rejected support (support margin down to -0.0113894 m and unknown support), then emitted a terminal terrain failure at 9.210 s (`failure=4`, failed leg RR). Execution stopped without rerun or tuning. No plan was adopted or replaced (`execution_plan_ids=0`, `plan_published=0`, `plan_consumed=0`); consequently no first measured terrain touchdown, support exchange, tabletop, or exit/recovery transaction was claimed. Planned and measured paths remained separately represented.

Measured evidence: planner latency p95 1277.164 us, deadline misses 0, ground-truth collision rows 0, torque saturation fraction 0.00012346, SRBD and ID-WBC validity 1.0, and actual-FK cross-check max error 2.57e-9 m. Actual-FK is harness-only development evidence. The run B1 terrain/Phase-1 quantitative status is FAIL because required crossing and profile completion did not occur. Per stop rule, Stage-C remains closed for this order and no acceptance claim is made.

Validation: simulator ctest 2/2 PASS; example ctest 31/31 PASS. Raw run artifacts remain in ignored `example/cpp/experiments/_runs/order110_c007_dev/`.
