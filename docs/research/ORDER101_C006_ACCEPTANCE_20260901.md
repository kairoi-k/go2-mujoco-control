# Order-101 C-006 acceptance record

**Status: NOT ACCEPTED (STOPPED ON AUTHORITATIVE FAILURE)**

The formal B0 gate was pre-registered for three independent serial fixed-pair runs at source `7861bf98cd32f454b3da6783a09b5571f4cfe037`, with frozen B0/Phase-1 contracts, scripts, scene, analyzers, controller/simulator binaries, and DDS Base=4000 preload. Stage-C execution remained off. Terrain sensor/planner plumbing and shadow diagnostics were enabled; no plan publish, plan consumer, gait/MPC/WBC consumer, terrain actuation, or measured-contact promotion occurred.

Pair 1 passed all authoritative terrain B0 and fixed 3 m/s gates, with a clean paired baseline. Pair 2 terrain independently passed, but its paired baseline hit the existing hard posture safety stop (`roll=178.557 deg`), yielding `safety_status=1`, `completion_status=1`, and fixed-analyzer `validation=FAIL`. The B0 analyzer therefore returned `acceptance_status=FAIL` via `paired_baseline_lifecycle=false`. Per Order-101, verification stopped immediately; pair 3 was not run. This prevents a 3/3 claim. The failure is retained as evidence and is classified as run-local baseline lifecycle/safety failure; terrain causality is not supported, while jitter versus startup/infrastructure cause remains unresolved without a prohibited rerun.

C-005 logging audit passed on both completed terrain rows: every FR/FL/RR/RL measured-FK record was valid with source `state_q+base_pose_fk`; planned/raw/filtered/fused contact fields remained distinct; and all plan/consumer/actuation counters stayed zero. Planner latency stayed within the frozen 5,000 us deadline with zero deadline misses. Paired numerical differences are listed separately as diagnostic-only fields under B0 contract v1.2.

No code behavior or acceptance threshold was changed. No B1 simulation was run. Rollback remains Stage-C flags off and prior verified SHA. See `evidence/order101_c006/PREREGISTERED_MANIFEST.json`, `FORMAL_MANIFEST.json`, `SUMMARY.md`, and `WILSON.json`.
