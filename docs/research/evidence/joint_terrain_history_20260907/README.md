# Bounded immutable terrain history
Current implementation uses up to four original capture-heading maps under an explicit stationary-terrain assumption. Real cell age includes elapsed observation time exactly once. Unknown cells remain unknown; only complete independently known historical support patches can supply a query. Fresh partial observations conflicting by more than 1 mm height or the declared normal dot threshold reject the query. Every selected future contact retains its actual source timestamp and sequence. Initial support patches also pass the same single-surface geometric screening as candidates.
Snapshot v2 retains all contributing original captures and policy metadata; the independent text transport/replayer supports both historical v1 and v2. Full controller build and 64/64 CTests pass. This is input/proposal infrastructure, with no joint command authority or new traversal claim. The preceding 0ff9dc8 flat/5cm results and actual contact mismatch audit are retained in joint_capture_view_20260907.
Next bounded diagnostic: one 32 s flat run joint_history_flat_20260907_0001 with state clock, joint shadow, map intervals V2, running period 0.14 s, contact point model off, seed11/domain231 and phase2_flat.xml / b1_v3_running_1mps.csv. Validate v2 actual snapshot extraction and replay, history provenance, and source bindings. Do not repeat the failed 5 cm legacy run before joint execution integration.

The same diagnostic now retains the exact winning centroidal problem/result, with independent certificate checking and deterministic cost/tie selection. New motor-envelope telemetry every20ms observes the final serialized command (PD plus feedforward) at the consumed motor state. Direct actuator control limits are read from the same MJCF; unsupported actuator semantics reject. Independent MuJoCo qfrc_actuator oracle verifies both signed saturation cases with zero Nm residual. This adds observation only, not torque enforcement or measured later-tick actuation claims.

Independent review found two pre-admission defects after the initial64-test pass: partial footprints at map edges skipped conflict evidence, and directly constructed public snapshots could bypass entry provenance checks. Both are now rejected/handled explicitly, with focused counterexamples; partial footprints remain unavailable as selected support patches. Invalid capture scope and malformed/unsorted/duplicate entries reject before selection. Conflict diagnostic reason is explicit. The initial test pass is not treated as proof that these paths were already correct.

Final frozen rebuild passes66/66 CTests. An earlier full build overlapped temporary red-test source variants and is discarded as experiment evidence. After all workers stopped editing, the affected header/test were forced newer, all dependent targets rebuilt, source hashes rechecked, and the full suite rerun. Only that final binary binding is registered below.

## Actual cc69623 flat result
Exact clean runtime cc696232eab2d184b5c98fa89674f8d029895dad completes normally.
16 complete captures yield15 reduced proposals and1 initial-contact-anchor
rejection. There are no terrain-query coverage rejections in these captures;
historical captures supply2 selected future contacts and2 initial queries.
This is not a matched-state causal comparison against prior runs.
Command-window18--24 topology diagnostic is35/42 good cycles, not acceptance.
The15 attempted pipeline samples have p50/p95/max1.072334/50.1456852/50.718449ms.
The two slow captures report2400 QP iterations each (two combinations at the
1200 cap); continuous residual verification passes, but timely/optimal planning
is not established. No joint command authority was enabled.
The registered first <=0.14s snapshot is the one contact-anchor rejection
(id360,state20.562). Real v2 snapshot extraction/replay reproduces that rejection;
it is not a feasible-case solver benchmark. The older retained f962 state20.100
snapshot remains feasible under current code with residual1.887379e-15, explicitly
a cross-version offline counterfactual rather than a new actual capture.
Independent raw data.csv recomputation of all1839 sampled final motor commands
matches runtime requested/clipped output within9.83745e-8Nm (CSV precision).
In STATE20--28,393 samples include6 saturations, max67.513173Nm beyond the model
limit. Worst: STATE27.900 / command25.896, requested/shaped speed1m/s, RL calf:
PD=-112.523849Nm, feedforward=-0.419324Nm, total=-112.943173Nm, model-clipped=-45.43Nm.
This validates a real final-composition mismatch, not a measured later-tick
actuator-force assertion or a causal proof of B1 failure. No motor command was
changed. The legacy phase1 profile KeyError/dependent analysis failure remain.
Reproduce the independent actuator audit with:
`python3 docs/research/evidence/joint_terrain_history_20260907/analyze_motor_envelope.py example/cpp/experiments/_runs/joint_history_flat_20260907_0001 --runtime-sha cc696232eab2d184b5c98fa89674f8d029895dad --model unitree_robots/go2/go2.xml --out /tmp/FRESH_MOTOR_AUDIT.json`.
Next: a separately bound short-horizon MuJoCo feedback-backend diagnostic,
then unified reference adoption. Do not repeat the unchanged failed5cm legacy run.
