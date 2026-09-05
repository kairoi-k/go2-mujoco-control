# Wall-clock B0 first divergence

Updated: 2026-09-05

## Verdict

The production wall-clock path has a reproducible runtime interference when
the terrain-sensor-only path is enabled. The first observed difference is the
LowState snapshot consumed by the controller's first logged `LowCmdWrite`:
the terrain run starts from a different `state_tick`, before any terrain plan
is consumed. This is a scheduling/snapshot provenance difference, not evidence
of a gait, WBC, MPC, or planner actuation effect.

The lidar raycast and HeightMap publish are not the necessary first cause.
With the terrain path enabled and `TROT_SIM_LIDAR_NOOP=1`, the lidar thread
still exists and takes the simulator lock, but performs no raycast and
publishes zero maps; the first consumed tick still differs from baseline.
The remaining exact boundary is the terrain-enabled DDS/controller worker and
simulator lidar-thread/lock scheduling around startup and latest-state
consumption. The evidence does not isolate those two scheduler sources from
each other; confidence is high for runtime interference and medium for the
exact subcomponent.

## Contract

Both accepted pairs used the same SHA within the pair, the same running-trot
parameters, domain 190, explicit CPU placement, serial execution, and
`lockstep=false`. The only normal-pair difference was terrain sensor-only:
baseline did not enable terrain; terrain enabled `--terrain-sensor-only`,
which also enables simulator terrain lidar. The controller's added telemetry
is read-only and does not gate control decisions.

## Evidence

| pair | baseline | terrain/validation | SHA | first consumed tick |
|---|---|---|---|---|
| normal A/B | `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_baseline` | `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_terrain` | `922cc89` | baseline `1786`, terrain `1734` |
| lidar no-op | `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_baseline` | `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_terrain_noop` (`TROT_SIM_LIDAR_NOOP=1`) | `cb920b4` | baseline `1730`, no-op `1728` |

The normal terrain run produced 1,094 controller-side lidar arrivals and
1,058 simulator publishes. Its simulator telemetry p95 values were
`lock_wait=0.0669 ms`, `lock_hold=0.000411 ms`, lidar operation `8.028 ms`,
and DDS publish `0.0460 ms`. The no-op run produced 1,576 lidar-thread
telemetry events, zero publishes, p95 `lock_wait=0.0654 ms`,
`lock_hold=0.000441 ms`, and operation `0.000020 ms`, yet its first consumed
tick still differed from baseline.

In the normal terrain CSV, the terrain map became valid, the planner updated
1,224 times, and `terrain_plan_consumed`, `terrain_gait_target_overrides`, and
`terrain_mpc_plan_consumed` stayed zero. In the no-op CSV the planner updated
1,219 times, but the same three actuation-consumption fields stayed zero.
The no-op map stayed invalid and its `terrain_plan_failure` flag stayed set;
this is the expected consequence of suppressing map production and does not
provide a terrain actuation path.

`HighState.stamp()` was zero in both pairs; controller-side HighState arrival
wall time and age are recorded, but the source message does not provide a
usable simulation timestamp. The first instrumentation pair was discarded
because its newly added CSV header was misordered; `_baseline` telemetry is
empty and `_terrain` contains 1,584 rows under that schema, so neither is used
above.

## Raw evidence

- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_baseline/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_terrain/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r1_terrain/wallclock_runtime_telemetry.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_baseline/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_terrain_noop/data.csv`
- `example/cpp/experiments/_runs/phase2_b0_wallclock_runtime_20260905_r2_terrain_noop/wallclock_runtime_telemetry.csv`

No gait/WBC/planner/B0 threshold or parameter tuning was performed, and no
production behavior change was made. The next architectural fix should make
the wall-clock state/command handoff time-indexed or otherwise record/replay
the exact state schedule; this report does not claim B0 acceptance.

## Lidar runtime isolation matrix

The first isolation matrix was run serially under `flock /tmp/go2_mujoco_experiment.lock`
from source SHA `2843d60c9345a36de799d3e6cffe5ce4b1fba704`, with identical
running-trot arguments, auto-pinning, `lockstep=false`, and a 12 s controller
duration. Domain 240 was rejected before launch; valid runs used 210--214.
The short harness is diagnostic only: its existing cycle-quality guard ended
each run with `quality_status=1`.

| mode | raw run | first consumed tick | last state tick | telemetry | key p95 |
|---|---|---:|---:|---:|---|
| baseline | `phase2_b0_lidar_matrix_20260905_r1_baseline` | 2208 | 14692 | 0 | — |
| terrain + none | `phase2_b0_lidar_matrix_20260905_r1_none` | 2202 | 14272 | 0 | — |
| terrain + park | `phase2_b0_lidar_matrix_20260905_r1_park` | 1752 | 11002 | 0 rows | — |
| terrain + snapshot | `phase2_b0_lidar_matrix_20260905_r1_snapshot` | 2202 | 14686 | 257 | wait 0.0789 ms; op 0.000030 ms |
| terrain + full | `phase2_b0_lidar_matrix_20260905_r1_full` | 1788 | 13976 | 192 | wait 0.0952 ms; op 9.282 ms; publish 0.1008 ms |

`none` created no simulator lidar thread. `park` created the thread but its
SCHED_IDLE worker did not reach telemetry before shutdown, so its telemetry
is header-only and is not treated as proof of scheduling behavior. `snapshot`
performed the model-state copy and simulator lock without raycast/publish;
`full` retained both. In this single repeat park/full had an earlier first
tick while none/snapshot matched baseline, but startup ordering is not stable
enough to assign the cause; a second repeat is required.

Raw directories are the five `phase2_b0_lidar_matrix_20260905_r1_*` entries
under `example/cpp/experiments/_runs/`.

## Replication and worker-order probe

The second lidar matrix used source SHA
`aa508cf0e28debf882ef32f7a88f90c057f21b85`, the same 12 s diagnostic harness,
and domains 215--219. Domain 219 failed before bridge startup because
CycloneDDS could not allocate a participant index; a domain-220 retry failed
with the same message. Both failed run directories and simulator logs are
preserved. A domain-190 retry completed.

| mode | raw run | first consumed tick |
|---|---|---:|
| baseline | `phase2_b0_lidar_matrix_20260905_r2_baseline` | 2212 |
| terrain + none | `phase2_b0_lidar_matrix_20260905_r2_none` | 1754 |
| terrain + park | `phase2_b0_lidar_matrix_20260905_r2_park` | 1762 |
| terrain + snapshot | `phase2_b0_lidar_matrix_20260905_r2_snapshot` | 1814 |
| terrain + full | `phase2_b0_lidar_matrix_20260905_r2_full_retry2` | 1794 |

Unlike the first repeat, all four terrain modes were about 0.40--0.46 s
earlier than baseline. Within terrain, adding a parked, snapshot, or full
simulator lidar path did not create a comparable extra shift. This replicates
the common terrain-on boundary more strongly than a lidar-specific boundary,
while still leaving short-run wall-clock startup nondeterminism visible.

The next probe used source SHA
`c161868e10efdc73aaec0805f85f36ef6d0b62a0` and set the diagnostic-only
`TROT_TERRAIN_WORKER_AFTER_WRITER=1`, with lidar mode `none` and otherwise
identical arguments. Baseline
`phase2_b0_worker_order_20260905_after_baseline` consumed tick 1800;
terrain-on `phase2_b0_worker_order_20260905_after_none` consumed tick 1794.
Both retained `quality_status=1` from the short harness, but the 6-tick
difference is materially smaller than the 458-tick default-order pair. The
probe reorders only thread startup; the production default remains unchanged
until a longer regression validates it.

## Candidate B0 acceleration probe

With source SHA `698284553f48ffe876683da51759eba15c8425ef`, the canonical
`accel_1_to_3` development pair was run under the required experiment lock
and `LD_PRELOAD=/home/che/dds_base8000_preload.so`, with
`TROT_TERRAIN_WORKER_AFTER_WRITER=1`. Raw directories are
`phase2_b0_development_accel_1_to_3_r0_20260905_185710_{baseline,terrain}`.
Both lifecycle, controller, quality, safety, dynamics, and analysis statuses
were zero. The terrain member passed Phase-1 quantitative/strict analysis;
the frozen `b0_analyzer.json` returned `acceptance_status=PASS`, with 24,201
terrain rows, map-valid fraction 1.0, 786 planner updates, and zero deadline
misses. The paired baseline Phase-1 diagnostic had torque saturation fraction
0.00320368597 and returned FAIL, while the terrain member was 0.00289586426;
this is retained as diagnostic context and was not hidden by the B0 analyzer.
This is one development profile under a diagnostic env, not full B0 and not
the final production-fix acceptance.

## Production fix and Phase-1 regression

Commit `34d4a626a21dda10e5aa5263761eb93a2daf7594` makes the worker-order
probe unconditional: the terrain observer is still created and updated, but
only after the wall-clock writer thread has been created. No gait, WBC,
planner math, threshold, analyzer, or contract file changed. The simulator
lidar default remains full. After the fix, both build trees were rebuilt and
focused CTest passed `32/32` plus `2/2`.

The first post-fix no-terrain Phase-1 varying regression is preserved at
`example/cpp/experiments/_runs/phase1_runtime_integrity_20260905/varying_20260905_190232`.
It completed with lifecycle/safety/quality status zero, but the analyzer
failed only `id_wbc_ok`: one row at state tick 64686 had a non-converged ID-WBC
QP and non-finite solution. Since terrain is disabled in this run, the changed
branch is unreachable. An independent identical retry at
`example/cpp/experiments/_runs/phase1_runtime_integrity_20260905_retry/varying_20260905_190453`
passed every strict and quantitative check with `id_wbc_ok_fraction=1.0`.
The failure is therefore retained as stochastic baseline evidence, not
attributed to the worker-order fix.

The first post-fix frozen B0 profile was `steps`, source SHA
`a6230b821056787267748e2fc530522e9d15fbd9`, run with the exact development
domains, serial lock, default full lidar, and the existing DDS preload. Raw
directories are
`example/cpp/experiments/_runs/phase2_b0_development_steps_r0_20260905_190808_{baseline,terrain}`.
Both members completed with lifecycle, controller, quality, safety, dynamics,
and analysis statuses zero. The baseline Phase-1 analysis passed. The terrain
member and frozen B0 analyzer failed only the terrain Phase-1
`steady_state_error` check: max error `0.43060309` versus frozen `0.40`.
ID-WBC, solver, map-valid fraction `0.9999798188`, 1,543 planner updates, and
zero planner deadline misses all passed. This is the first full-B0
information-bearing failure after the production fix; no threshold or
semantic change is inferred.

An independent rerun of the same frozen `steps` pair completed at
`example/cpp/experiments/_runs/phase2_b0_development_steps_r0_20260905_191350_{baseline,terrain}`
with source SHA `dee85333004d4f7b4fe9d05b9385127ad7d4239b`, the same preload,
domains, and production default. Both Phase-1 analyses and the frozen B0
analyzer returned PASS. The terrain member had steady-state max error
`0.29567809`, map-valid fraction `0.9999798192`, 1,462 planner updates, and
zero deadline misses. The pair therefore shows a boundary-sensitive but
reproducible pass/fail variation at the frozen metric, not a stable blocker;
the failed `190808` pair remains the negative evidence.

The next canonical production-default `accel_1_to_3` pair used source SHA
`4145ce9915971e3d2d86b2cbe3794d5922694ac9`, the same development domains,
serial lock, DDS base-8000 preload, and no diagnostic environment overrides.
Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_191913_{baseline,terrain}`.
The baseline Phase-1 analysis passed. The terrain member and frozen B0
analyzer failed only the terrain Phase-1 quantitative gate: contact loss
`0.2556372181`, steady-state max error `0.474359578` versus frozen `0.40`,
settling was not reached, and torque saturation was `0.0038382696`.
All runtime/contract checks still passed, including map-valid fraction
`0.9999586794`, 731 planner updates, zero planner deadline misses, no plan
publish/consume, and no terrain actuation. This is a production-default
negative result after the worker-order fix; the prior diagnostic-env
acceleration PASS is therefore not sufficient to close B0.

## Production-default lidar-mode diagnostic

The next same-profile pair used source SHA
`fc93eb5433837ea7b67fc0b2b0185025342c9c6f`, the same lock, domains, and
preload, but set `TROT_SIM_LIDAR_MODE=none`, so the simulator did not create
a lidar thread. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_193028_{baseline,terrain}`.
Both members completed with lifecycle, controller, quality, safety, dynamics,
and analysis statuses zero. The terrain Phase-1 diagnostic failed only the
frozen settling gate at `10.049994995 s`; contact loss, steady-state error,
torque saturation, solver, and ID-WBC checks passed. The B0 analyzer
structurally failed as expected because lidar observation and map telemetry
were absent (map-valid fraction `0.0`), despite 735 planner updates and zero
deadline misses. This is diagnostic evidence only, but it shows that removing
the simulator lidar thread does not by itself produce a B0 candidate.

The same-profile production-default pair then set
`TROT_SIM_LIDAR_MODE=park`: the simulator created the lidar thread, pinned
it, and parked it before any snapshot, raycast, or publish. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_193412_{baseline,terrain}`.
The baseline Phase-1 analysis passed. The terrain diagnostic failed contact
loss `0.2504374781`, steady-state max error `0.430223719` versus frozen
`0.40`, and torque saturation `0.0034229058`; settling and the remaining
checks passed. The parked thread emitted zero lidar telemetry rows, so the B0
analyzer failed the expected lidar/map structural checks. Compared with
`none`, this is a thread-creation-only diagnostic and remains consistent
with startup/runtime interference, but one pair is not enough to quantify
stability.

The next same-profile pair set `TROT_SIM_LIDAR_MODE=snapshot`: the simulator
created the thread, copied MuJoCo state under the simulation lock, and emitted
telemetry, but did no raycast or publish. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_193918_{baseline,terrain}`.
The terrain Phase-1 diagnostic passed every frozen check, with steady-state
max error `0.354981927`, settling `2.884051031 s`, contact loss
`0.2398260174`, and torque saturation `0.0025747425`; its B0 analyzer
failed only the expected lidar-observation/map-telemetry structural checks
(map-valid fraction `0.0`). It emitted 926 snapshot rows with approximately
`20 us` operation time and no publish. The paired baseline independently
failed settling and torque saturation, so this pair is not a clean acceptance
run; it does not show snapshot/lock alone as a sufficient terrain failure
cause.

An independent production-default full-lidar repeat used source SHA
`2d3189b8016308b991af238c6d216061dc36c81e` with the same canonical
`accel_1_to_3` pair, lock, domains, and preload. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_194248_{baseline,terrain}`.
The baseline Phase-1 analysis passed. The terrain member and frozen B0
analyzer failed only Phase-1 `settling` (`11.191999240 s` versus frozen
`10 s`) and `torque_saturation` (`0.0034938814`); contact loss,
steady-state error, solver, ID-WBC, and all other quantitative checks passed.
The analyzer's only false B0 check was `phase1_quantitative`; map-valid
fraction was `0.9999586794`, with 785 planner updates, zero deadline
misses, 935 lidar rows, and no plan publish/consume or terrain actuation.
Together with the earlier full failure, this makes the normal path a repeated
negative at the frozen Phase-1 boundary, not a deterministic crash.

The controller-side worker isolation pair used source SHA
`34d55567640f2066b18976d7a034ddc4ce2f70bd`, after focused CTest had again
passed 34/34. It kept full simulator lidar and the terrain subscriber, but
set `TROT_TERRAIN_WORKER_DISABLE=1`, so no asynchronous planner worker ran.
Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_195311_{baseline,terrain}`.
Both Phase-1 analyses passed every frozen quantitative check. The terrain
member had steady-state max error `0.130636073`, settling `3.007992284 s`,
contact loss `0.2418256774`, torque saturation `0.0028727896`, and 24,202
rows. Full lidar emitted 936 simulator telemetry rows and 751 controller
arrivals, but the B0 analyzer failed only the expected
`lidar_observation`, `map_telemetry`, and `planner_updated` checks because
the disabled worker produced no map/planner diagnostics. This is diagnostic
only: it implicates the worker's runtime presence/work as a contributor, not
its planner mathematics, and is not a B0 acceptance result.

The controller worker-park diagnostic used source SHA
`a276319865357d8a3020d6d79c89016ba943cda0`; focused CTest was again 34/34
PASS. It retained full simulator lidar and created/pinned the controller
terrain worker, but set `TROT_TERRAIN_WORKER_PARK=1`, so the worker consumed
no terrain work. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_195849_{baseline,terrain}`.
The baseline Phase-1 analysis passed. The terrain run became unsafe at
28.915796836 s: controller/safety status `0/1`, only 16,613 rows, minimum
base height `0.057560766 m`, and roll p95 `178.9433966 deg`; the controller
log repeated the hard-posture rejection at roll about `178.5 deg`. Phase-1
false checks were `base_height`, `id_wbc_ok`, `roll_p95`, `settling`,
`shaped_to_measured_p95`, `touchdown_x`, and `tracking_p95`. The B0
analyzer failed the expected no-map/no-planner structural checks plus Phase-1;
planner updates and map-valid fraction were zero, while full simulator lidar
still emitted 656 telemetry rows. This single pair shows worker creation/park
can be materially destabilizing, but is not yet a deterministic root-cause
proof.

The controller worker-park/no-pin diagnostic used source SHA
`8ba18f5fe0e4cf8051d3700d0260588b54616d55`; it set
`TROT_TERRAIN_WORKER_PARK=1` and `TROT_CPU_AFFINITY_TERRAIN=-1`, so the
worker was created and placed in SCHED_IDLE but did not consume terrain work
or receive a worker CPU pin. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_200926_{baseline,terrain}`.
The baseline Phase-1 analysis passed. The terrain run completed all 40 s
without a safety rejection and failed Phase-1 only on torque saturation
(`0.0030152339`); its other frozen quantitative checks passed, with 24,201
rows and 934 full-lidar telemetry events. The B0 analyzer failed the expected
no-map/no-planner structural checks plus Phase-1 (`terrain_map_valid_fraction=0`,
`terrain_planner_updates=0`). Compared with the pinned park run, this isolates
the pin/CPU placement path as a stronger contributor than worker creation alone,
but it is one diagnostic pair and not yet a production fix or B0 acceptance.

The active-worker no-pin diagnostic used source SHA
`81559bcef07ad8d05def1a636c6b2484f40077d9`, the canonical acceleration pair,
full simulator lidar, and no park override; only `TROT_CPU_AFFINITY_TERRAIN=-1`
was added. Raw directories are
`example/cpp/experiments/_runs/phase2_b0_development_accel_1_to_3_r0_20260905_201313_{baseline,terrain}`.
The terrain member completed 40 s and passed both frozen Phase-1 quantitative
analysis and the B0 analyzer: map-valid fraction `0.9999586811`, 792 planner
updates, zero deadline misses, and 24,202 rows. Full lidar emitted 935
telemetry rows including the header. The paired baseline diagnostic failed
only torque saturation (`0.0032921431`), so this is not yet a clean full-B0
acceptance pair. It is nevertheless the first B0 PASS with the real terrain
worker present after removing only its CPU pin, making auto affinity placement
the leading minimum production-change candidate.

## Recovery record

The initial audit was performed in `/home/che/dev/go2-workspace/current` without
reset, clean, or checkout. At that time the live worktree was on
`investigate/phase2-b0-wallclock-telemetry-20260905` at `de6fc6b`, while local
and origin `fix/phase2-b0-runtime-integrity` were both at `c42b1ee`; the latter
is an ancestor of the live wall-clock commits. The initial command capture was:

- `git status --short`: empty.
- `git diff`: empty.
- `git diff --cached`: empty.
- `git log -5`: `de6fc6bd6a8f948f5d5bbc6c4ee291aa0cbcc929 docs: record wall-clock first divergence`; `cb920b43d3cc0932c55f51f606692b90a30b1d8b diag: classify skipped ticks and isolate lidar worker`; `922cc89cb50778bec63dc270694267bf29b76b02 fix: align wall-clock telemetry columns`; `e07017cd19a7c77338836cdf41b080bd6f24dc38 diag: add wall-clock runtime telemetry`; `c42b1eec34f42baabe777a8c4b650bbf5957b0c7 docs: record lockstep handoff follow-up`.
- `git stash list`: `stash@{0}: On phase2-b1-b3: wip-base-height-hold-2026-08-28-unverified`.

The diff from `c42b1ee` to `de6fc6b` contains only wall-clock telemetry,
diagnostic/no-op instrumentation, and this report. It does not change B0
thresholds, gait/WBC parameters, or terrain actuation. All four run manifests
record `git_dirty=false` and the run status fields as zero; those lifecycle
fields are not acceptance evidence.

The local test logs at 17:47 CST record 32/32 `example/cpp` tests and 2/2
`simulate` tests, with 32 and 2 `Test Passed.` entries respectively:

- `example/cpp/build/Testing/Temporary/LastTest.log`
- `simulate/build/Testing/Temporary/LastTest.log`

This is the reported 34/34 test result only; it is not a B0 acceptance result.
