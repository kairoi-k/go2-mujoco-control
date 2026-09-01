# Order-116 bootstrap proof record

> Status: **RECORD-ONLY CLOSURE** — independent review verdict: **blocked**,
> outcome: **architecture_blocked**. No behavior implementation is authorized.
> next_safe_step is passive observability-only instrumentation (no behavior
> change, no actuation enablement, no simulator invocation).

## Bundle reference

- Proof bundle directory: `C:\Workspace\tmp\order116_proof_bundle` (record
  mirror delivered to `C:\Workspace\tmp\order116_record_bundle`).
- Bundle manifest hash (sha256 of concatenated per-file sha256, files sorted by
  path): `a5013c060813dfe2e5bfb0a8f10f73755f7b01432da941a11163a3720824b280`.
- Source baseline: `e3af0df` (`e3af0df7dc6d6fd5d0e5bde123a1e752b3501ab3`),
  clean working tree, simulator not invoked, no repository edits by the review.

## Independent review findings

### P0 — observation/C0 (insufficient, not provable)

- `roi_cell_observation.csv`: per-cell values and `cell_stamp` are empty;
  `rebuild_proof.py` only synthesizes rows from log `known=320`.
- `map_visibility_trace.json`: summary-only (32x10, known, height_range); no
  per-cell value/timestamp, lidar rays, transforms, or swept-volume evidence.
  C0 reachability cannot be proven from existing evidence.

### P1 — Dstop discrete implementation (open)

- `p1_calculations.json` verifies only the continuous jerk integral
  (0.183333 m). `velocity_command.h:139-169` actually applies discrete Euler
  updates on clamped dt with velocity saturation and overshoot correction; no
  discrete worst-case recomputation over the dt range was performed.

### P1 — full-chain latency (open)

- `p1_dstop.md` cites Order110 planner p95=1277.164us, while
  `runtime_timing_trace.csv` (same run) shows solver_p95=1281.301us; neither is
  worst-case. sensor/filter/queue/publish/adopt/actuation/halt lack paired
  timestamps or hard upper bounds. p95/raw-max must not serve as safety margin
  until closed.

### P2 — fallback seam (open)

- `terrain_plan_execution_adapter.h` `ApplyToKernel` (~lines 182-203) writes
  kernel gait setters only when `using_plan && last_request_.valid`; fallback
  only clears the execution request and does not prove overwrite of prior
  terrain gait parameters.

### P2 — certificate/ownership/ack (open)

- `TerrainMotionPlan`/`Identity` and lifecycle lockstep ack
  (`trot_experiment_lifecycle.cpp:336-364`) have no C0/C1 certificate, owner
  lease/token, per-consumer ack/deadline, or explicit C0 invalidation/
  preemption; atomic store proves snapshot atomicity only.

## Outcome

- Verdict: **blocked**; outcome: **architecture_blocked**. No behavior
  authorization follows.
- next_safe_step: passive observability-only instrumentation only — record each
  cell raw value/validity/sampling timestamp, lidar rays, pose/map transforms,
  ROI and swept-volume sets, and a sensor-to-halt monotonic sequence with
  timestamps; no behavior change, no actuation, no simulator invocation;
  re-review after evidence lands.
- Open obligations: per-cell sensor-observed C0 with observation-viewpoint
  causality chain; actual discrete Dstop and full-chain worst-case latency
  closure; C0/C1 certificate, lease, independent ack/deadline, invalidation,
  fallback and halt seams.
