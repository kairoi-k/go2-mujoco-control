# Phase 2 B0 epoch 19 result

Status: FAILED. This record preserves the formal verdict and the diagnostic follow-up without changing the frozen contract.

## Frozen provenance

* implementation HEAD: a49211fcf8fcd900397416d7834b7545d53b3ab5 (clean);
* code change under test: cc671a0, terrain callback affinity isolation;
* branch: research/phase2-stage-b-20260825;
* origin/main: 71d0e9ba7ca1097e840fe878aa30207f6f63600d;
* contract: b0-contract-v1.2, acceptance epoch b0-v1.2-epoch-19;
* build and CTest: build passed and 27/27 tests passed.

## Formal holdout

The frozen 18-pair holdout completed sequentially and every run directory is retained under example/cpp/experiments/_runs/.

* 16/18 terrain members passed all applicable gates;
* all three fixed 3 m/s terrain members passed their fixed analyzer;
* no passing terrain member consumed or published a plan, changed gait or v_cmd, or recorded a planner deadline miss.

## Failed members

1. phase2_b0_holdout_brake_3_to_0_r2_20260826_065614_terrain failed only paired_baseline_lifecycle. The terrain member quantitative result passed. Its paired no-terrain baseline stopped at the existing hard posture safety limit, with completion_status=1 and safety_status=1; its controller log records the hard posture stop. This is baseline-only lifecycle failure evidence, not terrain actuation.
2. phase2_b0_holdout_varying_r3_20260826_071346_terrain failed only inherited phase1_quantitative: steady_state_error_abs_max_mps=0.450988522 against the frozen 0.45 limit. The paired baseline passed. The terrain run had no plan consumer, no plan publication, no terrain actuation, zero deadline misses, and zero safe-stop requests; its maximum state-tick gap was not worse than the paired baseline.

The paired baseline quantitative misses in brake repeat 1 and ramp repeat 3 were diagnostic only because their lifecycle statuses were zero and the frozen contract does not use baseline quantitative output as an acceptance sample.

## Diagnostic follow-up

A fresh development varying pair at the same clean HEAD passed with steady_state_error_abs_max_mps=0.422210151, supporting the known wall-clock boundary variability diagnosis. A fresh development brake pair retained its own Phase 1-only undershoot/continuity variability and showed no terrain hard stop or terrain actuation. These runs are diagnostic only and do not alter the epoch19 verdict.

## Decision

B1 remains blocked because the frozen contract requires every B0 terrain member to pass. No threshold, safety envelope, or acceptance semantics may be changed to clear this result. Do not start B1 or rerun the full 18-member B0 solely to cover these known nondeterministic failures; a future full certification is required at the next meaningful control-path implementation epoch or when an explicitly controlled acceptance rerun is authorized.
