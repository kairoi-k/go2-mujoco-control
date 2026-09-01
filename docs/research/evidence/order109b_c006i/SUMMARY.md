# Order-109b C-006 exact-SHA lockstep verification

**Status: PASS.** Exact tested source SHA: `5b95e8265c885a81f8488e4930e682aa55f05674`; branch `phase2-b1-b3`; origin synchronized (`0	0`), clean before and after runs.

## Pre-registration and canary

The full tracked-file SHA-256 manifest was frozen before execution in `PREREGISTERED_MANIFEST.json`. Canary command: `SIM_LOCKSTEP=1 bash example/cpp/scripts/run_phase2_b0_lockstep_pair.sh development 0`. It passed with Stage-C execution off and terrain shadow diagnostics on. Both members had zero protocol violations, exact 2 ms tick cadence, exact state acknowledgements, one command update per tick after the lockstep handoff, controller `cmd_time/sim_time` slope 1.004 (within the frozen 1% timing gate; see formal manifest), fixed analyzer PASS, lifecycle PASS, terrain B0 PASS, and frozen equivalence comparator PASS.

## Formal sample

Three independent serial fixed pairs were run exactly as pre-registered: holdout-r1 (183/203), holdout-r2 (184/204), and holdout-r3 (185/205). Every baseline and terrain member passed lifecycle, fixed 3 m/s, lockstep protocol, authoritative B0, terrain sensor-only, planner deadline, no publish/consumer/actuation, and frozen non-regression checks. No pair was skipped or rerun.

| pair | baseline | terrain | result |
|---|---:|---:|---|
| canary | 222 | 223 | PASS |
| holdout-r1 | 183 | 203 | PASS |
| holdout-r2 | 184 | 204 | PASS |
| holdout-r3 | 185 | 205 | PASS |

## Validation

- `ctest --test-dir simulate/build --output-on-failure`: **2/2 PASS**.
- `ctest --test-dir example/cpp/build --output-on-failure`: **31/31 PASS**.
- No B1, threshold, analyzer, contract, configuration, or source behavior was changed.
- Evidence is docs-only; run data remains in the ignored experiment workspace and each run manifest records the tested SHA and clean state.

Detailed per-member trace, timing, analyzer, and B0 results are in `FORMAL_MANIFEST.json`.
