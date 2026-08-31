# Order-102 C-006b verification stop

Date: 2026-09-01. Exact preregistered source: `e422a53305f38101bf584982c40c71fdd5d49d53`. Phase A was reviewed as inherited Phase-1 stochastic failure triggered/exposed by wall-clock/state_tick_gap jitter; no code fix or startup workaround was justified. Order-101 remains permanent evidence (1/2 PASS; 95% Wilson [0.094531, 0.905469], diagnostic only).

## New preregistered sample

The frozen command was run serially with `LD_PRELOAD=/home/che/dds_base4000_preload.so bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0`, with domains 222/223, sensor-only terrain, Stage-C execution off, unchanged contract/analyzers, and no outcome-selected replacement.

| member | lifecycle | fixed 3 m/s analyzer | B0 | result |
|---|---|---|---|---|
| pair-1 baseline (`051311`) | all statuses 0 | **FAIL** | n/a | **STOP** |
| pair-1 terrain (`051311`) | all statuses 0 | PASS | PASS | paired member retained |
| pair-2 | not run | not run | not run | mandatory stop |
| pair-3 | not run | not run | not run | mandatory stop |

Baseline fixed-analyzer failure is explicit and authoritative: stop at `11.500 s`, only 44 cycle-health records, no 20 s good-speed window, speed median 1.170566 m/s, all-feet-low fraction 0.061914. The terrain member independently completed 504 cycles and PASS (median 3.240324 m/s); planner updates 2763, max latency 4358.228 us against 5000 us, deadline misses 0. All terrain publish/consumer/actuation counters were zero.

The B0 JSON reports `acceptance_status=PASS` because its paired-baseline-lifecycle predicate passed; this does not override the independent fixed 3 m/s analyzer failure required by Order-101's inherited quantitative gates. Therefore the new sample is 0/1 authoritative fixed-pair passes and C-006b is **BLOCKED**. Pair 2/3 were not selectively retried. Pooled with the permanent Order-101 outcomes, the retained statistic is 1/3 PASS with 95% Wilson interval [0.061492, 0.792340].

## Evidence

`PHASE_A_DIAGNOSIS.md`, `PREREGISTERED_MANIFEST.json`, `FORMAL_MANIFEST.json`, `WILSON.json`, `PAIR1_B0_ANALYZER.json`, `PAIR1_BASELINE_MANIFEST.json`, `PAIR1_TERRAIN_MANIFEST.json`. No B1 run, threshold/analyzer/contract change, or runtime behavior change occurred.
