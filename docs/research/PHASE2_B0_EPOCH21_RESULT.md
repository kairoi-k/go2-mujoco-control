# Phase 2 B0 epoch 21 result

Status: **FAILED**. This record preserves the complete formal result after the
safe-region consumption, sensor-only isolation, and planned/measured contact
separation changes. It does not change the frozen contract or any threshold.

## Frozen provenance

* frozen implementation HEAD during the holdout: ad6437096859da2d750e714d5cd45ee6570b3a96 (clean);
* code under test: 03d5fb786f57d822a5adcfac6b4d5c22a4b2d697;
* branch: research/phase2-stage-b-20260825;
* origin/main and merge-base: 71d0e9ba7ca1097e840fe878aa30207f6f63600d;
* contract: b0-contract-v1.2, acceptance epoch b0-v1.2-epoch-21;
* frozen manifest: docs/research/PHASE2_B0_HOLDOUT_MANIFEST.json;
* build and CTest before the holdout: build passed and 27/27 tests passed;
* B-class canary before the holdout: steps, accel_1_to_3, and brake_3_to_0
  all passed with the same implementation.

The formal holdout was run sequentially with the domains in the epoch-21
manifest. Every run directory remains under
example/cpp/experiments/_runs/ and contains its data.csv, run_manifest.json,
analyzer output where applicable, and controller logs.

## Formal result

* 15/18 terrain members passed all applicable B0 gates;
* 13/15 normal profile terrain members passed;
* 2/3 fixed 3 m/s terrain members passed;
* all terrain members had zero planner deadline misses, zero terrain safe-stop
  requests, zero plan publication, zero plan consumption, and zero terrain
  actuation;
* the formal B0 contract therefore did not pass.

The failed analyzer gates were two inherited Phase1 quantitative gates and one
fixed 3 m/s runtime analyzer gate. No failed member had a terrain planner
deadline, publication, consumption, or actuation gate failure.

## Failed members

1. `phase2_b0_holdout_brake_3_to_0_r1_20260826_101858_terrain` failed only
   `phase1_quantitative.undershoot`: the 3-to-0 transition excursion was
   -0.235920114 m/s against the frozen -0.20 m/s limit. Its paired baseline
   passed the same gate at -0.149139954 m/s. Both lifecycle and safety
   statuses were zero. The terrain member had 736 planner updates, zero
   deadline misses, and no plan publisher, consumer, or actuation.

2. `phase2_b0_holdout_varying_r1_20260826_103140_terrain` failed only
   `phase1_quantitative.steady_state_error`: the maximum steady-state error
   was 0.460990856 m/s against the frozen 0.45 m/s limit. Its paired
   baseline passed at 0.398582967 m/s. Both lifecycle and safety statuses
   were zero. The terrain member had 1307 planner updates, zero deadline
   misses, and no plan publisher, consumer, or actuation.

3. `phase2_b0_holdout_fixed_3mps_r1_20260826_104228_terrain` failed
   `fixed_3mps_analyzer`. Its
   `sustained_running_analysis.txt` records safety_status=1,
   completion_status=1, a hard-posture controller rejection, and final
   angle 178.738417 degrees. The paired baseline passed with final angle
   0.500622 degrees. The terrain member had 779 planner updates, zero
   planner deadline misses, and no plan publisher, consumer, or actuation.

## Failure attribution

The first two failures are near-boundary inherited Phase1 gates in a
sensor-only terrain member. They are not caused by a terrain plan reaching
MPC, WBC, gait, or the velocity arbitration path. The B0 analyzer confirms
that all corresponding terrain interface isolation checks passed.

The fixed r1 failure is a real acceptance blocker, not evidence that terrain
changed a control output. The failed terrain run had a 45.160950 ms maximum
wall-clock motion dt, a 44 ms maximum state-tick gap, and two motion-clock
pauses. The successful fixed r2 terrain run had 4.701496 ms, 8 ms, and one
pause respectively. The terrain worker was isolated to CPU 6; controller,
writer, physics, bridge, and lidar affinities were recorded in the run
metadata. This is consistent with wall-clock scheduling/nonlinearity, but the
terrain-versus-baseline timing difference means it should remain an explicit
runtime evidence item rather than be silently dismissed.

Historical epoch-20 evidence also had 15/18 normal terrain members passing and
all three fixed terrain members passing, with different near-boundary
failures. This supports nondeterministic timing as the current pattern, but
does not waive the epoch-21 contract.

## Decision

B1 remains blocked because the frozen contract requires all 18 terrain
members to pass. No threshold, safety envelope, Phase1 constraint, or
acceptance semantics may be relaxed. No successful retry may replace any of
the three failed directories.

Do not start B1 or perform terrain tuning from this HEAD. The next minimal
acceptance action is a separately controlled wall-clock/scheduler diagnostic
or acceptance rerun that preserves this epoch-21 evidence. A new full B0
certification is required before B1 after any meaningful control-path change
or any authorized correction to the acceptance runtime environment.

