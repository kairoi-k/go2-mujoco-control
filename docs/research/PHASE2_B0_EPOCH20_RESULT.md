# Phase 2 B0 epoch 20 result

Status: **FAILED**. This record preserves the complete formal result after the
planned-contact validity fix. It does not change the frozen contract or any
threshold.

## Frozen provenance

* implementation HEAD during the holdout: 5cef2fb01ae671a390a880d8f94d061a88ecc721 (clean);
* code under test: 9ef51230b74e362fb8756713e36df7b34be6ac39;
* branch: research/phase2-stage-b-20260825;
* origin/main: 71d0e9ba7ca1097e840fe878aa30207f6f63600d;
* contract: b0-contract-v1.2, acceptance epoch b0-v1.2-epoch-20;
* frozen manifest: docs/research/PHASE2_B0_HOLDOUT_MANIFEST.json;
* build and CTest before the holdout: build passed and 27/27 tests passed.

The formal holdout was launched sequentially from the frozen HEAD with the
domains in the epoch-20 manifest. Every run directory remains under
example/cpp/experiments/_runs/ and contains its data.csv, run_manifest.json,
analyzer output where applicable, and controller logs.

## Formal result

* 15/18 terrain members passed all applicable B0 gates;
* all three fixed 3 m/s terrain members passed;
* all terrain members had zero planner deadline misses, zero terrain safe-stop
  requests, zero plan publication, zero plan consumption, and zero terrain
  actuation;
* the terrain planner did execute after the planned-contact validity fix. Its
  observed status was kNoSafeFoothold (4) with terrain_plan_valid=0 in
  the sensor-only B0 scene. This is observer telemetry, not a control output.

## Failed members

1. phase2_b0_holdout_brake_3_to_0_r1_20260826_082008_terrain failed only
   phase1_quantitative.shaper_accel_continuity:
   accel_step_abs_max_mps3=0.020935180. The frozen gate is unchanged. Its
   paired baseline passed the same gate at 0.018843248; both lifecycle and
   safety statuses were zero. The terrain run had 681 planner updates,
   terrain_plan_status=4, no deadline miss, and no plan consumer/publisher or
   actuation. The largest state-tick gap was 0.008 s for both runs; the
   terrain wall-clock motion-dt maximum was 0.005233795 s versus 0.005191727 s
   for the pair. This is a near-boundary Phase1 timing result without evidence
   of a terrain control-path consumer.

2. phase2_b0_holdout_brake_3_to_0_r2_20260826_082153_terrain passed its
   terrain quantitative gate. The paired baseline failed lifecycle: its
   completion_status=1 and safety_status=1, and its controller log records
   the existing hard posture safety stop (roll about 179 degrees). The
   terrain member completed with zero safety status. This is baseline-only
   failure evidence and is not attributed to terrain.

3. phase2_b0_holdout_ramp_r3_20260826_082947_terrain failed only the frozen
   phase1_quantitative.undershoot gate: the 3-to-0 transition excursion was
   -0.217021273 m/s against the unchanged -0.2 m/s limit. Its paired
   baseline passed at -0.172398074 m/s. The terrain run had 904 planner
   updates, zero deadline misses, zero safe-stop requests, no publication,
   consumption, or actuation, and a maximum state-tick gap of 0.008 s versus
   0.034 s for the paired baseline. No new terrain timing or control-path
   coupling is evidenced by this pair.

## Planned-contact validity correction

Before this epoch, TrotExperiment::UpdateTerrainRuntime() filled the planned
contact bits but did not set TerrainContactSchedule::planned_valid. The terrain
builder therefore rejected the schedule as invalid on every update. Commit
9ef5123 explicitly sets this validity bit after the gait helper. Build, CTest,
and the B-class steps/accel/brake canary passed after the correction. This
correction is retained; the formal result above must not be mixed with epoch-19
evidence.

## Decision

B1 remains blocked because the frozen contract requires all 18 terrain members
to pass. No threshold, safety envelope, Phase1 constraint, or acceptance
semantics may be relaxed. The two terrain quantitative failures are retained as
diagnostic boundary evidence, while the paired baseline hard-stop failure is
retained as baseline-only evidence. Do not start B1 or run another full B0
solely to chase these known nonlinear wall-clock outcomes. A new full B0
certification is required after the next meaningful control-path change or an
explicitly controlled acceptance rerun.
