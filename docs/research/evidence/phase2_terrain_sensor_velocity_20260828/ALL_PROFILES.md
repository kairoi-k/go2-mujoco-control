# 2026-08-28 terrain-sensor-only profile campaign

This is a durable index of the broader raw campaign behind the documented
varying-only summary. It does not replace CURRENT.md, the frozen contract,
or a fresh current-head B0 acceptance.

Source: 70b7740c77dccd9b6610f772100f2df6d4d792e2 on phase2-b1-b3.
The original run manifests record git_dirty=true; the archive is therefore
historical evidence, not a clean reproducibility baseline. All runs were
MuJoCo wall-clock, terrain_lidar=true, and --terrain-sensor-only; terrain
actuation and obstacle crossing were disabled.

| Profile | Raw campaign result | Representative run IDs |
|---|---|---|
| steps | 2 PASS, 1 FAIL | ...steps_r1_20260828_213008 PASS; ...steps_r2_20260828_213416 FAIL; ...steps_r3_20260828_213824 PASS |
| accel_1_to_3 | 1 PASS, 2 FAIL | ...accel_1_to_3_r1_20260828_214130 PASS; ...accel_1_to_3_r2_20260828_214331 FAIL; ...accel_1_to_3_r3_20260828_214528 FAIL |
| brake_3_to_0 | retry campaign; final selected r1/r2/r3 PASS | ...brake_3_to_0_r1_20260828_211644 PASS; ...brake_3_to_0_r2_20260828_212523 PASS; ...brake_3_to_0_r3_20260828_212712 PASS; earlier retries include FAIL |
| ramp | 3 PASS | ...ramp_r1_20260828_214742, ...ramp_r2_20260828_215013, ...ramp_r3_20260828_215244 |
| varying | 3 PASS | ...varying_r1_20260828_215549, ...varying_r2_20260828_215929, ...varying_r3_20260828_220311 |

The fixed-3-m/s slice also has three PASS analyzer records at
...fixed_3mps_r{1,2,3}_20260828_*_terrain. The separate 2026-08-25 Phase-1
acceptance is the cleanly documented five-profile result: 5 profiles x 3
runs = 15/15 PASS. This campaign matrix explains the historical coverage
without upgrading mixed/retry results into a single all-green 8/28 claim.

Raw archive root (read-only):
/home/che/dev/go2-workspace/archive/phase2-b1-b3-misrouted-20260904/example/cpp/experiments/_runs/.
