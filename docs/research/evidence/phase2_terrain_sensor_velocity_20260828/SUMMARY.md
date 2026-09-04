# Phase 2 terrain sensor-only varying non-regression (2026-08-28)

Status: historical PASS; not current full B0 acceptance.

Bundle identity is recorded in `MANIFEST.json`. The archive path below is
historical provenance only; it is not a current checkout or execution path.

Source: exact commit 70b7740c77dccd9b6610f772100f2df6d4d792e2 on branch
phase2-b1-b3. The commit message is "phase2: retain hold for measured target
until schedule".

Mode: MuJoCo wall-clock running-trot with terrain_lidar=true and controller
flag --terrain-sensor-only. Terrain data was observed and published, but
production terrain actuation and obstacle crossing were not enabled.

Profile: example/cpp/configs/phase1_velocity_varying.csv, with targets
0.6, 1.4, 2.3, 2.8, and 0.6 m/s. Three independent terrain runs were
completed at the same source and passed the frozen Phase-1 analyzer:
acceptance_status=PASS, quantitative_pass=true, strict_pass=true, and all
lifecycle/safety/quality/completion/analysis statuses were zero.

Raw immutable archive evidence:
- r1: phase2_b0_holdout_varying_r1_20260828_215549
  manifest SHA256 34d3a13311271236aa9c41acb623ee624b072c8c1cad0f8370f4f7f9a2272091
- r2: phase2_b0_holdout_varying_r2_20260828_215929
  manifest SHA256 a5d16ae9567192db668c44f9a02f16b3a58228214107afe0013f1762766aa7f7
- r3: phase2_b0_holdout_varying_r3_20260828_220311
  manifest SHA256 c9aa4a04c48e7d6a8ef8802767411daa6b6539580d33a684618dbd507b4281d2

Archive root:
 /home/che/dev/go2-workspace/archive/phase2-b1-b3-misrouted-20260904/example/cpp/experiments/_runs/

Interpretation: this proves that the historical terrain observer/sensor-only
implementation supported the variable-speed profile. It does not prove the
current head: later terrain ownership, runtime, and repository-convergence
changes substantially changed the implementation. The current terrain-only
failure must therefore be compared against this passing boundary before a
specific component is named as causal. Keep the archive evidence immutable.
