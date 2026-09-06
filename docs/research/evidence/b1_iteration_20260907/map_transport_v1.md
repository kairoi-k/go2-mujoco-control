# Atomic capture-map transport V1

Raw validation: `example/cpp/experiments/_runs/b1_chunk_validation_20260907_0001/`.
The `dds/` packet contains standalone source, commands, original logs and hashes.
A native SDK String producer reports Write=1 for both 1300 and 13472 bytes,
but only 1300 arrives in the retained default-configuration comparison. The
exact lower-layer defect is not isolated; Write success is not delivery proof.

The new dedicated topic is `rt/go2/lidar_heightmap_capture_chunks_v1`.
The unchanged complete-envelope V1 codec is carried in <=1000-byte String
packets (800-byte payload). A strict header binds sequence, index/count,
canonical total size and full-wire FNV-1a-64 checksum. This is an integrity
checksum, not authentication. Maximum wire is 256 KiB and at most two partial
assemblies exist. New sequences evict the oldest partial assembly; conflicting
metadata/duplicates/checksum, clock regression, malformed size, expired and
replayed sequences cannot yield a wire. Reordered valid chunks are supported.
The receiver queue is bounded at 512 samples. Pending chunks never expose a
partial map; a completed wire must pass the original codec and outer/inner
sequence match. Capture simulation time and registration freshness checks are
unchanged. A previously complete map can only remain usable within the existing
200 ms registration freshness bound. Unknown input does not become flat ground.

Native DDS comparison: 100 valid 13472-byte maps at 20 Hz, 1700 packets,
queue512, all100 complete and byte-exact, zero rejects/codec failures/wrong
sequences, maximum observed packet899 bytes. No MuJoCo claim is inferred.
The controller/simulator production integration built successfully; controller
CTest40/40 and simulator3/3 passed. The final focused transport fixture uses
Release optimization with assertions explicitly enabled by go2_add_ctest;
`ndebug=false` is not a claim of an unoptimized build. For200 synthetic
13472-byte encode+reassembly iterations, final p50/p95/max were21/21/41 us.
This excludes sensor raycasting, DDS and planning. Source/binary hashes at the
dirty build snapshot are retained in source_binding.json and must match the
subsequent clean commit before closed-loop use.

The independent V3 wrapper audit found and fixed false PASS cases for truncated
truth tails, aggregate/negative contact forces, unregistered geometry, stale
state-clock progression, inactive rows hidden by filtering, gait changes during
interaction, nonfinite tail fields and fatal logs contradicting success status.
Profile reproduction uses1e-6; full runtime integrity uses quality/ground-truth/
dynamics status, while historical threshold analyzers remain separate reports.
Forty-six native focused Python tests pass. The normal post30s braking stage is
allowed for the declared32s trace, without hiding early braking or bad rows.
These are evidence-integrity corrections, not a relaxed physical threshold.

## Registered closed-loop raw runs (f6708ec, 2026-09-07)
The paired raw records are summarized in `registered_closed_loop_results.json`; its
`raw_sha256` entries cover every file retained in each run directory. Both runs
recorded dedicated lidar map registration during the active phase (defined as
`velocity_command_active > 0.5` and `velocity_command_gait_regime ==
continuous-trot`), with `terrain_map_registered=1`, `terrain_map_valid=1` and
source `lidar` on every active row. The registration stamp-age p95 was 86 ms in
`b1_chunk_registered_flat_20260907_0001` (12,504 active rows, map sequence
66--395) and 88 ms in `b1_v3_registered_step_f6708ec_20260907_0001` (10,089
active rows, sequence 68--320). The legacy `environment_map_valid` field stayed
zero; it is not the dedicated registration witness.
The flat run is a registered flat development control, not B0: it uses the
`phase2_flat.xml` scene and the Phase1 steps profile, has
`phase1_quantitative_status=1`, `terrain_analysis_status=1`, and its retained
Phase1 report has `acceptance_status=FAIL` and `quantitative_pass=false` (the
narrow legacy `strict_pass=true` does not promote it to B0). Active IMU posture
was roll p95/max 3.277/8.424 deg and pitch p95/max 2.919/4.624 deg; full-ID WBC
was 1.0 and the log has no IK or hard-safety message. Applied-mask nonzero
fraction was 0.2364; exact value counts and plan-mask counts are in the JSON.
The V3 step run is failed and is not a candidate. It reached first step contact
at 23.430 s, but corrected GT-time approach accounting gives 400 samples and
speed-in-band fraction 0.8475 (p05 0.71190 m/s, median 0.95133), below the
predeclared 0.95 gate. The old 0.85714 value came from collapsing distinct GT
samples sharing a controller state tick and is retained only as a superseded
diagnostic. The controller recorded 1,401 hard-posture and 1,401 hard-safety
lines; statuses are `controller=0`, `safety=1`, `completion=1`. Ground truth
first records nonfoot contact at 23.472 s (1,390 rows; max aggregate nonfoot
force 1027.795 N), with max exact foot-riser forces FR 375.1779 N and FL
158.2059 N. No terrain-top force witness was recorded, the whole-trace minimum base height
was 0.07960 m (including initial crouch; not an active-phase collapse metric), and no complete exit exists. Active full-ID WBC fraction was
0.9968, which is a runtime diagnostic and does not override the physical and
lifecycle failures. Applied-mask nonzero fraction was 0.1325.
These runs establish delivered registration and execution telemetry only. They
do not establish B0/B1 acceptance, and the flat result is not evidence that the
step scene is safe. The full legacy and V3 verdicts remain reportable alongside
this transport evidence.
