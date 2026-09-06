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
