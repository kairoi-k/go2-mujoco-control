# Joint map coverage and horizon diagnostic
Registered before the next run. Baseline runtime a180e605199999264b3b5e7d4ec1a54f9bbc5107
has no optimizer calls: state-clock captures reject initial map queries or
candidate coverage. Source audit additionally identifies a mismatched horizon
contract: event preview preserves the true stance end beyond the dynamics
window, while candidate generation historically requires that full end inside
its validity window. The new route explicitly certifies only in-horizon
coverage, preserving the real event and never extending surface validity.
Historical generator default remains unchanged.
Next bounded experiment: 32 s flat, seed 11/domain 231, 0.14 s running period,
state clock, point model off, joint shadow on, map intervals V2 on. Existing
Phase-1 shaping, legacy actuation and physical thresholds remain. Capture
queries every 0.5 state seconds in 18--24, including period ramp; preserve
one exact worker snapshot at the first 0.14 s capture. This is diagnostic,
not B1 acceptance or a solver benchmark before attempts actually occur.
The snapshot contains actual articulated state, measured force-contact flags,
nominal touchdown references, and the full registered sensor TerrainModel.
Unknown numerical map fields serialize as null and replay as unknown NaN.
No scene height, simulator truth or zero-height fill enters online queries.
Replay uses the same query and candidate code; it diagnoses reproducibility
and does not replace independent physical residual verification.
Each rejected query records event/leg/candidate, world/local XY, radius,
known/total/outside cell counts and age failure. Outside queries rejected
before footprint scanning have unpopulated counts. The first snapshot writes
once in the worker and its serialization cost is included in pipeline timing;
that capture must be separated from subsequent timings.

Independent all-known synthetic registration reproduces the observed 279/320
ceiling with only 1e-6 m displacement in both axes; see registration_synthetic.
This refutes attribution of the aggregate coverage deficit to ray misses.
Capture-side `JointTerrainSource` masks and sequence bind each query to the
same raw envelope, so the next run distinguishes source and registration loss.

Full controller build and 62/62 CTests passed before this probe. The exact
serializer/extractor/C++ reader roundtrip preserves nonzero pose/velocity and
unknown map numerics; malformed/truncated/extra input fields are rejected.
