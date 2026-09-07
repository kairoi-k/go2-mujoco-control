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

## Actual f9623e1 probe
Runtime `f9623e1486d03830101aa80d132fc8f3c04489bb`, exact clean source and
binaries independently verified; raw `joint_map_state_flat_20260907_0001`.
12 complete captures: 6 reduced-model proposals, 5 initial unknown, 1 candidate
coverage. All 6 attempted solves succeed; each evaluates two combinations.
Maximum reported continuous-equation residual is 5.71765e-15 (position,
velocity and momentum residual components are logged under one maximum).
No articulated body/contact-evolution or final actuator certificate follows.
Snapshot id350/state20.10, first 0.14 s capture: source318/320 known, registered
271/320. The full mask shows one row and one column lost, plus expansion of
two source holes. Independent patch-cell accounting agrees with all five
recorded rejected candidate queries: four hit the cropped low-Y row and one
hits an interior hole. Same-code offline replay reproduces exact feasible
status and residual1.8873791418627661e-15; epoch numbering restarts at1.
31 isolated standalone replays all match semantic output exactly. Capture
pipeline p50/p95/max=1546.845/1861.745/1953.679 us, including one snapshot
serialization and query logs each process. Model load is outside the timer.
Actual six attempted-capture timings p50/p95/max=1310.008/45866.77375/60611.528
us; the 60.6ms outlier is at period0.28 during the ramp, not solver iteration
proof. Report both isolated and closed-loop timing; no realtime guarantee.
The chosen next architecture change queries the immutable capture-heading
observation directly in world coordinates, preserving source unknowns and
actual age instead of losing extra cells in current-heading resampling.
