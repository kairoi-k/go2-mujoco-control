# Whole-body native horizon research V1
Status 2026-09-08: offline initialized privileged flat research only. No live
controller authority, new live flat, genuine5cm B1 or10cm acceptance.
## Native force kernel and immutable research packet
WholeBodyForceTracker optimizes direct12 motor torques nearest an existing
feedback command, with unchanged35Nm and180N strict pre/post-step limits.
Same-model nonlinear witness is independently replayed. Four derivative threads
reduce a fixed-state median17.034ms to5.674ms (five observations per mode);
eight threads6.171ms. Fast path median0.158ms. This is not a2ms guarantee.
Model contract/numerical reset tests retain all warnings as diagnostic evidence.
A reset-to-zero-contact state must not be treated as a feasible physical witness.
The packet uses absolute half-open coverage and model coordinate ordering;
nominal controls and gain evidence have distinct source hashes. Original failed
certificates remain failed. Parser/verification/feedback are shadow only; never
forge selected centroidal or ID-WBC force certificates. External trusted loading
must bind actual parsed bytes, model source closure, observation and identity.
check_whole_body_packet.py implements the fixture byte/source loader: pinned
packet/manifest, independently hashed sources, identical bytes via stdin. This
is a research test loader, not a controller model ownership implementation.
The corrected packet0002 receipt binds its manifest; earlier raw0001 receipt
had a model-digest overwrite and must not be used as a valid receipt.
## Completed20ms probe, dirty-source diagnostic
All exact source snapshots and independent replays are preserved in
whole_body_native_20260908/horizon_0001. Three candidates: nominal mismatch
control, actual+vy0.05 state, actual state with6ms already-published prefix.
Injected adoption0/2/6/20ms is simulation, not actual concurrent computation.
Actual/6ms-prefix generation11.230/9.910ms misses6ms.20ms is expired.
Nominal mismatch reaches188.715N. Independent candidate force/full integration
replay discrepancies are zero. One explicitly adaptive10ms adoption (ceil of
measured9.910ms to2ms grid) leaves10ms and stays within180N. Its first10ms
commands happen to agree although only6ms was committed: no deadline proof.
## Preregistered next single run: fixed12ms rolling schedule
Observation at each6-step boundary follows adoption of the previous candidate.
New candidate binds that active version; predicts its unchanged next12ms law,
then optimizes the next20ms (total32ms). No publication within a committed
prefix. Adopt only at observation+12ms, with measured generation<=12ms and
matching predecessor version. Discard late candidate. Continue only if the
previous active law covers the full next12ms prefix; otherwise stop immediately
with missing_committed_prefix. Never extend or rebase an expired law.
Single1400step(20cycle) continuous initialized replay, initial+vy0.05 only.
Periodic nominal/gains are explicitly reused solely as the offline reference;
this does not enable packet looping or provide a nonprivileged runtime seed.
First12ms bootstrap uses the existing nominal feedback. Actual states never
reset between iterations. Original model,35Nm and180N limits unchanged.
Stop at first useful failed prefix, infeasible candidate, coverage or actual
physical violation. No retry for favorable latency, gain sweep or threshold
change. Simulator advances serially after producer work; measured lateness gates
adoption, but neither this run nor its scheduler certifies live realtime.
Save exact source/inputs/binary hashes plus every observation, version, applied
command, state and pre/post-force witness. Independently replay actual states,
published control laws and candidate prefix consistency. Clean commit precedes
this run; the prior dirty-source probe is not relabeled clean.
## Observed collision reconstruction boundary
observed_collision_model.h accepts the existing immutable WorldTerrainSnapshot,
uses one complete lidar capture selected by existing conflict/freshness queries,
and freezes discrete cell-prism descriptors in world capture coordinates.
No scene ground-truth enters this interface. Scalar cell-top reconstruction,
extrusion depth and sidewalls are explicit assumptions; observed height bounds
are not continuous support or solid-volume certificates. Unknown holes are never
filled. QueryCoverage includes query time and checks descriptor geometry as well
as original source coverage. This is discrete footprint coverage, not a proof
of a continuous full-body swept volume.5/10cm fixtures are geometry tests only.
Live integration still needs robot-only collision model construction, complete
body collision coverage, nonprivileged trajectory generation and one atomic
execution owner. Initialized periodic references cannot supply those proofs.
