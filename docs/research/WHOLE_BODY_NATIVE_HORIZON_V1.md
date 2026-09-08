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

## Executed rolling0002 on clean54d750e
The registered single run completed1400steps and233adopted candidates with no
late results. Median/max generation3.168667/11.2341ms. Force and torque remained
within180N/35Nm; exact independent actual-state/force and candidate-state/force
replays match.19/19complete phase-zero cycles have both diagonals and aerial
contact episodes. Frozen V1 retains only time1.55209e-12 and full-dynamics
3.98340e-7 failures, not an unqualified V1 PASS. See rolling_0002/analysis_v1_0001.
The original40MB run is losslessly xz-compressed; curation.json binds raw digest.
observed_collision_mujoco.h now builds worker-local models from robot-only XML
plus the descriptor, with no imported world geometry. Fixtures verify preserved
inertia, passive dynamics, actuator and robot collision settings and cell poses.
The caller must bind canonical XML and recursive asset hashes: structural
RobotOnly validation is not cryptographic source verification. No trajectory
has yet been evaluated against this reconstructed model.

## Preregistered recorded observed-node probe
Run inspect_joint_observed_collision_node once on the complete immutable v2
initial_source.txt from joint_execution_flat_20260908/attempt_0012. Its frozen
SHA256 is b86474ca4e4e2bafa02d8db75e91499030865ab4fa67c46d7d664b8a94ad5a16.
Use canonical robot-only go2.xml with the packet0002 recursive XML/mesh hashes
independently checked first; no scene floor may enter compilation. Max cell age
is the existing kTerrainMapMaxAgeS=0.20s, explicit prism depth0.30m. Depth is a
research reconstruction choice, not measured support thickness. Complete q/v and
all captures are read; unavailable integration memory is not claimed recovered.
Current-node mj_kinematics uses true collidable geom centers and model rbound.
Report full-body enclosing-patch coverage and every geom/capture failure. Missing
coverage prevents descriptor/model construction, never induces flat fill or a
contact promotion. No mj_forward, mj_step, fullbody trajectory or B1 trial is
part of this probe. Stop after this informative result; no age/coverage sweep.
