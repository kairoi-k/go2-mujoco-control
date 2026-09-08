# Go2 current research checkpoint
Updated2026-09-08 at user-requested handoff. This is the live route entrypoint.
## Outcome and restart boundary
**Genuine5cm dynamic running-trot B1 remains NOT_CERTIFIED;10cm not started.**
Branch feat/stage-c-joint-planner; canonical native WSL worktree:
/home/che/dev/go2-workspace/feat-stage-c-joint-planner.
Resumed from0349725c90cf5e3a15addfc8715a32f21a3a98bd. Latest implementation source
is db10b5502110801fd3466b685f4dd3172c6d39d5; final documentation/evidence commit is
its descendant (use git rev-parse HEAD). No remote push was requested/performed.
Read docs/research/SESSION_HANDOFF_20260908.md top section before new work.
## Latest decisive results
The actual integrated controller remains source0f6ec6526f4fcde77f2b737b5a65d9855ed5365b,
flat attempt0012 stopping21.204s for roll28.80deg. No newer live controller run.
The new full-body backend is still initialized privileged offline research.
Clean54d750eb4b1730db9041abd384d126dbe1f7a028 ran one continuous20cycle/1400step
+vy0.05 diagnostic: fixed12ms already-published prefix plus20ms optimized tail.
All233candidates adopted, zero late; generation median3.168667/max11.2341ms.
Actual/candidate full-state and force replay discrepancies0; prefix command law
error<=4.44e-16Nm. Force179.99999999999952N, torque28.35590Nm, minheight0.36753384m,
nonfootcontacts0.19/19complete phase-zero cycles pass running-contact diagnostics.
Frozen V1 retains FAIL time1.55209e-12>1e-12 and dynamics3.98340e-7>1e-7;
19other checks pass. This is synchronous scheduling, not a liveRT guarantee.
Clean db10b55 observed-node0001 reads all11653tokens of the actual recorded
snapshot881/time21.020, two lidar captures295/294, age limit unchanged0.20s.
23collidable geoms: fourfeet and17others pass old rbound-square queries; two
right-thigh boxes fail outside. One whole-body r=.399793m square also fails.
No observed model was compiled for this actual snapshot because Build rejected.
The next analytic projection audit explains the conservative failures: true
FR/RR box XY extents leave70.206/69.375mm boundary margin in capture295, while
old spherical bounds exceed it by25.316/26.068mm. All23primitive projected
rectangles have a fresh complete capture with no unknown/stale/nonfinite cells.
This is NOT existing-API admission, history-conflict recertification, continuous
sweep or surface/support certification. No thresholds or map cells changed.
A separate exact-arithmetic audit locates the full-query invalid_query label:
nextafter(maxY,-inf) passes InBounds, but subtraction/division rounds cell index
to10==height, failing world_terrain_snapshot.h:461. The original broad square
really exceeds Y coverage; fixing only its error label cannot make it valid.
## Next work, not performed during handoff
First replace the overly broad research coverage query with shape-aware projected
regions while retaining unknown/stale/outside/history-conflict rejection. Keep
single-capture complete evidence per supported region; never fill holes, widen
sensor coverage or convert these analytic results directly into an admission.
Then construct the observed worker-local collision model for the actual snapshot
and validate force/contact behavior. Cell prisms reconstruct scalar tops with
explicit0.30m depth and inferred sidewalls; internal seams/contact multiplicity
and full-body swept coverage remain untested. Preserve robot XML/asset identity.
The eventual runtime still needs nonprivileged reference generation, command/
observation/terrain identity, one versioned atomic execution owner, shadow and
admission/command composition evidence. No forged centroidal selected or ID-WBC
force certificate, no consumer-local recovery, new gait or local swing retiming.
Only then new live flat, genuine5cm and independent10cm under versioned contracts.
Do not rerun successful initialized flat probes or resume gain/threshold sweeps.
## Tests, artifacts and execution
CMake builds/tests pass for whole-body force tracking(21checks), trajectory
packet, observed model, complete v2 reader(10checks).23focused Python checks pass
(packet10,20ms coverage6,rolling schedule7); earlier30Python checks remain bound
to their prior changes. The production real_trot target was not changed/built.
Native evidence: docs/research/evidence/whole_body_native_20260908, top MANIFEST.
rolling_0002 retains full40MB raw run losslessly as run.json.xz; other large
artifacts use gzip. Curation files bind original raw bytes; failed raw attempts
remain untouched. Earlier cycle/derivative/feedback/constrained packets remain
under their existing20260908 directories. Original V1 failures are retained.
Protocols: WHOLE_BODY_SHOOTING_V1.md, WHOLE_BODY_FEEDBACK_DIAGNOSTIC_V1.md,
WHOLE_BODY_CONSTRAINED_FEEDBACK_V1.md, WHOLE_BODY_NATIVE_HORIZON_V1.md.
Use pinned localhost SSH helper, not hanging wsl.exe. From PowerShell pipe a
Bash here-string to python at
C:/Users/w1881/Documents/Codex/2026-09-06/feat-stage-c-joint-planner-fetch/wsl_exec.py.
MuJoCo3.3.6, native SciPy/Eigen. OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
MKL_NUM_THREADS=1; native derivative threads explicitly4. Serialize builds and
physics/timing; hold /tmp/go2_mujoco_experiment.lock. Preserve all _runs/stashes/
other worktrees. User requested handoff; workers stopped, no new work pending.
