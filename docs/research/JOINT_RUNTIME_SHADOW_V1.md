# Joint runtime shadow V1
Research protocol, 2026-09-07. Not command authority or B1 acceptance.
`TROT_RESEARCH_JOINT_SHADOW=1` runs the event-indexed centroidal combination
backend on captured actual articulated states in the terrain worker. A private
instance loads the same production MJCF. The worker consumes the identical
immutable registered sensor map used by the reference planner. It never reads
simulator ground truth. Existing command generation remains the control.
For the first bounded probe, capture at most once per 0.5 s in state time
18--24 s, preview 0.28 s with event-aligned intervals no longer than 20 ms,
and evaluate at most two best-first whole combinations. The original SCP
centroidal model and independent original-equation verifier establish ONLY a
reduced-model proposal. This is not a realtime deadline or optimality claim.
Record clock epoch/error, candidate coverage/failure, event and combination
counts, QP/SCP iterations, residuals and elapsed time in `JointShadow` rows of
the controller log. A shadow worker can affect scheduling; measured gait
results remain diagnostics and require a same-source off control if compared.
The phase clock keeps one stable origin per fixed timing epoch. Observed drift
fails closed. A changed period/duty/offset creates a new epoch only without
commitments; changing one with commitments is a conflict. This does not yet
provide variable-timing commitment management or authoritative execution.
Initial support positions are model sphere-surface estimates conditioned on
measured force and known terrain normals. Their displacement from the sensor
map plane is explicitly reported; they are not copied from foot sites, snapped
to the terrain, or labelled ground-truth contact points. Future surfaces carry
known patch coverage and explicitly assumed mu=0.8/max normal=180 N and stationary
validity through this prediction horizon. Bounds are development model bounds;
no full geometry or actuator feasibility follows. Constant-height velocity
references are the first control before candidate-dependent terrain elevation.
The optional articulated WBC objective uses the same model's `[Jcom; Amom]`
and its configuration derivative. It replaces the base-acceleration objective
only when explicitly selected, leaving physical constraints in the same QP.
The feedback-step helper preserves actual q/dq and reports material-point
velocity and normal gap. Constant-acceleration configuration integration is a
preview, not a MuJoCo contact simulation or a certificate of contact evolution.
Production terrain remains an immutable heading-relative grid. World queries
transform XY by the registered pose, shift all height bounds by registered Z,
and rotate normals by registered yaw; no map relabelling or second resampling
is allowed. Native world fixtures retain identity coordinates. Measured contact
validity and the actual configured gait pattern are passed through explicitly.
Force-conditioned geometric anchors carry a separate provenance enum and the
reduced certificate records their use; unknown provenance is rejected.
The first live probe retains historical wall-clock motion. If stable-period
observations exhibit phase/state-time drift, a second same-source flat probe
sets `TROT_RESEARCH_STATE_CLOCK=1`, removing the wall-clock flag and selecting
the existing state-clock implementation. This registered branch changes the
clock authority explicitly; it does not silently rebase committed events.
Neither run is a B1 candidate. The next command experiment requires a complete
shared body/foot/force bundle and independent execution/geometry review.
