# First atomic joint execution flat canary

Registered before execution. Run `joint_execution_flat_20260908_0001`, 32 s,
phase2_flat.xml / b1_v3_running_1mps.csv, seed11/domain231, period.14/duty.44,
state clock, map intervals V2, shadow and joint execution on, old contact-point
probe off. Use run_b1_research_probe.sh under the shared experiment lock.
The joint worker runs at the existing single producer boundary with a 20 ms
snapshot cadence, binds accepted event commitments before solve, and publishes
an immutable selected proposal. LowCmdWrite is the only motor writer. New mode
bypasses old per-leg terrain transactions and legacy candidate planner execution.
Standing and normal stopping remain owned by the existing controller. There is
no new contact FSM or measured-contact promotion. Failed/expired accepted joint
execution requests the existing stop owner and records a failed diagnostic.

First command handover uses geom centers reconstructed from old commanded joint
positions and their consecutive world-position differences (<=10 ms). Actual
plant q/dq remain untouched. A named soft stance-reference Hermite settling
bridge preserves C1 and returns to the same p0 with zero velocity within 20 ms,
clipped before the next liftoff. It is a tracking reference, not a claim of
measured stance motion or a dynamic feasibility proof. Existing committed swing
polynomials survive the old body horizon, while body/force bundle expiry remains
strict. Motor output is zero extra PD and certified WBC torque in motor order.

Checks: actual adoption count and versions, continuity/expiry rejection reasons,
body/foot tracking, solver/certificate failures, final raw motor command and
MuJoCo physical safety/running results. No B1 acceptance is claimed by this flat
run, wrapper exit, focused tests or a short replay. Preserve the known old
profile-analyzer KeyError and all first failures. The simulator and full runtime
source/binary hashes belong to the new run manifest; historical manifests stay
unchanged. Current double-precision WBC certificate and final serialized motor
composition remain distinct quantities.

## Actual attempt 0001: runtime f5b2ef98364bbc01f934de575c850696236e1342
Joint execution adopts at STATE20.004, retains three accepted versions, and raw
CSV has 121 rows with every motor kp/kd zero in [20.004,20.246). Logged feedback
p50/p95/max 212.247/244.446/257.497 us are sampled, not full-loop deadlines.
Sampled COM/foot maximum errors are 8.114/78.542 mm. At20.068 the legacy kernel
changes period .16 to .14; PhaseClock changes epoch1 to2, then commitment
binding rejects every following proposal. At20.246 the old body bundle expires
and requests the existing stop owner. This is a FAILED flat diagnostic. A
small COM error and feasible per-tick QP do not establish locomotion success.
The old profile KeyError is separately retained. Full raw CSVs remain in _runs;
curated active-window rows and complete controller output have local hashes.

## Next fixed-schedule isolation (registered before execution)
Run joint_execution_flat_20260908_0002 with TROT_RESEARCH_JOINT_START_S=21 and
otherwise identical flags. This isolates continued joint feedback after the
legacy period has settled; it is NOT a repair or acceptance of variable-period
execution. Capture now correctly passes active commitments into PhaseClock,
so an incompatible timing request cannot silently mutate the accepted epoch.
A durable event-calendar transition design remains an open architecture item.

## Actual attempt 0002: runtime 8f3e04afa97507ff4151838c4edf608923d2702e
Fixed-period execution adopts at21.018 and applies46 commands, with46 raw CSV
zero-PD witnesses, across3accepted versions. The period remains .14/epoch1.
At21.112 WBC fails and requests stop; the first preceding planner rejection was
initial_contact_anchor_unavailable at21.062. Sampled COM/foot errors reach
1.926/63.569mm; sampled feedback latency p50/p95/max199.878/243.146/251.526us.
This is another FAILED flat diagnostic. It separates the numerical/feedback
problem from the earlier period transition; neither is considered solved.

Next registered diagnostic: joint_execution_flat_20260908_0003, identical fixed
start21s protocol. On a failed WBC call only, reconstruct the same deterministic
QP with original stage matrices, seed, and active-set failure detail. Do not
replace the rejected command with this second solve. This capture distinguishes
primary from secondary HQP failures without adding successful-tick matrix copies.

## Subsequent diagnostic index
All remain FAILED flat diagnostics, not B1 acceptance. CURRENT.md owns the live
next action; historical registrations above are retained.
- attempt_0003: exact primary-QP numerical failure, independent KKT and QR repair.
- attempt_0004: repaired solver reaches physical tracking/posture failure.
- attempt_0005: actual per-leg tasks and collision-truth early contact audit.
- attempt_0006: exact first-QP task compatibility, production/independent replay,
  and actual-source initial contact-motion audit. Its source-state audit is not
  the later control tick or a new closed-loop experiment.
- attempt_0007: shared orientation/swing-priority candidate fails posture; not
  promoted. Runtime default remains the previous hierarchy.

Terminal-binding follow-up: [terminal_binding_audit](terminal_binding_audit/README.md)
retains25 conditional terminal choices, full coverage and subsequent initial
condition rejection. This advances diagnosis, not closed-loop acceptance.

Actual coherent-body canary [attempt_0008](attempt_0008/README.md) fails posture
with four versions. Exact first-QP replay passes; initial legacy-command
handover reference differs substantially from actual foot velocities.

[attempt_0009](attempt_0009/README.md): proposal-based first acquisition avoids
legacy settling, but actual execution expires after154 commands. Contact anchor
failures and delayed-proposal commitment conflicts both need resolution.

[attempt_0010](attempt_0010/README.md): bounded admission yields15 observed
on-time versions without sampled commitment conflict, but actual posture fails.
Task-compatible acceleration alone does not establish body attitude stability.
