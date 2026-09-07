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
