# Executor tracking diagnostic: FAILED
Actual clean runtime4b09ccd217130c77b492040554c891d29746c7f5,
raw `example/cpp/experiments/_runs/joint_execution_flat_20260908_0005`.
Same fixed-start flat protocol. Logging is sampled and asynchronous start differs
from attempt0004; this is not an identical-state numerical replay.
At first sampled joint command STATE21.022, swing legs1/2 have requested versus
solved Cartesian acceleration norm errors41.76/70.42m/s2, while position errors
are12.8/12.0mm. Stance tasks are achieved in the model to displayed precision.
At21.060 leg1 is planned swing but measured contact; at21.066 all four measured
contacts coexist with planned aerial. Ground-truth confirmation is still needed;
these masks are sensors, not collision-truth replacement.
By21.086 leg1/2 position errors reach61.0/47.8mm. This evidence puts motion-task
compromise and actual contact timing before terminal numerical failure.
At21.402 the secondary QP fails active_KKT_numerical_failure, repeated identically;
planned mask6 versus measured1. Its exact matrices are retained separately.
191actual zero-extra-PD CSV commands,13versions, sampled maximum foot error
0.414984m. No B1 claim and no physical infeasibility claim from numerical failure.
Next use retained task/state evidence to investigate primary/secondary task
competition and contact-timing mismatch before further parameter experiments.

## Contact realization and task hierarchy review
Reproduce sampled-task calculations with `../analyze_tracking.py tracking_excerpt.txt
--out NEW_OUTPUT.json`. At21.022 the commanded stance bridge prescribes world-z
accelerations -23.4726/-26.8239m/s2 for legs0/3. WBC achieves corrected tasks
-22.9168/-26.2217m/s2 while assuming normal forces86.8356/36.2286N.
These values are model predictions, not measured force or acceleration.
Source review: `joint_feedback_reference.h::BuildWbcReplayInput` adds the same
reference acceleration and clipped position/velocity feedback to stance and swing.
`inverse_dynamics_wbc.h` uses soft stance tasks by default, and the primary HQP
preserves their optimum together with COM and orientation. It does not impose
normal contact acceleration compatibility. The independent inverse-dynamics
certificate checks equations, force cones and torque limits, not whether MuJoCo
will realize the assumed force under its compliant unilateral contact law.
Thus command C1 continuity plus a model-equation certificate is insufficient for
physical contact-consistent handover. This is a verified certificate coverage gap;
it is not yet proof that the bridge is the sole cause of the failure.
The secondary cost mixes swing acceleration tracking, angular momentum, joint
acceleration regularization and force/torque terms. `w_posture` penalizes qdd,
not posture error or joint velocity. First sampled maximum absolute qdd is
984.1924rad/s2 or m/s2 depending row. The named task hierarchy does not guarantee
swing-task preservation against these secondary costs. A fixed-state comparison
is needed before changing weights or replacing hierarchy. No such change is made
in this checkpoint.
Next experiment must isolate actual contact realization and task competition
at a retained state, with all original tasks/constraints and state provenance.
Do not continue long canaries or increase gains solely to reduce these residuals.

## Collision-truth cross-check
`../analyze_contact_divergence.py --out NEW_OUTPUT.json` reads the actual raw
run and uses FOOT-only world-z GRF fields, not terrain-obstacle mask or all-leg
collision force. FL sustained foot contact starts21.054s with188.3909N; RR
starts21.066s with118.4611N. Both are before their sampled planned support.
The independent raw checks confirm the per-leg and foot-only forces agree at
these onset samples. The first sampled state21.022 has actual FR/RL foot normal
forces64.1442/86.1232N, versus WBC model86.8356/36.2286N. This is a synchronized
state comparison, not a same-tick actuator-response identification: transport
and integration timing must be retained when interpreting the difference.
The audit retains preceding/following2ms truth rows, raw input hashes and source
SHA. No new simulation, controller setting or acceptance threshold changed.
The next fixed-state task comparison must first establish complete replay input;
source planner snapshots alone cannot stand in for later control-tick state.
