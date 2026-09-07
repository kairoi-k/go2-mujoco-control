# First actual QP task isolation
Runtime0a853346bff0005bc9175bd9d3f33eb01fa49e79, raw
`example/cpp/experiments/_runs/joint_execution_flat_20260908_0006`.
Flat diagnostic fails posture again; no QP failure. First actual secondary QP
captured at STATE21.020 includes the real primary optimum as equality rows,
seed, returned iterate and per-leg motion maps/bias/targets. No re-solve used
for capture; focused test verifies capture leaves torque bit-identical.
Independent SciPy HiGHS/SLSQP reconstruction reproduces baseline objective
-3226877.459268945 (actual -3226877.4592689294), original equality1.37e-12,
no inequality violation. Swing legs0/3 acceleration errors49.833/81.886m/s2.
Removing ONLY secondary qdd regularization (w_posture0.2 ->0) retains all
original constraints/primary outcome, gives50.013/74.381m/s2. This refutes
regularization as the main explanation at this state.
The equality-nullspace swing map has five substantial singular values and
one9.88e-16 singular value. At rank cutoff1e-10 the least-squares residual
floor is89.632m/s2 joint norm, with inequality slack14.648N. This is numerical
rank/attainability analysis, not an exact symbolic impossibility theorem.
The missing direction corresponds to task coupling under frozen primary
COM/orientation/stance accelerations; more secondary weight cannot restore it.
A separate DIAGNOSTIC removes only primary orientation locking (verified rows
9:12, sqrt40 times body angular acceleration); original orientation cost stays.
Swing errors fall28.363/42.613 but body angular acceleration becomes
[11.372,49.007,-9.422]rad/s2 versus[-0.438,0.768,-0.115]. This is NOT a selected
runtime fix: simply freeing body attitude trades leg error for large body motion.
The evidence calls for dynamically coordinated body/leg references and contact
realization, not a gain adjustment or unconditional removal of body regulation.
Reproduce each independent QP with the existing
`joint_feedback_replay_20260907/independent_qp.py MATRIX --out NEW_JSON`.
The variant matrices are derived explicitly by analyze_task_compatibility.py.
All variants are offline conditional problems, not new closed-loop evidence.

The production fixed-QP replay tool independently reproduces the captured
iterate exactly, and matches both SciPy swing-error results. Build target
`replay_joint_execution_qp`; CTest `test_joint_execution_qp_replay` fails if
baseline differs from captured iterate by more than1e-8. Its input schema is
explicitly scoped to this two-contact24-variable18-equality36-inequality layout.
Large instantaneous body angular acceleration alone is not a stability verdict:
natural periodic running can require substantial alternating acceleration.
Any relaxed-orientation candidate must therefore be judged on body excursion,
contact evolution and closed-loop behavior, not an invented acceleration gate.
No variant was activated by this offline study.

## Initial contact-motion audit
Independent Python MuJoCo evaluation of the exact retained source snapshot
STATE21.018 gives measured-support collision-center speeds FL0.28009067 and
RR0.41783440m/s. `audit_initial_motion.py` binds named actuator order and actual
sphere geom Jacobians, retains q/qvel unchanged, and calls no mj_step.
Its full vector results and source/model hashes are in initial_motion.json.
Measured contact cannot be equated with a stationary geom center.
Source admission audit confirms BuildExecutionProposal and execution-owner
ValidateProposal currently require the selected centroidal certificate, not
SolveJointTrajectoryCandidate. Runtime adds independent per-tick inverse
dynamics/torque checks, but these do not retroactively certify the planned
articulated trajectory. The existing body reconstruction fixes all four foot
velocities and rejects any change to source q/dq. A nominal zero-velocity stance
therefore cannot represent this moving measured support state without modeling
the transition. Do not reset state or silently loosen that initial-state check.

## Unmodified-state articulated diagnostic
Build `replay_joint_shadow_snapshot`, then run it on `articulated_source.txt`
with `--articulated-audit`. Exit 2 is the expected unverified trajectory result.
Runtime foot window21.018--21.218 is covered; extending to the complete
centroidal grid21.298 fails coverage_incomplete, before body reconstruction
(0 samples). Nested default body failure is not evidence of a numerical solve.
The separate source-state acceleration lift has rank18 and residual8.53e-14;
its conditional sample certificate has peak torque31.4344Nm, force residual
3.27e-10N and moment residual4.65e-10Nm. This does not certify physical contact
or a trajectory. It uses nominal foot acceleration without controller PD,
and recomputes the moment using actual application points. Both momentum
targets are recorded: they happen to agree exactly at this initial sample,
which does not establish agreement later. No state projection or mj_step occurs.
The tool builds and --roundtrip exits0. A requested three-test CTest regression
could not run because its executables are absent in this build directory;
no new regression pass is claimed. Runtime controller code is unchanged.
