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
