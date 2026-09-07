# Registered short-horizon joint feedback diagnostic
Decision before runtime. Source baseline9e3313103e74e2c19b06c4fd16a8e917e3d2e3e4.
The real cc69623 flat run confirms final PD/feedforward composition can saturate
far beyond the model limit despite a small feedforward torque. It also exposes
~50ms capped-QP proposal outliers. Neither is a proof of the sole B1 cause.
## Hypothesis and exact boundary
A selected event-indexed joint COM/momentum/force/foot reference can be tracked
by the existing articulated WBC under actual nonlinear MuJoCo contact evolution,
without uncontrolled extra motor PD. First test one0.20s prefix, not fullB1 or
production adoption. The new backend uses explicit torque-only motor control,
analytic foot acceleration feedforward plus feedback, and a separate attitude
objective in the same WBC. It is a new feedback-backend counterfactual, not a
replay of the legacy PD controller or a claim of improvement from tuning.
Input: immutable real f9623e1 state20.100 snapshot retained in
joint_map_coverage_20260907/actual, with raw/provenance hashes. Current code already
reproduces a feasible reduced proposal on that old snapshot. The cc69623 v2
state20.562 snapshot is a negative input: initial contact-anchor unavailability
must remain a rejection; do not fill it to manufacture a rollout.
Plant: phase2_flat.xml, same Go2 robot MJCF and actual recorded initial q/dq,
1:1 joint/actuator mapping,2ms mj_step. Contact solver warm start is reset,
explicitly a cold start. Subsequent feedback reads exact simulated state; this
is a privileged-state diagnostic, not an estimator or sensor robustness result.
Scene contact truth is evaluation only. Terrain references come from the
recorded observation; no scene height is injected into planner unknown cells.
Stationary nominal stance references may have zero reference velocity while
actual stance joint/material velocities remain untouched and logged.
Record nominal, controller, force-threshold and geom contacts separately.
Report true contact force AND torsional/rolling couples (foot condim6), since
control uses a point-force approximation. Track reference error, actual
pose/q/dq, solver/certificate residuals, requested/applied motor torque and
saturation, nonfoot contact, and any first failure. Completion alone is not a
tracking or safety certificate. A failed sample is retained, not filtered out.
Use the exclusive experiment lock; no concurrent build/archive/agent work
while timing or stepping. Bind clean source and binaries before execution.
Stop at the first useful failure and repair its cause. If this prefix works,
proceed to atomic production reference adoption and flat/5cm closed loop;
10cm follows credible5cm results. Historical acceptance and scenes stay intact.

## Implementation verification (2026-09-08)
The replay is integrated into replay_joint_shadow_snapshot --closed-loop.
A root-authored independent reference test reproduced the draft's erroneous
force.start==sample_time check, then passed after accepting the half-open
force interval. It also checks analytic COM velocity/momentum feedback
differences, unmodified actual dq, force frame/sign and plant clock retention.
The optional WBC attitude task has an analytic coupled-QP optimum fixture.
Actuator names and indices must both match before scene ctrl is written.
The robot model is the retained same-MJCF model; terrain references remain
observation-derived and actual force application uses the current sphere
material point. Contact forces/couples are observations, not nominal forces.
The original solver iteration cap and reduced feasibility semantics remain.

First exact b984105 attempt has zero physical steps: the tool rejected an
interior replay endpoint. This is a replay precondition defect, not physical
infeasibility. The next revision restricts only the foot schedule prefix and
keeps the original dynamics grid/result; independent off-grid prefix test added.

Second exact ac423a6 attempt also has zero physical steps: identical robot
models were rejected because the canonical free joint is unnamed. Match its
unique base body/type with unchanged qpos/dof layout checks; the actual scene
identity and deliberately changed gravity/gear now have focused tests.

Third exact30e2564 attempt matches the robot but still has zero physical
steps: source quaternion norm error is-1.5618637605463448e-8 from DDS float
representation. The replay had invented a1e-8 unit-norm gate inconsistent with
Go2RigidBody::SetState. Use that existing normalized representation; joint
q/dq, physical rotation and body velocities remain unprojected. This third
preflight failure triggered review: remove duplicated stricter representation
contracts rather than treating adapter defects as plant failure or tuning
dynamics. The focused test now includes a float-rounded quaternion.

Fourth exact45c36e4 attempt has one retained initial observation and no
applied step: foot references are unavailable. Source inspection identifies
another contract mismatch: candidate surfaces cover the280ms dynamics horizon,
while the foot sampler requires the full contact end beyond it. The opt-in
feedback prefix now requires surface coverage through min(contact_end,
max(replay_end,touchdown)); it does not modify event lifetimes or unknown
coverage. Legacy defaults remain strict. Independent fixtures cover both
contracts and reject missing touchdown coverage. Runtime next checks whether
this resolves the actual rejection; this cause is not yet physically tested.

Fifth exact37e900e attempt resolves reference coverage and reaches WBC.
The first sample is rejected before actuation: tau max35.0697429Nm exceeds
its35Nm QP bound, qp_converged=false, while dynamics residual is~1e-13.
This is now a numerical QP investigation. Preserve the physical certificate
and limits; export original matrices and compare with independent SciPy
feasibility/optimization before selecting a numerical fix. No step applied.

Sixth exacte18bf52 exports the identical rejected QP. Independent SciPy1.15.3
HiGHS finds a feasible witness (equality2.96e-12, inequality9.09e-13).
After nullspace/Cholesky/global coordinate scaling, independent SLSQP succeeds
in2 iterations, original objective-97581902.95771791, equality4.06e-12,
inequality9.78e-11. Original ADMM iterate has inequality0.0697428688Nm.
A separate full-M freefall witness has equality8.88e-16 and inequality0;
it proves feasibility only, not useful tracking. The opt-in replacement uses
that verified seed and a deterministic primal active set for the same SPD QP.
No friction/force/torque/certificate threshold is relaxed. Positive normal
floors or hard stance constraints require a verified feasible seed and must
fail closed when the freefall seed violates them. Historical solver stays default.

A separate reference risk is already identified: the source is mid-swing
with29.398932ms remaining; restarting a full30mm bump demands initial foot
accelerations959/1079m/s2. Keep this unchanged for the numerical comparison,
then investigate committed-curve continuation rather than hiding torque limits.

Seventh exact5e75b5e completes100 actual2ms torque-only MuJoCo steps. All
100 WBC solves converge and pass independent physical and final motor gates.
QP p50/p95/max86.069/116.202/179.612us; force/moment/joint residual maxima
2.84e-13/7.11e-14/7.11e-15; motor saturation0. The numerical defect is resolved
on this prefix, not globally certified. Tracking is poor: max COM error48.95mm,
max foot error54.80mm, base height falls366.69->312.97mm,23 nominal/geom mask
disagreements, no nonfoot contact. Completion is not tracking acceptance.

Next isolate the already-identified mid-swing bump restart: preserve measured
initial p/v and the absolute touchdown, but add no fresh clearance bump to an
already-inflight continuation. Future full swings retain30mm clearance. This
is still a measured-state counterfactual; production adoption must preserve
the actual commanded committed curve, unavailable in this old snapshot.
Body/stance task weighting remains unchanged for this comparison.

Eighth exact091399f also completes100 steps with every physical/motor gate.
Removing the repeated initial bump lowers max foot error54.80->25.57mm,
but COM error worsens48.95->89.88mm. No nonfoot contact, no saturation.
Thus the bump was not the sole tracking cause. The same low COM weight1
versus swing80/stance8 allows whole-body tracking to be traded away.

Next registered architecture change: opt-in two-level QP in the same WBC.
Primary tracks COM acceleration, absolute attitude acceleration, and stance
center acceleration under the same dynamics/cone/force/torque constraints.
A1e-6 diagonal regularizer makes the primary Hessian SPD. Secondary preserves
all achieved primary task values exactly and optimizes the existing full cost
(swing, nominal momentum/forces, posture/torque). Nominal momentum is a soft
reference; it cannot override body/stance to force limb angular momentum.
No contact truth is invented, no actuator bound changes, historical weighted
solver remains default. An analytic conflict test must show a swing target
cannot erase the achieved primary COM task. Then repeat the identical prefix.

## Body/support priority short feedback (runtime fb837929)
Attempt 0009 retains the original raw files plus independent audit and Python
MuJoCo replay. All 100 applied rows converge and pass physical/motor checks;
COM max 16.382 mm, feet max 29.753 mm. QP p50/p95/max 73.315/113.525/206.282 us.
Independent replay state, actuator and clock residuals are exactly zero.
The plant still reports 32 nominal-versus-geom mask disagreements and nonzero
contact couples; this is not proof of exact scheduled contact execution.
The historical weighted comparison 0008 had COM max 89.877 mm. These are
short counterfactuals, not traversal, robustness, or full-controller acceptance.
Negative 0001 retains the separate actual anchor rejection unchanged.
