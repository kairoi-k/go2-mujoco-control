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
