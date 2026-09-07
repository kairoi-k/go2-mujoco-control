# Joint articulated route (2026-09-07)
Status: implementation under validation. This document is not B1 acceptance.
The user authorized architecture replacement, 5 cm closed-loop validation first,
then an independent 10 cm challenge. Historical Phase-2/V3 and C0 evidence remain
separate. Neither module completion nor a model certificate implies traversal.
## Formulation and ownership
The outer search evaluates combinations of event-indexed terrain points. A
single absolute gait origin/epoch supplies liftoff, touchdown and stance-end
intervals; replanning cannot reset event times. Every candidate carries its own
world surface basis, friction assumptions, force bounds and map provenance.
For one combination, the existing centroidal SCP solver integrates world COM,
velocity and whole-body angular momentum under zero-order-held contact forces.
It uses original gravity, unilateral/conservative friction and force bounds;
aerial intervals are ballistic. Optional grid-indexed objective references
permit terrain-dependent COM elevation without changing hard dynamics or the
historical constant-height default. Original-equation residual verification is
independent of optimizer rows/status. The historical geometric 15 mm diagnostic
and frozen aerial conflict stay diagnostics, not dynamic feasibility tests.
A planning-only view of the **same** Go2 MJCF exposes COM Jacobian and whole-body
angular momentum matrix. Given all four foot-center trajectories, base XYZ and
joint positions are reconstructed at the current orientation. COM velocity,
angular momentum and foot velocities determine the full generalized velocity;
orientation is integrated on the MuJoCo quaternion manifold. Acceleration is
lifted with the same matrix and a centered configuration derivative. There is
no fixed COM/base offset or rigid-inertia substitution for moving-leg momentum.
Failure of this constrained lift is not a proof that all body motions fail.
Contact surface point P, sphere center C and foot site are distinct roles.
Future C = P + r*n uses the actual model radius and candidate normal. Force
Jacobians at P are separate from center motion Jacobians. The ID-WBC solver and
its independent certificate share selection of these force Jacobians; the old
center path remains the explicit default until the new runtime route selects
actual points. Tangential force at a 22 mm offset changes torque by 2.2 Nm per
100 N; a radial force has no such shift.
The candidate evaluator combines centroidal rollout, articulated reconstruction,
acceleration and the existing full-model ID-WBC residual/torque checker. It also
checks exact per-patch circular cones and joint bounds. Its current result is
**sample-level model evidence**. `execution_ready` remains false until geometry,
actual-state feedback, final command/actuator validation and coherent execution
are provided. Optimized inverse-dynamics tau is not the final PD motor torque.
## Temporal coverage
Foot references are C1 in absolute time. An already in-flight leg starts from
its observed center and velocity and reaches the original touchdown time.
A stance trajectory cannot silently erase measured initial joint/foot velocity.
A terminal swing whose touchdown lies beyond the dynamic horizon needs an
explicit continuation target, using the same event/candidate/surface types.
That continuation is not a beyond-horizon dynamics certificate or permission
to commit it. The terminal state query is separate from half-open force/reference
interval queries. Missing schedule, surface, point role, provenance or tail
coverage fails closed.
## Validation boundary
Synthetic oracles include actual-model finite differences, translating and
rotating articulated trajectories, virtual work at shifted contact points,
static support and aerial dynamics, multi-event time coverage, commitment and
unknown-input rejection. The V4 analyzer preserves physical/topology/speed and
stop-tail tests while versioning the fixed-period and registered-height rules;
it retains historical V3 output and has independent 5/10 cm scene registration.
A single passing run is still not a registered B1 campaign.
Open before empirical claims: full runtime adoption, snapshot/phase ownership,
contact compliance and rolling/slip semantics, swept robot/terrain geometry,
final actuator composition, latency and failure/replanning behavior. These are
research tasks to resolve with real MuJoCo feedback, not reasons to relabel a
partial model result as success.
## First controlled runtime probe
`TROT_RESEARCH_CONTACT_POINT_MODEL=1` opts the existing ID-WBC path into the
same model's sphere-surface force Jacobians. It uses the same captured state,
actual radius and declared terrain normal. Missing normals retain the existing
flat assumption and its certificate assumption mask; this is not a measured
contact-normal claim. `wbc_full_force_application_jacobian_used` records actual
selection per row. Default off preserves the historical center-force path.
This isolated probe is not joint-planner adoption. Its purpose is to measure
whether correcting the force-point convention preserves closed-loop stability
before the joint backend supplies authoritative points and complete references.
