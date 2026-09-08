# Whole-body MPC known-terrain diagnostic V1
Preregistered2026-09-08. Genuine B1 remains NOT_CERTIFIED. This route tests
whether event-spanning whole-body optimization can traverse the actual5cm scene;
known terrain, recorded full integration state and periodic nominal initialization
are privileged. It is not a sensor, realtime or new production command certificate.
The old packet is a nominal template only, never re-admitted as a runtime packet.
## Formulation
Same MuJoCo3.3.6 robot, gravity, soft contacts, joint/actuator transmission and2ms
physics. Decision variables parameterize joint motor torques across the whole
uncommitted horizon simultaneously. Body, feet and contact forces are dependent
trajectories of those torques under full articulated dynamics; no separately
prescribed zero touchdown velocity or subsequent force/limb stitching.
Native evaluator permits1..200steps, and exact immutable prefix0..steps. The
initial diagnostic targets70steps/140ms (one nominal running-trot period).
Every pre/post-forward sample has the existing48 physical inequalities: four
normal forces0..180N,12joint speeds<=30rad/s,24joint range bounds, baseheight>=.28m,
roll/pitch<=15deg, nonfoot/self-contact force<=1e-6N. Full friction/contact
semantics come from unchanged scene physics. Torque<=35Nm is checked on expanded
controls by solver and final replay, not silently clipped inside the evaluator.
This does not enforce running contact topology; measured phase/contact evidence
must be analyzed separately before claiming running traversal. Sampled constraints
are not a continuous-time collision or robustness certificate.
Cost is .5/N times sum over post-step states of squared normalized base position
(.025m), quaternion tangent(.10rad), base linear velocity(.30m/s), local freejoint
angular velocity(.60rad/s), four foot geom positions(.025m), plus .01 times
squared motor deviation from seed normalized35Nm. Angular references use MuJoCo
freejoint coordinates consistently. All references are soft objectives. They do
not turn force/dynamics violations into feasibility or change old thresholds.
## Verification and execution boundary
Native/Python full integration replay must agree in cost<=1e-10 and constraints
<=1e-9; these are cross-implementation audit tolerances, not feasibility slack.
Final inequality checks retain zero extra slack. Preserve full integration state,
absolute time, warmstart and unchanged physics callbacks/options. Reject missing
coverage, modified prefix, nonfinite values, numerical warning and late solves.
Only new synchronous known-scene research harness may use certified controls;
production owner remains untouched. No physical-state reset after initialization.
Known-scene success will require separate observed-terrain and real-time work.
First implementation audit uses actual5cm scene at original packet initialization,
1and70steps, nominal and deterministic perturbations, prefix/shape/nonfinite/
closed/unknown/quaternion rejection. This proves only evaluator equivalence.
Next experiment is rolling one-cycle optimization toward the5cm obstacle, recording
first useful failure, immutable prefix, full input/control/state and latency.

## Bounded near-obstacle canary
After harness smoke, use whole_body_oracle_step_5cm_near.xml: exact original
5cm scene except the plateau center translates from5.25to3.25m in X. Width,
height, friction, robot and initialization unchanged. Original frozen XML remains
untouched. This moves encounter from about2.6s to0.27s after initialization,
reducing repeated flat setup. It is a separate privileged diagnostic, NOT frozen
B1 acceptance or evidence of a naturally reached steady approach. No robot state
translation/reset. Bound first canary to100chunks (1s physics) and stop first
useful failure. Eventual candidate must run original approach and observed terrain.
## Diagnostic numerical consistency V1
The rolling oracle records the unchanged historical absolute1e-7 dynamics and
1e-12 clock verdicts separately. Admission into this privileged synchronous
experiment requires independent replay/physical checks plus component-scaled
force-balance residual<=1e-8, where each component denominator is1+sum absolute
terms in M*qacc+bias-passive-actuator-external-constraint, and absolute clock
error<=1e-10. This is a new diagnostic numerical rule, not a changed historical
contract or B1 acceptance. All physical inequalities retain strict zero slack.
The seed shifts the preceding accepted horizon by5steps and appends nominal
feedback under that actual seed rollout. Three linear correction knots are added
to the seed, with expanded35Nm inequalities, and the first5controls remain exact.
Foot support elevation uses the actual known box top at each fixed-phase target;
only swing interpolation and body height reference are smoothed.
## V1 implementation deviation and diagnostic numerical V2
Run0003 sourcecaae517 executed20chunks/100steps then search failed on predicted
RR189N. Independent sequential replay state/force/motor discrepancies0; absolute
balance2.266e-7 and componentwise balance1.412e-7 violate both historical/V1
numerical gates. Code erroneously gated only state/force/torque from the separate
verifier, while native sampled physical inequalities were strict. V1 numerical
admission was NOT implemented as registered:0002/0003 are retained raw diagnostics,
not conforming V1 certificates. Root owns this review failure.
V2 retains all old metrics/verdicts and fixes gating. For the explicit known-scene
experiment, use standard normwise generalized-force backward residual:
max(abs(sum(terms))) / (1 + sum(max(abs(term)))) <=1e-8, with terms
M*qacc,bias,-passive,-actuator,-external,-constraint. Components are expressed
numerically in1N translational/1Nm rotational units before this normalization.
This measures balance error relative to the full load scale, whereas V1 imposes
an extra near-zero-component relative demand unrelated to overall load accuracy.
The threshold is a new research numerical convention, not a solver optimality,
robustness or B1 certificate. Native/Python inequalities remain exact/no slack;
all other independent physical checks and clock<=1e-10 must pass. Old absolute
and componentwise failures remain visible. Every executed state/force is now also
compared against its admitted prediction before another physical step.
Read-only review found body reference height had an added terrain offset without
its vertical velocity derivative. V2 adds centered2ms derivative of the same
height reference; no velocity authority or local swing retiming changes.
Same failed state0003, same three knots, prefix and physical constraints, with20
instead of4SLSQP iterations yields a strictly sampled feasible witness in12.846s,
Python min inequality0 and exact prefix. This is an offline budget counterfactual,
not execution or resolution of numerical admission. Next bounded canary uses20
iterations/30s maximum per synchronous solve; preserve4iteration failure evidence.
## V3 task support affordance and geometry-derived swing envelope
Actual0004 contacts expose a structural defect: both front feet press on the
vertical step wall;31step contact samples, zero top contacts. Force/joint bounds
alone do not encode the intended support affordance. Do not call these contacts
successful touchdowns. The next change addresses this counterexample directly.
New optional wm_create_top_support ABI preserves legacy wm_create behavior.
Known world planes/axis-aligned boxes only; reject unknown/rotated/moving support.
Foot-terrain force is forbidden when the contact normal is not vertical to1e-6
or foot center is below the surface top. This deliberately excludes side/corner/
underside support for this flat-top traversal diagnostic; it is not a universal
terrain dynamic-infeasibility theorem. Existing nonfoot/self constraint adds
forbidden contact force with the same1e-6N near-zero contact tolerance. The default
canary requests this explicit backend; a legacy-only binary fails closed.
The soft swing reference uses continuous piecewise-linear nominal XY swept paths,
expanded by the actual sphere radius in XY. Segment/rectangle intersections find
first/last overlap with the known elevated box. A C1 surface-height envelope
reaches the required top elevation before first overlap, holds it during overlap,
and descends only after leaving. Initial/final footprint conflicts reject input.
No fixed extra height bump, local phase change or geometric15mm-as-dynamics rule.
Torques/body/actual feet remain one constrained full-body optimization; references
are not directly commanded trajectories. This still does not certify measured
running topology, observed terrain or realtime. Preserve all earlier diagnostics.
Before canary, seven geometry oracle tests plus native/Python flat fixture and
retained0004 side-wall witness must pass: legacy accepts the latter physical
bounds, top-support mode rejects it with identical cost and independently matching
constraint values. Then one bounded near5cm canary,20iterations/30s per solve.
