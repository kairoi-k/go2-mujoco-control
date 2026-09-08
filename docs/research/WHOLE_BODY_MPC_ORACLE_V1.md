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
