# Whole-body shooting V1: bounded full-cycle capability diagnostic
Registered 2026-09-08 before the first full-cycle solve. Historical acceptance,
controller, models, contracts and analyzers remain unchanged. B1 NOT_CERTIFIED.
## Hypothesis and scope
Centroidal feasibility plus independent prescribed foot interpolation does not
ensure actuator-feasible body/limb motion. Optimize the entire articulated
state trajectory through the SAME MuJoCo Go2 model and compliant contacts over
one complete 0.14s running-trot calendar, initially on flat. Start from saved
0012 source21.020 q/dq, not a substituted easy state. This source lacks original
integration memory: omitted fields use model defaults, explicitly serialized
as mjSTATE_INTEGRATION. This is a privileged initialized counterfactual, not
exact original simulator continuation, live sensor-based control or B1.
## Formulation and experiment
State x=(configuration on manifold, generalized velocity, activation); controls
are twelve actuator-order motor torques, held4ms and evaluated in the original
2ms physics timestep. Full MuJoCo dynamics determine body, limbs, contact forces,
landing velocities and contact transitions jointly, including passive friction,
gravity and actual collision geometry. No velocity reset or model-option change.
The immutable prior0.14s actual run supplies an initialization/reference only;
it is not a scripted execution route. Its trajectory is translated to current
state/command displacement; optimizing torque can change all resulting feet and
body states. Initial q/dq and the absolute horizon are fixed. A phase-relative
terminal reference retains initial joint/velocity/pose with commanded forward
translation. Body/foot tracking, joint limits, force excess and regularization
are named objective terms. Torque +/-35Nm is a solver bound; other penalties
DO NOT establish feasibility. Event schedule and measured/realized contact are
reported separately. This first diagnostic has no terrain selection or committed
bundle to certify; it must not be described as a delivered full terrain planner.
Use scipy bounded nonlinear least-squares with exact nonlinear rerollouts and
optional MuJoCo transitionFD chained derivatives. Check chained sensitivities
against independent whole-rollout perturbations at1e-4 and1e-5; report warmstart
memory exclusion and contact sensitivity. Failed derivatives require whole-rollout
finite differences or another validated method, not silent promotion.
The first run is one cycle,4ms controls,max35 function iterations, force excess
scale10N, source0012; all CLI overrides are bound to result metadata.
## Independent diagnostic success definition (not B1 thresholds)
Complete at least one cycle with each2ms state reproduced within1e-9 by a separate
verifier restoring the saved integration state. Bound recursive MJCF/assets,
source, seed, solver, verifier and exact source SHA. Recompute actuator effort,
full dynamics balance, radial friction, contact forces and joint ranges.
Require |tau|<=35Nm, per-foot summed normal force<=180N, |dq|<=30rad/s, model joint
limits, roll/pitch<=15deg, base height>=.28m, nonfoot contact force<=1e-6N.
At the phase-relative terminal reference require maximum base position error
<.02m, rotation error<.05rad, base velocity<.15m/s, body angular velocity<.3rad/s,
joint configuration<.15rad and joint velocity<3rad/s. Record all violations and
unknown coverage; solver success never overrides them. Contact realization,
diagonal episodes, aerial intervals and foot clearance remain measured outputs
requiring further gait/collision acceptance, not assumptions from phase masks.
A passing initialized open-loop cycle leads to multi-cycle/perturbation feedback
validation and realistic model/terrain input construction before any live flat
canary; then5cm and separately10cm under independently versioned empirical gates.
