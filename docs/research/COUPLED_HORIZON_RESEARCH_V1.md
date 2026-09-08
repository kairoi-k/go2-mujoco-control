# Coupled horizon research V1
Registered before actual-model execution. No controller/terrain actuation or B1
claim. This addresses the distinction between a sequence of independently
projected commands and a joint optimization with future-to-earlier influence.
The common optimizer takes one full-horizon evaluator. All uncommitted motor
controls form one decision vector; the prefix is excluded and copied exactly.
No second dynamics model is introduced. An independent two-step double-integrator
oracle has x2=1.5u0+.5u1, |u|<=1 and x2>=1.5. The analytic minimum-norm solution
is(.9,.3), cost.45. HiGHS proves joint feasibility and proves the greedyu0=0
continuation infeasible. Fixingu0=.8 instead yieldsu1=.6. These are algorithmic
counterexamples, not robot results. Unknown evaluation input, nonfinite/changing
constraint coverage and budget expiry cannot publish a witness. General solver
failure never becomes a global infeasibility claim.
First actual-model diagnostic reuses rolling0002 candidate1 actual full integration
state,6fixed2ms controls and10free2ms controls. Full original MuJoCo state and
contact dynamics determine all body/limb motion. The old sequence is only an
initialization/comparator. A terminal |vy|<=.02m/s constraint requires a future
outcome not enforced by one-step force correction; this is an explicitly new
research challenge, not a B1 threshold change. Cost is .5sum((tau-baseline)/35)^2
plus .5(vy_terminal/.02)^2. Terminal |bodyomega|<=.3rad/s, all sampled pre/post
foot normal forces in[0,180]N, |tau|<=35Nm, model joint bounds, jointspeed<=30rad/s,
roll/pitch<=15deg,baseheight>=.28m and no nonfoot/self contact force>1e-6N.
All force/body/joint constraints are independently recomputed from original
physics every stage; no post-hoc clipping or state resets. Prefix is fixed
commands for this exact initialization, not a robust feedback-law tube.
SLSQP uses full-horizon finite differences with explicit1e-4Nm step, maximum30
iterations and60s cooperative wall budget. Feasible witnesses must satisfy the
original inequalities strictly (zero extra acceptance tolerance). A numerical
solver status is not a dynamics or optimization certificate. Verify the returned
control sequence in a separate replay and preserve failure/budget evidence.
Latency is measured, not assumed realtime. No future nominal trajectory is
created from real observed terrain in this slice; scene use is explicitly
privileged and cannot supply runtime authority. No full-cycle/B1 conclusion from
a32ms result. Terrain merging is a separate independently tested research change.
