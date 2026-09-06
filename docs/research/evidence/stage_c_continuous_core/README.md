# Stage C continuous dynamics core — offline C0-02

The fetched remote `feat/stage-c-joint-planner` was exactly
`57dbd790de23107c870c5135e0fb18a8e57198b9`. The existing worktree was clean.
This packet and its code are delivered in one commit: the containing Git
commit is the source identity; `manifest.json` additionally binds each tested
source file, the binary, and frozen authority/analyzer files by SHA-256.
Synthetic measurements were made before that commit with the listed source
bytes. Final clean-commit verification is retained separately in the ignored
`example/cpp/experiments/_runs/stage_c_continuous_core_verify_<full-SHA>/`.
No robot/controller process, B0/B1, Atlas, terrain actuation or old greedy
planner modification is part of this work. A build of `real_trot_go2` was
attempted, not executed.

## Formulation and scope

The continuous subproblem takes one fixed discrete foothold combination,
explicit Phase-1 schedule intervals with a common epoch, event identities,
observed initial contact anchors, world-frame terrain bases and finite state
bounds. It does not regenerate period/duty or retime any leg. A grid must
include every touchdown/liftoff boundary and the complete consumer interval
endpoint. Contact intervals are half-open; the terminal state has no force
interval. `SampleCentroidalTrajectory` evaluates by absolute time and never
clamps an expired/uncovered sample.

State is `x=(c_world, v_world, L_world_about_COM)`, nine continuous variables
per node. The necessary body rotational freedom here is **centroidal angular
momentum**, not a fabricated base pose or roll/pitch trajectory. Each active
contact has a world-frame force held constant on its interval; swing forces
are removed from the decision vector and are exactly zero. This is a reduced
centroidal feasibility certificate, not full rigid-body/whole-body viability.
In particular it does NOT certify orientation, base position, inertia/limb
angular momentum decomposition, joint torque, IK, swing collision, impact,
tracking or release safety. `body_pose_valid` and `full_geometry_checked`
remain false. Force-supported landing impact is not represented by an impulse:
state is continuous and force switches at the prescribed touchdown.

The original continuous equations are:

```
c_dot = v
m v_dot = sum(f) + m g
L_dot = sum((p_i - c) cross f_i)
g = (0, 0, -gravity_mps2)
```

For interval duration h, define F=sum(f), P=sum(p_i cross f_i), a=g+F/m.
The exact zero-order-hold transition is:

```
c_next = c + h v + h^2 a/2
v_next = v + h a
L_next = L + h P - (h c + h^2 v/2 + h^3 g/6) cross F
```

The omitted apparent `(F/m) cross F` term is identically zero, not an
approximation. This map is quadratic in the force sequence. Piecewise
constant forces and fixed world contact points make c quadratic, v linear
and L cubic inside each interval. The checker tests exact polynomial extrema
against the linearly interpolated finite node bounds, including between-node
violations. An endpoint-only QP constraint is never treated as sufficient.

Force constraints are `min_normal <= n.f <= max_normal`, and
`abs(t1.f), abs(t2.f) <= mu/sqrt(2) * n.f`, using an explicit right-handed
orthonormal terrain basis. min_normal is nonnegative. This conservative
friction pyramid and normal-force units match flat SRBD/ID-WBC; a cone is
never built from an unknown, expired, malformed or non-world surface.
World contact *patch* positions are used directly: there is no foot-site
radius conversion or heading/body transformation hidden inside dynamics.
Upstream certified point geometry is an input, not re-inferred terrain.

The objective retains SRBD COM position/velocity and force weights and
horizontal nominal-reference advancement from the sole Phase-1 applied
velocity. The supplied anchor is initial model COM; predicted COM is a state,
not a new horizontal command. `w_momentum=4` is explicitly a new
(N m s)^-2 regularizer, not a mislabeled angular-rate weight. No posture weight
is claimed to constrain posture. The problem may provide finite COM/velocity/
momentum envelopes; synthetic pinned velocities are test constraints, not
new production velocity authority.

## Solver and certificate

The existing `SolveDenseQpEqNullspace`/DenseQP backend is reused unchanged.
A deterministic force-only SCP linearizes the quadratic rollout with fixed
central differences (1e-3 N perturbation; mathematically no truncation error
for a quadratic map). Fixed ordering, 80 N trust box, at most six SCP steps,
1200 QP iterations, rho=1 and 1e-9 absolute/relative tolerances define this
development solver. Equality nullspace elimination avoids softening hard
state/commitment constraints. The first independently feasible iterate is
returned; there is **no global optimality or stationarity claim**. A failed
or unverified iterate is `NumericalFailure`, not proof of physical infeasibility.
The current solver only enforces node bounds in the QP: if interior bounds
remain violated, the original checker rejects it. Thus this bounded solver
can return numerical unresolved on a feasible problem; it is not a complete
nonlinear feasibility decision procedure.

A physical `DynamicsInfeasible` result requires a separating support-function
witness: a required lower projection of a wrench/impulse box strictly exceeds
the attainable maximum over the contact force pyramids. First-interval angular
impulse is affine in forces under exact integration because initial c/v are
fixed; later-interval proofs use conservative linear-force bounds only.
Directions and both sides of the strict inequality are exported. This is a
sound but incomplete family of infeasibility proofs; other failures stay
unresolved. The Python oracle independently enumerates every cone vertex,
integrates its torque and reconstructs the required projection from inputs.

`VerifyCentroidalTrajectory` is a separate translation unit and does not use
QP matrices, Jacobians, backend return status or the optimizer's transition
helper. It integrates original torque by Simpson quadrature (exact for the
quadratic torque polynomial), checks c/v, every cone/normal/swing constraint,
all original initial/prefix values, time/contact identity and polynomial
extrema. Finite intermediate checks fail closed on overflow as well as NaN.
Fixed development certificate limits are 2e-7 for position/velocity/momentum,
2e-5 N for forces and 2e-6 for mixed-unit state-box violations. Individual
physical residual fields remain separate; these are NOT frozen B0/B1 gates.

Committed event target, touchdown time AND contact end cannot change. The
continuous state/force prefix carries original node/interval times and is
hard-constrained. An already measured touchdown cannot acquire a different
candidate at t0. There is no anchor revival or contact-target teleportation
without an intervening swing. Surface validity covers interval ends, not just
starts. Measured, planned and applied contacts are never promoted into each
other by this core.

## Independent foundation and design audit

The original T01–T07/T13–T15 foundation fixtures still pass and were not
reimplemented. Necessary, targeted corrections found by independent review:

* Committed-prefix compatibility omitted contact end; now locks it.
* `ExhaustiveOracle` inherited the request's combination budget; it now clears
  that budget. The bounded planner still reports its own budget exhaustion.
* Mixed numerical/physical leaf rejection could report proven exhaustion;
  numerical and other unresolved categories now survive aggregation.
* Duplicate time nodes, nonfinite event targets, out-of-range legs, inconsistent
  map counts and NaN/overflow time conversion now fail closed. Invalid seconds
  produce a negative sentinel instead of a plausible zero timestamp.

The foundation comparison helper remains a **partial field comparator**, not
full immutable-input hashing or command replay. The exhaustive helper shares
the search implementation and is not itself an independent dynamics oracle.
This work supplies analytic static/ballistic solutions, independent cone-vertex
proofs, exhaustive two-choice truth and external RK4 verification instead.
A fully enumerated discrete search does not imply its continuous leaves were
globally optimized. Existing partial/certification placeholders are not
accepted-execution authority.

Review of `STAGE_C_V1_DESIGN.md` confirms fixed topology/timing, exact event
identity, absolute interval endpoints, multi-touchdown, committed prefix,
fail-closed unknowns and original-equation verification. It rejects or narrows
these unproven assumptions:

* Merely freezing a COM lever arm and accepting QP success is not a dynamics
  certificate. The original c cross f coupling is retained and checked.
* Legacy SRBD forward Euler keeps the first aerial position unchanged; exact
  ballistic integration moves it by g*h^2/2 (1.962 mm at 20 ms). This isolated
  core intentionally uses exact integration, independently tested; no old
  MPC integrator changed and no joint-planning gain is attributed to it.
* An inertia-free COM solver cannot legitimately emit a body pose. Momentum
  is the minimum rotational state for this core; whole-body reconstruction
  and geometry must precede any execution certificate.
* The mild-slope WBC path chooses world X as tangent when abs(n.z)>=0.9;
  for tilted n this is not strictly perpendicular. New core terrain frames
  require orthonormality. Flat semantics match, sloped equivalence is NOT
  claimed. The WBC code was not patched; reconcile this at future adaptation.
* 15 mm support geometry neither implies nor replaces dynamics. The initial
  geometric conflict remains `ValidateInitialCondition`'s fixed-state
  conflict and is separately false in a dynamically feasible certificate.
  Full geometry remains unchecked; no acceptance gate has been waived.
* The frozen T13 transfer minimum of two contacts still conflicts with aerial
  running-trot intervals. The existing classifier is retained, reported and
  never used to change analyzer flags, contact masks or the contract.

## Tests and measured results

Focused CTest is 4/4: foundation, centroidal core, existing SRBD and DenseQP.
The core has 93 explicit checks with Release-safe failure handling.
Coverage includes symmetric feasible force oracle, friction infeasible,
force infeasible, an active normal-force cap with unequal load distribution,
aerial ballistic conservation, force transitions at two touchdowns of the
same leg, a full 0.24 s/duty-0.4 alternating-diagonal schedule, exact endpoint
queries, event-boundary coverage, committed event/state/force prefixes,
geometry versus dynamics conflict, nonfinite inputs, numerical/coverage/input
classification, tilted frames, world translation/yaw, corrupted trajectory
and an interior-only ballistic bound violation. Both point choices pass the actual unchanged production 15 mm support
diagnostic, but exactly one is dynamically feasible; exhaustive selection matches the
independent analytic/vertex result.

The external standard-library RK4(32 substeps/interval) + cone-vertex checker
passes 9/9 exported physical input/output cases. Maximum independently
reintegrated position/velocity/momentum errors are respectively
4.996e-16, 1.9984e-15, 4.44089e-16;
maximum force violation is 7.42034e-16 N. These are model
residuals, not measured robot tracking errors. Infeasible rows have no
trajectory residual claim; their nonzero separation gap is the certificate.

Release build, native Ubuntu-22.04 WSL on this Windows host, no added CPU
pinning or real-time priority. Each case has five warmups followed by 100
measured complete `SolveCentroidalSubproblem` calls. The timer includes input
validation, matrix construction, solve, rollout and original verification.
Problem construction, candidate enumeration, map preprocessing, queue time,
publication, external oracle and consumers are outside this **core** timer.
No percentile is claimed as the frozen whole-planner budget. Times below are
microseconds, with nearest-rank p50/p95 and maximum over all 100 samples.

| Fixture | Outcome | p50 us | p95 us | max us |
|---|---|---:|---:|---:|
| rest | none | 68.221 | 83.520 | 97.877 |
| friction | dynamics_infeasible | 0.541 | 0.571 | 0.581 |
| force | dynamics_infeasible | 0.451 | 0.471 | 0.481 |
| aerial | none | 1.483 | 1.523 | 14.107 |
| multi_td | none | 10.971 | 11.653 | 12.023 |
| choice | none | 5.120 | 5.311 | 5.441 |
| running_trot | none | 127.946 | 147.173 | 160.648 |
| active_force_cap | none | 215.994 | 260.550 | 508.436 |
| choice_infeasible | dynamics_infeasible | 0.791 | 0.811 | 0.842 |

The full build still fails on the absent MuJoCo header at
`simulate/mujoco/include/mujoco/mujoco.h`; see `full_build.log`.
The MuJoCo integration target was not counted. There were no B0/B1 runs.

## C0-03 disposition and stop

Ready to BEGIN C0-03's offline same-input comparison and larger-budget
measurement using this reduced continuous seam. This does not mean C0-03
passed: representative complete inputs, L0/L1/C0 comparison, bounded search
coverage, full-path Atlas latency and the missing geometry/body reconstruction
remain to be demonstrated. A core benchmark cannot authorize shadow consumers
or execution. T13, slope-basis adaptation, real observation/commitment
interfaces, command replay and MuJoCo remain explicit future integration
boundaries. No controller connection, terrain actuation, B0/B1, analyzer or
threshold change is authorized or performed here. Work stops after the clean
commit and verified push.

## Reproduce

```bash
cmake -S example/cpp -B example/cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build example/cpp/build --target test_stage_c_centroidal test_stage_c_foundations test_srbd_mpc test_dense_qp -j2
ctest --test-dir example/cpp/build -R 'stage_c|test_srbd_mpc|test_dense_qp' --output-on-failure
# Use a NEW output directory; do not overwrite retained raw evidence.
example/cpp/build/test_stage_c_centroidal <new-directory>/synthetic_results.json
python3 example/cpp/tools/analysis/verify_stage_c_centroidal.py <new-directory>/synthetic_results.json --output <new-directory>/independent_results.json
```
