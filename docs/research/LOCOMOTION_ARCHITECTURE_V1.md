# Locomotion architecture V1: shared contracts and replaceable backends
Decision date: 2026-09-07. Authorized by the user's latest instruction to own
architecture implementation and choose an efficient Astra/Luna division.
This is a versioned engineering/research decision, not an acceptance result.
Baseline: 06f002d7e04c0c753f813be10823d20f6fd11cc5; runtime 7a8ffc6.
## Capability objective and current evidence
The long-term objective is natural stable motion throughout the hardware's
achievable speed range, smooth transitions, complex terrain and diverse skills,
with a future embodied-intelligence interface. Hardware limits must be measured,
not defined by animal analogy or a model's assumed theoretical maximum.
B1 5 cm running traversal remains the first concrete closed-loop capability
probe, not the architectural ceiling. Existing tests or Stage-C milestones
cannot replace evidence that the robot actually accomplishes the task.
The F02 experiment fixes a real swing-acceleration error but does not improve
B1 empirically: no executable terrain plan precedes first front-foot impact.
WBC non-convergence is currently accepted on partial checks, and the final PD
command can differ from the optimized torque. Geometry points, future state,
event timing and observations also have unresolved contracts. These findings
justify staged replacement of shared semantics rather than more gain tuning.
They do not establish the winning control algorithm or a unique failure cause.
## Design rules and migration sequence
Define a contract only with a real producer, consumer, independent test and
migration path. Extend the existing Stage-C types and TerrainExecutionSnapshot/Commitment
where their semantics fit. Do not create a second map, robot model, contact
ledger or controller-local recovery authority. Keep the existing runnable
backend as a named historical reference. Never promote an untested new route
by renaming telemetry or changing a gate after seeing its outcome.
1. Independent validation and command lineage. Separate convergence, model
   feasibility, selected proposal validity, final actuator feasibility and
   measured task success. Start with a read-only WBC physical certificate
   evaluated against current dynamics, for both the raw attempt and selected
   candidate after reuse/overlays. Preserve old acceptance behavior for this
   measurement experiment; it is not a safe-command enforcement claim. Then
   use measured failure distributions to implement a coherent acceptance and
   fallback policy with current-state revalidation. A stale cached certificate
   must never certify a new state, contact mask or modified command.
2. Robot geometry, frames and time. Use explicit collision-geometry versus
   site/reference points and transformations. Physical quantities identify
   frame, origin and time. Observation time, plan generation time and absolute
   event time remain distinct. A future liftoff state is not the current
   force-supported FK foot. Unknown future state or terrain stays unresolved.
3. Event-indexed joint planning and execution. Plan body motion, multiple
   touchdown events and contact forces jointly through a shared physics model.
   Candidate generation can remain per-leg; selection cannot independently
   certify each leg as a complete plan. Coverage spans the meaningful obstacle
   interaction and committed events, with explicit terminal/coverage limits.
   Future timing or gait freedom must be represented in the same event contract,
   not hidden in local swing retiming. One execution owner preserves committed
   prefixes and revalidates adoption against current state. Test the whole chain
   from liftoff through touchdown, not only isolated feasible landing points.
4. Observation and actuator boundaries. Label simulator oracle observations
   separately from realistic sensor observations, preserving old oracle runs
   as privileged-input diagnostics. Final joint commands carry requested and
   applied values, PD/feedforward composition and an actuator envelope. Validate
   the actual plant command, including saturation, against that envelope. Ground
   truth belongs to evaluation and cannot silently become controller observation.
5. Replaceable control backends. A model-based backend may emit body/force/event
   trajectories; a learned backend may emit joint targets or residual actions.
   They share observation, intent, actuator and evaluation contracts, not a
   mandatory foothold-plan output. Compare learned and model-based routes with
   matched sensors, actuator limits, task distributions and compute budgets.
   Add a bounded whole-body optimization challenger when it resolves a concrete
   reduced-model limitation. No algorithm wins by architectural decree.
These are dependency ordering rules, not a requirement to finish an entire
platform before performing the next useful B1 experiment. Each slice must
replace or validate a concrete path and earn its place through measured results.
The Phase-1 shaper remains the current legacy backend's sole velocity authority;
future backend-neutral intent must have one explicitly chosen owner as well.
A different task/gait cannot inherit a running-trot acceptance claim by analogy.
## First implementation slice: current-model WBC physical certificate
Input: existing IdWbcParams, IdWbcInput (including current dynamics and contact
mask), and a proposed qdd/force/tau. No solver status or diagnostic residual is
trusted as the certificate's answer. Recompute full rigid-body balance,
normal-force limits, unilateral contact, radial Coulomb friction, the current
explicit swing-force allowance, joint feedforward-torque limits and optional
hard-stance physical acceleration. Split force and moment/torque residual units.
Report validity, checked coverage and violations independently of convergence.
Malformed declared surface normals and nonfinite data fail closed; absent normal
metadata uses the explicitly identified legacy flat assumption.
This certificate covers the stated rigid-body proposal and circular physical
cone. It does not certify the implementation's conservative pyramid rows,
joint reach/velocity, terrain geometry, final PD torque, actuator bandwidth,
thermal limits, observation truth, future contact execution or closed-loop
success. Those limits must be visible rather than hidden by a universal `ok`.
Tolerances are versioned diagnostic definitions, not edits to frozen acceptance.
The initial online path only measures; enforcement requires separate validation.
Tests include analytic static and aerial solutions, each violation family,
invalid/nonfinite inputs, force transitions, tilted normals and tampered outputs.
Use independent fixtures rather than reproducing the implementation formulas
as the sole oracle. Online logging checks raw and selected candidates separately
and includes coverage indicators. A clean exact-SHA flat diagnostic comes before
any step exposure. Record runtime overhead and preserve all failed evidence.
## Evidence and economical delegation
Astra owns contract semantics, experiment selection, failure interpretation and
acceptance. Luna receives bounded implementation/test/replay work with specific
file ownership, required inputs and a checkable output. It does not reread the
whole project or decide acceptance from its own summary. Escalate an ambiguous
physics or architectural decision with concrete evidence; do not spend repeated
agent turns guessing. Parallelize independent work; serialize simulator access.
Reuse retained manifests, targeted reads and deterministic scripts. Measure
rework and elapsed time along with quota consumption before expanding delegation.
For each runtime change: clean source/binary binding, focused regression,
registered experiment, immutable raw logs, independent replay, explicit legacy
and current verdicts, then a clean pushed checkpoint. Full B0/holdout admission
is still required for its historical campaign; development diagnostics do not
become holdouts. Any replacement acceptance remains separately versioned.
