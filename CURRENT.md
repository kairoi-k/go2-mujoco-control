# Go2 current research checkpoint
Updated: 2026-09-08. This is the only live route/status/handoff entrypoint.
## Objective and acceptance
Continue toward a long-term extensible locomotion architecture with genuine
joint body/foothold/contact-force planning and coherent execution. Establish
reproducible real MuJoCo 5 cm dynamic running-trot B1 evidence first, then an
independent 10 cm challenge. The user permits architecture/acceptance redesign;
version and retain historical baselines. A module checkpoint is not completion.
**B1 remains NOT_CERTIFIED. The new joint execution path is not stable on flat.**
## Current implementation and verified boundary
The Stage-C event-indexed search jointly selects touchdown combinations and
optimizes centroidal COM/velocity/angular momentum/contact forces. It shares
production model/state/frame semantics and preserves accepted touchdown targets.
The worker publishes immutable selected bundles to one atomic execution owner.
An actual feedback tick tracks COM/momentum/feet with the existing ID-WBC and
writes model-mapped torque with zero additional motor PD. This path has actually
actuated MuJoCo; it is not merely shadow plumbing. Historical per-leg planner
execution is bypassed in this opt-in mode; Phase-1 remains velocity authority.
Candidate admission currently checks the selected CENTROIDAL certificate.
`SolveJointTrajectoryCandidate` is NOT part of runtime candidate admission.
Per-tick full-model inverse-dynamics/torque certificates do not certify planned
whole-body trajectories, contact realization, tracking, or traversal. The
existing articulated reconstruction remains a conditional diagnostic: it cannot
silently change initial measured q/dq to impose stationary stance feet.
`execution_ready=false` in that diagnostic is not a release certificate.
The command-reference owner preserves C1 p/v, selected event commitments and
prepared swing-curve leases. Body/force validity is never extended by a curve
lease. An initial <=20ms stance-reference bridge currently propagates its
acceleration into WBC; contact-motion compatibility is unresolved. Planned,
measured, applied and collision-truth contact remain different quantities.
## Bounded attitude-feedback diagnostic
An opt-in replay mode now maps a body-frame angular acceleration correction
through the actual-state COM/body/feet Jacobian system into total momentum-rate
correction. The same corrected momentum target feeds WBC and the coherent lift.
COM/foot acceleration targets and natural feedforward body acceleration are
preserved; only the feedback increment is capped using existing gains/limits.
This is instantaneous feedback, NOT horizon body attitude planning or resolution
of moving initial support/contact realization. Production runtime is unchanged.
The mapper has an independent finite-difference kinematics test; body acceleration
and closed-loop reference focused tests pass. Two fixed-source 100-step MuJoCo replays completed: pitch peaks improve
5.19->4.31deg and4.50->3.70deg, foot errors rise less than1mm. Independent
saved-command plant replay agrees below3.3e-14 on recorded states. Evidence
`coherent_attitude_20260908`; no sustained flat or B1 claim.
## Latest actual runtime0012: physical posture failure
Clean0f6ec6526f4fcde77f2b737b5a65d9855ed5365b, absolute-state clock1 plus0011.
Four versions, first21.024,last21.204/count91; posture28.8021roll/7.15897pitch.
Sampled foot589.122mm/COM16.6688mm. No pre-stop clock rejection or executor QP
failure. Short attitude feedback improvement did not transfer to sustained
execution. Near-takeover has no>8ms gap, so gap fix not exercised in this run.
Evidence flat attempt_0012. Stop local correction/gain sweeps: inspect actual
body/foot reference compatibility and acceleration/singularity at first
tracking divergence, then represent body attitude in the planning horizon.
No new B1 or10cm result. Historical baseline remains preserved.
## Latest actual runtime0011: clock rejection and reference expiry
Clean b63f38f094d63ebdd69cdaf064099cc4b19961da, coherent attitude feedback1.
Three accepted versions, first21.016, expiry21.254 after119 commands. First
clock_rejected source21.086 has -10ms phase residual; later proposals rejected.
No sustained posture verdict is possible. Sampled foot66.8523mm/COM9.81005mm;
tick p50/p95/max285.569/418.9552/432.817us. Evidence flat attempt_0011.
Next reconstruct producer phase/state-time divergence without relaxing clock
checks or extending stale force validity. B1 remains NOT_CERTIFIED.
## Latest actual runtime: bounded-admission canary0010 FAILED
Clean97b6d02158a797f1bb02e9817495152964a3a322, admission budget80ms with0009 settings.
15 versions, first21.012, last sampled21.658/count325; then posture
roll23.8593deg/pitch14.4483deg. No executor expiry or QP stop. Foot242.760mm,
COM24.177mm sampled maxima. All15 observed admissions before deadlines,
max age52ms; no sampled commitment conflicts. Evidence flat packet attempt_0010.
The protocol now admits continuous replans in this case, but physical stability
is unresolved. Coherent acceleration has no absolute attitude feedback; next
supply body/limb-consistent momentum/attitude planning rather than restoring
an independent conflicting body task or extending stale forces.
## Previous actual runtime: proposal-initialized canary0009 FAILED
Clean6d0f75123d736f91c4ab114d37f8de178cfca54d. First21.026, five versions,
stop21.332/count154 for reference_expired_or_unavailable. No posture stop or
executor QP failure; max IMU roll15.47/pitch11.09deg over active window.
Sampled foot102.835mm/COM7.928mm. Evidence attempt_0009 in flat packet.
Source contact-anchor failures coexist with valid50ms proposals rejected as
commitment conflicts. Active-only exported commitments can become stale when
new swings activate during computation; reconstruct this race deterministically
before implementing a bounded computation-aware prefix/deadline protocol.
Do not extend stale force validity. Initial proposal reference is not measured
zero stance motion or a complete articulated trajectory certificate.
## Previous actual runtime: coherent-body canary0008 FAILED
Clean source9c1ed03c9099809bcb22123fc7599c87e2759752, actual coherent_body=1.
First21.004, four versions, last sampled21.278/count137; posture roll-22.0233deg,
pitch9.9677deg triggers stop. No executor QP failure; exact first-QP replay
matches. Foot139.332mm, COM11.6249mm; sampled latency287.864/334.1151/344.2us.
Evidence joint_execution_flat_20260908/attempt_0008. Short replay improvement
has NOT transferred to sustained actual execution; this candidate is not promoted.
Initial legacy command handover requests stance ax52.08/47.62m/s2, with opposite
sign commanded versus actual stance vx. This transition is absent from the short
state-initialized replay. Next compare matched first-takeover initialization,
retaining actual q/dq and later accepted commitments; no gain sweep.
## Previous actual runtime and causal evidence
Worktree `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`, branch
`feat/stage-c-joint-planner`. Git determines current source HEAD. Previous full
executed clean runtime is `9c5c836fe98370812f0f1bc94b9c2ec718d638e6`.
Its opt-in shared orientation/swing priority flat run (attempt0007) FAILS:
STATE21.024 first adoption, last sampled command21.168/count73, then posture
roll22.8689deg/pitch12.02deg; no QP failure. This candidate is not promoted.
The old hierarchy remains default; neither mode is an accepted running backend.
The durable packet is [joint_execution_flat_20260908](docs/research/evidence/joint_execution_flat_20260908/README.md):
- 0001/f5b2ef9: actual actuation, .16->.14 period change invalidates commitments;
  expiry stops execution. Passing commitment activity now prevents silent epoch
  reset, but a variable-period committed event calendar is still missing.
- 0002/8f3e04a and0003/f586950: fixed-start21s isolates WBC numerical failure.
  Independent exact-QP HiGHS/KKT proves feasibility. QR working-set projection
  removes active-row drift without relaxing original constraints.
- 0004/4bff757:10versions, last sampled21.418/count208, then posture failure;
  sampled foot error461.764mm. No WBC numerical failure.
- 0005/4b09ccd: actual task logs expose swing acceleration compromise before
  tracking loss. Foot-only collision truth confirms FL21.054/RR21.066 contacts
  before planned support. Secondary numerical failure21.402 is later.
- 0006/0a853346: first ACTUAL secondary QP captured at21.020. Production replay
  reproduces its iterate exactly; independent SciPy agrees. Removing qdd
  regularization leaves large swing errors. Numerical rank analysis finds a
  stable89.632m/s2 combined swing residual floor under frozen primary tasks.
  Releasing orientation locks improves foot tasks but changes body acceleration;
  instantaneous angular acceleration alone is not a stability verdict.
Actual source21.018 from0006 has measured-support collision-center speeds
FL0.28009/RR0.41783m/s in an independent unchanged-state MuJoCo Jacobian audit.
The nominal stationary-foot reconstruction cannot represent this source as-is.
Do not confuse this kinematic observation with proof of physical infeasibility.
Execution-window foot coverage (.2s) and complete planner-grid coverage must also
be distinguished when calling the full articulated diagnostic.
The curated articulated audit confirms full-grid coverage failure before body
reconstruction (0 samples). A separate unchanged-state nominal acceleration
lift passes a conditional sample certificate, with31.4344Nm peak torque; it
uses actual lever arms and no foot PD, not the actual feedback task. Both
momentum targets agree at this source sample. No new closed-loop run occurred.
## Coherent acceleration reference experiment
At ceed297, opt-in research config derives body angular acceleration from the
same COM/Ldot and feedback-corrected foot tasks at actual q/dq. Default remains
independent attitude PD; runtime opt-in is TROT_RESEARCH_JOINT_COHERENT_BODY=1 (exact string). A0.2s paired
MuJoCo counterfactual at source21.018 reduces max foot error58.12->24.12mm and
COM13.79->7.89mm, while pitch rises1.33->5.19deg. Both100-step replays complete;
independent saved-command plant replay agrees on recorded states. Contact-mask
mismatches remain30/25 rows. Evidence: coherent_body_acceleration_20260908.
This is a promising task-coherence result, not full-runtime stability. It does
not resolve strict trajectory initial conditions or absolute attitude control.
The opposite-phase paired replay also improves foot59.11->18.36mm and
COM10.44->5.39mm, with pitch3.06->4.50deg; contact mismatches remain32/22.
Both complete and independently reproduce. One actual flat canary0008 is
registered with coherent-body1, soft-orientation0 and all prior settings.
## Registered first-acquisition comparison
Opt-in TROT_RESEARCH_JOINT_INITIAL_PROPOSAL=1 skips only the legacy commanded
boundary before the owner has accepted any bundle. It uses the proposal's
existing observation-based nominal reference at current absolute time; actual
q/dq are unchanged. Nominal stance velocity is still zero, not measured truth.
Later adoptions require normal commanded boundaries and commitment checks.
Default remains legacy commanded handover. After focused tests, run one flat
canary0009 with coherent-body1 and all0008 settings, adding only this option.
An owner unit test does not establish5cm readiness or traversal acceptance.
## Registered bounded admission protocol
JOINT_PLANNING_ADMISSION_V1 adds an optional absolute admission deadline and
producer frozen prefix for events that can begin by that deadline. Live leases
remain authoritative; stale results cannot extend force validity. Deterministic
delayed-adoption test covers conflict, protected success and expiry. After
focused tests/build, run only flat0010 with admission budget0.080s added to
0009 settings. This is an enforced experimental budget, not a latency guarantee.
## Next research action
Conditional terminal binding at b49e9e8 now resolves the full-grid foot coverage
with observed next-event candidates under an explicit longer stationary-terrain
prediction assumption. All25 terminal combinations (fixed core selection)
then fail initial_condition_conflict in full-body reconstruction. Evidence:
`joint_execution_flat_20260908/terminal_binding_audit`. Original diagnostic
output remains unchanged; four focused tests pass. No new closed-loop run.
The next implementation must represent the actual moving initial support state
and coordinate body/leg acceleration with contact realization. Do not reset
initial q/dq, silently treat moving supports as stationary, or resume weight
sweeps. Validate coherent initial contact transitions/articulated references
offline before a bounded actual flat run, then5cm and independently10cm.
A model sample certificate is not trajectory or closed-loop acceptance.
## Reproduction and runtime switches
`example/cpp/scripts/run_b1_research_probe.sh` requires clean exact source,
unique raw name and holds the experiment lock. Recent runs use phase2_flat.xml,
32s, b1_v3_running_1mps.csv, seed11 and the preserved fixed-start protocol in
raw environment/metadata/manifests. Record all overrides; do not infer them.
`TROT_RESEARCH_JOINT_EXECUTION=1` enables the diagnostic actuator path; default off.
`TROT_RESEARCH_JOINT_START_S=21` is timing isolation, not a variable-period fix.
`TROT_RESEARCH_JOINT_SOFT_ORIENTATION=1` enables the FAILED0007 hypothesis;
default0 preserves the prior primary orientation hierarchy.
New `JointExecutionTracking` fields belong to actual executor tasks. Legacy WBC
CSV task fields computed earlier are not executor witnesses. `first_stop=null`
in analyze_execution.py only means no executor stop log; an upstream posture
stop can bypass it. Inspect raw safety output and final commands.
Native WSL via the pinned localhost SSH helper works; MuJoCo3.3.6 is available.
Historical missing-MuJoCo and old WSL launch blockers are not current blockers.
Pause competing builds/agents for timed runs. Preserve all raw input hashes,
source/binary bindings and target analyzer output, including legacy profile
KeyError/dependent analyzer failures. Wrapper exit status is not acceptance.
## Historical evidence and authority
The [articulated route](docs/research/STAGE_C_ARTICULATED_ROUTE.md),
[architecture decision](docs/research/LOCOMOTION_ARCHITECTURE_V1.md), and
[feedback route](docs/research/JOINT_FEEDBACK_EXECUTION_V1.md) are research context,
not proof that their end states have been delivered.
`joint_feedback_replay_20260907/attempt_0009` retains the exact100-step privileged
short replay:16.382/29.753mm COM/foot error and exact independent plant replay.
It does not prove full-runtime adoption stability. Earlier old-planner5cm runs
remain failures or incomplete acceptance; the OneDrive video depicts an older
runtime. Details remain in joint_capture_view_20260907, wbc_swing_bias_20260907,
joint_terrain_history_20260907 and b1_checkpoint_20260907 evidence packets.
Authority order: current explicit user instructions and CURRENT; AGENTS.md;
frozen PHASE2_ACCEPTANCE.md and PHASE2_HOLDOUT_MANIFEST.json for their campaign;
versioned protocols including B1_DYNAMIC_TRAVERSAL_V3.md and
B1_REGISTERED_INTERVALS_V2.md with bound analyzers/evidence.
Retain T13 frozen aerial conflict and15mm GEOMETRIC diagnostic separately from
dynamic feasibility. Unknown is not safe. Keep normal running-trot diagonal
support and one shared immutable terrain snapshot; no local recovery authority
or quasi-static/crawl fallback. User-authorized new contracts must be versioned.
Never commit, delete, overwrite or rename `example/cpp/experiments/_runs/`,
stashes, archived branches or other worktrees. Curated evidence has manifests.

## Registered bounded flat0011
Add only TROT_RESEARCH_JOINT_COHERENT_ATTITUDE=1 to0010 settings.
This enables the short-replay-verified same-state feedback correction; requires
coherent-body mode. Default off. One exact-source flat canary, no gain sweep.

## Clock loss localized after0011
CSV reconstruction proves MotionClockStep permanently drops10ms at state21.064:
state gap exceeds8ms, last tick advances but running_time/phase do not. Predicted
-10000000ns equals observed shadow rejection. Evidence attempt_0011/clock_loss.json
and reconstruct_clock.py. Transport stall cause unknown. Separate absolute event
time from bounded control integration; retain committed epoch and fail-closed
input quality. No controller change or new sim in this localization step.

## Absolute-state elapsed migration (opt-in, not yet simulated)
TROT_RESEARCH_ABSOLUTE_STATE_CLOCK=1 separates monotonically observed state
elapsed time from the original <=8ms integration allowance. Active only in
state-time mode without lockstep acknowledgment. Gap ticks preserve absolute
gait elapsed time and commitments; integration dt stays zero. Duplicate/rewind
never adds elapsed time; rewinds leave the high-water mark intact. Runtime
velocity shaper/scheduler/gates retain state when integration is unavailable.
No PhaseClock tolerance, validity deadline or accepted epoch is changed.
The pure seam test includes0011's10ms gap and existing PhaseClock with active
commitments. Production call-chain test verifies elapsed/integration separation;
it uses synthetic stand observations, not running gait or physical acceptance.
Independent review confirms Raibert advances phase from gait elapsed, but missed
cycles are not replayed and this cannot certify contact realization. Run one
flat canary with prior0011 settings plus this flag only after runtime build and
focused checks. No new simulation has yet exercised this option.

##0012 first-task incompatibility localized
First logged tick21.024 is all-leg planned/measured aerial. Leg1/2 reference
speeds5.728/5.976m/s vs actual1.968/2.021; reference acceleration451.808/464.338
m/s2 and solved-task residual207.449/219.053m/s2. See attempt_0012 tracking audit.
The attitude momentum correction is only kinematically compatible: aerial
external Ldot is constrained to zero, so arbitrary correction with fixed feet
need not be physically feasible. Do not promote or tune this as a complete
body planner. Next reconstruct initial remaining-swing reference from actual
q/dq and touchdown timing, then jointly coordinate limbs/body with conservation.
This observation does not attribute the entire residual to attitude feedback.

## Remaining-swing boundary reproduced
0012 source21.020 legs1/2 have7.532828ms until touchdown yet selected targets
are30.533/31.196mm forward with terminal velocity zero. Cubic initial ax
2258.800/2318.130m/s2; independent formula check<1e-9. Audit --swing-audit
replays recorded source, not physical feasibility. Next include actual-state
remaining-swing articulated reachability in candidate evaluation and preserve
initial commitments; no arbitrary clipping/retiming or further gain sweep.

## Initial target physical constraint proven
Shared VerifyArticulatedAccelerationTarget composes existing lift and full-model
certificate.0012 initial reference requires312.044Nm vs35Nm, with277.044Nm
violation, rank18 and well-resolved kinematics. Ballistic, excessive-aerial and
unknown-input focused fixtures pass. Evidence initial_target_certificate.json.
This checker is used in replay audit, not yet runtime selection. Next integrate
actual-state target verification into combination evaluation and jointly alter
body/limb references; rejection alone cannot create a successful B1 candidate.

## Initial articulated candidate gate (opt-in)
TROT_RESEARCH_JOINT_INITIAL_DYNAMICS=1 validates each centroidal-feasible
combination before ranking, using actual q/dq, nominal swing acceleration,
actual force application points and existing35Nm physical certificate.
Default retains historical centroidal-only selection. Explicit checked flags
and rejection counts distinguish these modes; passing is not full trajectory
or contact realization. Unknown/failed checks cannot expose a checked winner.
0012 recorded-source exact replay rejects the first two combinations at312.044
and506.433Nm. Overall budget_exhausted correctly means unsearched combinations
remain unknown, not global infeasibility. No new MuJoCo closed-loop run.
Next generate/optimize physically realizable remaining-swing footholds jointly
with body motion; a rejection gate alone cannot achieve the objective.

## Independent initial joint foothold oracle
Packet initial_joint_foothold_oracle_20260908 solves all four target positions
through shared actual-state qacc/tau, fixed touchdown times and zero terminal
velocity. Full production model bias crosschecks312.044Nm within1.7e-11Nm.
Local planar +/-50mm target box admits35Nm solution but body pitch acceleration
78.28rad/s2; forcing zero body angular acceleration is infeasible in this local
problem. Not trajectory/terrain/impact or B1 evidence. Earlier omitted passive/
friction oracle variants are explicitly superseded. Next jointly represent
landing velocity/contact transition and body/limb motion; do not infer global
infeasibility or patch gains from this instantaneous result.

## Landing velocity boundary experiment
The same initial oracle with normal touchdown velocities in[-2,0]m/s finds a
35Nm solution with zero body angular acceleration, unlike the zero-velocity
boundary. Chosen maximum descent0.249892m/s; residual1.13e-13. Tangential landing
velocity remains zero. Evidence landing_velocity_result.json in initial joint
foothold oracle packet. This is conditional instantaneous feasibility only.
Do not patch nonzero terminal velocity into a zero-velocity stance reference:
next explicitly model pre/post-contact transition and verify against MuJoCo.
No impact/horizon or B1 certificate, no new controller or simulation run.

## Soft-contact probe contradicts pure-aerial transfer
20ms held oracle torque at unchanged source in full scene has residual FR/RL
contact immediately; body angular acceleration is not zero. Original GT forces
at21.020 are2.447/7.674N, while counterfactual force under new torque is39N each;
keep those distinct. Soft-contact landing has nonzero foot velocity while force
builds. Evidence transition_probe_result and actual_source_contact in oracle
packet. This is not a planned-trajectory rollout or new B1 run. Next inspect
initial in-flight clearance by phase and model liftoff/landing force transition;
do not promote contact-free torque or impose a velocity reset.

## Phase-preserving in-flight clearance implemented
Opt-in preserve_inflight_clearance_phase retains actual p/v and original event
times, with exact quartic Hermite remainder scaled by remaining-phase^4.
Early/apex/late tilted-normal p/v/a equivalence tests and independent scalar
identity pass. Read-only --articulated-phase-audit is available; no controller
runtime switch wired.0012 initial torque310.601Nm still fails35Nm. Evidence
phase_preserving_clearance_20260908. Include this corrected shape in subsequent
joint body/landing-target/velocity optimization; it does not itself fix B1.
