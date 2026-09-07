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
## Latest actual runtime and causal evidence
Worktree `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`, branch
`feat/stage-c-joint-planner`. Git determines current source HEAD. Latest full
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
independent attitude PD; no runtime activation flag was added. A0.2s paired
MuJoCo counterfactual at source21.018 reduces max foot error58.12->24.12mm and
COM13.79->7.89mm, while pitch rises1.33->5.19deg. Both100-step replays complete;
independent saved-command plant replay agrees on recorded states. Contact-mask
mismatches remain30/25 rows. Evidence: coherent_body_acceleration_20260908.
This is a promising task-coherence result, not full-runtime stability. It does
not resolve strict trajectory initial conditions or absolute attitude control.
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
