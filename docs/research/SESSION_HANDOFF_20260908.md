# Native rolling horizon continuation,2026-09-08
Read CURRENT.md first. Latest executed initialized flat source54d750e:
fixed12ms commitment/20ms tail,20cycles,233candidates no late, physical limits
and19complete running-contact cycles pass. Frozen V1 numerical FAIL retained.
No live controller/B1/10cm run. New observed collision builder is research only;
real recorded snapshot coverage is the next diagnostic. Original evidence below
is historical and not an instruction to stop active authorized work.
# Resumed checkpoint after0349725,2026-09-08
Read CURRENT.md first: it now supersedes the original UNRUN draft status below.
Latest full initialized feedback source4023ff87732236b629a59c80f886bb2f22126248.
20cycles plus fixedvy/roll perturbations pass physical checks with one-step hard
force-constrained feedback. Exact independent pre/post replay and optimized full
trajectory equivalence are archived. Thirty focused tests pass. Constraint tick
still17.864ms>2ms; no production integration, new live flat,5cm B1 or10cm run.
Original V1 numerical failures remain, including floating-clock accumulation;
no acceptance thresholds or physical model options changed. B1 NOT_CERTIFIED.
Next is deadline-aware horizon/owner/observed-terrain integration per CURRENT,
not gain sweeps or calling privileged initialized results a terrain release.
All new raw directories use20260908 under _runs and are preserved. Code/evidence
committed locally, no push. Git determines final docs HEAD. Session workers are
finished and simulations/builds are not left running at this checkpoint.
# Original0349725 handoff (historical evidence follows)
# Session handoff: full-body B1 research, 2026-09-08
## Resume here
User requests handoff and a new session because spawned-agent handles cannot
be closed by the currently exposed collaboration tools. Do not restart old
workers or re-run completed audits. The next session may use Astra low workers
for independently useful tasks; root owns physics, architecture and acceptance.
Latest objective: genuine5cm dynamic running-trot B1 using a long-term extensible
architecture, then10cm independently. Research/acceptance redesign authorized;
retain versioned historical baselines. B1 remains NOT_CERTIFIED. No current
full-body challenger has achieved sustained live flat running or new traversal.
All user-facing communication <=300 Chinese characters, plain paragraphs only.
Canonical repo: /home/che/dev/go2-workspace/feat-stage-c-joint-planner
Branch: feat/stage-c-joint-planner
Windows workspace: C:\Users\w1881\Documents\Codex\2026-09-06\feat-stage-c-joint-planner-fetch
Use pinned localhost SSH helper, not the formerly unreliable wsl.exe launch:
PowerShell Bash here-string piped to python .\wsl_exec.py. MuJoCo3.3.6 and SciPy
work in native WSL. Hold /tmp/go2_mujoco_experiment.lock for simulations; avoid
competing builds. Preserve _runs, stashes, archived branches and other worktrees.
Read CURRENT.md and AGENTS.md authority. Read-only historical memory cannot
replace current code/evidence. Git HEAD and origin ref give the handoff SHA.
## Decisive evidence and architecture correction
Centroidal COM/momentum/force planning with separately prescribed foot cubics
was physically incomplete. Source0012 at21.020 had7.53ms to touchdown with a
30mm forward target and zero terminal foot velocity: required312.044Nm versus
35Nm. Not proof that all trajectories fail. Allowing landing normal velocity
restored instantaneous aerial feasibility, but actual residual soft contact
invalidated direct transfer. Phase-preserving clearance corrected a real
reference-shape issue but still needed310.601Nm with old targets.
Latest real controller flat0012 source0f6ec6526f4fcde77f2b737b5a65d9855ed5365b:
4versions,21.024 to21.204/count91, stopped for roll28.80deg. No new actual
controller run since. Prior0001-0012 failures are retained in
joint_execution_flat_20260908. Old video is an older planner and is not proof
that this new joint path works. Frozen T13 aerial conflict/15mm geometric
support diagnostic remain distinct from dynamics and real traversal.
## New complete-cycle oracle: ACTUALLY RUN
Runtime source6356c21c9474f58156065044a7fd1c2f82c61008, clean before solve.
Raw: example/cpp/experiments/_runs/whole_body_cycle_20260908_0001
Durable: docs/research/evidence/whole_body_cycle_20260908/attempt_0001
Protocol: WHOLE_BODY_SHOOTING_V1.md. Full SAME MuJoCo model including gravity,
passive friction, limits and compliant collision dynamics;12 motor torques,
35blocks held4ms,70physical2ms steps (one0.14s calendar). Actual sourceq/dq
retained (unit quaternion normalization); unavailable original integration
memory initialized to model defaults and serialized as mjSTATE_INTEGRATION.
Previous-cycle actual telemetry is a PRIVILEGED initialization/reference, not
an allowed future controller input. No measured state or landing velocity reset.
SciPy bounded least-squares uses transitionFD chained derivatives plus local
full-model observation differences. Directional whole-rollout checks at1e-4
and1e-5 yield relative2.50e-7/4.20e-7. Model/state/actuator ordering independently
reviewed; helper5tests and certificate8tests pass. Solver23function evaluations,
11Jacobians,21.496s elapsed; squared cost76.715->21.406. xtol termination despite
large reported first-order optimality7.77e3 is NOT an optimality certificate.
Independent replay: state/force/applied torque differences0; time7.76e-14s.
Actual torque max28.253Nm; joint speed11.955rad/s; base height min.367532m;
roll/pitch max.00704/.00239rad; no nonfoot contact; joint limits pass.
Terminal max errors: base position.001045m, orientation.002197rad,
base velocity.012405m/s, body omega.026747rad/s, jointq.013244rad,
jointv.138305rad/s. V1 diagnostic FAILS: force180.587912>180N and absolute
full dynamics residual3.583742e-7>1e-7. Preserve this verdict.
Worst residual baseZ; largest joint residual5.956e-8Nm. Relative scaled residual
about2.3e-9, model Newton tolerance1e-8,2iterations at worst row. Numerical
acceptance may deserve unit-aware versioning, but do not silently relax V1.
Independent contact replay: forward displacement.119244m; at10N threshold,
FR/RL diagonal68ms,FL/RR50ms,fourfeet12ms,nofoot8ms,other2ms. Only ONE8ms
netGRF<10N aerial episode. Other transfer overlaps; not two clean aerial
transitions or repeatable running certification. Soft penetration reaches
~3.8-7.5mm; maximum sphere-plane clearance~27-33mm. One open-loop cycle is not
feedback stability, multi-cycle viability, terrain planning or B1.
## Reproduction and provenance
attempt_0001/raw_result.json preserves original raw metadata/paths unchanged.
result.json changes only paths to portable relative references and exact
runtime_sources copies; it records the original result hash. Controls, states
and numerical results are unchanged. Recursive XML and mesh assets are bound.
curated_certificate.json was regenerated after curation and matches V1 failure.
From repo root run:
python3 docs/research/evidence/whole_body_cycle_20260908/attempt_0001/runtime_sources/verify_whole_body_shooting.py docs/research/evidence/whole_body_cycle_20260908/attempt_0001/result.json --out NEW_FILE.json
Exit1 is expected diagnostic failure; malformed input raises an error instead.
Full solve reproduction uses archived runtime_sources/whole_body_cycle.py,
--source docs/research/evidence/joint_execution_flat_20260908/attempt_0012/initial_source.txt
--seed docs/research/evidence/whole_body_cycle_20260908/previous_cycle_seed_v1.json
--scene unitree_robots/go2/phase2_flat.xml --out NEW_FILE.json
--check-jac --max-nfev35 --wall-budget-s240 (spell CLI options with spaces:
--max-nfev 35 --wall-budget-s 240). Set OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
MKL_NUM_THREADS=1. Archived solver defaults force objective180N, hard bound180N.
No repeated latency distribution exists for this full-cycle solve.
## Unrun research drafts INCLUDED, not promoted
Current tools/research/whole_body_cycle.py adds --force-target-n (default170N)
as an interior optimization objective; independent hard limit remains180N.
This change has only syntax validation, no actual solve yet. Intended next
experiment0002 is warm-start0001 with170N objective reserve. Independent
certificate decides feasibility, not the modified penalty. Keep0001 unchanged.
periodic_whole_body_feedback.py is an UNRUN prototype: periodic discrete TVLQR
from nominal full-model derivatives, unchanged2ms simulation,35Nm final clip,
multiple-cycle replay with optional initialvy/roll perturbations. Only py_compile
was run. Review/test Riccati convergence, quaternion conventions, trajectory/hash
path resolution (currently assumes absolute raw paths), imported source binding,
contact sensitivity, feedback saturation and periodic terminal mismatch before
using it. It is not a controller integration or validated feedback scheme.
analyze_shooting_contact_cycles.py is the independent worker's diagnostic script;
one locked replay produced0001/contact_diagnostic.json and its own script hash.
Its label1e6 denotes1e-6N threshold (calculation is correct); original output is
retained. Runtime numerical source6356 refers to the original solve, not a claim
that this later analysis script already existed in that commit.
## Recommended next decisions
First validate hard-force-feasible full cycle and actual contact pattern, then
multi-cycle feedback/perturbations with independent replay. If objective penalties
cannot produce adequate feasibility, use explicit nonlinear constraints or a
validated feasibility restoration step, not threshold tuning. Recheck derivatives
at later iterates/contact switches; initial directional agreement is not universal.
Do not spend another session on small controller gain patches. Do not integrate
an unexecutable nominal or treat periodic survival alone as B1.
Runtime architecture audit: retain ONE atomic execution owner; version its
payload/backend. Existing JointExecutionProposal insists on selected centroidal
problem identity, foot request and certificate; never forge those for a new
backend. A WholeBodyTrajectoryV1 alternative needs named model/observation/terrain
identity, absoluteq/v/tau(+K), tracking validity, events/commitments/certificates.
Producer seam CaptureModelPlanningObservation/TerrainControlSnapshot; model
wrapper needs worker-local forward/linearization data using same robot model.
Terrain collision geometry must come from same immutable OBSERVED terrain,
with explicit unknown coverage, never direct scene XML/GT at runtime.
Owner must sample this trajectory without regenerating cubics/20ms stance bridges.
Consumer can dispatch through existing ApplyJointExecutionTorque and final motor
certificate withkp=kd0; TVLQR does not inherit an ID-WBC solved-force certificate.
Validate shadow/adoption/state errors/deadlines/final commands before live flat,
then5cm and10cm under separately versioned empirical acceptance.
## Session agent/platform state
No robot simulation/build remains running at handoff. Latest enumeration had
acceptance_v2/phase_clearance_impl interrupted, execution_fix errored,
full_cycle_review/swing_bias_review completed, runtime pending_init despite
interrupt. No running worker. No close/despawn tool is exposed; handles were
NOT confirmed released. Two new spawn attempts failed thread limit. Do not
confuse interruption/completion/sidebar archive with actual handle release.
New session can create fresh Astra low workers for independent bounded work.
