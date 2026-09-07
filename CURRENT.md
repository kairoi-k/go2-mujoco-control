# Go2 current research checkpoint
Updated: 2026-09-07. This is the live route/status/handoff entrypoint.
## Current scope and conclusion
The user authorized the long-term architecture route, with real 5 cm B1
validation first and an independent 10 cm challenge next. Continue autonomous
research; a foundation checkpoint is not task completion. B1 remains
NOT_CERTIFIED; no new traversal result belongs to the work below.
The [joint articulated route](docs/research/STAGE_C_ARTICULATED_ROUTE.md) now
connects fixed absolute touchdown combinations, candidate-specific surfaces,
centroidal COM/velocity/angular-momentum/force optimization, actual-MJCF body and
joint reconstruction, acceleration lifting and independent full-model sample
certificates. Body position is not a fixed offset from COM. Initial joint
velocity cannot be silently reset. The solver accepts terrain-dependent COM
objective profiles; historical default references and frozen diagnostics remain.
Foot references have explicit liftoff/touchdown/stance lifetimes and a separate
terminal continuation when the next touchdown exceeds the dynamics horizon.
Missing contact, surface, timing and provenance coverage fails closed.
A shared production state conversion is used by WBC and the planner snapshot.
The actual sphere surface force Jacobian is separate from the geom-center motion
Jacobian. `TROT_RESEARCH_CONTACT_POINT_MODEL=1` enables a controlled WBC probe;
default off retains the old path. Applied mode is recorded in CSV. Force
redistribution uses that same selected Jacobian. Cartesian virtual-task torque
and final PD actuator composition remain separate from a dynamics certificate.
This flag is NOT evidence that the joint planner has taken execution authority.
The 5/10 cm V4 analyzer versions height/period assumptions while retaining
physical traversal gates and historical V3 output. The older baselines and
T13 frozen aerial conflict remain available unchanged. A single passing run
still cannot replace the registered campaign and coverage review.
Same-source e1e68de flat off/on probes completed: 40/42 versus 33/42 good
cycles in the identical 18--24 s window; 210 versus 372 sample certificate
failures. The point model remains opt-in. Historical 49-cycle counts used a
17--24 s window, not a different period. See the joint_point_pair_20260907 packet.
Actual f9623e1 state-clock flat shadow now yields 6/12 reduced proposals;
remaining captures reject unknown initial/candidate patches. The horizon-tail
contract is fixed for the opt-in new route. Source318/320 becomes271/320 after
current-heading registration; independent replay isolates low-Y edge cropping
and interior source holes. Direct capture-heading queries preserve source coverage in the next pair.
That flat run completes; its 5cm run fails physically. Remaining map holes
and scheduled-versus-measured initial contact mismatch are under review.
First failure in the new5cm run: FL non-top23.512s, nonfoot23.706s, hard
posture state23.874s/cmd21.868s. Historical23.216s is STATE time, not command.
31 isolated identical-state replays match semantic outputs; capture pipeline
p50/p95/max 1.547/1.862/1.954 ms. A live 60.6 ms outlier remains explicit.
The short replay backend now initializes an actual retained state in MuJoCo,
tracks joint COM/momentum/feet through WBC with explicit attitude feedback and
zero extra motor PD. The numerical failure was independently shown feasible and resolved through
an opt-in verified-seed primal active-set solver. Exact short runtime
fb837929e16e312e95611bf33ff3b1fe8e02d851 completes 100 MuJoCo steps with
body/support priority, no new inflight clearance bump, and no extra motor PD.
Maximum COM/foot errors are 16.382/29.753 mm; no nonfoot contact or actuator
saturation. Independent physical sample residuals are below 7e-13; independent
Python MuJoCo replay reproduces logged states and actuator torques exactly.
QP p50/p95/max are 73.315/113.525/206.282 us. This privileged-state, 0.2-second
cold-start counterfactual is not full runtime adoption or B1 acceptance.
The separately retained negative anchor input still rejects before stepping.
See joint_feedback_replay_20260907 attempts 0009 and negative_0001.
Next: complete the [joint feedback execution route](docs/research/JOINT_FEEDBACK_EXECUTION_V1.md).
The new input path retains bounded immutable capture history with explicit
stationary-terrain/freshness/conflict semantics and v2 exact-state replay.
Whole-combination search now retains its actual selected problem/result for
an execution consumer, rather than only scalar diagnostics. The atomic reference owner and shared FeedbackTick now have focused tests,
including command-only C1 handover, selected-combination commitments, curve
leases, expiry, independent ID-WBC and final motor-envelope gates. Capture accepts
commitments before solve, preserving exact targets under current-map verification.
Six rebuilt focused targets pass; see the adoption files in the feedback packet.
The opt-in `TROT_RESEARCH_JOINT_EXECUTION=1` runtime transport is now implemented:
worker publishes immutable selected bundles; control snapshots return commitments;
LowCmdWrite adopts a command-only geom-center reference and writes certified WBC
motor-order torque with zero extra PD. Old per-leg terrain transactions/planner
execution are bypassed in this mode. The existing stop owner handles failures.
Initial command velocity comes from consecutive commanded world-center positions;
an explicit <=20ms Hermite soft stance-reference bridge preserves C1 and settles
before liftoff. Prepared swing leases outlive old body horizons, without extending
body/force validity. Seven focused targets pass; full controller rebuilt.
The first actual flat canary f5b2ef98364bbc01f934de575c850696236e1342 is a
FAILED diagnostic: joint execution actually adopts3versions, with121 raw zero-PD
rows in STATE20.004--20.246. The .16->.14 legacy period change at20.068 resets
the planning epoch and invalidates commitments, starving new plans; expiry then
requests stop. Sampled COM/foot maxima8.114/78.542mm; no traversal result.
The fixed-start21s isolation (actual8f3e04afa97507ff4151838c4edf608923d2702e)
also FAILS:46actual zero-PD commands/3versions, then wbc_solver_failed at21.112,
with period.14/epoch1 unchanged. Earlier21.062 anchor observation rejection is
separate. Actual f586950 repeats this failure at STATE21.110 after53 commands.
Independent exact-matrix HiGHS/KKT analysis proves the QP feasible; active-row
numerical drift in the saddle-point search direction caused rejection. QR
nullspace projection passes both actual-matrix regressions without relaxing
constraints. New full-runtime validation is pending; sampled foot error in the
failed run reaches118.023mm and remains an execution concern. Variable-period calendar transitions remain an unresolved requirement. The clock caller
now passes commitment activity rather than silently resetting the epoch.
Default remains off. See joint_execution_flat_20260908.
Then run isolated flat and 5cm, followed by 10cm only after credible 5cm evidence.
Strict stationary-foot reconstruction remains a conditional fixture: actual
compliant running q/dq must not be projected to make it pass. Sample-level
model success alone keeps `execution_ready=false`.
See `docs/research/evidence/joint_articulated_20260907/` for model evidence.
The next source adds the existing WBC QP's optional articulated COM/momentum
objective and a non-projecting feedback sample preview. Material-point speed
and normal gap remain explicit diagnostics; contact evolution/execution remain
unverified. The [joint shadow protocol](docs/research/JOINT_RUNTIME_SHADOW_V1.md)
binds actual worker snapshots to multi-event candidate combinations. It has no
command authority. Unbound future event targets are now accepted by the new
combination path; historical bound-event defaults and commitments remain strict.
## Exact source and actual experiments
Active worktree: `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`.
Branch: `feat/stage-c-joint-planner`. Latest executed clean runtime source:
`f58695069e832a8c58acc4b5614245e11154d965`: fixed-period actual joint flat,
failed atSTATE21.110 on primary WBC numerical solve after53applied commands.
Prior `f5b2ef98364bbc01f934de575c850696236e1342`: first actual joint actuation flat,
failed on period-transition commitment conflict and reference expiry.
Prior `cc696232eab2d184b5c98fa89674f8d029895dad`: bounded-history flat diagnostic.
Normal completion,15/16 reduced proposals,35/42 cycle diagnostic; no joint
actuation. Pipeline p50/p95/max1.072/50.146/50.718ms includes two2400-QP-iteration
captures. Raw CSV independently confirms six sampled final-command saturations
in STATE20--28, max67.513Nm beyond model limit, dominated by joint PD. See
joint_terrain_history_20260907. Recorded v2 first snapshot reproduces its
initial-contact-anchor rejection; use the separately attributed retained f962
feasible snapshot for the next short-horizon feedback experiment.
Earlier `0ff9dc8fb755df231d26b4816d086fd77ed5d1b8`: capture-heading shadow pair.
Flat completes (9/16 proposals); 5cm FAILS with nonfoot collision/posture stop.
See joint_capture_view_20260907; joint command authority remains off.
Earlier f9623e1 is the horizon/query flat diagnostic with six proposals.
Earlier a180e605 wall/state pair has zero solver calls.
Earlier `e1e68de1cee4edb93d1094a39a52c8e4bb5347ec` is the contact-point pair. The earlier `43c5f5a91e5c075a69170bc8af7d5582d1031474` two flat
diagnostics retain their separate window/provenance; the first overlaps archival
load, the second was registered as isolated.
The previous observer-only runtime71d242a had 47/49 good running cycles.
The following step results belong to prior runtime
`7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6`. HEAD itself is given by Git.
WBC swing cost now uses physical `J qdd + Jdot qvel - a_des`, matching stance.
The caller does not pre-subtract bias. MuJoCo position finite differences and
an equivalent production QP fail on the old code and pass after correction.
44/44 controller tests and 3/3 simulator tests pass. No gait, gains, planner,
friction, acceptance thresholds or analyzer changes were made in this experiment.
32 s flat control: 43/49 good running cycles, normal completion/safety.
Same-source 5 cm step: full exit, four top-support witnesses, no nonfoot
collision, normal completion/safety, but all four feet have non-top contact.
Only 2/7 interaction cycles meet V3 running topology. Interaction speed
p05/median 0.559852/0.718292 m/s fails the existing median gate. Clock drift is
3.915 ms in step, 16.009 ms flat; this is not a clock-fix claim.
At first FR/FL impacts (23.216/23.262 s), all 100/101 rows in the preceding
inclusive 0.2 s have no-safe-foothold, no usable/applied plan and no in-flight
target. The immediate terrain planning/execution blocker therefore persists.
This does not prove the sole cause of impacts. The correction did not establish
an empirical speed/running improvement over the retained reference.
## Evidence and historical baseline
[The F02 packet](docs/research/evidence/wbc_swing_bias_20260907/README.md)
contains red/green tests, source/binary bindings, raw hashes, unchanged analyzer
results, independent review observations and a deterministic replay command.
Legacy profile analyzer KeyError and dependent Phase-2 failure remain recorded;
wrapper exit status does not establish physical acceptance. Independent physical constraint residuals are now logged; legacy solver
acceptance still does not guarantee physical feasibility. MuJoCo works via pinned localhost SSH.
[The prior bounded checkpoint](docs/research/evidence/b1_checkpoint_20260907/README.md)
preserves clean f5b5155 / runtime eeb5d75: flat 46/49 good cycles, step 1/7,
speed p05/median 0.673269/0.850097, all-leg non-top contacts and 28.318 ms drift.
That is a historical reference, not a contemporary randomized control. The
video in OneDrive still depicts that older runtime, not the new correction.
## Authority and provenance
1. Current explicit user instructions and this live CURRENT.
2. AGENTS.md and identified historical acceptance baselines.
3. Frozen `docs/research/PHASE2_ACCEPTANCE.md` and
   `docs/research/PHASE2_HOLDOUT_MANIFEST.json` for their campaign.
4. Versioned protocols, including `B1_DYNAMIC_TRAVERSAL_V3.md` and
   `B1_REGISTERED_INTERVALS_V2.md`, and their bound evidence/analyzers.
Keep the 15 mm geometric diagnostic separate from dynamic feasibility; retain
T13 aerial/old-contract conflict. Planned/applied contact is not measured truth.
Use native Linux and hold `/tmp/go2_mujoco_experiment.lock` for timed simulation.
Use one clean exact source and new raw name. Never overwrite, delete, rename
or commit `_runs`, stashes, archived snapshots or other worktrees. Raw hashes,
source/binary bindings and target results establish what was actually tested.
