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
Next: validate the opt-in event-indexed joint runtime shadow, then complete adoption,
coherent event commitment/execution and geometry/actuator validation using real
MuJoCo feedback. Sample-level model success alone keeps `execution_ready=false`.
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
`e1e68de1cee4edb93d1094a39a52c8e4bb5347ec`: the same-source contact-point
flat pair. The earlier `43c5f5a91e5c075a69170bc8af7d5582d1031474` two flat
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
