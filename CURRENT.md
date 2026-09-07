# Go2 current research checkpoint
Updated: 2026-09-07. This is the live route/status/handoff entrypoint.
## Current scope and conclusion
The user authorized the shared-contract architecture migration and delegated
implementation choices to the lead agent. The active route is
[Locomotion architecture V1](docs/research/LOCOMOTION_ARCHITECTURE_V1.md):
independent proposal/actuator validation, shared geometry/frame/time semantics,
event-indexed body/foothold/force planning, and replaceable control backends.
First slice is implemented: independent current-model WBC certificates and
raw/selected proposal telemetry. Clean runtime 71d242a flat diagnostic found
4201/16403 legacy-accepted proposals infeasible, dominated by swing forces.
The next registered intervention removes inactive force variables structurally;
remaining inequalities and final actuator validation are still open. See
`docs/research/evidence/wbc_certificate_20260907/README.md` and
`docs/research/evidence/wbc_active_forces_20260907/README.md`.
Luna implements bounded subtasks; the lead owns scientific decisions and review.

B1 remains NOT_CERTIFIED. The corrected model is a mathematical correctness
baseline, not a validated locomotion release. No fresh full B0 or holdout
campaign was run. The next evidence-driven work is the executable future
liftoff/touchdown and geometry/frame contract, alongside a full WBC constraint
validator. F03 friction basis, F04 inequality acceptance, final PD torque,
privileged sensing and shared observation/action architecture remain open.
## Exact source and actual experiments
Active worktree: `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`.
Branch: `feat/stage-c-joint-planner`. Latest executed clean runtime source:
`71d242a80e84356cf9dd786830a01ff425a1a838`: flat 47/49 running cycles,
normal runtime integrity, physical certificate failures described above.
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
