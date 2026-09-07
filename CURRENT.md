# Go2 current research checkpoint

Updated: 2026-09-07. This file is the live route and handoff entrypoint.

## Current instruction and scope

The user authorized moving the external architecture audit's analytic witnesses
into production regression tests, then a bounded single-variable correction
and controlled closed-loop evaluation. Current hypothesis: swing physical
acceleration must include Jdot*qvel, as stance acceleration already does.
Preserve the frozen baseline, thresholds, observation path and gait settings.
This is not authorization to claim B1 acceptance from a unit-test correction.
The previous closed checkpoint is f5b5155fc4750b358c39516c70a4e76014cf98b9.

## Exact source and result

Active worktree: `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`.
Branch: `feat/stage-c-joint-planner`. Latest executed runtime source:
`eeb5d757620712759604d8c51b2b9075d05625dc` (clean and pushed before experiments).
The containing documentation/evidence checkpoint has the same runtime sources
and binaries, independently checked against the 135-source build binding.

B1 is not accepted. The latest interval-V2 step run physically clears the
5 cm platform with sustained support from every foot, no nonfoot collision,
and no runtime safety termination. Interaction velocity p05/median is
0.673269/0.850097 m/s. However, all four legs have non-top step contacts and
only 1/7 complete interaction cycles meets the V3 running topology criterion.
The profile/state clocks also depart by 28.318033 ms, above the preserved
20 ms V3 gate. The flat control completes with 46/49 good steady cycles.
Videos are measured-state visualization, not an acceptance certificate.

At the first FL/FR impacts, the preceding 0.2 s contains no usable or applied
terrain plan: every row reports no-safe-foothold. The immediate engineering
blocker is supplying an executable terrain touchdown/swing plan before impact.
This does not yet identify the sole cause of the resulting physical impacts.
See the evidence packet for rejection attribution and separately untested
future-body, start-state, foot-radius and clock hypotheses.

Latest verification: 44/44 controller CTest, 3/3 simulator CTest, and 48/48
focused Python analyzer tests. MuJoCo works in this worktree. No fresh B0
acceptance campaign or B1 holdout was run, and no candidate is declared.

## Evidence and recovery

[The bounded closeout packet](docs/research/evidence/b1_checkpoint_20260907/README.md)
contains the three eeb5d75 experiments, raw hashes, reproducible analysis,
source audit, validation and interruption inventory. The full previous CURRENT
is preserved verbatim as
[historical context](docs/research/evidence/b1_checkpoint_20260907/CURRENT_at_eeb5d75.md);
its obsolete branch names, missing-MuJoCo statements, next steps and autonomous
continuation instructions are not the current route.

Earlier implementation/evidence remains in
[the iteration packet](docs/research/evidence/b1_iteration_20260907/README.md).
The offline Stage C continuous core is retained and tested, but its reduced
centroidal certificate is not a body-geometry or closed-loop B1 certificate.

## Authority and provenance

1. Current explicit user instructions and this live CURRENT.
2. AGENTS.md and the identified historical acceptance baselines.
3. `docs/research/PHASE2_ACCEPTANCE.md` and
   `docs/research/PHASE2_HOLDOUT_MANIFEST.json` for their frozen campaign.
4. Versioned research protocols and corresponding raw evidence/analyzers.

Current development diagnostics use `docs/research/B1_DYNAMIC_TRAVERSAL_V3.md`
and `docs/research/B1_REGISTERED_INTERVALS_V2.md`; they do not silently replace
the old frozen campaign. Geometric 15 mm support diagnostics remain distinct
from dynamics feasibility. Preserve the T13 aerial/old-contract conflict.

Use native Linux for build/simulation. The task's pinned localhost SSH path
works; the Windows WSL launcher hang is a separate environment issue.
For a future timed simulation hold `/tmp/go2_mujoco_experiment.lock`, use one
clean exact source and one unique raw directory. Do not overwrite, remove,
rename or commit `_runs`, stashes, other worktrees or unfinished isolated work.
Source hashes and analyzer outcomes, not wrapper exit status or videos,
establish what was actually tested. The packet specifies the bounded proposed
next experiment; it has not been launched by this closeout.
