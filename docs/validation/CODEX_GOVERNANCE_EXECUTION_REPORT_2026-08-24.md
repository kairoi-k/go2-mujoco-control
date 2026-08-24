# Codex repository-governance execution report

Date: 2026-08-24 (Asia/Shanghai)

This report records the ordered execution required by `docs/CODEX_GOVERNANCE_INDEX.md` and `docs/CODEX_REPO_CLEANUP_PROMPT.md` at governance source commit `2153c1d081a1d631a41c0de11732d60c3e893468` on `maintenance/repo-governance-spec-2026-08-24`.

## Phase A — read-only reconciliation

Canonical repository: `kairoi-k/go2-mujoco-control`.

Refs audited before mutation:

| ref | SHA prefix | result |
| --- | --- | --- |
| `main` | `97b6b0a0` | canonical baseline |
| `speed/1mps-2026-08-21` | `d55335b` | reconciled |
| `gait/natural-trot-1mps-2026-08-21` | `d591e179` | reconciled |
| `gait/sustained-sprint-running-2026-08-21` | `66dc3e8` | reconciled |
| `recovery/wbc-transition-20260818` | `eda7a7d0` | retained for review, not merged |
| `review/terrain-step-v1-wip-2026-08-24` | `efece291` | retained WIP |

The expected high-speed ancestry chain and listed merged candidates passed. Terrain merge-base was `d41143fa`; recovery was not contained in main/high-speed. No topology STOP condition occurred.

## Phase A local workspace hygiene — complete inventory

This inventory was captured after remote cleanup and before publishing this update.

Canonical clone:

- `/home/che/dev/go2-mujoco-control`, branch `gait/sustained-sprint-running-2026-08-21`, exact HEAD `66dc3e8`; tracked files clean, local upstream is gone because the remote branch was intentionally deleted. Local `main` points to the current remote main.

Remaining Git worktrees and state:

| path | branch/HEAD | tracked-state classification |
| --- | --- | --- |
| `/home/che/dev/go2-mujoco-control` | sustained branch / `66dc3e8` | clean; retained canonical research checkout |
| `/home/che/dev/go2-mujoco-control-rl-ready` | `rl/ready-contract-2026-08-21` / `515aa86` | clean; retained RL contract worktree |
| `/home/che/dev/go2-mujoco-control-terrain` | `terrain/adaptation-2026-08-21` / `de73edb` | retained; contains nested WIP worktree |
| `/home/che/dev/go2-mujoco-control-terrain-faststep` | `terrain/fast-step-reference-2026-08-24` / `de73edb` | dirty tracked terrain changes and untracked acceptance/build outputs |
| `/home/che/dev/go2-mujoco-control-terrain-minimal` | `terrain/fast-step-minimal-2026-08-24` / `0068b14` | dirty tracked terrain changes and ignored build output |
| `/home/che/dev/go2-mujoco-control-terrain-p2ref` | `terrain/p2-reference-2026-08-24` / `0068b14` | tracked-clean; ignored build output retained |
| `/home/che/dev/go2-mujoco-control-terrain/go2-mujoco-control-terrain-step-v1` | `review/terrain-step-v1-wip-2026-08-24` / `efece291` | tracked-clean WIP; retained |
| `/home/che/dev/go2-natural-ref` | natural-trot branch / `d41143f` | tracked-clean; ignored build output retained |
| `/home/che/dev/archive/go2-config-manifest-v1` | merged infra branch / `c86f85e` | tracked-clean; ignored build and smoke `_runs` retained as evidence |
| `/home/che/dev/archive/go2-governance-report-2026-08` | merged docs branch / `845a614` | tracked-clean; retained until report publication was verified |
| `/home/che/dev/archive/go2-highspeed-revalidation-2026-08-24` | detached / `66dc3e8` | tracked-clean; raw revalidation `_runs` retained |
| `/home/che/dev/archive/go2-wbc-base-probe` | detached / `2b82dae` | retained probe artifacts |

Other local Git roots audited: `/home/che/dev/go2-isaaclab-rl` (`main`, one local commit ahead), `/home/che/dev/go2-mujoco-control-dev` (archived historical clone, `main` four commits behind and `example/cpp/scripts/run_trot.sh` modified), `/home/che/dev/kine2go-research` (clean `main`), `/home/che/dev/kine2go-research-dev` (clean archived development clone), `/home/che/dev/unitree_sdk2` (dependency clone, `main` four commits behind), and `/home/che/dev/archive/go2-wbc-transition-2026-08-18` (detached archive probe). No stash entries were found in the audited roots.

Local branch inventory retained for provenance or active work:

`backup/main-before-auto-sensing-2026-08-21`, `backup/main-before-environment-adaptation-2026-08-20`, `backup/main-before-wbc-full-2026-08-19`, `cursor/cartesian-world-trot-a3ec`, `docs/codex-governance-report-2026-08`, `docs/wbc-full-mainline-claims`, `docs/wbc-full-repeat-2026-08-18`, `feature/auto-environment-sensing`, `feature/environment-adaptation`, `gait/natural-trot-1mps-2026-08-21`, `gait/sustained-sprint-running-2026-08-21`, `infra/config-manifest-v1`, `main`, `recovery/wbc-transition-20260818`, `review/terrain-step-v1-wip-2026-08-24`, `rl/ready-contract-2026-08-21`, `speed/1mps-2026-08-21`, `terrain/adaptation-2026-08-21`, `terrain/fast-step-minimal-2026-08-24`, `terrain/fast-step-reference-2026-08-24`, `terrain/p2-reference-2026-08-24`, and `terrain/step-v1-2026-08-24`. Local branches whose remotes were deliberately deleted were not force-cleaned where they still anchor a worktree or unique evidence.

Duplicate/companion clone inventory is intentional rather than accidental: the canonical control clone, RL clone, terrain variants, natural-reference clone, Isaac Lab RL clone, archived historical control clone, two kine2go clones, the Unitree SDK dependency, and archive/probe worktrees listed above. No unclassified duplicate control clone was deleted.

Retained `_runs` roots:

`/home/che/dev/go2-mujoco-control/example/cpp/experiments/_runs`, `/home/che/dev/go2-mujoco-control-dev/example/cpp/experiments/_runs`, `/home/che/dev/go2-mujoco-control-terrain-faststep/example/cpp/experiments/_runs`, `/home/che/dev/go2-mujoco-control-terrain-minimal/example/cpp/experiments/_runs`, `/home/che/dev/go2-mujoco-control-terrain-p2ref/example/cpp/experiments/_runs`, `/home/che/dev/go2-mujoco-control-terrain/go2-mujoco-control-terrain-step-v1/example/cpp/experiments/_runs`, `/home/che/dev/go2-natural-ref/example/cpp/experiments/_runs`, `/home/che/dev/archive/go2-config-manifest-v1/example/cpp/experiments/_runs`, `/home/che/dev/archive/go2-highspeed-revalidation-2026-08-24/example/cpp/experiments/_runs`, and `/home/che/dev/archive/go2-wbc-base-probe/example/cpp/experiments/_runs`. These are ignored runtime/evidence outputs and were retained rather than automatically deleted.

## Phase B — immutable milestones

Created and pushed:

- `milestone/sustained-running-3mps-2026-08-22` -> `66dc3e810dcf8766e4e2fd838e14fb772805c76d`
- `archive/wbc-bumpless-transition-2026-08-18` -> `eda7a7d0`
- `wip/terrain-step-v1-2026-08-24` -> `efece291`

The optional 1 m/s tag was not created because no exact acceptance commit was unambiguous.

## Phase C — exact high-speed revalidation

An isolated clean worktree was checked out at exact `66dc3e810dcf8766e4e2fd838e14fb772805c76d`. The simulator and example builds passed. Simulator CTest had no registered tests; example CTest passed 25/25.

The exact documented entrypoint and analyzer were run three times with only run name and DDS domain varied (domains 191, 192, 193). All three analyzer results were PASS; all safety, quality, analysis, contact, dynamics, and completion statuses were zero. This was simulation evidence only and was not represented as real-robot or sim-to-real validation.

## Phase D — validated promotion

PR [#18](https://github.com/kairoi-k/go2-mujoco-control/pull/18) promoted the validated simulation evidence into README, research index/history, experiment catalog, and `docs/validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`. It was reviewed and merged. Post-merge main was `6c64301771861a36c027a090cc7c1c7d5c59f7a0`.

## Phase E — remote cleanup

Tracking issue [#19](https://github.com/kairoi-k/go2-mujoco-control/issues/19) was created for later recovery semantic review. The recovery archive tag and issue were verified before deleting the stale recovery branch. Each fully merged model-control branch was ancestry-checked immediately before its individual remote deletion; no glob deletion was used. Terrain WIP was kept.

## Phase F — repository boundary cleanup

The seven stale kine2go cursor branches were individually ancestry-checked and deleted. `go2-isaaclab-rl` was not restructured. Historical `go2-mujoco-control-dev` and `kine2go-research-dev` were tagged/described with the canonical source and archived on GitHub.

## Phase G — governance PR

PR [#20](https://github.com/kairoi-k/go2-mujoco-control/pull/20), commit `861cc8c`, added repository governance, lifecycle guidance, analysis index, hygiene checker, workflow, and recording-tool portability cleanup. Local hygiene passed with `repo_hygiene=PASS`; the merged main workflow passed.

## Phase H — config and run-manifest infrastructure

PR [#21](https://github.com/kairoi-k/go2-mujoco-control/pull/21), commit `c86f85e`, merged as `d569585a6f8e0b5993df05e3c0da67be86e35bf6`, added:

- schema-versioned JSON profiles in `example/cpp/configs/profiles/`;
- a standard-library-only profile loader;
- compiled defaults < profile < explicit CLI precedence;
- semantic environment capture without migrating all `TROT_HS_*` behavior;
- `run_manifest.json` generation while retaining `run_metadata.txt`;
- profile, Git, artifact, analyzer, lifecycle, contact, dynamics, quality, safety, and completion metadata.

Verification passed: simulator build, example build, CTest 25/25, shell syntax, Python syntax, JSON validation, repository hygiene, diff check, and a short profile simulation. The profile simulation produced zero controller, safety, quality, analysis, ground-truth, dynamics, and completion failures. No controller tuning or research-semantic change was made.

## Phase I — final state

Final canonical main before this audit update: `d569585a6f8e0b5993df05e3c0da67be86e35bf6`; this report update is published in the subsequent documentation merge.

## Main branch protection

The initial report identified that GitHub still returned `404 Branch not protected` for `main`. This was corrected through the GitHub branch-protection API and verified by read-back. Current policy is:

- PR required, with zero mandatory approvals because this repository has one active collaborator and self-approval would deadlock maintenance;
- required status check `portable-checks`, strict up-to-date enforcement enabled;
- administrator enforcement enabled;
- force-push and branch deletion disabled;
- linear history not mandatory, preserving research merge provenance.

The protected-branch endpoint now returns `enforce_admins.enabled=true`, `allow_force_pushes.enabled=false`, `allow_deletions.enabled=false`, and `required_status_checks.contexts=["portable-checks"]`.

Retained by design: the governance source branch, terrain WIP branch, immutable milestone/WIP tags, dirty research worktrees, and unique high-speed raw evidence. The recovery semantic-review issue remains open. The only unresolved ambiguity is the intentionally uncreated optional 1 m/s tag. All required implementation PRs are merged and the post-merge repository-hygiene workflow passed.

STOP conditions encountered: none.

Research semantics changed: only the documentation promotion of already validated simulation evidence; no real-robot, sim-to-real, or unsupported gait-capability claim was added.

Merge safety: complete and verified.
