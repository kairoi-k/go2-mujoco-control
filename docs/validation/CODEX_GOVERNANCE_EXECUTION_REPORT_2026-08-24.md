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

Final canonical main: `d569585a6f8e0b5993df05e3c0da67be86e35bf6`.

Retained by design: the governance source branch, terrain WIP branch, immutable milestone/WIP tags, dirty research worktrees, and unique high-speed raw evidence. The recovery semantic-review issue remains open. The only unresolved ambiguity is the intentionally uncreated optional 1 m/s tag. All required implementation PRs are merged and the post-merge repository-hygiene workflow passed.

STOP conditions encountered: none.

Research semantics changed: only the documentation promotion of already validated simulation evidence; no real-robot, sim-to-real, or unsupported gait-capability claim was added.

Merge safety: complete and verified.
