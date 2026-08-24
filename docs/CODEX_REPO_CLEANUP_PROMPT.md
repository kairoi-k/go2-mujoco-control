# Codex execution specification — Go2 research repository governance

Use this as a staged task. Do not compress the phases or improvise research semantics.

## Global constraints

This is a repository-governance and infrastructure task.

Hard constraints:

- NEVER rewrite Git history.
- NEVER force-push.
- NEVER mutate `main` directly.
- NEVER merge `review/terrain-step-v1-wip-2026-08-24` as part of cleanup.
- NEVER treat WIP as accepted delivery.
- NEVER change controller gains, control laws, acceptance thresholds or physics merely to make tests pass.
- NEVER delete accepted research documents or compact result evidence.
- NEVER delete a remote branch unless its disposition has been verified against the current remote state.
- Preserve licenses, upstream attribution and provenance.
- Keep model-based MuJoCo, Isaac Lab RL and Kine2Go/AMP as separate repositories.
- If a scientific acceptance rerun fails, STOP that integration path and report the failure. Do not repair it inside the merge/cleanup task.

Primary canonical repository:
`kairoi-k/go2-mujoco-control`

Companion canonical repositories:
`kairoi-k/go2-isaaclab-rl`
`kairoi-k/kine2go-research`

Historical repositories to archive:
`kairoi-k/go2-mujoco-control-dev`
`kairoi-k/kine2go-research-dev`

Expected important model-control refs at audit time:
- main: `97b6b0a0abd36202171cff2f98ec1df69731860c`
- sustained high-speed integration head: `66dc3e810dcf8766e4e2fd838e14fb772805c76d`
- WBC transition recovery head: `eda7a7d0b678449daa69395510f4f896b31d8aea`
- terrain WIP head begins `efece29`

If remote refs moved, do not assume the old audit still applies. Recompute the comparisons.

---

# PHASE A — READ-ONLY STATE RECONCILIATION

Perform no writes first.

For every repository:

```bash
git fetch --all --prune --tags
git status --short
git remote -v
git branch -r
```

For `go2-mujoco-control`, produce a report containing for every remote branch:

- full ref
- HEAD SHA
- merge-base with `origin/main`
- ahead/behind count
- whether it is an ancestor of main
- proposed action
- reason

Expected fully merged deletion candidates from the 2026-08-24 audit:

```text
backup/main-before-wbc-full-2026-08-19
cursor/cartesian-world-trot-a3ec
cursor/cut-curation-meta-a3ec
cursor/hedge-cleanup-a3ec
cursor/hygiene-drop-named-bans-a3ec
cursor/portfolio-assets-a3ec
cursor/public-repo-polish-a3ec
cursor/public-review-fixes-a3ec
cursor/wbc-full-2ms-a3ec
cursor/wbc-mpc-complete-a3ec
cursor/wbc-mpc-lateral-a3ec
cursor/wbc-preview-wrench-a3ec
docs/wbc-full-mainline-claims
docs/wbc-full-repeat-2026-08-18
feature/auto-environment-sensing
```

For each candidate verify:

```bash
git merge-base --is-ancestor origin/<branch> origin/main
```

Do not delete anything in Phase A.

Verify the high-speed ancestry:

```text
main
  └─ speed/1mps-2026-08-21
      └─ gait/natural-trot-1mps-2026-08-21
          └─ gait/sustained-sprint-running-2026-08-21
```

Verify terrain is diverged from the sustained branch and record the exact merge-base.

Verify `recovery/wbc-transition-20260818` has a unique commit and is not contained in current high-speed/main.

Output:
`/tmp/go2_repo_branch_audit.md`

STOP if the remote topology materially differs from this specification.

---

# PHASE B — PRESERVE IMPORTANT IDENTITIES

Before deleting or merging remote work, create annotated tags **only after confirming the exact refs**.

Recommended tags:

```text
milestone/sustained-running-3mps-2026-08-22
archive/wbc-bumpless-transition-2026-08-18
wip/terrain-step-v1-2026-08-24
```

Optional accepted 1 m/s tag if its exact acceptance commit is clearly identified:
`milestone/natural-trot-1mps-2026-08-21`

Tag messages must include:
- source branch
- exact SHA
- acceptance/WIP/archive status
- relevant acceptance document
- statement that the tag does not upgrade WIP to accepted status

Do not move tags after creation.

---

# PHASE C — HIGH-SPEED EXACT-HEAD REVALIDATION

Work on an isolated local checkout of:

`66dc3e810dcf8766e4e2fd838e14fb772805c76d`

Requirements:

1. working tree clean;
2. build simulator;
3. build `example/cpp`;
4. run complete CTest suite with output on failure;
5. use the documented sustained-running entrypoint;
6. run at least 3 independent repeats;
7. run `analyze_sustained_running.py` on each;
8. retain per-run compact provenance;
9. do not edit thresholds between runs.

Documented entry:

```bash
bash example/cpp/scripts/run_sustained_running.sh --headless

python3 example/cpp/tools/analysis/analyze_sustained_running.py \
  example/cpp/experiments/_runs/<run-name>
```

Acceptance semantics must remain those recorded in:
`docs/SUSTAINED_RUNNING_3MPS_ACCEPTANCE_2026-08-22.md`

Do not weaken them.

For the new exact-HEAD verification, create compact records containing:
- commit SHA
- dirty state
- full effective argv
- relevant semantic environment variables
- binary hashes
- run name
- analyzer result
- key metrics
- final stop state
- artifact hashes where available

Do not commit raw `_runs`.

If any of the three required repeats fails:
- stop high-speed integration;
- preserve failure artifacts;
- report exact failure mode;
- do not tune or patch the controller in this task.

---

# PHASE D — CLAIM PROMOTION + INTEGRATION PR

Only if Phase C passes.

Create a new branch from current `origin/main`, for example:

`maintenance/promote-validated-high-speed-2026-08`

Integrate the sustained high-speed branch without mixing terrain/recovery changes.

Update canonical project state so these agree:

- top-level `README.md`
- `docs/RESEARCH_INDEX.md`
- `docs/RESEARCH_HISTORY.md`
- `example/cpp/experiments/CATALOG.md`
- relevant script/config README files

Rules for wording:
- make only evidence-backed claims;
- state simulation-only boundary;
- distinguish natural trot / diagonal sprint / running-trot where needed;
- preserve historical 0.13–0.15 m/s results as historical, not current upper bound;
- do not imply sim-to-real;
- do not promote terrain WIP.

Open a PR into main.

PR must include:
- source SHA `66dc3e8`
- exact revalidation commands
- test result
- three acceptance results
- compact manifest locations
- scientific claim changes

Do not merge automatically unless the full PR diff has been reviewed.

After merge, verify main contains the expected high-speed code/docs and re-run the lightweight checks.

Then the following branches become deletion candidates:

```text
speed/1mps-2026-08-21
gait/natural-trot-1mps-2026-08-21
gait/sustained-sprint-running-2026-08-21
```

Before deletion, verify each accepted commit remains reachable from main and/or its immutable milestone tag.

---

# PHASE E — REMOTE BRANCH CLEANUP

Destructive remote deletion is a separate step.

Generate the exact deletion command list first.

For every “fully merged” branch, re-run ancestry immediately before deletion.

Example:

```bash
git merge-base --is-ancestor origin/<branch> origin/main \
  && git push origin --delete <branch>
```

Never batch-delete an unverified glob.

For `recovery/wbc-transition-20260818`:
- do not merge it here;
- ensure the archive tag exists;
- create a tracking issue/note for later semantic review;
- only then may the stale branch be deleted.

For `review/terrain-step-v1-wip-2026-08-24`:
- KEEP it for now;
- ensure its WIP snapshot tag exists;
- no merge.

Expected model-control steady state immediately after cleanup:
- `main`
- `review/terrain-step-v1-wip-2026-08-24`
- at most any newly active maintenance branch/PR

---

# PHASE F — COMPANION REPOSITORIES

## `kairoi-k/kine2go-research`

Audit all branches.

The 2026-08-24 audit found these seven `cursor/*` branches fully contained in main:

```text
cursor/ckpt-lost-note-a3ec
cursor/cut-curation-meta-a3ec
cursor/hedge-cleanup-a3ec
cursor/hygiene-drop-named-bans-a3ec
cursor/portfolio-assets-a3ec
cursor/public-repo-polish-a3ec
cursor/public-review-fixes-a3ec
```

Re-verify ancestry and delete them.

Do not modify frozen evaluator semantics or research result artifacts.

## `go2-isaaclab-rl`

Do not restructure in this task.
It currently has only main and a clean repository boundary.

## `go2-mujoco-control-dev`

Treat as historical archive.

Do not merge current work into it.

If `baseline/2026-7-10-motion-control` is still desired as a named historical state, create:
`archive/motion-control-baseline-2026-07-10`

Add a clear archival banner/description pointing to canonical:
`kairoi-k/go2-mujoco-control`

Then archive the repository using GitHub settings/CLI if authorized.

If repository archival cannot be performed with current credentials, report the exact manual action needed; do not simulate it with arbitrary code changes.

## `kine2go-research-dev`

Same policy:
- preserve intentional diverged historical states with tags only if useful;
- add archival pointer to `kairoi-k/kine2go-research`;
- archive repository;
- no future feature work.

---

# PHASE G — GOVERNANCE/HYGIENE PR IN `go2-mujoco-control`

Start from the newly reconciled main.

Create:
`maintenance/repo-governance-2026-08`

This PR must not change locomotion behavior.

Add or update:

## 1. `docs/REPOSITORY_GOVERNANCE.md`

Document:
- source-of-truth repository map;
- branch lifecycle;
- tag policy;
- accepted/WIP/historical distinctions;
- evidence storage rules;
- config precedence;
- PR requirements.

## 2. Script lifecycle documentation

Update `example/cpp/scripts/README.md` with explicit categories:
- canonical entrypoint
- maintained reproducibility wrappers
- experiment helpers
- historical compatibility scripts

Do not mass-move scripts yet.

## 3. Analysis-tool index

Add:
`example/cpp/tools/analysis/INDEX.md`

For each analysis script or major family record:
- protocol/experiment owner
- current vs historical
- safe for new work? yes/no

Do not rename old analyzer paths in this PR.

## 4. Lightweight hygiene checker

Add a small dependency-free script, e.g.:

`example/cpp/tools/check_repo_hygiene.py`

Checks should include:
- build/cache/runtime outputs are not tracked;
- no `_runs` content is tracked;
- no accidental credentials;
- no obvious host-specific absolute paths in maintained configs/scripts;
- no newly introduced large media without explicit allowlist/justification;
- required governance/research-index files exist.

Grandfather existing historical large blobs. Do not fail merely because old videos are already present.

## 5. GitHub Actions

Add `.github/workflows/repo-hygiene.yml`.

Run only checks that are portable on hosted runners:
- shell syntax;
- Python syntax/compile;
- repo-hygiene checker;
- optional link/reference checks.

Do not pretend full MuJoCo/Unitree integration passed on hosted CI.

## 6. Branch policy

Recommend GitHub main rules:
- no force push;
- no main deletion;
- PR required;
- hygiene check required;
- no mandatory linear history if research merge provenance is useful.

Open PR, report diff and validation. Do not merge before review.

---

# PHASE H — CONFIG + RUN-MANIFEST INFRASTRUCTURE

This is a separate PR from governance-only cleanup.

Create:
`infra/config-manifest-v1`

Goal: one inspectable effective configuration per run.

## Configuration

Introduce versioned JSON profiles under:

`example/cpp/configs/profiles/`

Do not add YAML dependency.

Initial profiles should reproduce accepted baselines exactly.

The profile representation should contain ordered controller arguments and schema version.

Add profile loading to the canonical launcher while preserving old wrappers.

Desired precedence:

`compiled defaults < profile < explicit CLI override`

Semantic environment variables must be recorded until migrated.

Do not migrate all `TROT_HS_*` behavior in the same commit.

## Run manifest

Add `run_manifest.json` generation to the runner.

At minimum record:

- schema version
- run ID
- timestamps
- Git commit / branch / dirty state
- profile path + hash
- effective argv
- semantic env snapshot
- seed
- simulator/controller/scene hashes
- scenario/event-script hashes
- analyzer identities
- lifecycle/contact/dynamics/quality/safety/completion statuses

Keep `run_metadata.txt` during compatibility period.

## Equivalence gate

For baseline profiles:
- generated effective argv must match previous wrapper semantics;
- build + CTests pass;
- accepted smoke/benchmark behavior must not regress.

No controller tuning allowed in this infrastructure PR.

---

# PHASE I — REPORT BACK

For every phase report:

1. exact refs before;
2. files changed;
3. commands run;
4. test/acceptance results;
5. branches/tags created/deleted;
6. any unresolved ambiguity;
7. whether research semantics changed;
8. whether the phase is safe to merge.

Do not summarize a failed or incomplete phase as complete.

The desired end state is a clean research workflow, not merely a smaller branch list.
