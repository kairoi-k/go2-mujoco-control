# Codex addendum — Local workspace hygiene

This addendum is mandatory together with `CODEX_REPO_CLEANUP_PROMPT.md`. Apply it after remote state reconciliation and before final reporting. Local cleanup must never destroy unique code, uncommitted work, research evidence, or the only copy of an artifact.

## 1. Discover every local workspace first

For the model-control and companion repositories, inventory all accessible local clones/worktrees before deleting anything.

For each clone record:
- absolute path;
- repository remote URL;
- current HEAD SHA and branch/detached state;
- `git status --short`;
- local branches and upstream tracking state;
- worktrees (`git worktree list --porcelain`);
- stashes (`git stash list`);
- untracked/ignored disk usage where practical;
- whether the clone contains `_runs`, build products, videos, logs, checkpoints, or other research artifacts not present remotely.

Write a local-workspace audit report before mutation.

## 2. Choose one canonical local clone per active repository

After remote governance is settled, designate one canonical local clone for each active repository:
- `go2-mujoco-control`;
- `go2-isaaclab-rl`;
- `kine2go-research`.

Other clones are duplicate workspaces, not independent sources of truth.

Do not delete a duplicate clone until all of these are true:
- no uncommitted tracked changes;
- no unique local commits;
- no unique stash;
- no unique experiment/result artifact;
- no active worktree depends on it;
- its useful refs are reachable from canonical remote branches/tags.

If any condition is uncertain, keep it and report it.

## 3. Local branch hygiene

After remote branches/tags have been reconciled:

- run `git fetch --all --prune --tags`;
- list local branches with upstream and ahead/behind state;
- remove stale local branches only when their commits are reachable from canonical main or an intentional immutable tag;
- never delete a local branch with unique commits merely because its remote branch disappeared;
- prune stale remote-tracking refs;
- keep only `main` plus branches corresponding to genuinely active work.

Target steady state for the canonical model-control clone: `main` plus at most the currently active feature/WIP branches.

## 4. Worktree hygiene

Audit every worktree individually.

A worktree may be removed only if:
- `git status` is clean;
- its HEAD is safely reachable from main/tag/retained branch;
- it contains no unique untracked research artifacts;
- no active Agent is still using it.

Then remove with normal Git worktree commands and run `git worktree prune`.

Never manually `rm -rf` a worktree directory before Git metadata is reconciled.

The former speed Agent and terrain Agent worktrees must be treated as potentially active/valuable until explicitly verified safe. Terrain WIP must remain recoverable.

## 5. Stash policy

Never run `git stash clear` as cleanup.

For every stash:
- record repository/path, stash ID, creation message/date, and affected files;
- inspect whether it contains unique useful work;
- apply/commit/tag/archive it only when its purpose is understood;
- otherwise retain it and report it.

An unexplained stash is unresolved state, not garbage.

## 6. Generated-data cleanup

Safe cleanup candidates include reproducible build/cache artifacts such as:
- CMake build directories;
- Python `__pycache__` / `.pyc`;
- temporary simulator build products;
- stale generated caches.

Do not automatically delete:
- `_runs`;
- raw CSV/logs used by unpromoted research;
- videos;
- checkpoints;
- acceptance evidence;
- failure bundles;
- external-delivery staging directories.

For research artifacts, first classify them as:
1. promoted/recorded and disposable;
2. unique evidence requiring retention/export;
3. unknown — keep and report.

Only category 1 may be deleted automatically.

## 7. Duplicate binary/build detection

Large local generated directories may be measured and summarized. Cleanup should target reproducible outputs, not source history.

Do not use local disk cleanup as an excuse to rewrite Git history.

## 8. Agent workspace rule going forward

Each Agent task gets its own branch/worktree.

Rules:
- one task = one named branch/worktree;
- Agent must begin from an explicitly recorded base SHA;
- Agent must not silently switch to another research branch;
- task closeout must reconcile its branch/worktree before the Agent is abandoned;
- concurrent Agents may not both mutate the same branch/worktree;
- cross-line integration is performed by a dedicated integration/governance task, not opportunistically by one feature Agent.

## 9. Mandatory task closeout procedure

Every future research/feature task must finish with a closeout, even when the result is negative or WIP:

1. record final HEAD and clean/dirty state;
2. run required build/tests/acceptance or explicitly record why not;
3. promote accepted evidence to the canonical Research Index/experiment record, or mark the task WIP/negative;
4. create the appropriate milestone/WIP/archive tag if the state must be preserved;
5. close/merge the PR or close the tracking issue with final status;
6. delete the remote working branch when safe;
7. delete the local working branch when safe;
8. remove the worktree when safe;
9. classify and clean disposable generated outputs;
10. confirm canonical local `main` is updated and clean.

No task is operationally complete until closeout is complete.

## 10. Final local-state report

Final governance reporting must include:
- canonical clone path for each active repository;
- remaining local branches;
- remaining worktrees and owner/task;
- remaining stashes and why they remain;
- duplicate clones retained and why;
- disk artifacts intentionally retained;
- generated data removed;
- unresolved local-state risks.

Desired end state: one clearly canonical clone per active repository, a small number of intentional worktrees, no stale tracking refs, no unexplained local branches/stashes, and no deletion of unique research evidence.
