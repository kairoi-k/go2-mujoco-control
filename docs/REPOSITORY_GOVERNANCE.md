# Repository governance

## Source of truth

Root [`CURRENT.md`](../CURRENT.md) is the sole route/status authority for the
active Phase 2 research line. The repository rules below remain general
governance and cannot supersede that route.

| Scope | Canonical repository | Boundary |
|---|---|---|
| Model-based MuJoCo Go2 control | `kairoi-k/go2-mujoco-control` | C++ controller, simulator integration, MuJoCo scenes, and model-based evidence |
| Isaac Lab velocity RL | `kairoi-k/go2-isaaclab-rl` | RL environments, training, checkpoints, and RL evidence |
| Kine2Go / AMP imitation | `kairoi-k/kine2go-research` | Imitation, AMP, seam JSON, and imitation evidence |
| Historical development | `*-dev` repositories | Read-only provenance only; no new feature work |

Do not vendor or merge the RL or imitation tracks into the model-control repository. A claim is canonical only when its code revision, protocol, acceptance semantics, and evidence are recorded in the owning repository.

## Branch lifecycle

- `main` is the canonical integrated line. It is changed through reviewed pull requests only.
- Named feature, experiment, and maintenance branches start from an explicitly recorded base SHA and have one task owner.
- Accepted milestones use an immutable annotated `milestone/` tag. A milestone tag records the source branch, exact SHA, evidence document, and simulation/hardware boundary.
- Research in progress uses `review/` or `wip/` naming and an immutable `wip/` tag when the snapshot must remain recoverable. WIP is never presented as accepted delivery.
- Recovery or superseded work uses an `archive/` tag and remains unmerged until semantic compatibility is reviewed.
- Delete a remote working branch only after its current remote SHA is rechecked and is reachable from canonical `main` or an intentional immutable tag. Never delete a branch by wildcard.

## Accepted, WIP, and historical states

Accepted means the exact current code passed the protocol's unchanged analyzer and its evidence is indexed. WIP means the work is useful and recoverable but has not met acceptance. Historical means it is retained for provenance and must not be used as the current upper bound. Simulation results must state that they are simulation-only; they do not imply hardware performance, natural-animal gait, or sim-to-real transfer.

## Tags and provenance

Tags are annotated, immutable identity records. Never move or force-update a tag. Tag messages include source branch, exact SHA, status, relevant acceptance or archival document, and an explicit statement that the tag does not upgrade WIP to accepted status. A run record includes Git SHA, branch and dirty state, effective arguments, semantic environment, binary/scene hashes, analyzer identity, statuses, and artifact hashes where available.
The canonical ledger [`docs/RESEARCH_HISTORY.md`](RESEARCH_HISTORY.md) is the one-to-one index for every accepted result, historical non-regression result, `milestone/*` tag, and retired route. A raw PASS without a ledger row is an indexing defect. Legacy immutable tags are not rewritten; their complete provenance is backfilled in the ledger.

## Evidence storage

Keep compact protocols, summaries, manifests, and acceptance records in Git. Keep raw `_runs/`, build directories, caches, temporary logs, and generated media ignored unless a documented delivery requires a specific retained artifact. Do not delete unique failure bundles, checkpoints, videos, or unpromoted research evidence during cleanup. Classify local artifacts before removal: disposable/reproducible, unique evidence, or unknown.

## Configuration precedence

The effective runtime configuration is resolved as:

`compiled defaults < versioned profile < explicit CLI override`

Semantic environment variables remain recorded during migration. A profile must identify its schema version, ordered controller arguments, and hash. Infrastructure changes must prove baseline equivalence and must not tune controller gains, control laws, physics, or acceptance thresholds.

## Pull-request requirements

Every PR states its base and source SHA, scope, research-semantic impact, exact commands, tests and acceptance results, evidence locations, and unresolved ambiguity. Governance/CI PRs must not change locomotion behavior. Integration PRs keep terrain WIP, recovery work, RL, and imitation boundaries explicit. Do not merge before the full diff and required checks have been reviewed. Hosted CI may run portable syntax, hygiene, and reference checks; it must not claim MuJoCo or Unitree integration without running that environment.
