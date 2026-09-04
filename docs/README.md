# Documentation

For Phase 2, the authority chain is deliberately small. Start at
[`CURRENT.md`](../CURRENT.md); every other document is subordinate context,
implementation guidance, or history.

## Current authority

| Order | Document | Purpose |
|---:|---|---|
| 1 | [`CURRENT.md`](../CURRENT.md) | only current route, status, plan, and handoff |
| 2 | [`AGENTS.md`](../AGENTS.md) | hard execution boundaries |
| 2 | [`research/PHASE2_ACCEPTANCE.md`](research/PHASE2_ACCEPTANCE.md) | Phase 2 acceptance contract |
| 3 | [`research/PHASE2_HOLDOUT_MANIFEST.json`](research/PHASE2_HOLDOUT_MANIFEST.json) | frozen profiles, domains, and holdouts |
| 4 | [`research/evidence/`](research/evidence/), [`../example/cpp/tools/`](../example/cpp/tools/) | curated raw evidence and protocol analyzers |

Evidence, Git history, issues, experiment notes, and agent prose never override
this order.

## Implementation and operation

| Document | Purpose |
|---|---|
| [`ARCHITECTURE.md`](ARCHITECTURE.md) | simulator/controller data flow and module boundaries |
| [`CODE_GUIDE.md`](CODE_GUIDE.md) | smallest source entrypoint for a change |
| [`REPRODUCIBILITY.md`](REPRODUCIBILITY.md) | build, test, experiment lock, and evidence rules |
| [`WBC_MPC.md`](WBC_MPC.md) | Phase 1 ID-WBC and SRBD-MPC implementation |
| [`../example/cpp/README.md`](../example/cpp/README.md) | C++ build and runtime entrypoints |
| [`../example/cpp/scripts/README.md`](../example/cpp/scripts/README.md) | runner lifecycle and Phase 2-safe entrypoints |
| [`../example/cpp/tools/analysis/INDEX.md`](../example/cpp/tools/analysis/INDEX.md) | analyzer lifecycle and scope |

## Claims and history

| Location | Meaning |
|---|---|
| [`RESEARCH_INDEX.md`](RESEARCH_INDEX.md) | accepted repository claims and boundaries |
| [`RESEARCH_HISTORY.md`](RESEARCH_HISTORY.md) | canonical milestone ledger and supporting narratives, including rejected work |
| [`validation/`](validation/) | accepted Phase 1 protocols and revalidations |
| [`../example/cpp/experiments/CATALOG.md`](../example/cpp/experiments/CATALOG.md) | retained historical experiment artifacts |
| [`upstream/`](upstream/) | preserved upstream documentation |

Dated protocol/delivery files in `docs/` are historical Phase 1 records unless
`CURRENT.md` explicitly adopts them. They cannot define Phase 2 work.
