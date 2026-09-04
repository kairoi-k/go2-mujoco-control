# Contributing

Contributions are welcome. Because this repository doubles as a research record, changes that alter experimental meaning need explicit provenance.

Before any Phase 2 work, read [`CURRENT.md`](CURRENT.md) and [`AGENTS.md`](AGENTS.md).
`CURRENT.md` is the sole route/status/plan authority; architecture, history,
experiments, issues, commits, and agent handoffs cannot override it.

## Development

- Keep commits small and scoped.
- Follow the build/test instructions in `example/cpp/README.md` for the C++ control stack.
- Do not add host-specific paths to maintained instructions, credentials, raw bulk logs, or generated build products. The canonical WSL worktree and experiment lock may be named when operationally required; frozen evidence may retain provenance paths. Never modify or commit ignored `example/cpp/experiments/_runs/` evidence.
- Preserve upstream attribution and licensing when modifying inherited simulator/runtime code.

## Research-semantic changes

Call out any change that can alter controller behavior, evaluation semantics, experiment protocol, acceptance criteria, or a scientific claim. Such PRs should state:

1. what semantic behavior changes;
2. what validation was run;
3. which experiment/config/data/evaluator version supports the result;
4. whether prior results are superseded, invalidated, or remain comparable.

Do not rewrite a failed experiment out of the record when it matters to the evidence trail. Do not change a frozen protocol or acceptance gate in place after observing results.

## Pull requests

Use the pull-request template to separate repository maintenance from research-semantic changes. Documentation-only cleanup should not modify algorithms, experiment artifacts, or result files.
