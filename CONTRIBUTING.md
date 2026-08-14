# Contributing

Contributions are welcome. Because this repository doubles as a research record, changes that alter experimental meaning need explicit provenance.

## Development

- Keep commits small and scoped.
- Follow the build/test instructions in `example/cpp/README.md` for the C++ control stack.
- Do not commit machine-specific absolute paths, credentials, raw bulk logs, or generated build products.
- Preserve upstream attribution and licensing when modifying inherited simulator/runtime code.

## Research-semantic changes

Call out any change that can alter controller behavior, evaluation semantics, experiment protocol, acceptance criteria, or a scientific claim. Such PRs should state:

1. what semantic behavior changes;
2. what validation was run;
3. which experiment/config/data/evaluator version supports the result;
4. whether prior results are superseded, invalidated, or remain comparable.

Do not rewrite a failed experiment out of the record when it matters to the evidence trail. Do not change a frozen protocol or acceptance gate in place after observing results. Claims should remain narrower than the evidence; fixing known implementation defects does not prove that no undiscovered defects exist.

## Pull requests

Use the pull-request template to separate repository maintenance from research-semantic changes. Documentation-only cleanup should not modify algorithms, experiment artifacts, or result files.
