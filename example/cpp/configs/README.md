# Controller configurations

For Phase 2, configuration and DDS allocation come only from
[`CURRENT.md`](../../../CURRENT.md) and
`docs/research/PHASE2_HOLDOUT_MANIFEST.json`.

## Versioned runtime profiles

JSON files in `profiles/` store a schema version, ordered controller
arguments, and semantic environment. The launcher accepts `--profile FILE` or
`GO2_PROFILE_PATH`:

```bash
bash example/cpp/scripts/go2sim \
  --profile example/cpp/configs/profiles/go2sim_full.json --headless
```

Precedence is deterministic: compiled defaults, then the versioned profile,
then explicit CLI overrides. Runs write `run_manifest.json` beside
`run_metadata.txt` with the effective arguments, profile hash, repository
state, semantic environment, artifact hashes, analyzers, and lifecycle status.

## Historical sequence configurations

The root `*.txt` files configure the standalone `leg_lift/` action-sequence
experiment. They are retained for provenance and explicitly scoped historical
reproduction only. They are not Phase 2 profiles, a terrain route, or a design
source; do not use them to create fixed leg order, crawl, stop-to-arm, or
three-contact entry behavior.
