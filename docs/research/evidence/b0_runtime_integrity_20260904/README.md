# B0 runtime-integrity evidence

This bundle is an active diagnostic record, not a B0 or B1 acceptance result.
`MANIFEST.json` is the bundle identity; `DECISIONS.md` records the F0-F14
hypotheses, observed failures, and next contract review. F14 lockstep
diagnostic is in F14_LOCKSTEP_20260905.md; it is not a full B0 result. The
manifest pins the exact reviewed candidate SHA; diagnostic and acceptance
claims remain separate.

The [cloud review packet](cloud_review/README.md) contains the exact candidate
identity plus a compact, hash-pinned subset of the latest raw canary evidence
for a GitHub-only read-only review. Large CSVs remain in the local immutable
archive and are listed by hash in [the packet manifest](cloud_review/PACKET_MANIFEST.json).

The current route and acceptance thresholds remain in `CURRENT.md` and
`docs/research/PHASE2_ACCEPTANCE.md`. Raw run directories stay in the ignored
experiment workspace and are not instructions.
