# Terrain step-v1 review snapshot

This branch is a code-review snapshot, not an acceptance release.

## Baseline

- Base commit: `0068b14` (`P0` lidar height map, `P1` observation, `P2` 5 cm crawl 3/3).
- Review branch: `review/terrain-step-v1-wip-2026-08-24`.
- The stable `main` branch is intentionally unchanged.

## What this snapshot changes

- Samples the complete swing path against the simulated height map and checks IK.
- Checks a shrunken three-foot support polygon before reserving a front foothold.
- Holds the locomotion kernel while preparing a step.
- Adds a first staged weight-transfer experiment: rearward body shift, then lateral shift away from the released front leg, then sequential swing and neutral recovery.
- Adds explicit blocked/abort diagnostics and terrain fields to the CSV.

## Evidence already available

- Controller build succeeds.
- `ctest --output-on-failure`: 27/27 passed.
- P0/P1/P2 evidence is in the earlier commits and terrain reports.

## Known non-acceptance

- The prior dynamic 10 cm crossing did not pass: a kinematically valid unilateral transfer could still lose posture.
- The staged weight-transfer change in this snapshot has only compile/unit-test evidence so far; it must be runtime-tested on 5 cm, 10 cm, stairs, and flat regression before being merged or called stable.

## Review questions

1. Is the staged transaction the right bridge from a crawl kernel to coupled body/foothold planning?
2. Should body pose and footholds be represented as one receding-horizon `TerrainMotionReference` rather than per-leg offsets?
3. What minimum support-margin, contact-confirmation, and failure-recovery gates are required for 10 cm and continuous stairs?

See `TERRAIN_ADAPTATION_RESEARCH.md` for the literature and open-source design references.
