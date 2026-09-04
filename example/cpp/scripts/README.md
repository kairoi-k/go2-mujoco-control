# Controller runners

Phase 2 route, status, arguments, and acceptance come only from
[`CURRENT.md`](../../../CURRENT.md). A script's presence does not make it a
current or accepted route.

## Phase 2-safe entrypoints

| Script | Scope |
|---|---|
| `run_phase2_b0_pair.sh` | profile-based B0 development pair |
| `run_phase2_b0_fixed_pair.sh` | fixed B0 development pair |
| `run_phase2_b0_lockstep_pair.sh` | determinism diagnostic; not acceptance |

DDS domains come only from
`docs/research/PHASE2_HOLDOUT_MANIFEST.json`. Timed simulations must hold
`/tmp/go2_mujoco_experiment.lock`. Use the commands in `CURRENT.md`; do not
invent a profile, domain, retry loop, or alternative runner.

## Maintained non-Phase-2 runners

| Script | Scope |
|---|---|
| `go2sim`, `run_trot.sh` | general/Phase 1 controller launcher |
| `run_natural_trot.sh`, `run_running_trot.sh` | historical gait protocols |
| `run_sustained_running.sh`, `run_sustained_sprint.sh` | accepted high-speed protocol reproduction |
| `record_sustained_sprint.sh` | bounded capture helper |

Use these only with their matching indexed protocol and analyzer. They cannot
establish or alter Phase 2 status.

## Historical action-sequence helpers

`run_leg_lift_repeats.sh`, `run_leg_sequence.sh`,
`run_periodic_leg_lift.sh`, `run_single_step.sh`, `run_two_step.sh`,
`run_weight_shift_scan.sh`, `record_periodic_leg_lift.sh`, and
`record_reactive_acceptance.sh` are retained for provenance or an explicitly
scoped historical study. They are not Phase 2 entrypoints or design sources.
Do not use them to construct a crawl, fixed leg order, stop-to-arm transition,
or three-contact entry gate.

## Evidence and changes

Named curated records live under `experiments/go2_*/`. All disposable output
goes to ignored `experiments/_runs/`, which is immutable local evidence and
must never be committed, deleted, renamed, overwritten, or treated as
instruction.

A new or changed runner must document its owner protocol, effective arguments,
semantic environment, output path, analyzer, and lifecycle category. Update
this file and the analyzer index in the same PR.
