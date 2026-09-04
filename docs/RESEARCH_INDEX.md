# Research index

This repository records **model-based Go2 control in MuJoCo**. Isaac Lab velocity RL and Kine2Go imitation are maintained in separate companion repositories.

This index records accepted Phase 1 and general repository claims. It does not
define the current Phase 2 route; use [`CURRENT.md`](../CURRENT.md).

## Implementation

**Stand-walk-lie sequencing + `--wbc-full` Raibert trot**, 2026-08-18 (`2b82dae`).

| Item | Record |
|---|---|
| Interface | 500 Hz `LowCmd` / `LowState` over Unitree SDK2 DDS |
| Task | `go2sim task`: stand-up 3 s → settle 0.5 s → `--wbc-full` trot → 2 s blend to stand → lie-down 3 s (`--tau-limit 35`) |
| Transitions | Smoothstep interpolation of 12 joint targets; gait amplitude ramps over 0.8 s from the stand pose |
| Reliable cruise | `--wbc-full` 64-cycle n=5: **0.130 ± 0.011 m/s** (0.116–0.147); stand-walk-lie 8 s n=3: **0.139 ± 0.004 m/s**. Target `0.091/0.60 = 0.151667`. Evidence: [`example/cpp/experiments/go2_wbc_full_mainline_repeat_2026-08-18`](../example/cpp/experiments/go2_wbc_full_mainline_repeat_2026-08-18/README.md) |
| Beyond that | `go2sim fast` (~0.18 m/s, `--tau-limit 19`) and 0.21 m/s are `--wbc-primary` history; 0.21 was rejected. `go2sim walk` (`--wbc-primary`) failed the 2026-08-15 same-gate session (`q_error=0.296`) and is not claimed on this tree |
| WBC | `--wbc-full`: 18-DoF ID-WBC + SRBD MPC. `go2sim task` and `go2sim full` both turn it on. A 2026-08-15 single run hit 0.149 m/s (ratio 0.985); that is historical, not the n=5 number. See `docs/WBC_MPC.md` |
| Clip | `docs/media/stand_walk_lie_wbcfull.gif` / `.mp4`. The older `stand_walk_lie.gif` is the `--wbc-primary` homepage clip and was kept |

## Validated high-speed simulation profile

**Source:** `66dc3e810dcf8766e4e2fd838e14fb772805c76d` (`gait/sustained-sprint-running-2026-08-21`), exact-head revalidated on 2026-08-24 in an isolated clean checkout.

The separate `running-trot` wall-clock profile passed the strict analyzer in all three independent 40 s repeats. The analyzer requires a continuous 20 s window at 2.90–3.80 m/s, bounded attitude, running-gait contact/clearance structure, zero lifecycle/safety/quality/dynamics/contact status failures, and a completed stop hold below 0.10 m/s.

| repeat | median speed | good window | roll/pitch P95 | aerial fraction | minimum pair sync | stop-tail P95 |
|---|---:|---:|---:|---:|---:|---:|
| `codex_reval_3mps_r1` | 3.235254 m/s | 61.352 s | 2.818/2.325° | 0.290171 | 0.812456 | 0.003860 m/s |
| `codex_reval_3mps_r2` | 3.225791 m/s | 61.342 s | 2.971/2.500° | 0.284504 | 0.814020 | 0.003713 m/s |
| `codex_reval_3mps_r3` | 3.225737 m/s | 61.630 s | 3.081/2.562° | 0.289935 | 0.799793 | 0.003905 m/s |

This is a model-based MuJoCo simulation result for running-trot. It is distinct from the diagonal-trot sprint wording and does not establish natural-animal gait, hardware performance, or sim-to-real transfer. The compact provenance record is [`docs/validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`](validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md); raw `_runs/` remain ignored.

**Supported claim:** the sequenced `--wbc-full` task, the historical slow trot baseline, and the separately validated 3 m/s-class running-trot profile in this simulator stack.

**Outside the C++ claim:** natural animal gait, sim-to-real, command-conditioned imitation, A→B world goals, or a reproduced `--wbc-primary` homepage walk.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Companion research tracks

Isaac Lab / RSL-RL velocity curricula, their environment snapshot, clips, and
checkpoint record live in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).
Motion imitation, AMP, and seam JSON live in
[`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated
builds are not in git; RL assets belong to the RL companion repository.
