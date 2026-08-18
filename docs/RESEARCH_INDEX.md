# Research index

This repository records **model-based Go2 control in MuJoCo**. Isaac Lab velocity RL and Kine2Go imitation are maintained in separate companion repositories.

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

**Supported claim:** the sequenced `--wbc-full` task and a slow, measurable trot run in this simulator stack.

**Outside the C++ claim:** demo-speed running from this controller, natural animal gait, sim-to-real, command-conditioned imitation, A→B world goals, or a reproduced `--wbc-primary` homepage walk.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Companion research tracks

Isaac Lab / RSL-RL velocity curricula, their environment snapshot, clips, and
checkpoint record live in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).
Motion imitation, AMP, and seam JSON live in
[`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated
builds are not in git; RL assets belong to the RL companion repository.
