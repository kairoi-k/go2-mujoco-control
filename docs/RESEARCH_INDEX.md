# Research index

This repository records **model-based Go2 control in MuJoCo** as the promoted result, plus an Isaac Lab velocity-RL second track. It does not own the Kine2Go seam or AMP results.

## Promoted implementation

**Stand-walk-lie sequencing + Raibert trot**, development milestone `73ac543` (2026-08-13).

| Item | Record |
|---|---|
| Interface | 500 Hz `LowCmd` / `LowState` over Unitree SDK2 DDS |
| Task | `--task stand-walk-lie`: stand-up 3 s → settle 0.5 s → trot → 2 s blend to stand → lie-down 3 s |
| Transitions | Smoothstep interpolation of 12 joint targets; gait amplitude ramps over 0.8 s from the stand pose |
| Reliable cruise | about 0.15 m/s (`go2sim walk`); 0.18 m/s (`go2sim fast`) needs `--tau-limit 19` |
| Beyond that | 0.21 m/s rejected under the recorded torque/quality gates |
| WBC | incremental dynamics-informed components with position-control fallback, not a complete full-dynamics stack |

**Supported claim:** the sequenced task and a slow, measurable trot run in this simulator stack.

**Outside the C++ claim:** demo-speed running from this controller, natural animal gait, sim-to-real, or command-conditioned imitation.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Second track — Isaac Lab velocity RL

Isaac Lab / RSL-RL velocity tracking after the C++ speed ceiling. Configs: `rl/isaaclab_custom/`. Snapshot: `rl/isaaclab_custom/ENV_SNAPSHOT.md`.

| Item | Record |
|---|---|
| Task | Go2 flat velocity tracking, command curricula ±2.0 → ±3.5 m/s |
| Recorded policy | `model_54950` (not in git) |
| Gait | fast, short-stride; official velocity-task reward, no reference motion |
| Comparison clips | archive `speed_0.5ms` … `speed_3.5ms` |

**Supported claim:** the recorded policy follows high commanded speeds in that Isaac Lab stack.

**Outside the claim:** natural gait, sim-to-real, or that C++ in this repo can reach 3.5 m/s. `error_vel_xy` is tracking error, not body speed.

Kine2Go / AMP / seam JSON remain in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Omit private session logs, generated builds, and checkpoints. A new controlled result needs: revision, intervention, held-fixed conditions, configuration, protocol, and artifact paths.
