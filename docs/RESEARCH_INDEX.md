# Research index

This repository records **model-based Go2 control in MuJoCo**, plus an Isaac Lab velocity-RL second track. It does not own the Kine2Go seam or AMP results.

## Implementation

**Stand-walk-lie sequencing + Raibert trot**, 2026-08-13.

| Item | Record |
|---|---|
| Interface | 500 Hz `LowCmd` / `LowState` over Unitree SDK2 DDS |
| Task | `--task stand-walk-lie`: stand-up 3 s → settle 0.5 s → trot → 2 s blend to stand → lie-down 3 s |
| Transitions | Smoothstep interpolation of 12 joint targets; gait amplitude ramps over 0.8 s from the stand pose |
| Reliable cruise | about 0.15 m/s (`go2sim walk`); 0.18 m/s (`go2sim fast`) needs `--tau-limit 19` |
| Beyond that | 0.21 m/s rejected under the recorded torque/quality gates |
| WBC | `--wbc-full` is centroidal QP + foothold MPC; same-gate audit vs 0.15 walk is in `docs/WBC_MPC.md` (not faster; both failed the quality gate in that session) |

**Supported claim:** the sequenced task and a slow, measurable trot run in this simulator stack.

**Outside the C++ claim:** demo-speed running from this controller, natural animal gait, sim-to-real, or command-conditioned imitation.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Second track — Isaac Lab velocity RL

Isaac Lab / RSL-RL velocity tracking after the C++ speed ceiling. Gym-registered package: `rl/` (`pip install -e rl`). Snapshot: `rl/ENV_SNAPSHOT.md`.

| Item | Record |
|---|---|
| Task | Go2 flat velocity tracking, command curricula ±2.0 → ±3.5 m/s |
| Recorded policy | `model_54950` ([Release v0.1.0](https://github.com/kairoi-k/go2-mujoco-control/releases/tag/v0.1.0)) |
| Gait | fast, short-stride; official velocity-task reward, no reference motion |
| Comparison clips | `docs/media/rl_0.5ms.gif`, `docs/media/rl_3.5ms.gif` |

**Supported claim:** the recorded policy follows high commanded speeds in that Isaac Lab stack.

**Outside the claim:** natural gait, sim-to-real, or that C++ in this repo can reach 3.5 m/s. `error_vel_xy` is tracking error, not body speed.

Kine2Go / AMP / seam JSON remain in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated builds are not in git; `model_54950` is on Release v0.1.0.
