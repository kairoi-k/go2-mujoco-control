# Research index

This repository records **model-based Go2 control in MuJoCo**: a sequenced stand / walk / lie task and a diagonal-trot stack. It does not own the Kine2Go seam or AMP results.

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

**Outside the claim:** demo-speed running, natural animal gait, sim-to-real, or command-conditioned imitation.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Not in this repository

Isaac Lab velocity RL (including 3.5 m/s command curricula) and Kine2Go / AMP / seam JSON live elsewhere. `rl/` is a historical note; the implementations and checkpoints are omitted. Imitation evidence is in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Omit private session logs, generated builds, and checkpoints. A new controlled result needs: revision, intervention, held-fixed conditions, configuration, protocol, and artifact paths.
