# Research index

This repository records **model-based Go2 control in MuJoCo**. Isaac Lab velocity RL and Kine2Go imitation are maintained in separate companion repositories.

## Implementation

**Stand-walk-lie sequencing + Raibert trot**, 2026-08-13.

| Item | Record |
|---|---|
| Interface | 500 Hz `LowCmd` / `LowState` over Unitree SDK2 DDS |
| Task | `--task stand-walk-lie`: stand-up 3 s → settle 0.5 s → trot → 2 s blend to stand → lie-down 3 s |
| Transitions | Smoothstep interpolation of 12 joint targets; gait amplitude ramps over 0.8 s from the stand pose |
| Reliable cruise | about 0.15 m/s (`go2sim walk`); 0.18 m/s (`go2sim fast`) needs `--tau-limit 19` |
| Beyond that | 0.21 m/s rejected under the recorded torque/quality gates |
| WBC | `--wbc-full`: 18-DoF ID-WBC + SRBD MPC; 64-cycle 0.15 gate pass at 0.149 m/s (ratio 0.985), ID 100%, eq residual ~1e-7 N. See `docs/WBC_MPC.md` |

**Supported claim:** the sequenced task and a slow, measurable trot run in this simulator stack.

**Outside the C++ claim:** demo-speed running from this controller, natural animal gait, sim-to-real, or command-conditioned imitation.

Entry points: `example/cpp/`, `docs/ARCHITECTURE.md`, `example/cpp/experiments/CATALOG.md`.

## Companion research tracks

Isaac Lab / RSL-RL velocity curricula, their environment snapshot, clips, and
checkpoint record live in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).
Motion imitation, AMP, and seam JSON live in
[`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Evidence policy

Keep artifacts needed to understand the controller. Bulk logs and generated
builds are not in git; RL assets belong to the RL companion repository.
