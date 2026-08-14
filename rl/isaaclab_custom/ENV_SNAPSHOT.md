# Isaac Lab environment snapshot (2026-08-10)

Recorded stack for the velocity-curriculum runs, including `model_54950`:

- Isaac Sim 6.0.1.0, Isaac Lab v3.0.0-beta2, rsl_rl 5.4.2
- Newton 1.4.0, MuJoCo 3.11, mujoco_warp 3.11, PyTorch 2.7.1+cu128
- physics backend: Newton / MuJoCo Warp
- custom configs: `flat_fast*.py` (command curricula ±2.0 … ±3.5 m/s)
- local RSL-RL NaN guards: PPO ratio clamp, `compute_returns` guard, loss skip; distribution sample guard
- evaluated checkpoint identity: `model_54950` (not distributed in git)

This snapshot describes one working environment. It is not a container or lockfile.
