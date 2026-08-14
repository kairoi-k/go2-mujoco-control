# Isaac Lab environment snapshot (2026-08-10)

Recorded stack for the velocity-curriculum runs, including `model_54950`:

- Isaac Sim 6.0.1.0, Isaac Lab v3.0.0-beta2, rsl_rl 5.4.2
- Newton 1.4.0, MuJoCo 3.11, mujoco_warp 3.11, PyTorch 2.7.1+cu128
- physics backend: Newton / MuJoCo Warp
- custom configs: `go2_velocity_fast.tasks.flat_fast*` (command curricula ±2.0 … ±3.5 m/s)
- local RSL-RL NaN guards: PPO ratio clamp, `compute_returns` guard, loss skip; distribution sample guard
- evaluated checkpoint: `model_54950.pt` (GitHub Release `v0.1.0`)
- SHA-256: `c2009f890e5b575a8832021ab717dd2dcc23678a64f423d2f4e793d861ed4b42`
