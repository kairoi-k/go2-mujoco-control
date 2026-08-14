# Isaac Lab velocity RL (second track)

This folder is **not** the stand-walk-lie C++ result. After the model-based trot hit a low speed ceiling (~0.15 m/s), Isaac Lab / RSL-RL velocity tracking was trained on Unitree Go2.

**Supported claim.** Command curricula of `lin_vel_x ∈ ±2.0, ±2.5, ±3.0, ±3.5 m/s` produced a fast policy (`model_54950`) that follows high speed commands in Isaac Lab. Recorded clips are the `speed_0.5ms` … `speed_3.5ms` comparison set.

**Outside the claim.** This is short-stride velocity tracking, not a natural animal gait and not sim-to-real. Coarse tracking metrics can look good while the gait stays 碎步. `error_vel_xy` in Isaac Lab logs is tracking error, not body speed.

**Not in git.** Checkpoints (`model_54950.pt`) and videos stay in the development archive. `*.pt` is gitignored.

## What is in tree

Drop-in configs for Isaac Lab’s Go2 velocity task package:

| File | Command range (`lin_vel_x`) |
|---|---|
| `isaaclab_custom/flat_fast_env_cfg.py` | ±2.0 m/s |
| `isaaclab_custom/flat_fast25_env_cfg.py` | ±2.5 m/s |
| `isaaclab_custom/flat_fast30_env_cfg.py` | ±3.0 m/s |
| `isaaclab_custom/flat_fast35_env_cfg.py` | ±3.5 m/s |

Recorded training environment: `isaaclab_custom/ENV_SNAPSHOT.md` (Isaac Sim 6.0.1, Isaac Lab v3.0.0-beta2, rsl_rl 5.4.2, Newton / MuJoCo Warp). Local RSL-RL NaN guards (ratio clamp, return guard, loss skip, sample guard) were applied in that environment; they are not vendored here.

To train, copy the four `flat_fast*.py` files into Isaac Lab’s `isaaclab_tasks/.../velocity/config/go2/` next to the official `flat_env_cfg.py`, then launch the `FlatFast35` task with RSL-RL.

Early handwritten MuJoCo PPO in this project is omitted. Imitation / AMP evidence is in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research), not here.
