# Isaac Lab velocity RL (second track)

After the model-based trot hit a low speed ceiling (~0.15 m/s), Isaac Lab / RSL-RL velocity tracking was trained on Unitree Go2.

**Supported claim.** Command curricula of `lin_vel_x ∈ ±2.0, ±2.5, ±3.0, ±3.5 m/s` produced a fast policy (`model_54950`) that follows high speed commands in Isaac Lab.

**Outside the claim.** Short-stride velocity tracking, not a natural animal gait and not sim-to-real. Coarse tracking metrics can look good while the gait stays 碎步. `error_vel_xy` in Isaac Lab logs is tracking error, not body speed.

![0.5 m/s](../docs/media/rl_0.5ms.gif)
![3.5 m/s](../docs/media/rl_3.5ms.gif)

## Install

This is a gym-registered extension. It subclasses Isaac Lab's official Go2 flat velocity env; you do not copy files into `isaaclab_tasks`.

```bash
# inside the Isaac Lab Python environment
pip install -e rl
export ISAACLAB_PATH=/path/to/IsaacLab
```

Registered task ids:

| Task | `lin_vel_x` |
|---|---|
| `Isaac-Velocity-Flat-Unitree-Go2-Fast-v0` | ±2.0 m/s |
| `Isaac-Velocity-Flat-Unitree-Go2-Fast25-v0` | ±2.5 m/s |
| `Isaac-Velocity-Flat-Unitree-Go2-Fast30-v0` | ±3.0 m/s |
| `Isaac-Velocity-Flat-Unitree-Go2-Fast35-v0` | ±3.5 m/s |

Each has a `-Play-v0` variant.

## Play the recorded policy

```bash
python -m go2_velocity_fast.download -o model_54950.pt
python -m go2_velocity_fast.play --task Isaac-Velocity-Flat-Unitree-Go2-Fast35-v0 --num_envs 16 --checkpoint model_54950.pt
```

`play` / `train` register the gym ids, then run Isaac Lab's RSL-RL scripts. Checkpoint SHA-256: `c2009f890e5b575a8832021ab717dd2dcc23678a64f423d2f4e793d861ed4b42` ([Release v0.1.0](https://github.com/kairoi-k/go2-mujoco-control/releases/tag/v0.1.0)).

## Train

```bash
python -m go2_velocity_fast.train --task Isaac-Velocity-Flat-Unitree-Go2-Fast35-v0 --headless
```

Recorded training environment: [`ENV_SNAPSHOT.md`](ENV_SNAPSHOT.md) (Isaac Sim 6.0.1, Isaac Lab v3.0.0-beta2, rsl_rl 5.4.2, Newton / MuJoCo Warp). Local RSL-RL NaN guards used for `model_54950` are not vendored here.

Imitation / AMP evidence is in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).
