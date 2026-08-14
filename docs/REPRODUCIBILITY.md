# Reproducibility

This repository combines inherited simulator/runtime code, research-specific C++ control code, and retained experiment artifacts. Reproducibility has two layers: rebuilding the controller stack and reconstructing the evidence behind a specific research claim.

## Environment

The research environment used MuJoCo 3.3.6 and Unitree SDK2 on Linux/WSL2. The C++ targets also depend on CMake, Eigen, yaml-cpp, spdlog/fmt, Boost, GLFW/OpenGL, and the dependencies required by the upstream Unitree simulator.

`scripts/setup_ubuntu_env.sh` is a bootstrap helper for a clean Ubuntu environment. Review it before execution: it installs system packages, downloads MuJoCo, clones/builds Unitree SDK2 when needed, and builds the simulator and C++ examples.

## Build from a configured checkout

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j"$(nproc)"

cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"
```

Basic kinematics checks:

```bash
./example/cpp/build/test_go2_forward_kinematics
./example/cpp/build/test_go2_inverse_kinematics
```

The simulator/controller pair inherits the DDS/runtime assumptions of the Unitree stack. See `example/cpp/README.md` and the preserved upstream documentation under `docs/upstream/`.

## What a reproduction should preserve

The sequenced controller and trot gates were finalized on 2026-08-13. Claims for this tree are in [`RESEARCH_INDEX.md`](RESEARCH_INDEX.md).

A valid extension should preserve or version:

- the stand-walk-lie durations and Smoothstep transitions;
- trot period / duty / step-length / torque-gate settings used for a quoted speed;
- which experiment directory or CSV supports the quote.

Kine2Go seam and AMP numbers belong in the companion imitation fork.

Isaac Lab velocity RL uses a separate stack (see `rl/ENV_SNAPSHOT.md`). Configs are a gym-registered package (`pip install -e rl`). The recorded `model_54950` checkpoint is on [Release v0.1.0](https://github.com/kairoi-k/go2-mujoco-control/releases/tag/v0.1.0).

Bulk logs, build trees, local checkpoints, and machine-specific caches are not in this tree.
