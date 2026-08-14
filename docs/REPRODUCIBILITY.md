# Reproducibility

This repository combines inherited simulator/runtime code, research-specific C++ control code, and curated experiment evidence. Reproducibility therefore has two layers: rebuilding the controller stack and reconstructing the evidence behind a specific research claim.

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

The sequenced controller and trot gates were finalized at development milestone `73ac543`. Claims for this tree are in [`RESEARCH_INDEX.md`](RESEARCH_INDEX.md).

A valid extension should preserve or version:

- the stand-walk-lie durations and Smoothstep transitions;
- trot period / duty / step-length / torque-gate settings used for a quoted speed;
- which experiment directory or CSV supports the quote.

Do not attach Kine2Go seam or AMP numbers to this repository. Those artifacts are in the companion imitation fork.

Isaac Lab velocity RL uses a separate stack (see `rl/isaaclab_custom/ENV_SNAPSHOT.md`). Configs are in git; the `model_54950` checkpoint is not.

Bulk logs, build trees, local checkpoints, and machine-specific caches are not in this tree.

The companion Kine2Go repository owns the motion-imitation evidence; do not mix numbers across the two projects.

## Hosted CI

The full simulator/controller build depends on external simulator/runtime packages and is not a generic GitHub-hosted compile gate. The build and kinematics commands above are the smoke path.
