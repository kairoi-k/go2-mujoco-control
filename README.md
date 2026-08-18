# Unitree Go2 MuJoCo control research

Research fork of [`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) for Go2 **stand → walk → lie** and model-based diagonal trot control in MuJoCo.

![stand-walk-lie](docs/media/stand_walk_lie_wbcfull.gif)

The C++ result is a 500 Hz LowCmd state machine: stand-up, settle, `--wbc-full` trot, blend back to stand, lie-down. On this tree the sequenced / 64-cycle plant is **18-DoF ID-WBC + SRBD MPC**. Repeated `--wbc-full` cruise is about **0.12–0.15 m/s** (`go2sim full` n=5: 0.130 ± 0.011). Watch [`docs/media/stand_walk_lie_wbcfull.mp4`](docs/media/stand_walk_lie_wbcfull.mp4). The independent Isaac Lab velocity-RL track is maintained in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl); imitation work is in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

## Contents

`example/cpp/` is the research code:

- stand / walk / stop / lie sequencing at 500 Hz;
- diagonal-trot generation and Raibert footstep planning;
- world/support feedback and CSV logging;
- constrained contact-force allocation and an 18-DoF `--wbc-full` path (ID-WBC + SRBD MPC); `--wbc-primary` is still incremental feedforward with PD fallback.

## Quick start (C++)

Needs MuJoCo, Unitree SDK2, and the upstream simulator dependencies. `scripts/setup_ubuntu_env.sh` installs system packages; read it before running.

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j"$(nproc)"

cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"

./example/cpp/build/test_go2_forward_kinematics
./example/cpp/build/test_go2_inverse_kinematics
```

Stand-walk-lie (after the simulator is up): see [`example/cpp/README.md`](example/cpp/README.md). `go2sim task` is the sequenced `--wbc-full` entry; `go2sim full` is the same plant for 64 cycles. `go2sim walk` / `go2sim fast` are the older `--wbc-primary` trot-only paths and are not the current homepage claim. See [`docs/WBC_MPC.md`](docs/WBC_MPC.md).

## Related track

Isaac Lab / RSL-RL velocity curricula for Go2 are maintained in the separate
[`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl)
repository. This control repository does not vendor or install that package.

## Tracks

| Track | Where |
|---|---|
| Model-based C++ stand-walk-lie + slow trot | `example/cpp/` |
| Isaac Lab velocity RL (fast, short-stride) | [`go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl) |
| Kine2Go imitation, AMP, seam JSON | [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research) |

## Map

```text
example/cpp/       controller, tests, runners, retained experiment artifacts
simulate/          Unitree MuJoCo simulator base
unitree_robots/    Go2 MJCF only (other Unitree robots stay upstream)
docs/media/        README clips
patches/           simulator/runtime patches
scripts/           environment helpers
docs/              architecture, reproducibility, research history
```

## Docs

- [`docs/RESEARCH_INDEX.md`](docs/RESEARCH_INDEX.md) — claims vs this tree
- [`go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl) — Isaac Lab velocity RL companion
- [`docs/RESEARCH_HISTORY.md`](docs/RESEARCH_HISTORY.md) — milestone history
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — runtime/control architecture
- [`docs/CODE_GUIDE.md`](docs/CODE_GUIDE.md) — source map
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md) — build notes
- [`example/cpp/experiments/CATALOG.md`](example/cpp/experiments/CATALOG.md) — experiment artifacts
- [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md) — upstream boundary

## License

BSD 3-Clause from the Unitree simulator base. Third-party assets keep their own terms: [`NOTICE.md`](NOTICE.md), [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md). Citation: [`CITATION.cff`](CITATION.cff).
