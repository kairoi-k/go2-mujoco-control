# Unitree Go2 MuJoCo control research

Research fork of [`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) for **stand → walk → lie** sequencing and a model-based diagonal trot on Unitree Go2. Not a product, not a general control library, and not the Kine2Go imitation work.

> **What this repo actually is.** A 500 Hz LowCmd state machine that interpolates through stand-up, a settle hold, trot, a blend back to stand, then lie-down. Reliable cruise in this stack is about **0.13–0.18 m/s**. Faster running in this project was Isaac Lab velocity RL (omitted here); motion imitation lives in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

Development milestone `73ac543` is provenance for the archive. It need not appear in a clean public snapshot. This repository is that snapshot.

## Contents

`example/cpp/` is the research code:

- stand / walk / stop / lie sequencing at 500 Hz;
- diagonal-trot generation and Raibert footstep planning;
- world/support feedback and CSV logging;
- constrained contact-force allocation;
- incremental dynamics-informed WBC **components** with fallback to position control — not a complete full-dynamics WBC stack.

## Quick start

Needs MuJoCo, Unitree SDK2, and the upstream simulator dependencies. `scripts/setup_ubuntu_env.sh` installs system packages; read it before running.

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j"$(nproc)"

cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"

./example/cpp/build/test_go2_forward_kinematics
./example/cpp/build/test_go2_inverse_kinematics
```

Stand-walk-lie (after the simulator is up): see [`example/cpp/README.md`](example/cpp/README.md). `go2sim task` is the sequenced entry; `go2sim walk` / `go2sim fast` are trot-only (fast ≈ 0.18 m/s with a looser torque gate).

## Other tracks (not this tree)

| Track | Where |
|---|---|
| Model-based C++ locomotion | this repository |
| Isaac Lab velocity RL (fast, short-stride) | development archive; `rl/` is a note only |
| Kine2Go imitation, AMP, seam JSON | [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research) |

## Map

```text
example/cpp/       controller, tests, runners, retained experiment artifacts
simulate/          Unitree MuJoCo simulator base
unitree_robots/    robot models
rl/                pointer to omitted exploratory RL
patches/           simulator/runtime patches
scripts/           environment helpers
docs/              architecture, reproducibility, research history
```

## Docs

- [`docs/RESEARCH_INDEX.md`](docs/RESEARCH_INDEX.md) — claims vs this tree
- [`docs/RESEARCH_HISTORY.md`](docs/RESEARCH_HISTORY.md) — milestone history
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — runtime/control architecture
- [`docs/CODE_GUIDE.md`](docs/CODE_GUIDE.md) — source map
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md) — build notes
- [`example/cpp/experiments/CATALOG.md`](example/cpp/experiments/CATALOG.md) — experiment artifacts
- [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md) — upstream boundary

## License

BSD 3-Clause from the Unitree simulator base. Third-party assets keep their own terms: [`NOTICE.md`](NOTICE.md), [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md). Citation: [`CITATION.cff`](CITATION.cff).
