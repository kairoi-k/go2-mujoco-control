# Unitree Go2 MuJoCo control research

Research fork of [`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco) for **stand → walk → lie** sequencing, a model-based diagonal trot, and an Isaac Lab velocity-RL second track on Unitree Go2. Not a product, not a general control library, and not the Kine2Go imitation work.

> **What this repo actually is.** Two tracks. The onboarding result is a 500 Hz LowCmd state machine: stand-up, settle, trot, blend back to stand, lie-down. Reliable C++ cruise is about **0.13–0.18 m/s**. The second track is Isaac Lab velocity RL (`rl/`): command curricula up to **±3.5 m/s**, fast but short-stride. Motion imitation lives in [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research).

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

## Tracks

| Track | Where |
|---|---|
| Model-based C++ stand-walk-lie + slow trot | `example/cpp/` (promoted) |
| Isaac Lab velocity RL (fast, short-stride) | `rl/isaaclab_custom/` (configs; checkpoints not in git) |
| Kine2Go imitation, AMP, seam JSON | [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research) |

## Map

```text
example/cpp/       controller, tests, runners, retained experiment artifacts
simulate/          Unitree MuJoCo simulator base
unitree_robots/    robot models
rl/                Isaac Lab velocity-curriculum configs (second track)
patches/           simulator/runtime patches
scripts/           environment helpers
docs/              architecture, reproducibility, research history
```

## Docs

- [`docs/RESEARCH_INDEX.md`](docs/RESEARCH_INDEX.md) — claims vs this tree
- [`rl/README.md`](rl/README.md) — Isaac Lab velocity RL second track
- [`docs/RESEARCH_HISTORY.md`](docs/RESEARCH_HISTORY.md) — milestone history
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — runtime/control architecture
- [`docs/CODE_GUIDE.md`](docs/CODE_GUIDE.md) — source map
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md) — build notes
- [`example/cpp/experiments/CATALOG.md`](example/cpp/experiments/CATALOG.md) — experiment artifacts
- [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md) — upstream boundary

## License

BSD 3-Clause from the Unitree simulator base. Third-party assets keep their own terms: [`NOTICE.md`](NOTICE.md), [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md). Citation: [`CITATION.cff`](CITATION.cff).
