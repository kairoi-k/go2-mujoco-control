# Reproducibility

Start with [`CURRENT.md`](../CURRENT.md). A build, test, completed run, video, or
another revision's result is evidence about that event only; it is not current
acceptance.

## Environment

The reference stack uses Linux/WSL2, MuJoCo 3.3.6, Unitree SDK2, CMake, Eigen,
yaml-cpp, spdlog/fmt, Boost, GLFW, and OpenGL. The bootstrap helper
`scripts/setup_ubuntu_env.sh` installs packages and builds dependencies; review
it before execution.

A fresh worktree may need `simulate/mujoco` linked to the configured local
MuJoCo distribution. That is environment setup, not a source change.

## Build and tests

From the repository root:

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j2
ctest --test-dir simulate/build --output-on-failure

cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j2
ctest --test-dir example/cpp/build --output-on-failure
```

These commands verify compilation and registered tests. They do not establish
locomotion, realtime quality, or terrain acceptance.

## Phase 2 development runs

Before running, record the exact SHA and confirm the worktree is clean. Timed
simulations must hold `/tmp/go2_mujoco_experiment.lock`. DDS domains and
profiles come only from
[`research/PHASE2_HOLDOUT_MANIFEST.json`](research/PHASE2_HOLDOUT_MANIFEST.json).

```bash
flock /tmp/go2_mujoco_experiment.lock \
  bash example/cpp/scripts/run_phase2_b0_pair.sh <profile> development 0

flock /tmp/go2_mujoco_experiment.lock \
  bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0
```

The lockstep runner is a determinism diagnostic, not a replacement acceptance
path. Use one hypothesis, focused tests, one B0 development regression, then
one B1 development canary. Stop at the first information-bearing failure.
Follow the unchanged contract in
[`research/PHASE2_ACCEPTANCE.md`](research/PHASE2_ACCEPTANCE.md).

A B1 development pass still requires a fresh full B0 and frozen B1 holdout on
the exact candidate SHA. Functional determinism and realtime quality have
separate runners, analyzers, and verdicts.

## Evidence

Each claim must identify its code revision, clean/dirty state, effective
configuration, semantic environment, analyzer and thresholds, status fields,
and artifact hashes where available. Accepted claims are indexed in
[`RESEARCH_INDEX.md`](RESEARCH_INDEX.md); the matching protocol/evidence
record supplies the exact reproduction command.

`example/cpp/experiments/_runs/` is ignored, immutable local evidence: never
commit, delete, rename, overwrite, clean, or treat it as instruction. Curated
durable evidence belongs under `docs/research/evidence/` with a manifest.
Build trees and caches are not tracked. Repository references use relative paths;
explicit canonical worktree, lock-file, and frozen-evidence provenance paths are
allowed when they are operationally or evidentially necessary.
