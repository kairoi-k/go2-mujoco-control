# Go2 model-based locomotion in MuJoCo

Model-based locomotion research for the Unitree Go2 in MuJoCo: a **500 Hz LowCmd controller** with online velocity commands, gait scheduling, **SRBD MPC**, and **18-DoF inverse-dynamics WBC**.

Research fork of [`unitreerobotics/unitree_mujoco`](https://github.com/unitreerobotics/unitree_mujoco).

![stand-walk-lie](docs/media/stand_walk_lie_wbcfull.gif)

## Current results

| Capability | Status |
|---|---|
| **3 m/s-class running trot** | Three independent strict revalidations; median speed about **3.23 m/s** |
| **Runtime arbitrary velocity** | Online 0–3 m/s command path; **15/15** frozen benchmark runs pass quantitative and legacy gates |
| **Stand → walk → stop → lie** | 500 Hz sequenced `LowCmd` state machine |
| **Model-based control** | SRBD MPC + 18-DoF ID-WBC with contact, torque, posture, solver, and plan diagnostics |
| **Terrain-aware locomotion** | Active Phase 2 research; no terrain-crossing claim is made on `main` |

All results above are **MuJoCo simulation results**. They do not establish hardware performance, sim-to-real transfer, or natural-animal gait.

## Runtime velocity control

The Phase 1 command path is fully online:

```text
requested velocity profile
        ↓
jerk / acceleration-limited shaper
        ↓
continuous gait scheduler
        ↓
SRBD-MPC velocity reference
        ↓
18-DoF ID-WBC
        ↓
LowCmd
```

The frozen benchmark covers:

- stepped commands: `0 → 1 → 2 → 3 → 1 → 0 m/s`;
- acceleration: `1 → 3 m/s`;
- braking: `3 → 0 m/s`;
- continuous ramp: `0 → 3 → 0 m/s`;
- non-integer commands: `0.6, 1.4, 2.3, 2.8 m/s`.

Each profile was evaluated in three valid runs. All 15 runs passed both the legacy safety/status gate and the frozen quantitative acceptance gate.

See [`PHASE1_RUNTIME_VELOCITY_CLOSEOUT_2026-08-25.md`](docs/validation/PHASE1_RUNTIME_VELOCITY_CLOSEOUT_2026-08-25.md) and [`PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md`](docs/validation/PHASE1_QUANTITATIVE_ACCEPTANCE_2026-08-25.md).

## Sustained running

A separate immutable running-trot protocol validates sustained 3 m/s-class locomotion.

Three independent strict revalidations passed with median speeds around **3.23 m/s**, bounded attitude, valid running contact structure, clean lifecycle/safety/quality status, and a completed low-speed stop tail.

This benchmark remains separate from the arbitrary-velocity benchmark and retains its own analyzer and thresholds.

See [`SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md`](docs/validation/SUSTAINED_RUNNING_3MPS_REVALIDATION_2026-08-24.md).

## Architecture

The controller and MuJoCo simulator run as separate processes and communicate through the Unitree SDK2 / DDS interface.

```text
MuJoCo simulator
        │
        │ LowState / SportModeState
        ▼
state snapshot + filtering
        │
        ▼
runtime velocity command
        │
        ▼
gait / foothold generation
        │
        ▼
SRBD MPC
        │
        ▼
18-DoF ID-WBC
        │
        ▼
safety gates + saturation
        │
        ▼
LowCmd
        │
        └──────────────► MuJoCo
```

The main research implementation lives under `example/cpp/`:

```text
example/cpp/
├── gait/          gait generation, foothold preview and scheduling
├── contact/       contact filtering and wrench allocation
├── kinematics/    Go2 FK / IK / Jacobians
├── wbc/           SRBD MPC, inverse-dynamics WBC, dense QP
├── trot/          runtime controller and 500 Hz execution path
├── experiments/   retained experiment records and evidence
├── tests/         controller and math tests
└── tools/         analysis and research utilities
```

For the runtime data flow, see [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). For source-level navigation, see [`docs/CODE_GUIDE.md`](docs/CODE_GUIDE.md).

## Quick start

Requires MuJoCo, Unitree SDK2, and the upstream simulator dependencies. `scripts/setup_ubuntu_env.sh` installs system packages; inspect it before running.

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build -j"$(nproc)"

cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"

ctest --test-dir example/cpp/build
```

Start the MuJoCo simulator, then use the C++ runners documented in [`example/cpp/README.md`](example/cpp/README.md).

Validated sustained-running entry:

```bash
bash example/cpp/scripts/run_sustained_running.sh --headless
```

Phase 1 arbitrary-velocity benchmark and analyzer:

```text
example/cpp/scripts/run_phase1_velocity_benchmark.sh
example/cpp/scripts/analyze_phase1_velocity.py
```

## Research and validation

The repository keeps implementation claims separate from exploratory results.

- [`docs/RESEARCH_INDEX.md`](docs/RESEARCH_INDEX.md) — current supported claims;
- [`docs/RESEARCH_HISTORY.md`](docs/RESEARCH_HISTORY.md) — research milestones, including rejected results;
- [`docs/validation/`](docs/validation/) — frozen acceptance and revalidation records;
- [`example/cpp/experiments/CATALOG.md`](example/cpp/experiments/CATALOG.md) — retained experiment artifacts;
- [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md) — build and reproduction notes.

Bulk generated runs are not committed by default. Reviewable analyzers, manifests, selected evidence, and bounded claims remain in the repository.

## Active research

Phase 2 extends the model-based stack toward sensor-derived terrain representation, foothold/contact planning, and terrain-aware execution.

It remains active research. `main` does **not** currently claim completed or repeatable obstacle-crossing capability.

## Companion tracks

Learning-based work is kept separate from this controller:

| Track | Repository |
|---|---|
| Isaac Lab / RSL-RL velocity locomotion | [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl) |
| Kine2Go motion imitation / AMP | [`kairoi-k/kine2go-research`](https://github.com/kairoi-k/kine2go-research) |

These repositories are independent research tracks and are not runtime dependencies of the C++ model-based controller.

## Repository map

```text
example/cpp/       research controller, tests, runners, experiments
simulate/          Unitree MuJoCo simulator base
unitree_robots/    Go2 MJCF assets
docs/              architecture, research records, validation
docs/media/        README media
patches/           simulator/runtime patches
scripts/           environment helpers
```

Upstream boundaries and project-specific changes are documented in [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md).

## Scope

Supported claims in this repository are deliberately narrow:

- model-based Go2 control in MuJoCo;
- 500 Hz stand / locomotion / stop / lie sequencing;
- SRBD MPC + 18-DoF ID-WBC;
- validated 3 m/s-class running-trot simulation;
- quantitatively validated online arbitrary forward-velocity commands.

Not claimed:

- hardware validation;
- sim-to-real transfer;
- natural-animal gait;
- completed terrain traversal;
- results from the separate RL or imitation repositories.

## License

BSD 3-Clause from the Unitree simulator base.

Third-party assets retain their original terms. See [`NOTICE.md`](NOTICE.md) and [`UPSTREAM_AND_CONTRIBUTIONS.md`](UPSTREAM_AND_CONTRIBUTIONS.md). Citation metadata: [`CITATION.cff`](CITATION.cff).
