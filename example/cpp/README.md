# Go2 C++ control stack

This directory contains the primary model-based locomotion implementation and its retained research artifacts.

## Build

From the repository root, after MuJoCo and Unitree SDK2 are configured:

```bash
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"
```

The main locomotion target is `real_trot_go2`. Kinematics and other test targets are defined in `CMakeLists.txt`. `test_go2_forward_kinematics` is built only when `simulate/mujoco/lib/libmujoco.so` is present.

```bash
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build -j"$(nproc)"
ctest --test-dir example/cpp/build --output-on-failure
```

See [`../../docs/REPRODUCIBILITY.md`](../../docs/REPRODUCIBILITY.md) for environment assumptions and simulator build notes.

## Running the simulator/controller pair

`example/cpp/scripts/go2sim` wraps the simulator and controller launch used by the research experiments. Examples:

```bash
bash example/cpp/scripts/go2sim walk
bash example/cpp/scripts/go2sim walk --view
bash example/cpp/scripts/go2sim task
bash example/cpp/scripts/go2sim full
bash example/cpp/scripts/go2sim turn 0.3
```

The runner uses dedicated DDS domain IDs. Inspect it and `scripts/run_trot.sh` before adapting the setup to another machine or network.

## Source layout

Sources live in named modules under `example/cpp/`. `#include "foo.h"` still works because CMake adds every module directory.

| Area | Files |
|---|---|
| Entry point / CLI | `trot/real_trot_go2.cpp`, `trot/trot_cli.*` |
| Stand / walk / lie sequencer | `trot/trot_task.*` |
| Controller lifecycle | `trot/trot_experiment_lifecycle.cpp` |
| 500 Hz control loop | `trot/trot_experiment_control.cpp` |
| Gait and velocity targets | `trot/trot_experiment_gait.cpp`, `gait/raibert_trot_kernel.h`, `gait/raibert_footstep_planner.h`, `gait/footstep_mpc.h` |
| Dynamics-informed feedforward | `trot/trot_experiment_wbc.cpp`, `trot/trot_true_dynamics.h`, `wbc/centroidal_wbc.h`, `kinematics/go2_rigid_body.h`, `wbc/srbd_mpc.h`, `wbc/inverse_dynamics_wbc.h` |
| Diagnostics and safety gates | `trot/trot_experiment_diagnostics.cpp`, `trot/trot_types.h` |
| Kinematics | `kinematics/go2_forward_kinematics.h`, `kinematics/go2_inverse_kinematics.h`, `kinematics/go2_leg_jacobian.h` |
| Contact / wrench handling | `contact/contact_*`, `contact/contact_wrench_qp.h`, `wbc/dense_qp.h`, `contact/go2_contact_torque_mapping.h`, `wbc/wbc_runtime_gate.h` |
| Leg-lift / multi-step sequence | `leg_lift/real_leg_lift_go2.cpp`, `leg_lift/leg_lift_*` |
| Small apps | `apps/` |
| Tests | `tests/` |
| Experiment runners | `scripts/` |
| Retained evidence | `experiments/`, `experiments/CATALOG.md` |
| Offline analysis | `tools/analysis/` |

Maintained runners are in `scripts/`. Historical batch/parameter-sweep launchers are not in this tree.

For a higher-level map, see [`../../docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) and [`../../docs/CODE_GUIDE.md`](../../docs/CODE_GUIDE.md).

## Experiment records

A research-relevant run should identify the code revision, intervention/configuration, seed, input/reference identity, completion status, metric semantics, and evidence path. Disposable outputs belong in ignored local directories.

Claims and their scope are in [`../../docs/RESEARCH_INDEX.md`](../../docs/RESEARCH_INDEX.md). History: [`../../docs/RESEARCH_HISTORY.md`](../../docs/RESEARCH_HISTORY.md).
