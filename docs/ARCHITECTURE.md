# Architecture

The model-based control path uses the same Unitree SDK2 / DDS message interface as the simulator: the controller publishes `LowCmd`, receives `LowState`, and the MuJoCo process bridges those messages to simulated robot dynamics.

## Runtime data flow

```text
MuJoCo simulator
  simulate/build/unitree_mujoco
          │
          │ DDS: LowState / LowCmd
          ▼
C++ locomotion controller
  example/cpp/build/real_trot_go2
          │
          ├─ state snapshot and filtering
          ├─ gait phase / Raibert target generation
          ├─ foot targets → IK → joint targets
          ├─ world/support feedback
          ├─ optional contact-force / WBC feedforward
          ├─ runtime safety gates and saturation
          └─ diagnostics / CSV logging
```

The core control loop runs at 500 Hz. The simulator and controller remain separate processes and communicate through the Unitree runtime interface rather than a repository-specific in-process simulation API.

## Main C++ modules

| Area | Files | Responsibility |
|---|---|---|
| CLI / lifecycle | `trot_cli.*`, `trot_experiment_lifecycle.cpp` | configuration, DDS setup, startup/shutdown |
| Control loop | `trot_experiment_control.cpp` | phase sequencing and motor-command publication |
| Gait generation | `trot_experiment_gait.cpp`, `raibert_trot_kernel.h`, `raibert_footstep_planner.h` | diagonal-trot phase logic and footstep targets |
| Kinematics | `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`, `go2_leg_jacobian.h` | foot/joint transforms and Jacobians |
| Contact / force allocation | `contact_*`, `go2_contact_torque_mapping.h` | contact-state filtering, wrench allocation, torque mapping |
| Dynamics-informed feedforward | `trot_true_dynamics.h`, `dynamic_acceleration_target.h`, `wbc_runtime_gate.h` | dynamics terms, acceleration targets, runtime gating |
| Diagnostics | `trot_experiment_diagnostics.cpp`, `trot_types.h` | safety checks, metrics, structured logging |
| Quasi-static sequence | `leg_lift_*`, `real_leg_lift_go2.cpp` | leg-lift and multi-step experiments |

## Model-based control path

At a high level, each control update:

1. snapshots joint, IMU, base, and contact observations;
2. advances the action/gait phase;
3. computes desired foot positions from the gait kernel and world-frame corrections;
4. maps foot targets to joint targets with inverse kinematics;
5. optionally computes contact-force/dynamics-informed feedforward terms;
6. applies runtime gates, limits, and fallback behavior;
7. publishes `LowCmd` and records diagnostics.

The repository describes these components as **incremental dynamics-informed WBC components**, not as a complete full-dynamics whole-body controller.

## Process and asset boundaries

- `simulate/` and `unitree_robots/` originate from the Unitree simulator stack and are kept close to upstream.
- `example/cpp/` contains the main research-specific control work.
- `rl/` is an exploratory research track and is not coupled to the model-based controller at runtime.
- the Kine2Go / Genesis motion-imitation work is maintained in a separate companion repository.

For source-level navigation, see [`CODE_GUIDE.md`](CODE_GUIDE.md). For upstream provenance, see [`../UPSTREAM_AND_CONTRIBUTIONS.md`](../UPSTREAM_AND_CONTRIBUTIONS.md).
