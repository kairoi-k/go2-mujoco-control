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
          ├─ gait phase / Raibert / preview footholds
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
| Gait generation | `trot_experiment_gait.cpp`, `raibert_trot_kernel.h`, `raibert_footstep_planner.h`, `preview_footstep_horizon.h`, `footstep_mpc.h` | diagonal-trot, Raibert, receding-horizon foothold MPC |
| Kinematics | `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`, `go2_leg_jacobian.h` | foot/joint transforms and Jacobians |
| Contact / force allocation | `contact_*`, `contact_wrench_qp.h`, `dense_qp.h`, `go2_contact_torque_mapping.h` | contact-state filtering, wrench QP, torque mapping |
| Dynamics-informed feedforward | `trot_true_dynamics.h`, `dynamic_acceleration_target.h`, `wbc_runtime_gate.h`, `centroidal_wbc.h`, `go2_rigid_body.h`, `srbd_mpc.h`, `inverse_dynamics_wbc.h` | `--wbc-primary` incremental feedforward; `--wbc-full` 18-DoF ID-WBC + SRBD MPC |
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

The repository describes `--wbc-full` as a controller-side 18-DoF inverse-dynamics WBC plus receding-horizon SRBD MPC. `--wbc-primary` remains incremental dynamics-informed feedforward.

## Process and asset boundaries

- `simulate/` and `unitree_robots/go2/` originate from the Unitree simulator stack and are kept close to upstream. Other Unitree robot MJCFs are not vendored here.
- `example/cpp/` contains the main research-specific control work.
- Isaac Lab / RSL-RL velocity RL is maintained in the separate
  [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl)
  repository and is not coupled to this controller at runtime.
- the Kine2Go / Genesis motion-imitation work is maintained in a separate companion repository.

For source-level navigation, see [`CODE_GUIDE.md`](CODE_GUIDE.md). For upstream provenance, see [`../UPSTREAM_AND_CONTRIBUTIONS.md`](../UPSTREAM_AND_CONTRIBUTIONS.md).
