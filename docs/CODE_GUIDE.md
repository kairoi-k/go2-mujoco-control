# Code guide

This guide points contributors to the smallest relevant source area for common changes. The primary research implementation is under `example/cpp/`.

## Entry points

| Task | Start here |
|---|---|
| Understand the runtime/control data flow | `docs/ARCHITECTURE.md` |
| Build or run the C++ stack | `example/cpp/README.md` |
| Change command-line configuration | `example/cpp/trot_cli.*` |
| Change stand / walk / stop sequencing | `example/cpp/trot_experiment_control.cpp` |
| Change gait phase or foot targets | `example/cpp/trot_experiment_gait.cpp`, `example/cpp/raibert_trot_kernel.h` |
| Change Raibert landing adjustment | `example/cpp/raibert_footstep_planner.h` |
| Change contact-force allocation | `example/cpp/contact_wrench_*` |
| Change centroidal wrench / foothold preview | `example/cpp/centroidal_wbc.h`, `example/cpp/preview_footstep_horizon.h`, `example/cpp/contact_wrench_qp.h`, `example/cpp/footstep_mpc.h` |
| Change `--wbc-full` ID-WBC / SRBD MPC | `example/cpp/go2_rigid_body.h`, `example/cpp/srbd_mpc.h`, `example/cpp/inverse_dynamics_wbc.h`, `example/cpp/dense_qp.h`, `example/cpp/trot_experiment_wbc.cpp` |
| Change dynamics-informed feedforward | `example/cpp/trot_experiment_wbc.cpp`, `example/cpp/trot_true_dynamics.h` |
| Change safety gates / diagnostics | `example/cpp/trot_experiment_diagnostics.cpp` |
| Change leg-lift / multi-step experiments | `example/cpp/leg_lift_*`, `example/cpp/real_leg_lift_go2.cpp` |
| Inspect retained experiment evidence | `example/cpp/experiments/`, `example/cpp/experiments/CATALOG.md` |

## Trot controller

`real_trot_go2.cpp` is the executable entry point. Most implementation is split across the `TrotExperiment` modules:

- `trot_experiment_lifecycle.cpp` — initialization, DDS, shutdown;
- `trot_experiment_gait.cpp` — velocity estimation and gait targets;
- `trot_experiment_wbc.cpp` — contact-force / dynamics-informed feedforward;
- `trot_experiment_diagnostics.cpp` — limits, quality gates, logging;
- `trot_experiment_control.cpp` — 500 Hz phase/control loop;
- `trot_types.h` — shared configuration and diagnostic structures.

## Leg-lift controller

The quasi-static action-sequence implementation is split into:

- `leg_lift_cli.*` — CLI parsing;
- `leg_lift_lifecycle.cpp` — setup and lifecycle;
- `leg_lift_world.cpp` — world-frame feedback;
- `leg_lift_diagnostics.cpp` — logging and acceptance diagnostics;
- `leg_lift_control.cpp` — control phases and command publication;
- `leg_lift_types.h` — sequence types and constants.

## Supporting headers

- Kinematics: `go2_forward_kinematics.h`, `go2_inverse_kinematics.h`, `go2_leg_jacobian.h`
- Gait: `locomotion_kernel.h`, `raibert_trot_kernel.h`, `raibert_footstep_planner.h`, `preview_footstep_horizon.h`, `footstep_mpc.h`
- Contact / WBC: `contact_*.h`, `contact_wrench_qp.h`, `dense_qp.h`, `wbc_runtime_gate.h`, `go2_contact_torque_mapping.h`
- Filtering / frames: `velocity_filter.h`, `motion_frame_utils.h`, `contact_state_filter.h`

When a change can alter research semantics, record the affected configuration/evidence and follow [`../CONTRIBUTING.md`](../CONTRIBUTING.md).
