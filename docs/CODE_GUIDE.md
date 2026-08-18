# Code guide

This guide points contributors to the smallest relevant source area for common changes. The primary research implementation is under `example/cpp/`.

## Entry points

| Task | Start here |
|---|---|
| Understand the runtime/control data flow | `docs/ARCHITECTURE.md` |
| Build or run the C++ stack | `example/cpp/README.md` |
| Change command-line configuration | `example/cpp/trot/trot_cli.*` |
| Change stand / walk / stop sequencing | `example/cpp/trot/trot_task.*`, `example/cpp/trot/trot_experiment_control.cpp` |
| Change gait phase or foot targets | `example/cpp/trot/trot_experiment_gait.cpp`, `example/cpp/gait/raibert_trot_kernel.h` |
| Change Raibert landing adjustment | `example/cpp/gait/raibert_footstep_planner.h` |
| Change contact-force allocation | `example/cpp/contact/contact_wrench_*` |
| Change centroidal wrench / foothold preview | `example/cpp/wbc/centroidal_wbc.h`, `example/cpp/gait/preview_footstep_horizon.h`, `example/cpp/contact/contact_wrench_qp.h`, `example/cpp/gait/footstep_mpc.h` |
| Change `--wbc-full` ID-WBC / SRBD MPC | `example/cpp/kinematics/go2_rigid_body.h`, `example/cpp/wbc/srbd_mpc.h`, `example/cpp/wbc/inverse_dynamics_wbc.h`, `example/cpp/wbc/dense_qp.h`, `example/cpp/trot/trot_experiment_wbc.cpp` |
| Change dynamics-informed feedforward | `example/cpp/trot/trot_experiment_wbc.cpp`, `example/cpp/trot/trot_true_dynamics.h` |
| Change safety gates / diagnostics | `example/cpp/trot/trot_experiment_diagnostics.cpp` |
| Change leg-lift / multi-step experiments | `example/cpp/leg_lift/*`, `example/cpp/leg_lift/real_leg_lift_go2.cpp` |
| Inspect retained experiment evidence | `example/cpp/experiments/`, `example/cpp/experiments/CATALOG.md` |

## Trot controller

`real_trot_go2.cpp` is the executable entry point. Sequencing lives in `TrotTask`; the remaining `TrotExperiment` modules own DDS, gait, WBC, and diagnostics:

- `trot/trot_task.*` — stand / walk / stand / lie sequencer;
- `trot/trot_experiment_lifecycle.cpp` — initialization, DDS, shutdown;
- `trot/trot_experiment_gait.cpp` — velocity estimation and gait targets;
- `trot/trot_experiment_wbc.cpp` — contact-force / dynamics-informed feedforward;
- `trot/trot_experiment_diagnostics.cpp` — limits, quality gates, logging;
- `trot/trot_experiment_control.cpp` — 500 Hz control loop;
- `trot/trot_types.h` — shared configuration and diagnostic structures.

## Leg-lift controller

The quasi-static action-sequence implementation is split into:

- `leg_lift_cli.*` — CLI parsing;
- `leg_lift_lifecycle.cpp` — setup and lifecycle;
- `leg_lift_world.cpp` — world-frame feedback;
- `leg_lift_diagnostics.cpp` — logging and acceptance diagnostics;
- `leg_lift_control.cpp` — control phases and command publication;
- `leg_lift_types.h` — sequence types and constants.

## Supporting headers

- Kinematics: `example/cpp/kinematics/go2_forward_kinematics.h`, `go2_inverse_kinematics.h`, `go2_leg_jacobian.h`
- Gait: `example/cpp/gait/locomotion_kernel.h`, `raibert_trot_kernel.h`, `raibert_footstep_planner.h`, `preview_footstep_horizon.h`, `footstep_mpc.h`
- Contact / WBC: `example/cpp/contact/contact_*.h`, `contact_wrench_qp.h`, `example/cpp/wbc/dense_qp.h`, `wbc_runtime_gate.h`, `example/cpp/contact/go2_contact_torque_mapping.h`
- Filtering / frames: `example/cpp/util/velocity_filter.h`, `motion_frame_utils.h`, `example/cpp/contact/contact_state_filter.h`

When a change can alter research semantics, record the affected configuration/evidence and follow [`../CONTRIBUTING.md`](../CONTRIBUTING.md).
