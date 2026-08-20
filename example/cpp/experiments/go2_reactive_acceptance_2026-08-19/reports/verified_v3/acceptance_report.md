# Reactive acceptance — verified evidence

This report is generated from the synchronized controller CSV and run metadata. `PASS` requires zero status codes, a complete event window and recovery window, and the event-specific response gate.

The scripted turn/stop/obstacle cases validate continuous reference updates; they are not perception tests. `impact` is a physical velocity-push test. `low_friction` validates robustness under a friction change; it does not claim automatic terrain perception.

| case | event/source | gate | duration (s) | yaw Δ (rad) | vx drop (m/s) | jump (m/s) | transitions |
|---|---|---:|---:|---:|---:|---:|---|
| `go2_reactive_acceptance_baseline_video_v3_2026-08-19` | nominal / none | **REF** | 23.108 | - | - | - | `none` |
| `go2_reactive_acceptance_turn_left_video_v1_2026-08-19` | turn_left / scripted_reference | **PASS** | 23.106 | 0.207 | 0.144 | 0.019 | `none -> turn_left -> none` |
| `go2_reactive_acceptance_turn_right_video_v1_2026-08-19` | turn_right / scripted_reference | **PASS** | 23.102 | -0.209 | 0.149 | 0.025 | `none -> turn_right -> none` |
| `go2_reactive_acceptance_obstacle_right_video_v1_2026-08-19` | obstacle_right / scripted_reference | **PASS** | 23.104 | -0.161 | 0.167 | 0.026 | `none -> obstacle_right -> none` |
| `go2_reactive_acceptance_emergency_stop_video_v2_2026-08-19` | emergency_stop / scripted_reference | **PASS** | 23.102 | 0.003 | 0.147 | 0.019 | `none -> emergency_stop -> none` |
| `go2_reactive_acceptance_impact_video_v3_2026-08-19` | impact / push_dv=0.802mps | **PASS** | 23.104 | -0.018 | 0.242 | 0.802 | `none -> impact -> none` |
| `go2_reactive_acceptance_low_friction_video_v3_2026-08-19` | low_friction / simulator_friction | **PASS** | 23.106 | 0.003 | 0.082 | 0.014 | `none` |

## Data-quality checks

- All listed runs have complete metadata status codes equal to zero.
- Controller timestamps are checked for finite values and non-monotonic records; duplicate timestamps are retained because the controller and recorder run asynchronously.
- The analyzer checks the exact active event transition and requires at least 50 samples in both the event and recovery windows.

## Interpretation boundary

These results demonstrate that the current WBC/MPC pipeline can accept changed velocity/yaw references and survive the tested physical disturbance. Automatic external-event detection and a broader disturbance sweep remain separate validation work.
