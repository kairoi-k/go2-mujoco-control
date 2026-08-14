# Go2 Stand Tracking With State Start

This experiment records `LowCmd` joint targets and `LowState` feedback while Go2 stands up.

Key setup:
- The controller waits for the first `LowState`.
- The stand-up trajectory starts from the measured current joint positions.
- The target then moves smoothly to `stand_up_joint_pos`.

Files:
- `go2_tracking_v2.csv`: raw recorded data.
- `go2_tracking_v2_target_state.png`: target vs feedback joint positions.
- `go2_tracking_v2_error.png`: joint tracking error.

