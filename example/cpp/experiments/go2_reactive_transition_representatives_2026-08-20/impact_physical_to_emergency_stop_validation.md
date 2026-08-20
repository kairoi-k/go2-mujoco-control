# Physical impact -> emergency stop validation

Run: _runs/reactive_representative_impact_physical_to_emergency_stop

This is a real MuJoCo velocity push, not a scheduled impact token. The run used push-vel-x 0.8 m/s for 0.2 s with reactive event detection enabled; the emergency-stop event was scheduled after the physical response window.

Observed controller event sequence: none -> impact -> none -> emergency_stop. Impact was detected for 0.800 s (controller clock); emergency stop started at controller t=12.004 s and remained latched through the terminal WBC stance hold. The simulator log records the physical push as PUSH active t=8.002 with qvel0=0.921188; simulator and controller clocks are different.

Impact-window evidence: world vx max 0.936 m/s, world vx min -0.221 m/s, absolute yaw-rate max 0.579 rad/s, imu z-acceleration max 28.069 m/s^2. Peak roll and pitch were 0.088 rad and 0.145 rad. All WBC SRBD/ID status samples were valid; run metadata controller, safety, quality, analysis, ground-truth, dynamics and completion statuses were all zero.

Acceptance conclusion: the final AB clip now demonstrates a physical disturbance, automatic impact response, a later emergency-stop reference, and terminal WBC stance hold. It does not claim perception or autonomous planning.
