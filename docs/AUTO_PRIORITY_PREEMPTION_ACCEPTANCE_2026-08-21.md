# Automatic priority preemption acceptance

This acceptance proves safety ordering under a real physical obstacle and a later physical push. It is deliberately different from a scripted token-transition test.

The run uses `scene_reactive_obstacle.xml`, `--auto-environment`, `--wbc-full`, and a 0.8 m/s simulator push at MuJoCo `state_tick_s=7.002`. The detector observes the height-map obstacle first, then the velocity jump:

| transition | controller clock | MuJoCo state clock | priority |
|---|---:|---:|---:|
| none | 0.000 | 1.652 | 0 |
| obstacle_left | 5.040 | 6.692 | 80 |
| impact | 5.354 | 7.006 | 100 |
| emergency_stop | 5.854 | 7.506 | 100 |

Strict gates: 4 ms impact latency, 0.5 s emergency-stop delay, target `vy=0.45 m/s` and yaw `0.18 rad/s`, velocity jump 0.806 m/s, map valid rate 1.000, maximum map age 0.020 s, zero obstacle contact, maximum roll/pitch 0.051/0.162 rad, WBC residual `1.6963e-5`, all status codes 0, and final emergency-stop hold complete.

The analyzer compares physical timing in `state_tick_s`; `cmd_time_s` is retained for controller-event ordering. This avoids falsely labelling the two clocks as a premature impact.
