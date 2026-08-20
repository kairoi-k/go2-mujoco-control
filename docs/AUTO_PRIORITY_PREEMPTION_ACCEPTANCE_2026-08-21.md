# Automatic priority preemption acceptance

This acceptance proves safety ordering under a real physical obstacle and a later physical push. It is deliberately different from a scripted token-transition test.

The run uses `scene_reactive_obstacle.xml`, `--auto-environment`, `--wbc-full`, and a 0.8 m/s simulator push at MuJoCo `state_tick_s=7.002`. The detector observes the height-map obstacle first, then the velocity jump:

| transition | controller clock | MuJoCo state clock | priority | source |
|---|---:|---:|---:|---|
| none | 0.000 | 1.204 | 0 | none |
| obstacle_left | 5.040 | 6.244 | 80 | sensor |
| impact | 5.800 | 7.004 | 100 | sensor |
| emergency_stop | 6.300 | 7.504 | 100 | scheduled |

Strict gates: 2 ms impact latency, 0.5 s emergency-stop delay, target `vy=0.45 m/s` and yaw `0.18 rad/s`, velocity jump 0.798 m/s, map valid rate 1.000, maximum map age 0.020 s, zero obstacle contact, maximum roll/pitch 0.078/0.162 rad, WBC residual `1.6964e-5`, all status codes 0, and final emergency-stop hold complete.

The analyzer compares physical timing in `state_tick_s`; `cmd_time_s` is retained for controller-event ordering. It also requires the obstacle and impact sources to be `sensor`, and the follow-up stop to be `scheduled`. This avoids falsely labelling the two clocks as a premature impact or a scripted event as autonomous sensing.
