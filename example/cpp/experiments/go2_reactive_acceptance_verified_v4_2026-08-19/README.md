# Reactive acceptance — verified v4

This package replaces the earlier v3 media. Every case was rerun with the
same controller binary, scene, CSV, metadata, and annotated video; the strict
analyzer and 1 fps contact-sheet review were both completed.

## Contents

- `media/acceptance_demo_verified_v4.mp4`: short collection demo.
- `media/demo_*_v4.mp4`: event-focused clips with a stable pre-window, active
  event, and recovery window.
- `media/*_verified_v4.mp4`: full audit clips with synchronized trace plots.
- `reports/verified_v4/acceptance_report.md`: quantitative gates and limits.
- Each case directory contains `data.csv`, logs, metadata, and contact-ground-
  truth analysis.

## What is actually tested

`emergency_stop`, `turn_left`, `turn_right`, and `obstacle_right` are scripted
reference events. They validate the continuous reference/transition layer, not
automatic perception. The obstacle case additionally uses the red/yellow
visible obstacle proxy in `unitree_robots/go2/scene_reactive_obstacle.xml`;
the trigger is still scripted, so this is not an obstacle detector test.

`impact` is an injected physical velocity push. `low_friction` changes the
simulator floor friction to `mu=0.05` for one second. `slip` is a scripted
protective slowdown. These disturbance cases demonstrate response/robustness,
not complete environment understanding.

The red band in each trace is the active event window. For low friction the
