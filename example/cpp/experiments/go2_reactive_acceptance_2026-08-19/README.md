# Verified reactive acceptance media (2026-08-19)

All complete demo clips are 1280x720, 20 fps, 20 s. Each run was checked against its controller metadata: controller, safety, quality, CSV, contact-ground-truth, dynamics, and completion statuses are all zero.

- `baseline_annotated.mp4`: no-event regression.
- `event_turn_left.mp4`: isolated turn-left reference event.
- `event_emergency_stop.mp4`: isolated emergency-stop reference event.
- `event_obstacle_right.mp4`: isolated right-obstacle reference event.
- `event_slip.mp4`: isolated slip reference event.
- `event_impact.mp4`: external 0.8 m/s velocity kick.
- `low_friction_annotated.mp4`: ground friction mu=1 -> 0.05 -> 1.
- `failure_boundary_annotated.mp4`: 1.5 m/s kick, expected safety-stop boundary.
- `acceptance_demo.mp4`: complete 140 s concatenation of the seven full clips.

The `event_scripts/` directory contains the deterministic reference-event inputs. The corresponding experiment directories retain CSV, simulator/controller logs, and metadata.
