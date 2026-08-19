# Verified reactive acceptance media (2026-08-19)

The verified package is the `v3` set. Full clips are 1280x720, 20 fps, about 25 s; short demos keep the stable-before, active-event, and recovery windows. Each run is checked against synchronized CSV/metadata, with all seven status codes equal to zero.

- `acceptance_demo_verified_v3.mp4`: 88 s concatenation of the clean short demos.
- `demo_*_verified.mp4`: event-focused clips with synchronized response plots.
- `*_verified_v3.mp4`: full-length annotated audit clips.
- `reports/verified_v3/acceptance_report.md`: quantitative gates, CSV/JSON metrics, and interpretation limits.

The scripted turn/stop/obstacle clips validate continuous reference updates; they are not perception tests. `impact` is a physical velocity-push test. `low_friction` is a robustness test under changed friction, not an automatic terrain-perception claim.


The `event_scripts/` directory contains the deterministic reference-event inputs. The corresponding experiment directories retain CSV, simulator/controller logs, and metadata.
