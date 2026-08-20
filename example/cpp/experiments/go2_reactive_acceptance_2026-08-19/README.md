# Reactive acceptance experiment assets (2026-08-19)

The final verified package is `../go2_reactive_acceptance_verified_v4_2026-08-19/`. The older v3 media is retained as history only; do not cite it as the final result.

The v4 package fixes the previously false-positive emergency-stop gate and adds a visible obstacle scene. It contains synchronized CSV/metadata, full audit clips, short demos, trace plots, and a quantitative report.

The scripted turn/stop/obstacle clips validate continuous reference updates; they are not perception tests. `impact` is a physical velocity-push test. `low_friction` is a robustness test under changed friction, not an automatic terrain-perception claim.


The `event_scripts/` directory contains the deterministic reference-event inputs. The corresponding experiment directories retain CSV, simulator/controller logs, and metadata.
