# Evidence bundles

These bundles are durable provenance, not route instructions. `CURRENT.md` and
`PHASE2_ACCEPTANCE.md` remain the authorities for the active line.

| Bundle | State | Use |
|---|---|---|
| `b0_runtime_integrity_20260904` | active diagnostic; not acceptance | F0-F14 decision log and current B0 runtime review |
| `order109b_c006i` | historical fixed-3 m/s lockstep slice | PASS at `5b95e826`; not current full B0 and not a B1 authorization |
| `phase2_terrain_sensor_velocity_20260828` | historical sensor-only campaign | PASS subset at `70b7740`; varying summary plus an indexed all-profile raw matrix, no terrain actuation or crossing |

Each bundle has a summary plus an identity manifest. Frozen manifests may retain
legacy paths or packaging fields from the run environment; those are provenance
and are not instructions to restore deleted files. Raw `_runs/` remain ignored
and immutable in the experiment workspace.
