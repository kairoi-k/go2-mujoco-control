# Capture-pose map registration V1

This is a new runtime observation path, not a mutation of frozen acceptance.
The old HeightMap topic and world-cache construction remain historical
observations. The new `rt/go2/lidar_heightmap_capture_v1` String topic carries
one bounded, versioned envelope: sequence, capture physics time, dimensions,
origin/resolution, base-link world xyz/yaw, per-cell heights and observation
times. No arrival-time/nearest-HighState pairing is used.

The new envelope uses only the CURRENT dense downward-ray hits. This is a
material observation-policy change in addition to frame registration: the old
world cache retains minimum historical height but refreshes time after a
higher observation, so that time cannot honestly be attached to the old value.
Missed current rays become unknown. The old cache is not silently relabeled.
No scene obstacle coordinate or collision truth enters controller planning.
Capture pose is simulator sensor/localization provenance, of the same kind as
the existing HighState pose; this is not hardware localization validation.

The source map is heading-aligned XY with world-parallel Z relative to capture
base height, despite the legacy `base_link` string. Registration transforms
each current-grid cell's four corners through world coordinates into the source
frame. Every bounding source cell must be known, fresh and agree in height
within 1e-6 m; otherwise the destination is unknown. It does not interpolate
across a riser. Curved/sloping/mixed cells can conservatively become unknown;
this path has not established general sloped-terrain support. The default
map/cell bound here is .20 s (old baseline unchanged). Finite future map/cell
observations and changed registration state stamps are rejected.

The resulting model retains capture pose, registration pose/time, sequence
and true cell ages. CSV exposes these fields. Missing pose/envelope is rejected
by the planner input path. Existing executable commitments may outlive a plan;
legacy kernel fallback remains UNCERTIFIED. Thus this is fail-closed map
registration, not a claim of fail-closed whole-robot terrain execution.

Remaining independent issues: planner FK/body-to-heading roll/pitch conversion,
future-body displacement, checked/executed swing curve and MPC committed-prefix
coverage. They are not repaired by this transport/registration change. A
separate default-OFF nominal-COM-reference ablation remains independently
selectable, so its effect must not be attributed to this map change.

Tests exercise wire shape/malformed numbers, unknown/age pairs, future/stale
observations, xyz/yaw frame transforms, discontinuity/unknown propagation,
registration stamp binding and extreme inputs. Release latency is measured for
320 cells over warmup plus repeated decode/register/model builds; raw build,
CTest and benchmark output lives in the ignored run directory named
`b1_map_registration_build_20260907_0001`. Runtime results are recorded only
after the clean source commit and rebuilt simulator/controller are used.
