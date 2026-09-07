# Immutable capture-heading joint shadow
Registered protocol before runtime. Prior f9623e1 yields six reduced proposals
but loses extra source coverage through current-heading raster registration.
The new joint path uses the same immutable envelope and production V2 identity
registration at CAPTURE pose. World queries transform into that capture frame;
state time still controls real observation age. Unknown source cells remain
unknown, and a failed view does not fall back to the legacy registered map.
The legacy controller continues its existing registered-map path, so this
experiment evaluates joint input representation with no new command authority.
Source319/320 is not promoted to320/320. No history, ground truth, fabricated
height or map/feasibility threshold change is introduced.
Registered next pair on one clean source: `joint_capture_flat_20260907_0001`
then `joint_capture_b1_20260907_0001`, 32 s, seed11/domain231, 0.14 s running
period, state clock, point model off, joint shadow on, map intervals V2 on.
Use phase2_flat.xml then b1_v3_running_step_5cm.xml (the historical B1 V3
scene, explicitly admitted by the harness). Capture at >=0.5 state seconds in STATE20--28 s; this is
an explicit window revision to cover steady running and the historical B1
impact near COMMAND23.216 s (approximately STATE25.216 s). The old18--24
state window and its results remain versioned. One snapshot is retained at
the first0.14s capture. Queries report original source and view coverage.
The 5 cm run is a diagnostic of candidate/continuous proposal coverage under
actual traversal states, not adoption, candidate acceptance or proof of body,
geometry/contact evolution/final actuator feasibility. B1 remains unaccepted.

63/63 CTests passed before runtime. Tests include the production32x10 float
grid, immutable unknowns/age, world height and rotated slope normals, and
comparison with coverage loss under a displaced current-body registration.
