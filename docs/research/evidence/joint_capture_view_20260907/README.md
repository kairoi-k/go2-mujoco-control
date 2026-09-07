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
impact at simulator/state23.216 s. The prior command-time attribution was
incorrect: direct CSV joins show state and command time differ. The old18--24
state window and its results remain versioned. One snapshot is retained at
the first0.14s capture. Queries report original source and view coverage.
The 5 cm run is a diagnostic of candidate/continuous proposal coverage under
actual traversal states, not adoption, candidate acceptance or proof of body,
geometry/contact evolution/final actuator feasibility. B1 remains unaccepted.

63/63 CTests passed before runtime. Tests include the production32x10 float
grid, immutable unknowns/age, world height and rotated slope normals, and
comparison with coverage loss under a displaced current-body registration.

## Actual results
Runtime `0ff9dc8fb755df231d26b4816d086fd77ed5d1b8` (ad2b833 source plus
curated replay-log retention), exact clean source/binary bindings verified.
Flat completes normally: 16 captures,9 reduced proposals,4 initial-contact
anchor unavailable,2 source-unknown initial patches,1 candidate coverage.
First snapshot preserves319/320 known exactly. Nine attempted-capture pipeline
p50/p95/max=1000.66/1140.8392/1142.96us. Command-window18--24 cycle diagnostic
36/42 good; this is not an acceptance or causal improvement claim.
The 5cm diagnostic FAILS physically: first hard posture limit reports pitch
22.0445deg, later reaching about57.8deg; no controlled stop, nonfoot collision,
no complete exit. V4 NOT_CERTIFIED. Stable approach gate passes but interaction
fails. Joint shadow remains command_authority=0; this is not joint execution.
13 captures including stopped-clock rejection:4 reduced proposals,4 source
unknown initial patches,5 clock rejections after failure. Attempt pipeline
p50/p95/max974.0645/1166.288/1199.339us. First snapshot preserves319/320.
No further run is warranted until first failure and execution semantics are
reviewed. Remaining contact/map gaps must not be hidden by promoting planned
contacts to measured contacts or by filling unknown cells.

Timebase erratum after direct raw joins: historical first contact23.216s is
contact_ground_truth.time_s = simulator state time, NOT command time. Earlier
packet commentary assigning it to command time was incorrect. STATE18--24
already covered first contact, but did not cover the full interaction. The
new20--28 window covers more of the interaction; its protocol rationale is
corrected here, without changing any raw time fields or analyzer thresholds.
Current first FL non-top23.512s -> first nonfoot23.706s -> first hard posture
state23.874s/cmd21.868s. Historical same-state and same-x comparisons show
opposite pitch evolution but are NOT a causal clock/view attribution; see
actual_b1/first_divergence.md. Terrain execution is absent in both routes.
