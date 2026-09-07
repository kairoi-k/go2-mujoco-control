# Synthetic V2 registration coverage check (a180e60)
Scope: temporary /tmp fixture only. No repository files changed, no production CMake target built, and no MuJoCo simulation run.
Fixture: a TerrainMapEnvelope with the production published shape (width=32, height=10, resolution=0.05 m, origin=(-0.45,-0.225)), all 320 heights finite, all observation stamps equal to map_stamp=10.0, capture pose (0,0,0.5), yaw 0. RegisterTerrainMap(..., state_stamp=10.04, policy=kRegisteredIntervalsV2), then BuildRegisteredTerrainModel(..., source=kLidar). The fixture links the existing SDK and includes the production terrain headers.
Result counts (registered map and built model counts were identical):
case                         dx                 dy                 yaw              known
identity                     0                  0                  0                320
x only +1e-6                 +1e-6              0                  0                310
x only -1e-6                 -1e-6              0                  0                310
y only +1e-6                 0                  +1e-6              0                288
y only -1e-6                 0                  -1e-6              0                288
x+y +1e-6                   +1e-6              +1e-6              0                279
x+y -1e-6                   -1e-6              -1e-6              0                279
small yaw +1e-6              0                  0                  +1e-6            277
small yaw -1e-6              0                  0                  -1e-6            277
wall first registered delta  -1.239e-6         -4e-9              -4e-8            279
state first registered delta -2.366e-6         -15e-9             -146e-9           278
Unknown cells show the exact edge pattern: x-only removes one full column (10 cells), y-only removes one full row (32 cells), and x+y removes one column plus one row (41 cells), yielding 320-41=279. The production registration code conservatively requires every source cell touched by each destination-cell bounding box to be in-grid; any nonzero translation at this grid alignment can therefore clip an edge. Small yaw clips several edge fragments and yielded 277 here.
The real wall run's first valid registered row has capture (-0.091077916, 0.000005640), registration (-0.091079155, 0.000005636), yaw delta -4e-8; state has dx=-2.366e-6, dy=-15e-9, yaw delta=-146e-9. These are near-zero but not exact zero and reproduce 279/278 with a full-known source. Therefore the observed 279 ceiling is fully explainable by production registration alone. The previous report's direct-ray miss/filter claim is not established as the primary cause; direct ray misses remain possible but are unmeasured.
The run CSV field terrain_known_cells is post-registration model aggregate, not a direct 32x10 capture mask. It cannot distinguish registration edge clipping from capture unknown cells. Existing shadow initial_patch_unknown remains unresolved until the new per-query snapshot includes direct known mask, registered known mask, patch outside/inside counts, and query coordinates.
Reproduction command:
g++ -std=c++17 -O2 -I/home/che/dev/go2-workspace/feat-stage-c-joint-planner/example/cpp -I/home/che/dev/go2-workspace/feat-stage-c-joint-planner/example/cpp/terrain -I/opt/unitree_robotics/include -I/opt/unitree_robotics/include/ddscxx /tmp/wbc_registration_fixture.cpp -L/opt/unitree_robotics/lib -lunitree_sdk2 -o /tmp/wbc_registration_fixture
/tmp/wbc_registration_fixture
