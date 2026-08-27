# Phase 2 Stage B2 Handoff — 2026-08-27

Status: handoff only. No further canary, diagnostic instrumentation, or
controller change is authorized in this checkout after this document. This
handoff does not claim formal B1 or B2 acceptance.

Repository: `kairoi-k/go2-mujoco-control`

Worktree: `/home/che/dev/go2-mujoco-control-b1`

Branch: `research/phase2-b1-5cm-20260826`

Required implementation base:
`d77a16b9ed2c914de96899e599f254032148c873`

Phase 1 origin/main reference and merge-base:
`71d0e9ba7ca1097e840fe878aa30207f6f63600d`

Implementation HEAD before this handoff document:
`ca32942ed6f040041057051b9490c7279f8c8a01`

The implementation commits are already pushed to the branch. The handoff
document and evidence archives are the final repository changes to push.

## 1. B1 status and evidence

B1 5 cm full dynamic crossing is **not complete**. The robot did not reach a
verified upper-surface touchdown, measured support transfer, all-leg crossing,
body clear, or stable exit in any current-branch development run. There is no
formal B1 acceptance result and no claim is made here.

The useful capability already demonstrated is the sensor/planner side of the
chain:

* MuJoCo lidar publishes a sensor-derived height map through DDS. The latest
  map diagnostic in run 131 reports `source=lidar`, `frame=base_link`,
  `epoch=1`, `known=320`, `resolution=0.05`, and `relief=0.05`.
* The map diagnostic reports the observed local maximum at
  `max_cell_local=(0.825,-0.2)`, consistent with the sensed upper surface. The
  controller did not receive XML step coordinates or scene ground truth.
* `TerrainModel` receives and versions the map; `TerrainFeasibility` and
  `SafeFootholdRegion` produce sensor-derived candidate counts and reject
  reasons; some runs commit valid terrain plans.
* `TerrainMotionPlan` carries plan id, map epoch, timestamps, validity,
  contact schedule, future foothold predictions, body references, and velocity
  cap. Plan publication and consumption counters are nonzero in the useful
  runs.
* The future contact/foothold fields are wired into the terrain-aware SRBD-MPC
  and full ID-WBC path. Planned contact remains distinct from live measured
  contact for touchdown promotion and support verification.
* Swept foot/shin clearance, IK/reachability, support feasibility, atomic
  touchdown timing, and measured support handling are implemented and covered
  by the interface tests, but their combined dynamic crossing behavior is not
  proven.

The most recent run, `b1_dev_step5_support_reference_20260827_131`, is a
failure before touchdown. It used period `0.50 s`, duty `0.75`, step length
`0.15 m`, lift `0.08 m`, 0.3 m/s requested velocity, the 5 cm scene, and
terrain planning. It produced the correct lidar map and intermittent valid
plans/caps, but repeatedly hit support/deadline rejection, began target
overrides too late, lost support, and stopped on the hard posture limit. Its
metadata records the pre-final source head `58cb31238b85b715a81d9e8b8504d887477f5067`
with `git_dirty=true`; it must not be read as a run of `ca32942`.

The latest run and the preceding causal-failure set are archived and pushed
with this handoff:

* `docs/research/evidence/PHASE2_B1_DEV_RUN127_130_20260827.tar.gz`
  SHA-256:
  `486a75fdec1756351c133cd6c9196461bbf9d56b0ceb2d8dfc5be31e1a322652`
* `docs/research/evidence/PHASE2_B1_DEV_RUN131_20260827.tar.gz`
  SHA-256:
  `4477af14908b31760e40444512912b8847cfd5a50c600d224b84c1289573e7bc`

The corresponding raw directories remain under the ignored
`example/cpp/experiments/_runs/` directory for local inspection.

## 2. B2 actual capability

B2 10 cm is **not demonstrated**. The current code has the same sensor,
feasibility, atomic-plan, future-horizon, swing, SRBD-MPC, and ID-WBC
interfaces that are needed for B2, but there is no complete 10 cm dynamic
crossing, no repeat evidence, and no formal B2 result.

The latest preserved 10 cm exploration is
`b2_dev_step10_start_cell_20260827_113`. It is a pre-feature exploratory run
from the required base (`d77a16b...`), stopped with safety and completion
failure before a verified crossing. It is archived as:

`docs/research/evidence/PHASE2_B2_DEV_RUN113_20260827.tar.gz`

SHA-256:
`8bfcfdf9f6b393a21600aeeab75f53324932080e3fade2edbb43993ae328b79c`

B2 and B3 must therefore be treated as future development targets, not as
capabilities inherited from the existence of the interfaces or the old
exploration directories.

## 3. Recent effective commits

The relevant pushed implementation sequence is:

* `5c6d854` — terrain touchdown becomes a measured-support transaction.
* `8ed2e25` — terrain swing horizontal motion remains continuous.
* `689ba5b` — terrain swing contact transaction is retimed coherently.
* `50df7c1` — terrain edge replanning after a contact gap.
* `5bfe50f` — both future diagonal terrain swing pairs are considered.
* `b232589` — touchdown timing and schedule changes are atomic.
* `58cb312` — terrain height reference uses measured support rather than an
  airborne foot's map sample.
* `ca32942` — terrain-planner support contact fuses force evidence with the
  scheduled stance phase; WBC's separate early/late touchdown evidence is
  preserved.

No Phase 1 gain, hard safety threshold, or quantitative threshold was widened.
No main branch was modified, no old stash was applied, and no history was
rewritten.

## 4. Causal-failure runs worth retaining

These are development failures, not acceptance evidence:

* Run 127, `continuous_swing`: lidar map correct; continuous terrain path
  present; body stopped before the step and the WBC contact mask stayed on the
  old diagonal pair.
* Run 128, `retimed_contact`: atomic schedule work was exercised, but support,
  unknown, and deadline failures occurred before touchdown.
* Run 129, `front_replan_cap`: a zero velocity cap and a front candidate were
  observed, but only the immediately selected pair was handled; the next
  front leg encountered the riser without a promoted terrain touchdown.
* Run 130, `atomic_event`: the event transaction was coherent in the plan
  object, but support feasibility rejected the plan before touchdown.
* Run 131, `support_reference`: lidar geometry remained correct; valid plans
  and caps appeared, but target handoff was late and the run fell before
  touchdown. The raw force/hysteresis state showed swing-phase legs still
  looking loaded, motivating `ca32942`; that final patch has not had a new
  runtime canary.

## 5. Directions already falsified

The front step is not being confused with the ground by the lidar/TerrainModel
geometry in the current evidence. Run 131 reconstructs the 5 cm relief with
the expected sensor frame, resolution, epoch, and local location. Reworking
the map from scene coordinates is not the next action.

The old endpoint-only or centered/bell swing is insufficient: runs 122, 124,
and 127 either started too late, reverted at an endpoint check, or failed to
clear the leading edge. A late endpoint jump is not a valid terrain solution.

A plan for only one front/diagonal pair is insufficient: run 129 reached the
cap/replan path but left the following swing without a coherent terrain target.

Consumer-side touchdown extension and partial schedule changes are invalid:
run 128 exposed the new-foothold/old-contact mismatch; run 130 showed that an
atomic event alone does not create support feasibility.

Force hysteresis alone is not measured support: run 131 showed high force on
swing-phase legs. `ca32942` removes those legs from the planner's loaded-support
reference while keeping WBC touchdown confirmation independent. This is a
semantic correction, not runtime proof of crossing.

Repeated threshold relaxation, Phase 1 gain tuning, ground-truth input, fixed
scene coordinates, fixed leg order, and a scripted crawl are not valid paths
and were not adopted.

## 6. Unique current first causal blocker

The first causal blocker is a **fixed-timing terrain swing contract**, not
terrain detection: the latest experiment uses a `0.50 s` gait period and
`0.75` duty factor, leaving only `0.125 s` for swing. The sensor-derived
foothold path must move the foot to the upper surface, clear the leading edge
with the foot and shin, remain IK/reachability-valid, preserve support, and
arrive at the same planned contact event. The current path-duration bound and
support checks cannot satisfy those requirements in that fixed window.

The observed consequence is a sequence of support/deadline rejections or a
late target handoff before the first measured upper-surface touchdown. Delaying
or locally stretching one leg creates the forbidden mismatch between swing,
planned contact, MPC preview, and WBC support. This is an architecture-level
timing/geometry incompatibility under the current fixed running-trot schedule;
it is not evidence that the step was mapped as the robot's current ground.

## 7. Current architecture/dataflow

The implemented runtime path is:

```text
MuJoCo lidar rays
  -> DDS height map (world z relative to base, frame=base_link)
  -> TerrainModel (timestamp, epoch, resolution, known/stale/confidence)
  -> TerrainFeasibility / SafeFootholdRegion
       (surface, slope, roughness, edge, support, IK, reachability,
        joint margin, swept foot/shin clearance, confidence)
  -> TerrainPlanner
       (atomic plan_id/map_epoch/times/status/body refs/contact schedule/
        future footholds/velocity cap)
  -> plan store/publication
  -> terrain swing target adapter
  -> future contact and foot lever-arm horizon
       -> terrain-aware SRBD-MPC
       -> full ID-WBC torque output
```

The parallel measured path is deliberately separate:

```text
LowState force evidence + live foot pose + scheduled stance phase
  -> fused loaded-support reference for planning
  -> independent WBC touchdown/support confirmation
  -> support transfer promotion or abort/safe stop
```

Terrain speed influence is bounded to a velocity request/cap entering the
existing acceleration/jerk-limited Phase 1 shaper. Terrain does not directly
write nominal velocity, motion reference, gait setters, reactive events, or a
normal-action leg sequence. The terrain FSM remains observe/plan/commit/execute/
verify/abort/safe-stop.

Implemented and wired does not mean dynamically successful here. The missing
runtime result is the uninterrupted approach → sensor-derived terrain target →
swept-clear swing → touchdown → measured contact → support transfer → all legs
over → body clear → stable exit transaction.

## 8. Next three actions, in priority order

1. Replace the implicit fixed-window/consumer-retiming compromise with a
   first-class terrain contact-timing contract. Evaluate a hybrid contact
   planner or an explicitly negotiated terrain swing window that can preserve
   support while giving every terrain target enough time. Keep one atomic
   schedule consumed identically by gait, MPC, and WBC; do not lower safety
   margins or tune Phase 1 gains.

2. Re-design the terrain swing geometry around the measured support frame and
   the observed height profile: pre-edge clearance, foot plus lower-leg swept
   volume, reachable upper-surface touchdown, and continuous endpoint velocity.
   Prove the geometry against the negotiated timing contract before attempting
   a dynamic run; do not use scene-specific coordinates or a fixed leg order.

3. After actions 1 and 2 produce one real B1 crossing, run only the smallest
   capability sequence needed to advance: B1 repeat, then B2 10 cm, then B3
   mixed/repeated terrain. Keep raw evidence and failed runs, vary initial
   state/phase/terrain placement outside the controller, and defer formal B0
   recertification and B1/B2 acceptance to the stabilized capability chain
   and determinism workstream.

## 9. Final validation and reproduction

The final validation command was run once under the shared lock
`/tmp/go2_mujoco_experiment.lock`:

```text
cmake --build simulate/build -j2
cmake --build example/cpp/build -j2
ctest --test-dir example/cpp/build --output-on-failure
```

Both builds reported no work to do. CTest passed `27/27`; total test time was
1.51 s. This is a build/test result only and is not terrain acceptance.

The most useful local raw reproduction is the run-131 command recorded in its
archived `run_metadata.txt`. It uses `scripts/run_trot.sh`, the
`unitree_robots/go2/phase2_step_5cm.xml` scene, terrain lidar/planner, domain
230, and the exact CPU affinities recorded by the run. Any future runtime
experiment must first acquire the shared lock; this handoff itself ran no
additional canary.

Final source state: implementation `ca32942ed6f040041057051b9490c7279f8c8a01`
plus this handoff document and the three evidence archives. Formal B0
recertification remains required later after the capability chain and
determinism work are stabilized. Formal B1 acceptance: **no**.
