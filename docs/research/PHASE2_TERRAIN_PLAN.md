# Phase 2 Terrain Plan

Status: architecture plan and audit record, 2026-08-25. This document is the
implementation handoff for the next Phase 2 agent. It is deliberately written
before a new terrain implementation, tuning pass, or simulation campaign.

This round changed no controller, planner, analyzer, physics, scene, or
experiment code. The only intended repository change in this round is this
document. All old Phase 2 worktrees, branches, stashes, logs, and evidence
remain preserved.

## 1. Current baseline

### Accepted Phase 1

The accepted baseline is origin/main at:

    71d0e9ba7ca1097e840fe878aa30207f6f63600d

It contains the Phase 1 runtime chain:

    v_cmd
      -> acceleration/jerk-limited command shaper
      -> continuous gait parameterization
      -> Raibert foothold logic
      -> SRBD-MPC
      -> ID-WBC

The accepted Phase 1 command and tracking coverage includes:

* 0 -> 1 -> 2 -> 3 -> 1 -> 0;
* 1 -> 3 and 3 -> 0;
* continuous ramps;
* 0.6, 1.4, 2.3, and 2.8 m/s profiles;
* fixed 3 m/s regression and its quantitative acceptance evidence.

Phase 1 is a frozen dependency for this plan. Phase 2 must not re-optimize
Phase 1 tracking, settling, or high-speed gait. Terrain may request a bounded
velocity change through the existing runtime v_cmd API; it must not silently
rewrite gait parameters or create a second velocity authority.

The accepted baseline evidence is recorded in the existing Phase 1 validation
documents and analyzer outputs, including:

* docs/validation/PHASE1_RUNTIME_VELOCITY_CLOSEOUT_2026-08-25.md;
* docs/validation/PHASE1_BENCHMARK_FREEZE_2026-08-24.md;
* example/cpp/scripts/analyze_phase1_velocity.py;
* the exact run manifests and data.csv files used by the Phase 1 closeout.

The 3 m/s running-trot evidence is target-specific. It is not evidence that
terrain, 1 m/s natural gait, or sim-to-real is accepted.

### Phase 2 audit heads

The audit was performed from WSL and fetched origin with prune and tags. The
important observed heads are:

* origin/main: 71d0e9ba7ca1097e840fe878aa30207f6f63600d;
* research/phase2-terrain-from-phase1-2fd5888:
  8bbb366905d65b73c65b84564b6233a5e31eb8c2;
* research/phase2-terrain-from-048eb1b:
  37f3c23;
* review/terrain-step-v1-wip-2026-08-24:
  efece291;
* its review base before the staged WIP:
  0068b14.

The plan is authored in the non-main worktree
/home/che/dev/go2-mujoco-control-phase2-from-phase1-2fd5888, on
research/phase2-terrain-from-phase1-2fd5888. No main worktree is modified.

## 2. Audit evidence and conclusions

### 2.1 Latest flat planner-enabled run

The last run preserved by the stopped agent is:

    example/cpp/experiments/_runs/phase2_A_flat_current_steps_fix1_20260825/

In the WSL checkout used by that run, the absolute evidence directory was:

    /home/che/dev/go2-mujoco-control-phase2-from-phase1-2fd5888/example/cpp/experiments/_runs/phase2_A_flat_current_steps_fix1_20260825/

The required evidence files were found:

* data.csv;
* run_manifest.json;
* phase1_quantitative_analysis.json;
* controller.log;
* simulator.log;
* run_metadata.txt.

The manifest records branch
research/phase2-terrain-from-phase1-2fd5888, HEAD
8bbb366905d65b73c65b84564b6233a5e31eb8c2, and git_dirty=true. The effective
run enabled terrain-planner, WBC-full, running-trot, the Phase 1 runtime
velocity profile, and domain 231. The manifest also records the analyzer,
controller, simulator, and scenario hashes. This is diagnostic evidence, not a
clean release artifact, because the source was dirty when the run was made.

The analyzer result is:

    acceptance_status = FAIL
    strict_pass = true
    quantitative_pass = false

All lifecycle/status fields were zero in the manifest: controller, safety,
quality, completion, dynamics, ground-truth, and analysis. Thus this was not a
process crash or an analyzer infrastructure failure. It completed while
failing the quantitative locomotion contract.

The failed quantitative gates in phase1_quantitative_analysis.json are:

* shaped_to_measured_p95;
* steady_state_error;
* tracking_p95;
* undershoot.

The measured facts are decisive:

* shaped command maximum: 3.0 m/s;
* measured speed maximum: about 0.559887 m/s;
* shaped-to-measured absolute P95: about 2.983720 m/s;
* steady-state maximum error: about 3.014755 m/s;
* tracking P95: about 2.983705 m/s;
* requested-profile error remained approximately zero;
* roll and pitch gates, torque saturation, SRBD validity, ID-WBC validity, and
  solver validity were not the quantitative failure.

The data.csv path was also checked independently. Throughout the active rows:

* terrain_contact_plan_active was zero;
* terrain_planner_safe_stop was zero;
* terrain_speed_limit_mps was infinite;
* terrain_pattern_blend was zero;
* event_active, event_type, and event_source were zero;
* the planner did not select a terrain crawl or an elevated contact plan;
* wbc_full_velocity_target_x_mps stayed zero while the shaped command stepped
  to 1, 2, and 3 m/s.

The controller log simultaneously reported:

    terrain_planner=on
    auto_environment=on
    sensor_map=lidar
    reactive_events=on
    runtime_velocity_command=on

The clean parent code explains the failure path. The terrain CLI enabled the
auto-environment path, which made motion_event_response_enabled_ true. In
trot_experiment_gait.cpp, the runtime command first sets
kernel_nominal_velocity_x_mps_ from the shaped/applied v_cmd and then the
motion-event path overwrites it with motion_reference_.vx_mps. In
trot_experiment_wbc.cpp, the full-WBC velocity task is also gated by the
motion-event response mode. The comparison flat run with the event path off
retained a non-zero WBC velocity target and did not show this same collapse.
The last stash contains a direct attempted fix that removes the implicit
reactive-event enable, makes runtime v_cmd the owner of the nominal speed, and
prevents the event layer from setting gait step/duty/lift during a runtime
command.

Answer to the five required questions:

1. The quantitative failure is the speed path: shaped-to-measured P95,
   steady-state error, tracking P95, and undershoot. It is not the roll/pitch,
   torque, SRBD, or ID-WBC validity gate.
2. This run does not show terrain geometry contaminating locomotion. The
   terrain contact plan was inactive and the speed limit was infinite. It shows
   an interface coupling: the terrain-planner flag activated a response path
   that changed velocity-reference and full-WBC task selection even with no
   active terrain event.
3. It is not caused by the swing-foot-below-ground elevated misclassification
   in this particular run: no elevated plan or terrain contact activation was
   recorded. That misclassification is a separately verified historical defect
   in the old architecture.
4. The flat regression is nevertheless a systematic architectural problem. A
   planner-enabled sensor mode was not observationally orthogonal to runtime
   velocity and reactive-event behavior. A flat scene can therefore fail before
   terrain geometry is used.
5. The new architecture must make sensing, planning, event response, velocity
   arbitration, gait scheduling, and safety separate interfaces. A plan must
   be an atomic, versioned object. Terrain can submit a v_cmd request or cap
   through the Phase 1 shaper, but it cannot enable a hidden event mode, write
   gait setters, overwrite the kernel velocity, or modify WBC task gates.

### 2.2 Historical foot-height false activation

The old planner compared a candidate foothold height with the instantaneous
kinematic foot height. During swing, that height is intentionally above the
surface; on flat ground, the comparison can therefore make a valid flat
candidate appear elevated or make a return-to-ground path look like an
excessive step down. The current old-WIP source contains a compensating
comment and uses a nominal surface reference for the patch check, while
stash@{0} additionally changes:

    elevated = terrain_obstacle_detected &&
               output.world_z_m > foot_world_z + 0.01

This is direct code evidence that the issue was real enough to receive a
targeted WIP fix. It is not evidence that the fix is accepted. The new
planner must compare a touchdown candidate with a sensed terrain surface and
state estimate, never with an arbitrary in-flight foot sample. The map epoch,
frame, sensor confidence, and swing phase must be logged with the decision.

### 2.3 Historical 10 cm dynamic failure

The old 10 cm evidence was checked rather than accepted from a prose summary.
Representative evidence is:

    /home/che/dev/go2-mujoco-control-terrain/
      go2-mujoco-control-terrain-step-v1/example/cpp/experiments/_runs/
      stepv1_10cm_1/

Its metadata identifies review base 0068b14, a crawl/terrain-act run, and
safety_status=1 with a hard posture stop. The controller log shows valid
front-pair z=.1 targets, then a staged transfer. Later health lines show roll
and pitch growth, including a hard posture limit around roll=13.47 degrees and
pitch=22.36 degrees. Other 10 cm attempts in the same review tree show the
same pattern: IK and patch selection can be valid while the dynamic support
exchange is not.

The failure is therefore not simply “the foothold was unreachable”. The
planner, body reference, contact schedule, support polygon, force
redistribution, and touchdown timing were not one jointly feasible plan.
Unilateral staged weight transfer was treated as a scripted action and was
allowed to proceed without a sufficiently robust predicted support margin.

### 2.4 Sensor and oracle boundary

simulate/src/unitree_sdk2_bridge.h contains two different map publishers:

* PublishLidarHeightMap casts rays, retains observed cells in a world cache,
  publishes a base_link-relative window, and uses NaN for unknown/stale cells.
  This is the reusable sensor adapter path.
* PublishEnvironmentHeightMap enumerates MuJoCo obstacle geometry and writes
  top heights into a map. It is a simulator oracle, not a controller sensor.
  It may be used by the experiment harness for scoring or a test fixture, but
  the controller must not consume its ground-truth geometry.

The old P1 observation analyzer and tests verify map/status sequences and
scene geometry. They do not establish dynamic terrain acceptance.

## 3. Old Phase 2 asset classification

The classification below is based on the actual source, tests, docs, branches,
stashes, and preserved run directories. “Reuse” means the concept or adapter
is worth carrying into the new interface; it does not mean copy the current
mutable fields unchanged.

| Area | Classification | Decision |
|---|---|---|
| HeightMap cell storage, known bit, local sampling, support-patch statistics in example/cpp/terrain/terrain_adaptation.h | REUSE | Refactor into TerrainModel with frame, epoch, age, provenance, normal, roughness, and uncertainty. Keep unknown explicit. |
| Lidar ray/map path in simulate/src/unitree_sdk2_bridge.h | REUSE | Keep as a sensor adapter and test source. Make the map source explicit; never expose the oracle geometry path as controller input. |
| SampleSupportPatch and conservative swept-clearance sampling | REUSE | Keep as feasibility primitives after separating touchdown surface from swing clearance and adding map confidence/age. |
| Go2 inverse kinematics and clamped IK path | REUSE | Use as a candidate feasibility filter and log the exact failure reason. |
| Existing swept-volume clearance checks | REUSE | Keep the geometric test, but run it on an atomic candidate trajectory with a terrain epoch and a real swing start/landing state. |
| Support polygon, contact-state hysteresis, support margin, contact loss, slip, torque, SRBD, ID-WBC diagnostics | REUSE | Preserve and extend logging; do not infer measured support from a planned foothold. |
| Phase 1 runtime velocity shaper and v_cmd profile | REUSE | This is the only terrain-to-speed control boundary. Terrain submits a request/cap to it. |
| Existing SRBD-MPC and ID-WBC execution | REUSE | Make the smallest horizon-input extension; preserve first-force/first-acceleration output. |
| Flat Raibert and preview foothold logic | REUSE | Keep as the no-terrain baseline and fallback. It is not a terrain planner. |
| data.csv, run_manifest.json, analyzer hashes, unit-test style, and experiment provenance | REUSE | Extend with plan id, map epoch, source, latency, deadline, and fallback fields. |
| PlanTerrainFoothold fixed dx/dy candidate lattice | BASELINE ONLY | Retain as a deterministic prototype/regression oracle. Do not expand it into the final planner without a body/contact horizon. |
| P1 lidar-observation route and analyze_terrain_observe.py | BASELINE ONLY | Keep for sensor regression; it is not a locomotion acceptance analyzer. |
| TerrainMotionReference and generic slew helper | BASELINE ONLY | The bounded-reference idea is useful, but fields must be replaced by plan outputs and the v_cmd API. |
| TerrainApproachFsm, crawl switching, terrain pattern blending | BASELINE ONLY | Keep only for historical comparison and a controlled fallback experiment. It must not own normal crossing behavior. |
| Scripted staged unilateral transfer, body height/pitch overlay, and terrain_contact_recovery | BASELINE ONLY | Keep as failure/regression baselines. Do not add more scene-specific stages. |
| Scheduled/measured contact merge from old WIP | BASELINE ONLY | The timing problem is real, but use a named planned-vs-measured contact interface with epochs and hysteresis, not an ad hoc union. |
| 5 cm/10 cm scene XMLs, old crawl/step scripts, and old terrain CSVs | BASELINE ONLY | They are development fixtures and regression references; acceptance must randomize scene parameters and use holdout sets. |
| StaircaseSpec, StaircaseHeightAt, FillStaircaseHeightMap, ClassifyStaircasePhase, and PlanStaircaseReference | REJECT | These use known world-x, fixed tread/riser coordinates, or a scene phase. They must not be in the controller path. |
| Known obstacle world-x, fixed step positions, controller access to scene geoms, and oracle-map activation | REJECT | Ground truth is permitted only in the harness scorer. |
| Implicit terrain-planner -> auto-environment/reactive-events activation | REJECT | Feature flags must be orthogonal. Sensor-only terrain mode must be a no-actuation flat regression. |
| Direct planner calls to SetGaitPeriod, SetGaitDuty, SetGaitStepLength, SetGaitFootLift, phase offsets, or motion_reference overwrite | REJECT | Planner outputs a plan and v_cmd request; an adapter owns gait scheduling. |
| Per-leg mutable target edits spread across refreshes and current-knot-only contact replacement | REJECT | Replace with one atomic short-horizon plan consumed consistently by swing, support, MPC, and WBC. |
| Phase freeze, “front pair then rear pair” scripts, and normal-crossing if/else action scripts | REJECT | FSM may manage lifecycle and safety only; it cannot be the terrain policy. |

## 4. Identified architectural failures

1. There are multiple velocity authorities. Runtime v_cmd, static kernel
   nominal speed, MotionEventResponse, and WBC gates can overwrite or disable
   one another. The latest flat FAIL demonstrates this without terrain
   activation.
2. Terrain enablement is not orthogonal. A sensor/planner CLI option changes
   reactive-event state and therefore changes the WBC/MPC reference path.
3. A per-leg foothold candidate is not a whole-body plan. IK validity does not
   imply support transfer, contact wrench feasibility, body/CoM feasibility,
   or touchdown timing feasibility.
4. The current planner mutates mutable fields at refresh time and then blends
   them into a gait kernel. There is no plan id, map epoch, valid-until time,
   atomic acceptance, or latest-valid snapshot.
5. Measured contact, scheduled contact, planned contact, and recovery contact
   are mixed. A late force sample can remove a real support foot; a planned
   foothold can be treated as if it already carries force; a single transient
   contact can change the WBC plant.
6. The current SRBD-MPC has a contact horizon but not a foothold horizon. A
   single foot_from_com_world vector is reused at every knot, and terrain only
   overrides contact[0]. Future elevated touchdown geometry is therefore not
   represented consistently in the force prediction.
7. Map semantics are under-specified. The same local map path can contain
   oracle geometry, lidar returns, stale cells, sparse interpolation, or a
   lifted-foot comparison. Unknown, age, frame, confidence, and provenance are
   not first-class inputs to planning.
8. Planner timing and failure handling are inline and implicit. A failed or
   late update can clear fields, request recovery, and change gait state in the
   same control tick. There is no deadline contract or bounded stale-plan grace.
9. The old FSM encodes a known scene and a prescribed leg sequence. This is
   vulnerable to scene memorization and cannot naturally grow into contact
   topology planning.
10. Acceptance evidence mixes kinematic unit tests, sensor observation, and
    dynamic crossing. A valid candidate or a 5 cm crawl repeat is not an
    accepted terrain locomotion contract.

## 5. Proposed target architecture

The near-term chain is:

    state estimation
      + local TerrainModel
      -> TerrainFeasibility
      -> online foothold/body/contact TerrainMotionPlan
      -> terrain-aware SRBD-MPC
      -> ID-WBC

The existing Phase 1 gait kernel remains the execution adapter in Stage B. It
owns phase continuity and consumes a bounded v_cmd request. The planner owns
terrain feasibility, selected footholds, body/CoM references, and a preview
contact/foothold plan. The WBC owns force/torque execution and measured-contact
safety. None of these layers may read obstacle ground truth.

The design has four separation rules:

1. A map update is observation; it is not an event, gait switch, or velocity
   command.
2. A plan is a versioned value object; consumers use one accepted snapshot,
   never a collection of independently updated mutable fields.
3. Planned contact schedule and measured contact state are distinct. The
   schedule predicts where force may be applied; measured state decides what
   is actually carrying force.
4. Terrain can request a v_cmd target/cap or a safe stop through the Phase 1
   command interface. It cannot call gait setters, alter a motion-event
   reference, change a WBC task gate, or select a gait topology in Stage B.

### 5.1 TerrainModel

The first implementation may remain a 2.5-D local elevation representation,
but the public object must not be just HeightMap. It should contain:

    TerrainModel {
        frame_id
        state_stamp
        map_stamp
        epoch
        origin and resolution
        cells: height, known, age, normal/slope, roughness, variance
        observed_source: lidar, estimator, test-fixture
        confidence/provenance per cell or region
        transform quality and map bounds
        stale/unknown policy
    }

Required behavior:

* unknown and stale are different from height zero;
* all positions have an explicit frame and timestamp;
* a candidate records the map epoch used to evaluate it;
* the controller rejects oracle/test-fixture provenance in production mode;
* patch estimates expose height range, plane/normal, slope, roughness, and
  confidence rather than only mean height;
* the model can later be backed by a learned terrain belief without changing
  the planner contract.

The simulator's lidar publisher may continue to publish NaN for unknown cells.
The controller adapter must preserve that information. It must not silently
fill unknown terrain with the MuJoCo oracle map.

### 5.2 TerrainFeasibility and safe foothold regions

TerrainFeasibility should express a safe region, not only one selected point.
For each leg and planning horizon, expose a set of small convex or
convexified regions in the local terrain frame:

    SafeFootholdRegion {
        leg
        map_epoch
        xy convex polygons or inward-safe cells
        height interval and local support plane/normal
        slope and roughness bounds
        edge margin
        reachability/IK result and margin
        swing swept-volume clearance margin
        collision margin
        confidence and valid_until
        hard_reject reasons and soft costs
    }

The region is safe only if all hard conditions hold:

* the foot patch is known and sufficiently fresh;
* height range, slope, and roughness are within the configured contract;
* the point is inside an edge-inset support region;
* IK and joint/velocity limits are valid;
* the swing swept volume has clearance from observed terrain;
* the candidate does not create a body or support-polygon violation;
* its uncertainty is below the stage's allowed limit.

Reachability and terrain support are separate reasons. A reachable point on a
riser edge is not a safe foothold. A safe patch outside the leg workspace is
not a candidate.

### 5.3 TerrainMotionPlan

The plan is an immutable snapshot exchanged between planner and consumers. A
concrete initial C++-style contract is:

    TerrainMotionPlan {
        plan_id
        map_epoch
        state_stamp
        generated_at
        valid_until
        frame_id
        status: valid, degraded, stale, rejected, safe_stop
        uncertainty summary
        solver status, iterations, elapsed_us, deadline_miss

        body/com reference samples[k]:
            pose, velocity, acceleration, height, pitch/yaw bounds

        planned_contact_schedule[k][leg]:
            planned stance/contact mask and transition time

        predicted_foothold[k][leg]:
            valid, touchdown time/phase, world position, surface normal,
            source region id, edge/clearance/reachability margins

        current_support_anchor[leg]:
            measured/confirmed status, position, confidence

        velocity_request:
            target or cap, acceleration/deceleration/jerk limits,
            reason, priority, expiry

        fallback policy and diagnostic reasons
    }

The exact array container can follow repository conventions. The semantic
requirements are more important than the spelling: one plan id, one map epoch,
one validity window, and time-indexed footholds/contact schedule.

### 5.4 Planned versus measured contact

Use two explicit types:

    PlannedContactSchedule: phase/time prediction from the plan;
    MeasuredContactState: force/kinematic/contact-filter evidence.

The SRBD force model consumes a time-indexed planned schedule. ID-WBC and
safety consume measured contact plus a documented, hysteretic fusion policy.
Fusion may preserve a scheduled stance foot for a short sensor-latency grace or
admit an early measured touchdown, but every promotion/demotion must be logged.
The fusion policy must never claim that a planned foothold has already produced
force.

## 6. Planner formulation

### 6.1 Alternatives

| Formulation | Benefit | Cost/risk | Decision |
|---|---|---|---|
| Fixed discrete candidate lattice per leg | Deterministic, easy to budget, works with non-convex IK/terrain tests | Can be sparse and greedy; no body/contact coupling by itself | Keep as candidate generator and fallback |
| One convex QP over footholds and CoM | Fast and transparent for convex regions | Region construction, contact changes, IK, and clearance are not naturally convex | Use only after hard filtering, for local projection |
| Full short-horizon nonlinear or mixed-integer optimization | Can jointly choose timing, contact, and body | Too fragile and expensive for current SRBD/ID-WBC stage | Defer to Stage C |
| Candidate search plus small convex body/CoM projection | Preserves hard geometric tests and adds whole-body coordination | Needs a good support/stability model and deterministic tie-breaking | Recommended Stage B |

### 6.2 Recommended Stage B planner

Use a short receding horizon covering the next one to three touchdown
opportunities per leg, or the equivalent bounded time window established by
measurement. The horizon length is a design parameter to freeze at B0; it must
be long enough to contain the next support exchange but not so long that
uncertain terrain is treated as known.

At each update:

1. Read one state snapshot and one TerrainModel snapshot.
2. Generate nominal Raibert candidates and candidates from each safe region.
3. Reject unknown, stale, edge, slope, roughness, step, IK, collision, and
   swept-clearance violations.
4. Form a small set of joint candidate combinations for the imminent support
   exchange.
5. For each combination, project or solve a small convex body/CoM adjustment
   subject to support-polygon and body-rate bounds.
6. Score forward progress, nominal-foot displacement, body/CoM error, support
   margin, edge margin, clearance, slope, uncertainty, touchdown timing, and
   v_cmd deviation.
7. Accept one complete plan only if all hard constraints and the solver
   deadline hold. Publish it atomically with plan_id and map_epoch.

The short-horizon optimizer must not use obstacle coordinates, scene labels,
or a known step index. It receives only state estimation and TerrainModel.
The experiment harness may separately know the ground truth for scoring.

At the first implementation, touchdown timing and gait topology remain inputs
from the accepted Phase 1 schedule. A candidate may be restricted to the
current swing window. Do not make a new gait switch just to make the optimizer
appear more capable. Timing offsets become explicit candidate variables only
after B0/B1 establish that the fixed topology path is clean.

### 6.3 Body and CoM coordination

Body/CoM is planned together with footholds, not as an independent average of
the currently measured feet. For each candidate combination:

* form the predicted support polygon from confirmed current contacts and
  candidate future contacts;
* shrink it by the required stability margin and uncertainty margin;
* keep the projected CoM inside that region over the support exchange;
* bound base height, roll, pitch, angular rate, and body displacement;
* reject a unilateral transfer if the predicted support/force margin is not
  positive for the entire transition;
* prefer a velocity cap or hold over a body shift that is only kinematically
  possible.

The old staged transfer failure is exactly why “two feet have valid IK” is not
an adequate body reference. A support polygon and force-feasibility check must
be part of candidate selection.

### 6.4 Touchdown footholds in the MPC horizon

The plan must represent:

* current confirmed stance anchors;
* the planned touchdown for each swing leg;
* the predicted position after touchdown;
* the contact schedule at every MPC knot.

Before a leg touches down, its force-model position is the current confirmed
anchor only when the contact schedule says it is in stance; otherwise its force
is constrained to zero. At the predicted touchdown knot, the new foothold
becomes the force application point for subsequent knots. A measured touchdown
may update the anchor only through the contact-state filter and a new plan
epoch; it must not mutate one field halfway through a solve.

## 7. Terrain-aware SRBD-MPC integration

### 7.1 Audited current structure

The current solver in example/cpp/wbc/srbd_mpc.h has:

* maximum horizon 12 and a caller-selected horizon of 8;
* caller dt_s = clamp(gait_period / 8, 0.020, 0.05);
* state x = roll, pitch, yaw, CoM position, angular velocity, linear
  velocity;
* contact[k][leg] over the horizon;
* one input.foot_from_com_world[leg] vector, reused by SrbdBd at every knot;
* condensed force decision variables with friction-pyramid, normal-force, and
  swing-foot zero constraints;
* first_force, first_linear_acc, first_angular_acc, and predicted_state output.

The caller in trot_experiment_wbc.cpp fills the nominal trot contact schedule
over the horizon, then may replace contact[0] for terrain/recovery behavior.
The terrain foothold field terrain_mpc_foot_world_[leg] is converted to one
foot_from_com_world vector; it is not a time-indexed future foothold sequence.
The first linear and angular accelerations are passed into ID-WBC, which
produces the torque command.

This structure is sufficient for a minimal Stage B interface upgrade. It is
not sufficient to claim terrain-aware future contact dynamics while the same
foot vector is used at every knot.

### 7.2 Minimal, clean upgrade

Do not rewrite SRBD-MPC or jump to whole-body NMPC. Add an optional,
time-indexed input contract:

    mpc_in.contact[k][leg]
    mpc_in.foot_from_com_world_horizon[k][leg]
    mpc_in.foot_valid[k][leg]
    mpc_in.plan_id / plan_epoch

The implementation may initially keep the old single-foot field as a
backward-compatible fallback. The solver must select either the old flat
input or the complete new horizon input; it must never mix a new contact
schedule with an old foot anchor.

A later small extension may add reference[k] for body/CoM and velocity
trajectory samples. For the first terrain milestone, a constant/linear
reference generated from the accepted TerrainMotionPlan is enough. Keep the
existing first-force, first-linear-acceleration, and first-angular-acceleration
outputs so ID-WBC remains unchanged.

The minimal integration order is:

1. Define and log time-indexed contact/foothold input and plan epoch.
2. Preserve the current flat schedule and confirm numerical equivalence in
   B0 with the new fallback path.
3. Add future foothold positions to the condensed SRBD input at the knot where
   contact begins.
4. Add body/CoM reference samples only as needed by B1/B2 evidence.
5. Keep measured-contact fusion and safety outside the optimizer; the solver
   reports invalid/late/failed, and the safety adapter decides fallback.

## 8. Runtime rates, solver budgets, and latest-valid-plan

The existing controller path is a high-rate loop (the repository records the
LowCmd/LowState interface as 500 Hz). The initial Stage B scheduling contract
should be:

| Component | Initial rate/trigger | Initial budget contract |
|---|---:|---|
| State/contact estimation | existing controller rate | must not block on planner |
| Terrain map update | 20-50 Hz, on new sensor/map epoch | timestamp and age every map |
| Terrain planner | nominal 20 Hz, plus map/phase trigger with coalescing | soft p95 <=2 ms, hard deadline <=5 ms; measure and freeze at B0 |
| SRBD-MPC | existing caller cadence; terrain path must log actual tick period | keep current solver budget first; no unmeasured horizon expansion |
| ID-WBC | existing controller rate | unchanged execution deadline |

The numbers above are initial engineering budgets, not achieved evidence.
The B0 contract must record actual wall-clock distributions, CPU affinity,
solver iterations, missed deadlines, and queue age. If the host cannot meet
the hard budget, the planner must degrade or safe-stop; it must not block the
500 Hz control loop.

### Latest-valid-plan protocol

1. Planner reads a coherent state/map pair and computes a private plan.
2. It validates finite values, map epoch, state age, support margin, all hard
   candidate constraints, solver status, and deadline.
3. It publishes the entire plan by atomic snapshot replacement.
4. Consumers keep the last accepted snapshot only while now < valid_until and
   the state/map epoch conditions remain within the declared grace.
5. A stale plan may finish the current bounded swing or at most the declared
   one-touchdown grace. The exact grace is frozen at B0; it must not be
   unlimited.
6. After grace expiry, the velocity request is driven toward zero through the
   Phase 1 shaper and the gait remains in a support-safe mode. No new swing
   target is invented.
7. A plan_id and map_epoch are written on every planner, swing, MPC, WBC, and
   fallback diagnostic row.

There must be no partial update such as a new foot target with an old body
reference or a new contact mask with an old foothold. A failed planner update
does not clear the currently valid plan before the fallback decision is made.

## 9. Safety and fallback behavior

| Condition | Required response |
|---|---|
| Map absent, stale, unknown at required touchdown, or frame invalid | Do not commit a new terrain touchdown. Hold the last valid plan only within grace; submit a v_cmd cap/zero request through the Phase 1 shaper. |
| Candidate has no safe region, bad edge/slope/roughness/clearance, or IK failure | Reject that candidate. Keep current confirmed support; do not script a replacement leg action. |
| Body/CoM or support margin infeasible | Reject the combination. Slow/hold through v_cmd if a safe plan exists; otherwise safe-stop. |
| Planner solver failure or deadline miss | Keep latest-valid plan within grace, then bounded brake and support hold. Record the failure reason and latency. |
| Planned versus measured contact disagreement | Use the documented hysteretic fusion policy. Do not claim a planned touchdown is loaded; do not remove the last robust support pair on one noisy sample. |
| SRBD-MPC invalid or stale | Keep the last valid force reference only for the existing bounded WBC grace, then use the accepted safe stop/fallback. |
| ID-WBC invalid, torque saturation, collision, slip, or posture guard | Invoke the existing safety path and terminate/hold according to the safety contract. Terrain must not bypass it. |
| Uncertainty grows beyond the stage budget | Increase v_cmd braking demand or stop. Uncertainty is a first-class safety input, not a soft logging field. |

### v_cmd ownership

The planner-facing API should be conceptually:

    VelocityCommandRequest {
        target_or_max_vx
        max_accel, max_decel, max_jerk
        priority/reason
        valid_until
        plan_id
    }

An arbitration layer combines the user command, terrain request, and safety
request before the existing Phase 1 shaper. Terrain may request “slow to this
cap” or “target zero by this deadline”. The shaper remains the sole source of
the shaped/applied runtime velocity. The planner must not:

* call SetGaitPeriod, SetGaitDuty, SetGaitStepLength, SetGaitFootLift, or phase
  offset setters;
* write kernel_nominal_velocity_x_mps_;
* write motion_reference_;
* enable reactive_events or emergency-event processing;
* directly switch running-trot, crawl, bound, or any other topology in Stage B.

The resulting effective v_cmd, cap reason, plan_id, and arbitration decision
must be logged. A terrain cap is not allowed to silently alter the acceptance
target without appearing in the manifest and analyzer input.

### FSM boundary

An FSM is allowed for plan lifecycle and safety:

    OBSERVE -> PLAN -> COMMIT -> EXECUTE -> VERIFY -> ABORT/SAFE_STOP

It may enforce map/plan validity, contact confirmation, timeout, and recovery.
It must not encode normal terrain crossing as a fixed sequence of world-x
phases, front-pair/rear-pair scripts, or a collection of scene-specific
if/else actions. Normal crossing behavior comes from feasibility and the
short-horizon plan. The FSM is a supervisor, not the terrain policy.

## 10. Milestones

### B0: flat planner-enabled regression

* Run the accepted Phase 1 profile with terrain sensing and planner plumbing
  enabled in sensor-only/no-actuation mode.
* The planner may build TerrainModel and Feasibility telemetry, but it must
  not change footholds, body references, gait topology, event response,
  contact plan, or WBC task gates.
* Compare the exact Phase 1 quantitative contract, not a newly chosen terrain
  threshold.
* Verify that terrain flag, sensor source, event response, runtime v_cmd,
  kernel target, WBC target, MPC input, and ID-WBC output remain separate.
* Exit only when flat planner-enabled regression passes all Phase 1 gates and
  the plan/fallback instrumentation is complete.

### B1: 5 cm single step

* Use sensor-derived terrain only in the controller.
* Commit a safe-region/foothold/body plan while preserving the Phase 1 gait
  topology and phase continuity.
* Test approach, first touchdown, support exchange, and exit separately in
  logs.
* A valid 5 cm run requires dynamic support and all geometric/runtime gates;
  “three kinematic candidates passed” is not enough.

### B2: 10 cm single step

* Use the same interfaces as B1 with no scene-specific code.
* Demonstrate predicted support margin through the transfer and correct
  future footholds in the SRBD horizon.
* Any unilateral transfer must be justified by the joint plan and support
  margin; otherwise the planner must slow, hold, or stop.

### B3: mixed/repeated steps

* Mix step heights, approach distances, lateral offsets, repeated rises/falls,
  and unknown/stale patches.
* Require plan replacement and fallback across multiple map epochs.
* Include randomized initial x, gait phase, obstacle position, and seed.
* No change to the controller may identify the scene or step index.

No B0-B3 experiment is part of this audit round. They are future milestones.

## 11. Acceptance contracts

### 11.1 Metric groups

Every run must produce a manifest, data.csv, planner diagnostics, analyzer
output, and a machine-readable status for:

| Group | Required measures |
|---|---|
| Completion and lifecycle | completion, controller/safety/quality/dynamics status, intended stop, safe-stop events |
| State | roll/pitch peak and P95, base height, velocity tracking, settling and stop tail |
| Foothold geometry | candidate reachability/IK, safe-region membership, edge margin, slope, roughness, swing swept clearance, collision, touchdown x/y/z error |
| Support and contact | predicted/confirmed support margin, support polygon area, contact loss, single-support exposure, slip, planned/measured disagreement |
| Actuation and models | torque limit/saturation, SRBD validity, ID-WBC validity, force feasibility, solver iterations/cost |
| Planner runtime | planner success, accepted/rejected plan count, plan age, solver latency, deadline misses, stale-plan grace, safe-stop reason |

Ground-truth contact, geom, collision, and terrain values may be consumed by
the experiment harness and scorer only. They must not be readable by the
controller or planner process. Both source provenance and this separation are
recorded in the manifest.

### 11.2 Proposed pre-freeze gates

The following are proposed starting values for a contract, not results. They
must be frozen before any B0/B1 tuning and may not be moved toward an observed
result. The exact Phase 1 B0 thresholds remain inherited from the accepted
Phase 1 analyzer; current diagnostic output includes tracking P95 <=0.40 m/s,
steady-state error <=0.40 m/s, overshoot <=0.50 m/s, undershoot >=-0.25 m/s,
and settling <=8.2 s.

| Gate | B0 | B1 proposal | B2/B3 proposal |
|---|---|---|---|
| Completion/status | exact Phase 1 contract; zero unplanned status failure | 100% completion; zero unplanned safe stop | same across every holdout |
| Roll/pitch | exact Phase 1 contract | P95 <=12 deg, max <=18 deg | P95 <=15 deg, max <=20 deg |
| Base height | exact Phase 1 contract | P95 error <=0.04 m from plan/contract | same, with terrain plane documented |
| Velocity | exact Phase 1 contract against shaped v_cmd | P95 <=0.40 m/s against effective approved v_cmd | same; all caps visible in manifest |
| Committed footholds | no terrain actuation; 100% flat baseline | 100% hard-feasible at commit | 100% hard-feasible at commit |
| Edge margin | telemetry only in sensor-only mode | >=0.02 m or the frozen foot-specific margin | same |
| Slope/roughness | telemetry only | within frozen feasibility bounds | same, with holdout variation |
| Swing clearance/collision | no new actuation; baseline collision gate | clearance >=0.02 m and zero collision | same |
| Touchdown | baseline Phase 1 touch metrics | x/y <=0.05 m, z <=0.03 m unless a tighter sensor contract is frozen | same |
| Support/stability | baseline Phase 1 contact gates | minimum predicted/confirmed margin >0.02 m; no unplanned support loss | same over every transfer |
| Slip/contact loss | exact Phase 1 baseline limit | no unplanned slip/contact-loss gate failure | same |
| Torque/model validity | exact Phase 1 limit; SRBD/ID-WBC valid | no new saturation gate failure; SRBD/ID-WBC valid on all active rows | same |
| Planner success/deadline | planner telemetry only | 100% committed-plan success; zero hard deadline misses after warm-up | same |
| Safe stop | zero in a successful B0 run; injected-failure test must stop | zero in successful holdout; failure-injection test must stop | same |

The purpose of the B1/B2 numbers is to give the next agent a concrete
pre-registration starting point. They are not claims that old WIP met them.
Before terrain tuning, create a versioned acceptance contract containing
these values, the source analyzers, seed list, scene generator version, and
the exact definition of each metric. Hash that contract into every
run_manifest.json. If an inherited Phase 1 gate is inconsistent with a clean
baseline, stop and revise the contract once, with a written rationale, before
terrain tuning or holdout evaluation. Never change a threshold after seeing
the corresponding holdout result.

## 12. Development versus holdout protocol

### Development/tuning set

The development set is used for debugging and tuning only:

* flat planner-sensor mode;
* 5 cm and 10 cm single-step fixtures;
* multiple initial x positions and gait phases;
* a bounded range of obstacle positions and step widths/heights;
* deterministic seeds plus sensor-noise/replay cases;
* injected planner timeout, stale map, unknown patch, and contact-lag cases.

Development results may guide code changes, but no threshold or holdout case
may be selected after looking at a holdout result.

### Holdout acceptance set

Before tuning, freeze a separate manifest containing:

* initial x and y offsets not used as a single fixed start;
* gait phase offsets not used in development;
* obstacle/step positions sampled from the allowed range;
* step height, width, slope, roughness, and repeated-sequence combinations;
* seeds not used for tuning;
* the exact scene generator and analyzer hashes.

Holdout scenes may be generated at run time, but the controller receives only
state estimation and sensor-derived TerrainModel. The experiment harness alone
may read MuJoCo ground truth to score collision, true contact, and true
terrain. A clean checkout and a frozen config/contract are required for
holdout. Holdout thresholds are evaluated exactly as frozen.

## 13. Anti-hack and anti-scene-memorization rules

The controller/planner must not:

* read scene XML, geom names, geom positions, step index, or obstacle
  ground-truth arrays;
* call a staircase generator or fixed world-x classifier;
* select behavior from a scene filename or known obstacle coordinate;
* encode a fixed “front pair then rear pair” action sequence as the crossing
  policy;
* infer elevated terrain solely from an in-flight foot height;
* use the oracle map in a sensor-mode run;
* silently change the acceptance target when v_cmd is capped.

The harness may read all ground truth for scoring, randomized scene creation,
and post-run diagnosis. The manifest must identify which process produced each
field.

## 14. Stage C hybrid contact-planner roadmap

Stage B keeps the accepted Phase 1 topology and schedule as the execution
baseline while making terrain footholds/body/preview contacts coherent.

Stage C introduces a hybrid contact planner. Its candidate/optimization
variables become, progressively:

* gait topology;
* contact sequence;
* period;
* duty factor;
* phase offset;
* touchdown timing;
* foothold position and surface orientation;
* body/CoM reference.

Running-trot, bound, and gallop may be topology candidates. They must not be
implemented long-term as:

    if speed > X: choose bound

Instead, topology feasibility, predicted support, flight/aerial fraction,
body dynamics, terrain uncertainty, and transition cost must select among
candidates. The planner must preserve phase/contact continuity during a
topology transition and expose the chosen topology in the plan and manifest.

The Stage C interface should be compatible with a hybrid contact sequence
without forcing Stage B to implement a hybrid solver.

## 15. Conditions for whole-body NMPC and later learning

Do not move to whole-body NMPC merely because terrain looks difficult. First
show with controlled evidence that:

* TerrainModel and safe regions are correct and sensor-only flat regression
  remains accepted;
* the Stage B planner produces dynamically feasible body/foothold/contact
  plans;
* the SRBD horizon with time-indexed footholds is the measured bottleneck;
* failures remain after contact schedule, support margin, touchdown timing,
  and ID-WBC execution have been instrumented and validated;
* the solver budget and failure fallback are known.

Only then consider Stage D whole-body NMPC. After that, and only with separate
evidence, consider:

* learned terrain belief;
* optimizer warm starts;
* learned prior/value functions;
* residual dynamics;
* recovery policy.

Learning must augment a known-safe model and fallback, not replace the
interface contracts or become a hidden terrain-specific policy.

## 16. Risks and unresolved questions

These are intentionally left as questions for implementation/evidence, not
silently fixed by this plan:

1. The actual end-to-end controller and WBC scheduling period must be measured
   from the current kDt and manifest; comments about 20/50/100 Hz are not
   acceptance evidence.
2. The DDS HeightMap frame/origin/age semantics and map rotation under body yaw
   need a dedicated contract test.
3. The current simulator publishes both lidar and oracle maps; production
   wiring must make the selected source unambiguous.
4. Force sensor latency, threshold hysteresis, early touchdown, and contact
   loss timing may require a better measured-contact filter before terrain
   contact scheduling is trusted.
5. A time-indexed foothold may expose SRBD linearization or friction-model
   limits at a step. This must be measured before changing solver class.
6. Body/CoM references from two or three contacts need an explicit friction and
   support-margin model; averaging measured feet is not sufficient.
7. Planner p95/hard deadlines and allocation behavior under WSL are unknown.
8. The latest flat run is dirty-source evidence. A clean reproduction of the
   isolated interface bug is a future diagnostic task, not performed here.
9. No 5 cm or 10 cm dynamic terrain acceptance exists on the accepted main.
   Old WIP passes are prototypes or baseline-only evidence.
10. The final foot patch model, terrain roughness definition, and threshold
    values for different foot materials remain to be frozen before holdout.

## 17. Concrete implementation order

The next agent should implement in this order, stopping at each gate:

1. Freeze the Phase 2 interface and acceptance contract. Add plan/map/source/
   epoch/latency fields to the manifest and CSV without changing behavior.
2. Split terrain sensor-only enablement from auto environment, reactive events,
   and runtime velocity. Make B0 flat planner-enabled mode actuation-free.
3. Add TerrainModel as an adapter around the existing lidar/HeightMap path.
   Preserve unknown, age, frame, source, and epoch.
4. Add TerrainFeasibility regions and unit tests for flat, unknown, stale,
   edge, slope, roughness, IK, clearance, and uncertainty.
5. Add deterministic candidate generation plus body/CoM support projection.
   Keep PlanTerrainFoothold only as a baseline oracle during comparison.
6. Add TerrainMotionPlan with atomic latest-valid-plan publication and
   explicit fallback.
7. Add the velocity request/cap adapter into the Phase 1 v_cmd shaper.
   Remove all direct terrain writes to gait setters and motion references.
8. Add the minimal SRBD-MPC horizon foothold input, preserving the old flat
   fallback and first-acceleration/ID-WBC connection.
9. Add planned/measured contact fusion with epochs, hysteresis, and logs.
10. Run B0 and freeze its evidence before any terrain tuning.
11. Implement B1, then B2, with development-only tuning and pre-frozen
    holdouts.
12. Implement B3 mixed/repeated randomized scenes and failure injection.
13. Only after B0-B3 evidence, decide whether Stage C topology planning is
    warranted.

This list is an implementation order, not an instruction to execute it in
this audit round.

## 18. Exact files likely to change in a future implementation

Expected new or changed files, subject to repository conventions:

### New terrain interfaces

* example/cpp/terrain/terrain_model.h;
* example/cpp/terrain/terrain_feasibility.h;
* example/cpp/terrain/terrain_motion_plan.h;
* example/cpp/terrain/terrain_planner.h;
* optionally example/cpp/terrain/terrain_contact_state.h.

### Controller integration

* example/cpp/trot/trot_experiment.h;
* example/cpp/trot/trot_experiment_control.cpp;
* example/cpp/trot/trot_experiment_gait.cpp;
* example/cpp/trot/trot_experiment_wbc.cpp;
* example/cpp/trot/trot_experiment_diagnostics.cpp;
* example/cpp/trot/trot_cli.cpp, only for orthogonal flags and the v_cmd
  request interface;
* example/cpp/gait/locomotion_kernel.h, only for an adapter contract if
  required;
* example/cpp/gait/raibert_trot_kernel.h, only to consume a plan-approved
  touchdown without adding terrain policy;
* example/cpp/wbc/srbd_mpc.h;
* example/cpp/wbc/inverse_dynamics_wbc.h only if the existing input contract
  needs a named plan/contact epoch, not for a wholesale WBC rewrite.

### Sensor, tests, analyzers, and harness

* simulate/src/unitree_sdk2_bridge.h for explicit sensor-source/frame tests;
* example/cpp/tests/test_terrain_adaptation.cpp and new focused interface
  tests;
* example/cpp/tools/analyze_terrain_observe.py or a new Phase 2 analyzer;
* a future acceptance-contract/manifest helper;
* future scene generators and experiment harnesses, which may read ground
  truth only for scoring.

The exact old files must not be extended blindly. In particular,
terrain_adaptation.h's StaircaseSpec path and current terrain mutable fields
are migration references, not the target interface.

## 19. Definition of DONE for Phase 2

Phase 2 is done only when all of the following are true:

1. B0 flat planner-enabled mode passes the exact accepted Phase 1 quantitative
   contract, with terrain sensing/planning telemetry on and no hidden
   velocity/event/WBC coupling.
2. B1 5 cm, B2 10 cm, and B3 mixed/repeated holdout sets pass all hard
   completion, posture, base-height, velocity, foothold, clearance, collision,
   support, contact, slip, torque, SRBD, ID-WBC, planner-runtime, and
   deadline gates.
3. Initial x, gait phase, obstacle position, step parameters, and seeds vary
   according to a pre-frozen anti-memorization protocol.
4. No controller or planner path consumes obstacle ground truth, scene
   coordinates, or the simulator oracle map.
5. Every accepted plan and every fallback is traceable by plan_id, map_epoch,
   state/map age, source provenance, solver status, latency, and reason.
6. Planner failure, stale map, unknown terrain, contact disagreement, SRBD
   failure, and deadline overrun produce the documented safe behavior.
7. The time-indexed foothold/contact interface is consumed coherently by
   SRBD-MPC and its first acceleration continues to feed ID-WBC.
8. Clean holdout manifests contain exact HEAD, clean-source status, config,
   scene-generator, contract, analyzer, and solver versions.
9. The results are reported as terrain-specific evidence; they do not claim
   Stage C hybrid topology, whole-body NMPC, learning, hardware, or
   sim-to-real.

Whole-body NMPC, RL, and new terrain experiments are not prerequisites for
this definition of done.

## 20. Audit trail index

The next agent should consult these preserved sources while implementing:

* accepted baseline: origin/main 71d0e9b and docs/validation;
* old Phase 2 source: example/cpp/terrain/terrain_adaptation.h;
* old sensor path: simulate/src/unitree_sdk2_bridge.h;
* old terrain integration: example/cpp/trot/trot_experiment_gait.cpp and
  trot_experiment_wbc.cpp;
* current SRBD contract: example/cpp/wbc/srbd_mpc.h;
* flat FAIL:
  example/cpp/experiments/_runs/phase2_A_flat_current_steps_fix1_20260825/;
* 10 cm dynamic failure:
  review/terrain-step-v1-wip-2026-08-24 run
  _runs/stepv1_10cm_1;
* old architectural notes: docs/TERRAIN_SENSING_MASTER_PLAN.md,
  docs/terrain_adaptation_route.md, and docs/PROGRESS_LOG.md;
* preserved WIP changes: stash@{0} through stash@{3}; all remain read-only
  audit inputs and must not be applied or deleted as part of this plan.
+

## 21. Git topology snapshot

This is the read-only topology captured after fetch/prune/tags. Plus signs in the
original branch output indicate branches checked out by a worktree.

Worktrees:

* /home/che/dev/go2-mujoco-control at 66dc3e8,
  gait/sustained-sprint-running-2026-08-21;
* /home/che/dev/archive/go2-config-manifest-v1 at c86f85e,
  infra/config-manifest-v1;
* /home/che/dev/archive/go2-highspeed-revalidation-2026-08-24 at 66dc3e8,
  detached;
* /home/che/dev/archive/go2-wbc-base-probe at 2b82dae, detached;
* /home/che/dev/go2-mujoco-control-20260824T150934Z at 048eb1b,
  feature/phase1-runtime-arbitrary-velocity;
* /home/che/dev/go2-mujoco-control-phase1-clean at 2fd5888,
  feature/phase1-runtime-arbitrary-velocity-clean;
* /home/che/dev/go2-mujoco-control-phase1-quant-acceptance at 6e34f99,
  feature/phase1-quantitative-acceptance;
* /home/che/dev/go2-mujoco-control-phase2-048eb1b at 37f3c23,
  research/phase2-terrain-from-048eb1b;
* /home/che/dev/go2-mujoco-control-phase2-from-phase1-2fd5888 at 8bbb366,
  research/phase2-terrain-from-phase1-2fd5888;
* /home/che/dev/go2-mujoco-control-rl-ready at 515aa86,
  rl/ready-contract-2026-08-21;
* /home/che/dev/go2-mujoco-control-terrain at de73edb,
  terrain/adaptation-2026-08-21;
* /home/che/dev/go2-mujoco-control-terrain-faststep at de73edb,
  terrain/fast-step-reference-2026-08-24;
* /home/che/dev/go2-mujoco-control-terrain-minimal at 0068b14,
  terrain/fast-step-minimal-2026-08-24;
* /home/che/dev/go2-mujoco-control-terrain-p2ref at 0068b14,
  terrain/p2-reference-2026-08-24;
* nested review worktree
  /home/che/dev/go2-mujoco-control-terrain/go2-mujoco-control-terrain-step-v1
  at efece291, review/terrain-step-v1-wip-2026-08-24;
* /home/che/dev/go2-natural-ref at d41143f,
  gait/natural-trot-1mps-2026-08-21.

Local branches not checked out by a worktree include:

    backup/main-before-auto-sensing-2026-08-21
    backup/main-before-environment-adaptation-2026-08-20
    backup/main-before-wbc-full-2026-08-19
    cursor/cartesian-world-trot-a3ec
    docs/wbc-full-mainline-claims
    docs/wbc-full-repeat-2026-08-18
    feature/auto-environment-sensing
    feature/environment-adaptation
    main
    recovery/wbc-transition-20260818
    speed/1mps-2026-08-21
    terrain/step-v1-2026-08-24

Fetched remote branches:

    origin/HEAD -> origin/main
    origin/main
    origin/feature/phase1-quantitative-acceptance
    origin/feature/phase1-runtime-arbitrary-velocity-clean
    origin/maintenance/repo-governance-spec-2026-08-24
    origin/review/terrain-step-v1-wip-2026-08-24

Stashes, all preserved and not applied, popped, deleted, or rewritten:

    stash@{0} c2119258  phase2 WIP paused 2026-08-25 11:13 CST
    stash@{1} d49b6b70  preserved before Phase 1 quantitative acceptance
    stash@{2} 7d6704d0  preserved before Phase 1 PR closeout
    stash@{3} 708e12a6  preserved before Phase 1 completion

The canonical worktree was clean on the sustained-sprint branch at audit time.
The Phase 2 worktree was clean before this document was added. Existing dirty
terrain worktrees and untracked build/evidence directories were left untouched.
