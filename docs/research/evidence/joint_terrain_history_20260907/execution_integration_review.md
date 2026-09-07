# Joint feedback execution integration design (read-only source audit)
Target source: `/home/che/dev/go2-workspace/feat-stage-c-joint-planner`, working source based on HEAD `0ff9dc8`. This is an integration map only; no repository edit, build, or simulation was performed.
## 1. Current single command path and insertion point
The only LowCmd writer is `TrotExperiment::LowCmdWrite` (`example/cpp/trot/trot_experiment_control.cpp:546-724`). Each control tick is:
1. `SnapshotState`, then lockstep state-tick validation.
2. `MotionClockStep`.
3. Clear/load this tick's `terrain_tick_plan_` (`582-597`).
4. `PhaseRunGait`, which calls `BuildGaitTargets` (`control.cpp:1497-1745`, `gait.cpp:205-230,1710`).
5. `UpdateWbcShadowAndTorqueFf`, then `WriteMotorCommands` (`control.cpp:690-706`).
6. `PublishLowCmdWithCrc`, ack, log, and terrain snapshot.
Adoption must happen once after step 2 and before `PhaseRunGait`: the worker only `atomic_store`s the newest `shared_ptr<const PendingExecutionBundle>`; LowCmdWrite `atomic_load`s it once per tick, validates it, and promotes or retains one owner-side `AcceptedExecutionBundle`. The same pointer/version must flow through `PhaseRunGait -> BuildGaitTargets -> UpdateWbcFull -> WriteMotorCommands/LogSample`. These consumers must not read pending, adopt again, or rebuild events. `TerrainPlanStore` remains the legacy `TerrainMotionPlan` store and must not become a second bundle ledger.
The worker path is `PublishTerrainControlSnapshot` (`control.cpp:107-222`) -> `UpdateTerrainRuntime` (`224-307`) -> `TerrainPlannerWorker` (`309-462`). `JointPlanningShadow::Capture` only emits reduced proposals/logs and explicitly has no command authority. `TrotExperiment` currently has only `terrain_tick_plan_`; it has no Stage-C bundle store. `PendingExecutionBundle` and `AcceptedExecutionBundle` are only type declarations in `stage_c/types.h:574-588`.
## 2. Producer and minimum bundle payload
The worker-produced `PendingExecutionBundle` needs, at minimum:
* `PlanningIdentity`: source state tick/time, map epoch, schedule epoch, source proposal/plan id, source freshness, and coverage `[start,end]`.
* Event table: `(schedule_epoch, leg, sequence)`, liftoff/touchdown/contact-end, target, and surface provenance.
* Event-indexed reference curve/sample (position, velocity, acceleration, curve identity/hash, frame, point role). Do not place a full curve in `PlanCandidate` and later guess its event.
* Selected centroidal `cdd/Ldot` reference and weights, plus nominal `ContactForceInterval`; nominal force is only a soft WBC reference.
* Collision-center to foot-site geometry binding, IK shadow admission, and reference validity. Keep source time and applicability time distinct.
Existing `PlanCandidate` contains reduced `RolloutKnot` COM/velocity/angular-momentum/contact-mask data and force intervals; `RolloutKnot.body_pose_valid` defaults false, and `JointRollout` has no motor-order q/dq/ddq. It cannot be treated as an articulated execution trajectory or command trigger.
`TerrainControlSnapshot` (`trot_experiment.h:262-292`) has measured anchors, nominal body feet, gait phase/period/duty, joint positions, and measured contact, but no `previous_commanded_world_feet_`, commanded world velocity, source time, or schedule epoch. If the worker creates the first proposal, extend this typed snapshot. Otherwise the owner must construct the first bootstrap from live command fields; it must not use measured FK or nominal stand feet as a command reference.
## 3. First adoption and in-flight events
A gated first C1 bootstrap is feasible: inside a stable kernel/`PhaseClock` epoch, use each leg's current commanded world foot position/velocity as the curve start and preserve that leg's existing absolute touchdown time. Later accepted proposals may change only uncommitted events. This does not modify actual `q/dq` and does not turn command position into measured contact.
Reusable current sources:
* At the end of `BuildGaitTargets` (`gait.cpp:1773-1785`), when pose is valid, the final commanded `feet` are transformed into `previous_commanded_world_feet_`, with `previous_commanded_time_s_`. These are the bootstrap position/time sources.
* World velocity is not stored there. At the same owner/tick, compute it from `commanded_body_feet_velocity_` plus state base linear velocity, IMU angular velocity, and body rotation, using the existing WBC formula at `wbc.cpp:977-1000`: `v_base + R*(omega x r + rel_v)`. Reject bootstrap if command position/time/velocity provenance is invalid; do not fall back to `AllFootPositions(stand_up_joint_pos_)`.
* Absolute TD/LO comes from the fixed phase/schedule/`PhaseClock`, never worker generation time or plan arrival time. With active commitments, `PhaseClock` must reject period/duty changes. Therefore flatclosedloop first adoption must lock a stable period (the current joint-shadow capture window is 20-28 s) or reject.
`TouchdownEventTable::committed_prefix_compatible` (`types.h:387-414`) already compares committed event id, TD/contact-end/liftoff, and target. Reuse it and add curve identity/hash to the compatibility check. Existing `TerrainExecutionCommitment` and `TerrainExecutionConsistency` (`terrain_commitment_lifecycle.h:44-74`, `terrain_execution_consistency.h:258-433`) can adapt current expiry/inheritance semantics. Plan expiry must not clear an accepted in-flight curve; release it at liftoff/completion. A new proposal that changes a committed event's time, target, surface, or curve is rejected as a whole. Only uncommitted future events may change. Do not add a second contact FSM.
## 4. Sampling, gait/IK, and WBC must use the same reference
Sample an accepted bundle at `state_snapshot.tick()*1e-3`. `foot_trajectory.h:329-559` `Prepare`/`SamplePrepared` already define event coverage, C1 Hermite, clearance, stance endpoints, and provenance. `SampleFootTrajectory` (`640-685`) is only a deterministic sampler; it does not prove execution. Runtime use requires bundle admission first.
`BuildGaitTargets` currently performs the legacy terrain endpoint transaction at `gait.cpp:1483-1718`, then updates body-foot velocity, then runs IK at `1752-1771`. In flatclosedloop, the accepted sample must be the sole `feet` input at that point. The old terrain transaction must be bypassed or serialized under the same gate; otherwise it can overwrite the accepted target. Keep velocity bookkeeping and world-command diagnostics. Clamped IK alone is not geometric admission: independently evaluate the achieved collision-center FK residual.
Stage-C `FootTrajectorySample.center_world` is a collision center; candidate touchdown is target plus radius times normal. Go2 runtime distinguishes geom center and foot site (`go2_rigid_body.h:84-112,276-315`), while `AllLegInverseKinematics*` expects the foot site. Use actual model geometry for center-to-site conversion and run IK/clamp shadow admission. Do not feed center directly to IK or substitute a fixed 22 mm offset for the model frame. Reject the bundle if conversion, IK, finite, or clearance checks fail; keep the old path and log the rejection.
`UpdateWbcFull` (`wbc.cpp:48-121`) builds dynamics from actual rigid state, then creates swing/stance tasks at `915-1019`. Swing WBC tasks must use the accepted collision-CENTER p/v/a sample to match dyn.foot_pos_world; gait IK consumes a model-consistent foot-SITE conversion of that same reference. Stance legs retain existing actual-contact stance feedback. Once actual dynamics and contact mode are selected, `SetCentroidalWbcTask` (`terrain/stage_c/centroidal_wbc_task.h:7-35`) may supply COM acceleration/angular-momentum-derivative map/bias/desired/weights. Do not use a `body_pose_valid=false` rollout as a body command. Do not use `ReconstructBodyTrajectory` as a runtime hard gate: it zeroes seed linear/angular velocity and dq and requires near-stationary stance, which conflicts with compliant running.
## 5. Contact, force, and final actuator semantics
Current WBC `qp_contact` is measured foot-force/hysteresis plus gait schedule/merge (`wbc.cpp:123-190`); the same mask enters MPC horizon (`544-572`) and `wbc_in.contact` (`663-697`). Planned, measured, fused, and applied diagnostics remain separate. The bundle nominal schedule/event table must not replace `qp_contact`, promote planned/applied to measured, or create another contact authority. The existing WBC/phase path remains the sole actual-contact state machine; the event table only commits target/time.
A bundle force interval can be copied to `IdWbcInput::force_ref` with nonzero `w_force_track`; `SolveInverseDynamicsWbc` treats it as a soft quadratic reference (`inverse_dynamics_wbc.h:344-355`) and has no hard force-equality field. Nominal force or a `PlanCandidate` certificate cannot claim exact force execution.
`WriteMotorCommands` (`control.cpp:1831-1991`) is the final LowCmd owner: q/dq come from gait/IK targets, kp/kd are stance/swing blends, and tau receives primary ramp, contact/swing scale, and candidate scaling. ID-WBC may reuse `last_id_wbc_` (`wbc.cpp:1101-1153`) and add pitch/force/Cartesian PD overlays (`1155-1318`); the selected certificate is recorded after overlays (`1321-1328`). Accepted bundle must not write tau directly. Record actual dyn/contact -> ID-WBC tau -> overlays -> scaled final LowCmd tau, plus plant finite/saturation envelope, separately from the solver certificate.
## 6. flatclosedloop blockers and minimum gate
Current CLI has `--terrain-b1-execution`; `--stage-c-execution` is retired. There is no flatclosedloop opt-in. Add an independent default-off gate requiring running-trot, wbc-full, flat profile, and valid high-state/model geometry. Each tick must validate pending identity/freshness, source tick/time, fixed schedule epoch, complete horizon/curve coverage, committed-prefix/curve compatibility, surface/map provenance, center-to-site/IK admission, solver/torque finite state, and final actuator envelope. Any failure rejects the whole bundle, keeps the old baseline path, and records a reason.
The concrete blockers are: no atomic bundle store/consumer; `AcceptedExecutionBundle` lacks event refs/curve/identity/coverage; runtime `TerrainMotionPlan`/MPC/ID-WBC identity lacks schedule epoch; snapshot lacks command world p/v; `PlanCandidate` lacks articulated q/dq/ddq; center/site geometry gate is absent; ID certificate excludes final PD/scaling/saturation; and the legacy terrain transaction can overwrite a new sample. Strict body reconstruction is not a workaround because it conflicts with actual running q/dq.
## 7. Exact first-tick shape (pseudocode)
```text
LowCmdWrite:
  snapshot state; MotionClockStep
  pending = atomic_load(pending_bundle)                 # once
  if flat_gate and Validate(pending, state_time, schedule_epoch,
                            coverage, committed_prefix, geometry, IK):
      owner.accepted = Promote(pending)                 # once
  tick_bundle = owner.accepted                           # one pointer
  PhaseRunGait(..., tick_bundle)
    BuildGaitTargets(..., tick_bundle.sample(now), IK)
    UpdateWbcFull(..., tick_bundle.sample(now), existing qp_contact)
    WriteMotorCommands(..., only final q/dq/kp/kd/tau composition)
  log bundle version/reference hash + measured/nominal/applied masks + final actuator envelope
  PublishLowCmdWithCrc
```
## 8. Minimum test boundary
Add `test_stage_c_execution_bundle` in the Stage-C test block of `example/cpp/CMakeLists.txt`, or place focused cases in existing `test_stage_c_foot_trajectory.cpp` and `test_terrain_execution_consistency.cpp`:
* Atomic pending-to-accepted adoption occurs once per state tick; stale source, expiry, coverage, or identity rejects closed.
* Bootstrap uses command world p/v and unchanged absolute TD; actual q/dq stay untouched; missing command provenance rejects.
* A committed in-flight event retains id, TD, LO, contact-end, target, surface, and curve hash after replan; only uncommitted future events update.
* The same bundle version/reference hash reaches BuildGaitTargets, WBC, and WriteMotorCommands; the old terrain transaction cannot overwrite it.
* Measured, nominal planned, and applied masks stay independent; nominal force is soft tracking; existing `qp_contact` remains authoritative.
* Collision-center-to-foot-site, geometry epoch, clamped IK shadow, C1, and clearance checks reject closed.
* Final q/dq/kp/kd/tau, feedforward scaling, PD overlay, and plant saturation are traceable separately; model-sample success does not set `execution_ready`.
Existing `test_stage_c_foot_trajectory` covers in-flight initial velocity, C1 endpoint, tail/unknown provenance, and committed-prefix retime rejection. `test_stage_c_feedback_step` covers non-projecting actual q/dq and moving support. `test_terrain_execution_consistency` covers target/time inheritance. Keep these and the old flat/5 cm baseline unchanged before runtime integration.
