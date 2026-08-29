# Phase2 B1 Timing-Contract Redesign (2026-08-28)

Author: Kimi (taking over phase2-b1-b3). Status: design, pre-implementation.
Evidence base: `_runs/b1_baseline_53211b9_epoch8_20260828` (HEAD-equivalent
baseline at 53211b9), B2 handoff §6/§8, terrain plan §8/§9.

## 1. Fresh diagnosis from run data (not code reading)

Timeline of the baseline B1 dev run (5 cm step, 0.30 m/s approach):

- t≈6.4 s: surface transition declared, required_mask=15 (all legs).
- t=6.46-6.90: terrain velocity cap brakes the body 0.30 → 0.00. The robot
  never moves again; the cycle-quality guard aborts at 10.5 s.
- FL executes its terrain swing and gets a measured touchdown on the plateau
  (committed_mask=2). Its exec target z=0.05 (correct upper surface).
- FR had an exec target z=0.05 at t=6.5 that vanished by t=6.75 and never
  returned. Prepare rejections alternate between code 3
  (required_swing_duration > available: timing infeasible) and code 4
  (foothold not classified as a terrain target).
- Plan stream keeps refreshing (plan_id 209→377); intermittent whole-plan
  rejections (status=4) at t≈5.5 and 8.5, where BuildRetimedPlanInput could
  not fit the delay inside the 24-knot horizon.
- single_vcmd_authority violation (0.2999) is dominated by the first
  transfer row where wbc_mpc_update_count=0 (reference fields still 0 while
  applied=0.30). During the actual transfer the MPC reference vx tracks
  applied exactly.

## 2. Root causes, in causal order

R1. **The contact schedule is not stretched for the co-event pair.**
BuildRetimedPlanInput delays only the touchdown of legs whose own foothold
rises (plus same-knot co-event members). The opposite diagonal pair keeps its
phase-derived swing start. Execution follows the phase machine, so when the
extended swing overruns, the opposite pair's swing starts anyway → support
collapses or the terrain leg's window collapses → reject-3 forever. The plan
schedule and execution diverge by construction.

R2. **The crossing brakes to zero and never recovers.** The contract
(B123 common: "a brake ... is FAIL") requires a seamless crossing at the
approach speed. Any design whose happy path includes cap-to-zero cannot PASS.

R3. **Five overlapping consumer-side state expressions** (endpoint_held,
measured_touchdown, transfer_hold_contact, surface_transition_committed,
effective_transfer_hold in WBC) each re-derive "this leg is still waiting".
They diverge (FR's transaction vanished while its candidate stayed required).

R4. Plan validity 0.15 s vs required swing durations ~0.15-0.20 s at
max_swing_speed 2.5 m/s (path ≈ 0.2 m L1 × 1.875 / 2.5): executable windows
are borderline; retimed touchdowns beyond valid_until are skipped by
find_planned_foothold. The schedule must place the terrain touchdown within
reach of a fresh plan.

## 3. Redesign (the B2-handoff §8 action 1, made concrete)

**One atomic timing contract, planner-owned, consumed identically by gait,
MPC, and WBC.**

S1. Planner: replace BuildRetimedPlanInput with an explicit event-stretch
model. When a terrain swing needs duration D > nominal window:
- the swinging diagonal pair gets contact=false for [start, touchdown] with
  touchdown-start = D (start moved earlier, touchdown later, both bounded);
- ALL subsequent contact events of ALL legs shift by the stretch s so the
  schedule never overlaps the two pairs' swings (no zero-support knot);
- if the stretch cannot fit the horizon or violates support margin, reject
  the plan (fail-closed via v_cmd cap is legal; a silent half-schedule is
  not).

S2. Gait: during an active terrain plan, contact state per leg comes from
the plan schedule knots (not raw phase). Stance legs world-anchor at their
measured foothold; swing legs execute the terrain transaction (existing
quintic path + TerrainSwingDurationForPath timing); touchdown is a measured
support confirmation (existing WBC logic). After the plan horizon/crossing,
control returns to the phase machine (plans re-anchor to live phase at 20 Hz).

S3. One owner: replace terrain_swing_execution_/pending_,
terrain_transfer_hold_*, terrain_surface_transition_* with a single
TerrainExecutionState { per-leg: Stance | SwingTransaction | HeldSupport,
window [start, touchdown], plan_id } owned by the gait↔WBC snapshot seam
(TerrainControlSnapshot already exists). WBC/MPC read it; they do not
re-derive it.

S4. v_cmd single authority: MPC reference vx always from
velocity_command_state_.applied_mps (shaper output); remove the
kernel_nominal_velocity_x_mps_ preference on the runtime-command path
(wbc.cpp:588-599, gait.cpp:336-344 keep kernel_nominal as diagnostics only).

## 4. Verification protocol (anti-thrash discipline)

- One hypothesis, one change, one canary. Canary command frozen as
  b1_baseline_53211b9_epoch8_20260828's argv (period 0.50/duty 0.75 path via
  the new time-qualified scheduler; lift from runtime schedule).
- First gate after each change: flat-ground regression — B0 brake pair +
  steps pair must stay green (scheduler fix already PASS on brake r1).
- Then B1 dev run; read phase2_terrain_analysis.json; fix the FIRST failing
  gate in causal order only.
- Unit tests: extend test_terrain_interfaces.cpp for S1 (schedule stretch
  cases: single-leg rise, co-event group, horizon overflow rejection).
- No threshold changes, no Phase-1 gain tuning, no scene edits, no oracle
  inputs. Analyzer and contract files are hash-frozen: do not touch.

## 5. Implementation slices

1. S4 (smallest, independent). Canary: vcmd authority metric ≤0.02.
2. S1 + unit tests. Canary: plan rejections at 5.5/8.5 disappear; FR/RL get
   executable stretched windows.
3. S2+S3 (the large one; only if slice 2 leaves consumer divergence).
4. B1 dev PASS → 3 holdout scenes (PHASE2_B1_HOLDOUT_MANIFEST.json) →
   B0 full-matrix regression (contract: any code fix re-certifies B0).

## 6. Two-contact support gate recalibration (epoch9/10 evidence)

Evidence: the epoch9/epoch10 canaries reject essentially every terrain plan
with kSupportInfeasible at support margins 0.0146-0.0150 against
min_support_margin_m = 0.015 (controller.log "Terrain support reject",
masks 6/9 = the two trot diagonals).  epoch8, which predates the S1 timeline
stretch, shows the same signature at 0.0105-0.0144, so the gate is
structurally too tight for this gait, not a stretch regression.

Mechanism: for a two-contact set SupportMargin2D returns
min(endpoint_margin, max_two_contact_line_error_m - line_error).  A trot
diagonal sits ~30 degrees off the direction of travel (nominal stance
x = +/-0.19-0.20 m, y = +/-0.10-0.14 m, sin ~ 0.45-0.59), so while the pair
holds and the body translates at v, the COM drifts off the support line at
v·sin ~ 0.13-0.18 m/s.  The gate requires line_error <= 0.040-0.015 = 25 mm,
i.e. a pure diagonal hold may last at most ~140-190 ms at 0.30 m/s.  The
nominal duty-0.75 / period-0.50 trot already holds each diagonal alone for
~125 ms (~17-22 mm line_error at hold end, up to 88% of budget).  Any S1
terrain stretch (+100-200 ms) must cross the gate, and the rejection then
blocks the very plan whose touchdown would end the hold — a deadlock at the
gate boundary, which matches the observed margin clustering.

Excluded alternative: yaw-only foot projection (RotateBaseToWorld) drops
pitch/roll, but at the reject instants the measured |roll|,|pitch| <= 0.5 deg
(epoch10 data.csv at the 57 failure=5 plan timestamps), biasing line_error
by <= 0.3 cm — two orders below the 25 mm gap.  The rest of the stack
(ComputeWorldFeet, gait support anchors) uses the full quaternion, so the
yaw-only helper remains a latent inconsistency, but it is not this failure.

Provenance: min_support_margin_m = 0.015 and max_two_contact_line_error_m =
0.040 both date from scaffolding commit 81ad07a with no derivation; nothing
in docs/ or the analyzers references them.  They are planner-internal
geometry constants, not acceptance-contract gates (the analyzer and contract
files stay hash-frozen; the "no threshold changes" rule in section 4 covers
those, not this).

Decision: keep min_support_margin_m = 0.015 unchanged (the physical safety
band); recalibrate the lateral capsule half-width
max_two_contact_line_error_m 0.040 -> 0.060.  Sizing: nominal end-of-hold
line_error ~ 22 mm + committed-stretch allowance ~ 23 mm (~130 ms of added
hold at 0.30 m/s) + 15 mm margin = 60 mm.  A total diagonal hold beyond
(60-15)/0.176 ~ 255 ms still fails closed, as it should — holds that long
must slow the body instead of being rubber-stamped.  The endpoint term (COM
must stay between the two stance feet along the line) is untouched and
remains the true static tip-over gate; degenerate near-coincident foot pairs
self-reject through the zero-length line.

## 7. Crux recalibration after epoch11 (2026-08-28)

epoch11 (capsule 0.060) relieved the approach deadlock (required plan
rejection rows 1181 -> 411, posture restored) but still failed at the crux.
Quantification of `_runs/b1_margin2_epoch11_20260828` (contact ground truth
time-aligned to data.csv with a +1.29 s offset):

- Realized two-contact holds during the crux are 22-50 ms (running trot
  with aerial phases) — far below the 125 ms nominal stance and the 255 ms
  allowance.  The long-hold/drift framing does not apply.
- v_applied stayed 0.30 m/s with the terrain cap at inf; v_meas physically
  decayed from 0.39 m/s at t=6.3 to 0 at t=6.8.  Every plan was rejected
  (failure=kSupportInfeasible) from t~=6.32, freezing the execution plan at
  id 231 before FL landed; the stall and the t=6.79-6.95 borderline rejects
  are downstream of that freeze.
- The three borderline rejects all encode the same forced geometry: with
  the front foot on the plateau and the rear foot still on flat ground, the
  support line of the landing diagonal sits 45-46 mm off the COM path
  (recomputed from the logged selected_xy: e.g. (0.225,0.10)-(-0.125,-0.15)
  gives 45.9 mm; the endpoint margin stays ~190 mm).  The 0.060 capsule
  leaves a 45 mm budget, so the gate rejected the forced crossing geometry
  by ~1 mm.  This is geometry-bound, not drift-bound: measured velocity was
  already collapsing at two of the three reject instants.
- Deceleration would not fix it (the line error is positional, not
  accumulated drift) and the contract forbids brake-to-zero mid-crossing
  (section 2, R2); the velocity script commands a fixed 0.30 m/s.

Decision: recalibrate max_two_contact_line_error_m 0.060 -> 0.070.
Sizing: observed forced straddle 45-46 mm + the unchanged 15 mm safety
margin + ~9 mm placement-variation headroom = 70 mm.  The endpoint term is
untouched and remains the static tip-over gate; the genuinely degenerate
states seen during the epoch11 fall (line errors 110-230 mm, negative
margins) remain rejected.  Regression test: test_terrain_interfaces.cpp
pins the straddle accept / corridor reject / endpoint reject trio directly
against SupportMargin2D.

Residual consumer-side defect recorded for the next round: FL physically
landed on the plateau lip at t=6.64 (ground-truth contact 50-127 N,
foot at x=0.685, z=0.052-0.068) but its terrain swing never completed —
the touchdown evaluation in trot_experiment_wbc.cpp (~line 315) is gated on
execution.endpoint_held, and with the plan stream frozen the schedule never
declared the swing over, so in_flight hung for >1 s while the foot was
loaded.  Evaluating measured contact for in-flight executions (keeping the
at_endpoint geometry guard) is the candidate fix once the planner gate no
longer freezes the stream.

## 8. Height-conditional straddle corridor after epoch12 (2026-08-28)

epoch12 (capsule 0.070) still failed plan_support at the crux
(`_runs/b1_crux_epoch12_20260828`): required plan rejection rows 367,
target_prepare_rejections 3556.  A single flat capsule cannot express
the difference between a committed crossing — front and rear feet
contracted to different terrain heights, support line forced 45-59 mm
off the COM path by the straddle geometry — and genuine lateral drift
on flat ground, where a widened flat corridor would rubber-stamp real
instability.

Decision: make the two-contact lateral bound height-conditional.  New
helper TwoContactLineErrorBound (terrain_planner.h, after
SupportMargin2D) returns two_contact_straddle_corridor_m = 0.120 when
exactly two feet are in contact and their world-z difference exceeds
two_contact_straddle_height_m = 0.030 (on the 5 cm step a plateau foot
and a flat-ground foot differ by ~50 mm; flat-ground pairs stay well
under 30 mm), and max_two_contact_line_error_m = 0.070 otherwise.  Both
call sites (SupportFeasibleSelection, SupportFeasible) route through
it.  The min_support_margin_m safety band and the endpoint term are
unchanged, and flat-ground behavior is bit-identical to epoch12.
Regression coverage in test_terrain_interfaces.cpp: the crux straddle
geometry (line error ~56 mm) is rejected on flat ground, the same
geometry with one foot raised +50 mm is accepted under the corridor,
and a far-COM variant is still rejected under the corridor; two further
assertions pin the bound selection itself.  ctest 27/27.

epoch13 canary (`_runs/b1_corridor_epoch13_20260828`, same command
contract): plan_support FAIL -> PASS — the primary objective of this
round.  posture_hard still FAIL (roll max 21.5 deg);
surface_transition_transaction regressed PASS -> FAIL (transition
completions still 1; timing/counting detail not yet diagnosed).
Required plan rejection rows 367 -> 451, target_prepare_rejections
3556 -> 4579: the planner now survives the crux geometry and keeps
proposing, so more plans are attempted and rejected later.

The rejection signature after t=5.5 changed accordingly:
kNoSafeFoothold (fail=4) 1486 rows vs kSupportInfeasible (fail=5) 230
rows — the margin gate is no longer the binding constraint.  Prepare
rejection codes: 4 (1892) > 3 (389) > 6 (21).  The controller log
shows fail=4 coming from two distinct sources: (a) swing-clearance
dominated rejects, where the rear leg's swing path onto the plateau
sweeps through the riser; and (b) 8-swing-candidate kNoSafeFoothold
rejects (plan ids of the 247/257/290 pattern) consistent with
BuildRetimedPlanInput returning false on horizon overflow rather than a
foothold-quality verdict.  Separating these two sources is the next
round's first task.

Trajectory: the base reached x=0.598, z=0.428 at t=7.27 — the deepest
crossing so far, front legs against/on the plateau — before falling
backward-sideways at t~=7.7 (y -> -0.51, z -> 0.06).

Residuals carried forward:
- 12 residual "Terrain support reject" lines (margin 0.011-0.0149,
  line_error 55-59 mm under the 70 mm band, knots 7-9, masks 6/9) did
  not trigger the corridor because both feet were at the same height
  (dz <= 0.03): either the measured-anchor branch scored against a
  flat-ground anchor, or both feet were already on the plateau with
  skewed geometry.  Knot geometry not yet reconstructed.
- The endpoint_held gate recorded in section 7 still applies: when the
  plan stream freezes (now via fail=4 instead of fail=5), the touchdown
  evaluation in trot_experiment_wbc.cpp (~line 315) stays gated on
  execution.endpoint_held and a loaded swing leg can hang in_flight.

## 9. Horizon-overflow graceful truncation after epoch13 (2026-08-28)

Source separation of the epoch13 fail=4 (kNoSafeFoothold) rejects, using
the per-cycle data.csv columns (terrain_failed_leg,
terrain_dominant_foothold_reject) grouped by distinct plan id, in both
`_runs/b1_corridor_epoch13_20260828` and a same-command redo.  Three
previously conflated buckets separate cleanly:

- dominant=none with failed_leg=-1: the BuildRetimedPlanInput
  return-false path (terrain_planner.h) — past the candidate loop, so no
  foothold diagnostic exists.  15 (redo) / 22 (original) distinct plans
  inside the crux window (t=5.5-8.0).  These fire while the measured
  state still shows a healthy 2-3 contact diagonal: a far retimed event
  falls off the 24-knot horizon and kills the whole plan.
- dominant=swing_clearance: 18 (original) / 26 (redo) crux plans.
- dominant=unknown: 12 approach-phase plans at x~=-0.09 with
  known_cells=318 vs 320 — a startup map-coverage transient, plus
  post-fall stragglers; not a crux constraint.
- dominant=reachability: 1-2 plans.

(Note: single canary runs are not bit-reproducible near the failure
boundary — the redo diverged from the original epoch13 run in
plan_support and rejection counts.  Bucket structure and crux counts
agree across both runs; check-level flip-flops between runs should be
read with that variance in mind.)

Decision: fix the horizon-overflow path first (structural and cheap).
BuildRetimedPlanInput no longer returns false when a retimed touchdown
lands beyond the horizon: the event-level guard is removed, and the
stretch insertion truncates at the horizon end (each written row remains
a copy of an original schedule row, so a partial stretch stays internally
consistent; shifted selections beyond the horizon were already dropped
silently by the selection shift loop).  The near-term schedule still
publishes and the dropped far event is replanned once it slides into the
window.  Fail-closed is preserved where it matters: a truncated retime
over a measured state with fewer than two contacts still rejects with
kNoSafeFoothold (this keeps the three-legs-airborne contact-gap case
rejected — it was previously rejected only as a side effect of the
overflow).  Regression: a new test in test_terrain_interfaces.cpp places
the elevated touchdown event at the last horizon knot; it is red without
the fix (plan rejected) and green with it (plan publishes, near-term
schedule rows 0-15 unchanged, event shifted off knot 23).  ctest 27/27.

epoch14 canary (`_runs/b1_foothold_epoch14_20260828`, same command
contract) vs epoch13:

- plan_support PASS -> PASS; posture_hard FAIL -> PASS (roll abs max
  21.5 -> 13.3 deg); posture_p95 still FAIL;
  surface_transition_transaction still FAIL (completions 1 -> 0).
- required_plan_rejection_rows 451 -> 403; target_prepare_rejections
  4579 -> 3406.
- Crux horizon-overflow (none/failed_leg=-1) rejects 15-22 -> 2; the
  remaining two are the new measured-support gate firing during flight
  phases (e.g. plan 241 at state 7.788, all four legs holding 8 swing
  candidates) — intended brief fail-closed, not a freeze.
- The robot still does not cross: max base x 0.578, down at t~=7.3.
  swing_clearance is now the dominant crux reject (47 distinct plans).

swing_clearance quantification (scene geometry + planner path model):

- The step box in unitree_robots/go2/phase2_step_5cm.xml spans
  x=0.70-1.20 at z=0.05: the riser face is at x=0.70.
- The failing legs are the FRONT legs (0=FR, 1=FL) swinging from the
  flat approach onto the plateau lip — not the rear legs as previously
  assumed.  Candidate supply is ample (19-106 regions/leg) and every
  candidate is rejected with swing_clearance (dominant count 32), so
  this is geometry-bound, not sampling-bound.
- The planner's swing model (terrain_feasibility.h): horizontal progress
  is TerrainSwingEase (smoothstep); the vertical arch is
  TerrainSwingProfile (eased triangle peaking at a terrain-chosen phase)
  on top of the linear start->end height; the lift solve searches
  test_lift, and interior samples must clear the swept terrain patch by
  clearance_m for BOTH the foot and two shin interpolation points.  With
  foot_lift 0.08 over a 0.05 riser the foot apex is not the binding
  element; the suspect is the shin segment sweeping the riser corner at
  (0.70, 0.05) while the foot is still climbing — the check's
  worst_foot_phase / minimum_shin_clearance diagnostics exist but are
  not logged.  Next round: log them in the failure diagnostic to
  separate foot-clip from shin-clip before choosing between a higher
  crossing lift and a lift-then-advance via point.
- The 12 residual "Terrain support reject" prints persist (margin
  0.011-0.015 under the unchanged 15 mm band, knots 7-14, masks 6/9);
  one new instance (plan 229, knot=1, margin=-0.018) is a genuine
  negative-margin reject — the safety band still bites.

## 10. Swing-speed model calibration after epoch14 (2026-08-28)

Instrumentation round first.  Three additions, all kept permanently
(env-gated or count-capped, zero cost when off):

- terrain_feasibility.h: the TROT_TERRAIN_DEBUG_SWING reject print was
  dead code (placed after the reject returns); moved to fire after the
  lift-solve loop, and added matching prints at the two estimation-pass
  exits (anchor penetration, degenerate profile shape).
- trot_experiment_control.cpp: the "Terrain support reject" print now
  carries the failing knot's actual feet (world) and body reference, so
  endpoint vs line terms can be decomposed offline.

Findings, in the order they falsified the standing hypotheses:

1. The swing_clearance rejects are 100% the anchor-penetration guard
   (i==0 sample): all 64 epoch15a anchor rejects show the identical
   operands start.z=-0.3893 vs terrain[0]=-0.3506 (gap 38.7 mm, and
   terrain[0]==end.z exactly per row).  start.z is the live encoder-FK
   foot (trot_experiment_control.cpp:243 fills
   input.current_feet_base from AllFootPositions; terrain_planner.h:578
   passes it as the guard's start), not a planned landing z.  Revised
   attribution (epoch16 trace, 2026-08-29; supersedes both the epoch15
   "map content bias" note and the first "52 mm z-origin lag"
   estimate): decomposed in world frame, gap = (cell_world -
   foot_world) + (base_now - base_snapshot).  The map cell under the
   start anchor reads ~= +0.05 (the anchor sits in the riser's own
   5 cm cell, which the sparse lidar sweep fills with the riser top)
   vs the flat-standing foot origin at ~= +0.021 (including the ~20 mm
   foot-site offset) — ~= 29 mm.  The z-reference lag (publisher
   re-references to base_link xpos at every 50 ms publish; map ages
   0.01-0.1 s; base rising 0.2-0.3 m/s at the crux) contributes only
   ~= 8-15 mm.  FK == MuJoCo truth (worker, n=15094, median 0); map
   content is geometrically right, it is the single-cell anchor
   comparison that is fragile.  It is neither foot-vs-riser mid-swing
   nor shin clip.
2. Timeline re-analysis (epoch14 + 15a runs): swing_clearance appears
   only POST-stall (t>=6.7, base already sliding, contact masks
   degraded).  The early-crux blocker is fail=5 kSupportInfeasible:
   margin 0.011-0.015 vs the 15 mm band at knots 7-16.  Decomposition
   of the printed knot geometry: the LINE term binds (line_error
   55-58 mm vs the 70 mm band; endpoint term 93-174 mm healthy), and
   both feet read the same height (z~=0.02, map-blended riser-edge
   cells), so the epoch13 straddle corridor (dz>30 mm) never triggers.
3. Root driver of the long holds: max_swing_speed_mps=2.50 predicts a
   330 ms swing for the crux step-up path (L1 ~0.41 m), forcing ~10-knot
   stretches and ~300 ms diagonal holds — exactly what the 70 mm drift
   band (sized for ~310 ms at 0.30 m/s) is designed to reject.  But the
   legs physically swing much faster: the realized crux step-up swing
   took 162 ms (eased-profile peak ~4.6-4.7 m/s), and flat nominal
   125 ms swings already peak ~3.5 m/s.  The planner's duration model
   was 2x conservative, and the same constant gates the gait consumer's
   prepare checks (codes 3/6, required > available + tolerance).

Fix: max_swing_speed_mps 2.50 -> 4.50 (one default; drives the planner
retime and both gait prepare gates).  Flat paths (L1 ~0.23 m) still
predict under the nominal 125 ms, so flat-ground behavior is unchanged.
Regression test: default-config duration for the crux path is
<= 195 ms (red at 2.50) and flat stays exactly nominal; ctest 27/27.

Canary (`_runs/b1_swing_epoch15_20260828` + one repeat `_r2`, same
command contract) vs epoch14:

- r1: plan_support PASS, posture_hard PASS (roll max 10.0 deg),
  surface_transition_transaction PASS (completions 1 — first PASS of
  the whole campaign), posture_p95 FAIL; rejection rows 471, prepare
  3392; max base x 0.436; front foot committed onto the plateau, then
  the base stalled and slid back, down at t=7.45.
- r2: plan_support FAIL, posture_hard PASS, posture_p95 PASS (first
  PASS), surface_transition FAIL; rejection rows 172 (lowest yet),
  prepare 3336; max base x 0.424, down at t=7.09.
- epoch14 reference: plan_support PASS, posture_hard PASS, surface_tx
  FAIL, posture_p95 FAIL; rows 403, prepare 3406; max x 0.578.

Read: individual check flips are within the documented run-to-run
variance; the durable trends are prepare rejections 4579 -> 3406 ->
~3390, the end of multi-second plan-stream freezes, and a new terminal
signature — physical contact collapse at the riser base (masks degrade
to 0-2 contacts) around x~=0.42-0.44, not a planner deadlock.

## 11. Handoff snapshot (2026-08-28, workspace state)

All work is uncommitted on branch phase2-b1-b3 (HEAD 70b7740).
ctest 27/27 green.  Uncommitted changes by file and mechanism:

- example/cpp/terrain/terrain_feasibility.h — swing-clearance reject
  diagnostics made reachable/complete (TROT_TERRAIN_DEBUG_SWING);
  max_swing_speed_mps 2.50 -> 4.50 (epoch15, this section).
- example/cpp/terrain/terrain_planner.h — epoch10: retime-shifted
  footholds carry forward with base travel; epoch11/12:
  max_two_contact_line_error_m 0.040 -> 0.070; epoch13:
  height-conditional straddle corridor (TwoContactLineErrorBound,
  two_contact_straddle_height_m=0.030, corridor 0.120); epoch14:
  BuildRetimedPlanInput horizon overflow degrades gracefully (truncate
  at horizon, drop far events) with a fail-closed gate when the
  measured state has <2 contacts.
- example/cpp/tests/test_terrain_interfaces.cpp — regression tests for
  all of the above (moving-stretch carry-forward, SupportMargin2D trio,
  corridor trio, horizon-overflow truncation, swing-speed calibration).
- example/cpp/trot/trot_experiment_control.cpp — support-reject print
  carries failing-knot feet + COM.
- example/cpp/trot/{trot_experiment.h,trot_experiment_gait.cpp} —
  pre-epoch9: time-qualified low-speed support-rich scheduler (avoids
  the ramp-through-low-band instability, B0 brake_3_to_0 regression).
- example/cpp/trot/trot_experiment_wbc.cpp — pre-epoch9: MPC reference
  vx first-window logging fix.
- example/cpp/trot/velocity_command.h + tests/test_velocity_command.cpp
  — pre-epoch9: low-speed schedule qualification made time-based.

Canary evolution (same B1 command contract throughout):

- epoch9/10: near-total plan rejection at the crux (section 6);
  capsule 0.040 -> 0.060.
- epoch11: rejection rows 1181 -> 411, approach deadlock relieved,
  still fails at the crux; capsule 0.060 -> 0.070 (section 7).
- epoch12: rows 367; still plan_support FAIL (section 8).
- epoch13: straddle corridor; plan_support FAIL -> PASS; rows 451,
  prepare 4579; roll max 21.5 deg; down at t~=7.7, max x 0.598-0.600.
- epoch14: horizon-overflow truncation; posture_hard PASS; rows 403,
  prepare 3406; crux overflow rejects 15-22 -> 2; down at t~=7.3.
- epoch15: swing-speed calibration; surface_transition_transaction and
  posture_p95 each PASS for the first time (in separate samples);
  prepare ~3390; terminal failure is physical, at the riser base.

Remaining failure signatures and next steps, in priority order:

1. Anchor-guard terrain reference (revised twice; epoch16 trace and
   canary): the 38.7 mm guard gap is NOT a map content bias and the
   "52 mm z-origin lag" estimate did not reproduce — the sim publisher
   (simulate/src/unitree_sdk2_bridge.h, PublishLidarHeightMap) converts
   world hits to base-relative with the base_link xpos at each 50 ms
   publish snapshot, so the true z-reference lag is map-age x
   base-rise-rate ~= 8-15 mm at the crux.  The dominant term is the
   anchor cell itself: the start anchor stands in the riser's own
   5 cm cell, which the sparse sweep fills with the riser top
   (~= +0.05 world) while the flat-standing foot origin reads
   ~= +0.021 (incl. the ~20 mm foot-site offset) — ~= 29 mm.  epoch16
   shipped the frame unification (map re-referenced to the planner
   snapshot's base height on ingest, trot_experiment_control.cpp) and
   swing_clearance remained the dominant crux reject, confirming the
   z-ref lag is the minor term.  Next hypothesis: make the anchor
   guard use a patch-min / proprioceptive reference instead of the
   single cell under the anchor.  RESOLVED in epoch17 (section 14):
   neighborhood-minimum anchor; crux swing_clearance storm eliminated.
   The riser-edge foothold pairs at z~=0.02 still evade the 30 mm
   straddle-corridor condition.
1b. RESOLVED in epoch18 (section 15): crux plan starvation by kUnknown
   rejects was the FOV-fringe vs in-window unknown conflation.  Fixed
   (outside_cells / HasUnknownInside; anchor i==0 patch gate removed;
   fringe shin skip); confirmed on two canaries (no fall, plan stream
   alive).  The new head blocker is the support-feasibility deadlock
   at the crossing: every crux plan fails the two-contact support gate
   with margin ~= -0.10 m at knot 2 (mask 9) — knot COM 10 cm past the
   FR-RL support line before the front feet commit to the plateau.
   The riser-edge foothold pairs at z~=0.02 (still evading the 30 mm
   straddle corridor, item 1 tail) are the prime suspect for why the
   crossing sequence never commits.
2. Physical trip at the riser base: with the plan stream alive, the
   robot now reaches the riser and collapses there (masks -> 0-2).
   Compare the executed toe trajectory against the riser corner in the
   ground-truth contact log; check whether the 160 ms committed swings
   actually track the planner's cleared path (endpoint_held gate in
   trot_experiment_wbc.cpp ~line 315 is still in effect).
3. Run-to-run variance near the failure boundary makes single-canary
   check flips unreliable; any future gating decision should sample at
   least two runs (epoch15 r1/r2 differ on plan_support, posture_p95,
   and surface_transition_transaction simultaneously).
4. B0 flat-ground matrix has not been re-run since epoch13.  The
   epoch13-15 changes are analytically flat-invariant (corridor needs
   dz>30 mm, truncation needs terrain retiming, flat swing durations
   stay nominal), but an actual B0 run is owed before merge.

## 12. Unattended worker protocol (worker ant <-> main agent)

> **DEPRECATED (2026-08-29):** this cross-CLI protocol (kimi session
> report channel + WORKER_ORDERS polling) was designed for unattended
> cross-process collaboration; it never closed the loop automatically
> in practice.  After handoff it is replaced by the harness's native
> subagent dispatch.  The text below is retained for provenance only —
> do not follow it.

Setup: a DeepSeek-class worker agent is launched from the Windows side
and enters WSL via `wsl -e bash -lc`, operating in
/home/che/dev/go2-workspace/current, running unattended overnight.  The
main agent is Kimi Code CLI on the Windows side (C:\Workspace).  Any
situation not covered by this document must stop the worker and be
reported to the main agent; improvisation is forbidden.

Verified channels (do not change):

- Report channel (executed on the Windows side):
  `kimi -p "<report>" --session session_de7ac201-eee8-4cd2-9f23-6c1b547b937b`.  The main agent's
  reply arrives on stdout; use `--output-format stream-json` to parse
  it.  `kimi --continue -p` under C:\Workspace can resume the most
  recent session, but it carries a real risk of resuming the WRONG
  session — always prefer the explicit session id.  session_de7ac201-eee8-4cd2-9f23-6c1b547b937b
  is a placeholder: the main agent supplies the id and it is filled in
  here before the first unattended shift.
- Orders channel: the main agent writes the next instruction to
  /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/WORKER_ORDERS.md;
  the worker polls that file and continues from it.

Stop-and-report triggers (hitting ANY of them means stop immediately
and report; no freelancing):

- T1: a failure signature not recorded in this document appears (a new
  reject code, or a new FAIL-check combination).
- T2: any urge to touch the acceptance-contract files or analyzer
  thresholds.  Absolute red line; the thought itself triggers a report.
- T3: two consecutive canaries show the same signature with no
  improvement.
- T4: flat-ground B0 regression — any previously PASS flat member
  turns FAIL.
- T5: simulation environment fault (DDS crash, domain 220/221
  anomalies, build failure).

Report format (fixed fields, in this order):

1. timestamp (local, with timezone offset);
2. run_id;
3. trigger id (T1-T5);
4. failure-signature digest (one line: codes, checks, key numbers);
5. evidence file paths (absolute): controller.log, data.csv,
   phase2_terrain_analysis.json of the offending run;
6. workspace `git status --short` digest;
7. worker's suggested next step (optional).

Worker behaviour rules:

- After reporting, stop ALL simulation and compilation activity and
  poll WORKER_ORDERS.md.  Do not restart any attempt without a new
  order.
- Every change still follows the one-hypothesis-one-canary discipline
  of section 4 (unit test + build + 27/27 ctest + canary comparison).
- Append every escalation to
  /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/ESCALATION_LOG.md
  using the same field format as the report, one block per event.

### 12.1 Quota throttling and run rules

1. Reports carry structured summaries only: the seven fixed fields plus
   evidence file paths.  Never paste whole logs or csv content into a
   report; the main agent reads the files itself when it needs detail.
2. Each failure signature is reported ONCE: if the same signature
   recurs before a new order arrives, do not report again — just append
   the event to ESCALATION_LOG.md.
3. Rate limit: at most 2 proactive reports per hour.  Events beyond
   that accumulate in ESCALATION_LOG.md and wait for the main agent to
   read them on its own initiative.
4. A live main-agent session is a precondition of this protocol.  If
   `kimi -p` fails (command missing, timeout, non-zero exit), degrade
   to file-only mode: write ESCALATION_LOG.md, stop, and wait for a
   human.  No retry bombing — the same report may be retried at most
   2 times, with >= 10 minutes between attempts.
5. Serialization: while the main agent is processing (WORKER_ORDERS.md
   carries a new unexecuted order), the worker must not report again —
   except trigger T2 (contract red line), which always stops and
   reports immediately, unrestricted.

## 13. HeightMap z-reference trace and frame unification (epoch16, 2026-08-29)

Trace (worker order 001, executed by the main agent):

- Publish side (simulate/src/unitree_sdk2_bridge.h): TerrainLidarLoop
  (TROT_SIM_LIDAR_CPU, SCHED_IDLE, 50 ms wall period) snapshots qpos,
  ray-casts the scene into a persistent world-frame grid
  (lidar_world_z_, 1.5 s memory), and publishes cells as
  world_z - base_link.xpos[2] evaluated at the snapshot.  The z
  reference is therefore re-locked to the true base height at EVERY
  publish; message stamp = snapshot sim time.  There is no startup
  latch and no first-scan origin.
- Consume side (trot_experiment_control.cpp): the message was copied
  verbatim into the planner work item; BuildTerrainModel used the raw
  cell heights with no re-referencing, while the swing-anchor guard
  compares against live encoder-FK feet in the current base frame.
- Measured lag: map ages 0.01-0.1 s (terrain_map_age_s), base rising
  0.2-0.3 m/s through the crux => z-reference mismatch ~= 8-15 mm.
  The earlier "map z-origin ~= world 0.365 vs base_z 0.417 (52 mm)"
  estimate is RETRACTED: it mis-paired the cell reading with a flat
  ground-truth assumption.  Decomposed in world frame, the 38.7 mm
  epoch15a anchor gap is ~= 29 mm anchor-cell content (the start
  anchor stands inside the riser's own 5 cm cell, which the sparse
  sweep fills with the riser top ~= +0.05 world, vs the flat-standing
  foot origin ~= +0.021 incl. the ~20 mm foot-site offset) plus
  ~= 8-15 mm z-reference lag.

Fix (frame unification, minimal, flat-invariant):

- example/cpp/terrain/terrain_model.h: RereferenceHeightMapZ()
  shifts every finite cell by -dz, NaN cells preserved.
- example/cpp/trot/trot_experiment.h: base_height_history_ member
  (2 s window of (state_stamp_s, base_position_world.z)).
- example/cpp/trot/trot_experiment_control.cpp: InterpolatedBaseHeight()
  helper; history push at planner-input build; at the map copy the
  cells are shifted by base_z(now) - base_z(map.stamp).  On flat
  ground base_z is quasi-static, dz ~= 0, behavior unchanged.
- Test: RereferenceHeightMapZ cases added to
  example/cpp/tests/test_terrain_interfaces.cpp.  ctest 27/27 green.

Canaries (18 s / 35 s, scene phase2_step_5cm.xml, domain 229, same
params as epoch15):

- b1_zref_epoch16_20260828: down at t~=6.75 (base_z < 0.25), base_x
  at fall 0.522 m (epoch15 r1: 7.27 / 0.043 m; r2: 7.05 / 0.381 m),
  dominant crux reject still swing_clearance (1274 rows vs 1034/1119),
  reachability 12, prepared 3 vs 1/2.
- b1_zref_epoch16_diag_20260828 (TROT_TERRAIN_DEBUG_SWING=1): the
  re-reference is confirmed engaged — during the terminal fall
  (base dropping ~1 m/s, map age 0.028 s) the printed anchor cell
  reads exactly flat world 0.000 in the CURRENT base frame
  (terrain0=-0.307220 at base_z~=0.307); unshifted it would have
  lagged by ~8 mm.  64 anchor rejects recur but only POST-fall
  (foot genuinely below the ground plane while toppling).

Verdict: the frame fix works as designed but clears only the minor
term; swing_clearance remains the dominant crux reject.  This
falsifies "z-reference lag is the primary anchor-guard poison" and
promotes the next hypothesis (section 11 item 1): the anchor guard
must not compare the FK foot origin against the single cell under the
anchor — use a patch-min / proprioceptive anchor reference.  Per
epoch15 finding 3, single canaries near the failure boundary are not
gating evidence; the epoch16 fall time sits inside the r1/r2 spread.

## 14. Anchor-guard neighborhood minimum (epoch17, 2026-08-29)

Complete explanation of the 38.7 mm anchor gap (closing the epoch15/16
thread): both guard operands are base-frame quantities, so no base_z
estimate enters the comparison at all.  start.z = encoder FK foot
(verified == MuJoCo truth, median 0) = foot_world - base = 0.021 - 0.410
= -0.389.  terrain[0] was the SINGLE cell at the anchor xy
(terrain_feasibility.h); the foot stands at the riser base inside the
riser's own 5 cm cell, which the sparse sweep fills with the riser
top (world ~= +0.05), so terrain[0] = 0.05 - base - reref_residual
~= -0.351.  Gap = (0.05 - 0.021) + z-ref residual ~= 29 + 8-10 mm.
No transform-chain bug anywhere: the content difference at the anchor
cell (quantization) plus the epoch16-fixed frame lag is the whole story.

Fix (chosen: patch-min over pure proprioception, to preserve the
genuine "start below ALL nearby ground" rejection): the i==0 anchor
sample now takes the minimum known cell height over the swept-radius
neighborhood (3x3 at 5 cm resolution) instead of the single cell
(terrain_feasibility.h, CheckSwingClearance).  A measured support foot
stands on the lowest local surface; comparing against the neighborhood
minimum is the proprioceptive fact expressed in map coordinates.  The
mid-path swept-volume, shin, and landing checks are untouched.

Tests: new case pins the epoch15a geometry (anchor in a riser-filled
cell, foot 38.7 mm below the cell but above the flat neighbors ->
accepted; anchor below the neighborhood minimum -> still rejected).
The pre-existing uniform-flat "low swing" rejection survives.
ctest 27/27 green.

Canary b1_anchor_epoch17_20260828 (params identical to epoch16, domain
229): the crux swing_clearance storm is GONE — dominant swing_clearance
rows 1274 (epoch16) -> 210, and those 210 all occur at t>=6.70 with
base_z already < 0.32 (post-fall-onset, a toppling robot reading real
penetration).  The exposed crux blocker is plan starvation by
kUnknown rejects: dominant unknown rows 117 -> 1245, plan publication
freezes at plan 220 from t~=6.2 while the map window stays fully known
(known_cells 320/320), so the unknowns are SamplePatch bound/all-known
failures along the swing/shin sampling path — masked until now by the
anchor reject firing first on every candidate.  Prime suspect: the
shin/knee segment samples of extended rear legs leaving the local
window (x < -0.45).  Outcome: down at t~=6.75, base_x at fall 0.195 m
(epoch15 r1/r2: 7.27/0.043, 7.05/0.381; epoch16: 6.75/0.522) — same
terminal class, inside the run-to-run spread; per epoch15 finding 3
this single canary is not gating evidence.

Next hypothesis: locate the exact kUnknown source in the swing
estimation pass (path SamplePatch vs shin segment vs IK-adjacent
sampling), then either clip sampling to the window or treat
out-of-window interior samples conservatively instead of rejecting all
32 regions.  B0 flat matrix still not re-run since epoch13 (T4 watch).

## 15. FOV-fringe vs in-window unknown (epoch18, 2026-08-29)

Mechanism of the epoch17 kUnknown starvation ("map fully known, 320/320
cells, yet unknown rejects"): TerrainPatch.all_known required
known_cells == total_cells AND map_edge_margin_m >= radius, so any swept
patch touching the grid boundary counted as unknown.  Two distinct
populations, separated by debug prints (TROT_TERRAIN_DEBUG_SWING,
caps raised 64 -> 512 after the first diag run exhausted its budget at
startup):

- Startup/stand-up: folded-leg knees exit the lateral window
  (y half-width only 0.225 m); harmless, pre-walk.
- Crux: the front-right foot drifts outboard to y ~= -0.20..-0.21
  while the robot slips at the riser base; the i==0 path patch at the
  anchor then fails the edge-margin gate and rejects all 32 regions
  with kUnknown before the epoch17 anchor patch-min even runs.  Plan
  publication froze (epoch17: plan 220 from t ~= 6.2) while the map
  stayed fully known.

The structural point: the lateral fringe is never observable by this
sensor window at any time, so treating FOV-edge cells as fatal unknown
makes planning impossible exactly when the feet drift sideways.

Fix (terrain_model.h + terrain_feasibility.h):

- TerrainPatch gains outside_cells (out-of-grid cells) and
  HasUnknownInside() (in-grid unobserved cells); SamplePatch counts
  them separately.  all_known semantics unchanged for the foothold
  gate.
- CheckSwingClearance first loop: the i==0 SamplePatch gate is removed
  (its result was always overwritten by the anchor neighborhood-min);
  the anchor block alone governs the start.  i>0 path samples reject
  only on HasUnknownInside() — FOV-fringe patches use the observed
  subset max.
- Estimation pass: foot interior samples same relaxation (reject only
  on in-grid holes; fully unobserved patches still reject); shin
  samples keep the epoch18 fringe skip when nothing is observed and
  reject only on in-window holes.
- Debug prints at every kUnknown site, env-gated, zero cost when off.

Tests (test_terrain_interfaces.cpp): CoversPatch boundary semantics;
narrow-window (32x10, y +/-0.225) rear-leg swing accepted; fringe
anchor at y=-0.21 accepted; 2x2 NaN occlusion hole inside the window
still rejects with kUnknown.  ctest 27/27 green.

Canaries (params identical to epoch16/17, domain 229):

- b1_unknown_epoch18_20260828: NO FALL over the full 18 s; plan
  publication stayed alive through the crux (pub 299 vs 220-231);
  dominant rejects: unknown 180 rows (border candidate cells), no
  swing_clearance or unknown storm.  But the run still FAILs the
  contract: the robot stalls at base_x ~= 0.52 m (riser at x=0.70),
  velocity tracking error 0.198 mps mean — a support-feasibility
  deadlock: support rejects with NEGATIVE margins (-0.08..-0.10) at
  knot 2, mask 9, COM driven ahead of the diagonal support line while
  the front feet remain on flat ground at x ~= 0.53.
- diag2 (shin-skip only, intermediate build): fell at 7.96 with
  swing_clearance still dominant (610) — confirms the path/anchor
  restructure, not the shin skip alone, cleared the storm.
- b1_unknown_epoch18_20260828_r2 (double-sample per epoch15 finding
  3): same shape — no fall, pub 300, dominant unknown 180 rows only,
  stall at base_x ~= 0.51 m.  Fix confirmed on two samples.

Verdict: kUnknown starvation fixed; the head blocker is now the
support-feasibility deadlock at the riser base (negative margins,
mask 9 knot 2) — the crossing itself, not the map plumbing.  Note the
stall prevented the fall, so "no fall" is not "crossed"; the step was
never climbed.

## 16. Handoff quick-start (2026-08-29)

- Current state: epoch16-18 DONE (z-reference unification, anchor
  patch-min, FOV-fringe vs in-window unknown); ctest 27/27 green; all
  work uncommitted on phase2-b1-b3 (10 modified files, this doc new).
- Head blocker: support-feasibility deadlock at the crossing — crux
  plans rejected at knot 2, mask 9 (FR+RL diagonal), margin
  ~= -0.08..-0.10 m; COM 10 cm past the support line while front feet
  stay on flat ground; robot stalls at base_x ~= 0.51 (riser x=0.70).
- Next hypothesis: why the 30 mm straddle corridor never activates —
  prime suspect is riser-edge mixed cells reading z ~= 0.02, evading
  the dz>30 mm trigger, so the crossing sequence never commits.
- Active instruction: Order 002 in
  example/cpp/experiments/_runs/WORKER_ORDERS.md (acceptance: file/line
  evidence + unit test + ctest 27/27 + double canary
  b1_support_epoch19_20260828{,_r2}).
- Red lines: never touch the contract or the analyzers; never commit;
  serial simulation only (CPU-pin env fixed, see _runs/run_trot.sh);
  never delete anything under _runs/.
