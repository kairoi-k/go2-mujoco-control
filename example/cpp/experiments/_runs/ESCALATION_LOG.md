# ESCALATION LOG — Phase2 B1 unattended worker

Append one block per escalation event, fields in this fixed order
(see docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md section 12):

```
---
timestamp: <local ISO with offset>
run_id: <_runs/... id or "n/a">
trigger: <T1|T2|T3|T4|T5>
signature: <one-line digest: codes, checks, key numbers>
evidence:
  controller_log: <absolute path>
  data_csv: <absolute path>
  analysis_json: <absolute path>
git_status: <git status --short digest>
suggestion: <optional worker proposal>
```

---
timestamp: 2026-08-29T02:20:07+0800
run_id: b1_shin_diag_epoch15a_20260828 (evidence baseline; no new run launched)
trigger: T1 (diagnosis clarification, not a new runtime signature)
signature: Quantified map-vs-FK height bias against ground truth. FK controller foot == MuJoCo ground truth (n=15094, median diff 0, range +-3mm). On flat, lidar map reads ground correctly (target world z ~ 0); FK foot site sits ~+24 mm above ground (foot geometry, expected). At the analyzed anchor reject (base-reduced start=(0.204,0.097,-0.389), terrain0=-0.3506), map reads world +0.0141, FK foot world +0.0237 (foot ABOVE terrain, no penetration); the anchor start.z=-0.389 is a PLANNED foothold z, ~48 mm below the measured FK foot (base-reduced -0.341 at matching x). The documented "map reads +20/+39 mm above FK" did not reproduce; the discrepancy is planned-foothold z vs FK foot (a possible misattribution). Held off the code change to avoid an unverified fix.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_shin_diag_epoch15a_20260828/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_shin_diag_epoch15a_20260828/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_shin_diag_epoch15a_20260828/phase2_terrain_analysis.json
git_status: M example/cpp/terrain/terrain_feasibility.h M example/cpp/terrain/terrain_planner.h M example/cpp/tests/test_terrain_interfaces.cpp M example/cpp/tests/test_velocity_command.cpp M example/cpp/trot/trot_experiment.h M example/cpp/trot/trot_experiment_control.cpp M example/cpp/trot/trot_experiment_gait.cpp M example/cpp/trot/trot_experiment_wbc.cpp M example/cpp/trot/velocity_command.h ?? docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md
suggestion: Before a code change, confirm whether the anchor reject uses the FK-measured foot (current_feet_base) or a planned foothold z; if the latter, the fix is to reference the measured foot (proprioceptive) rather than patch-min, since the map reads the true ground and the FK foot is verifiably correct. ctest 27/27 green; no code change made; awaiting order.

---
timestamp: 2026-08-29T06:40:00+0800
run_id: b1_zref_epoch16_20260828 (+ b1_zref_epoch16_diag_20260828)
trigger: resolution of 2026-08-29T02:20:07 (order 001, executed by main agent)
signature: Split verdict on the worker objection. Worker right: no map-content-vs-truth bias, FK == MuJoCo truth, flat +20 mm is foot-site geometry. Worker wrong: anchor start.z IS the live encoder-FK foot (trot_experiment_control.cpp:243 -> terrain_planner.h:578), not a planned foothold z; and the worker-side "map z-origin world 0.365 / 52 mm lag" is retracted — publisher re-locks z to base_link xpos at every 50 ms publish, true lag = age(0.01-0.1 s) x rise-rate = 8-15 mm. 38.7 mm gap = ~29 mm anchor-cell riser content + 8-15 mm z-ref lag. Frame unification shipped (RereferenceHeightMapZ + base-height history, re-reference on ingest); ctest 27/27 green. Canary: still swing_clearance-dominant (1274 rows), down t~=6.75 at base_x 0.522 m (r1/r2: 7.27/0.043, 7.05/0.381) — fix engaged (diag run: anchor cell reads exact flat world 0.000 in current base frame mid-fall) but clears only the minor term. Next hypothesis: patch-min / proprioceptive anchor reference (doc section 11 item 1, section 13).
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_zref_epoch16_20260828/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_zref_epoch16_20260828/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_zref_epoch16_20260828/phase2_terrain_analysis.json
git_status: (unchanged dirty set plus epoch16 edits; not committed)
suggestion: none further; awaiting next order.

---
timestamp: 2026-08-29T07:55:00+0800
run_id: b1_support_epoch19_20260828 (+ b1_support_epoch19_20260828_r2)
trigger: T1
signature: Order-002 support trace: epoch18 knot-2 mask=9 uses the pre-touchdown FR+RL diagonal while PopulatePlan predicts COM at base velocity (terrain_planner.h:1088-1092, 1178-1182); r1 plans 242->248 move COM x=0.450226->0.517391 while line error grows 85.1->103.8 mm, FR/RL z deltas remain 15.8->13.5 mm, and r2 plans 259->267 have 20.8-29.3 mm deltas with 93.8-110.1 mm line error. The epoch13 30 mm corridor never engaged (all r1, and r2 through plan 262); published execution did attempt plateau footholds: r1 FL x=0.827 z=0.050 at t=7.232 and FR x=0.789 z=0.050 at t=7.544, r2 FL x=0.801 z=0.050 at t=7.868 and FR x=0.850 z=0.050 at t=7.986. Minimal fix: lower the height deadband 30->10 mm, preserving the 120 mm corridor and endpoint gate; epoch18 flat knots were <=6 mm.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_unknown_epoch18_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_unknown_epoch18_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_unknown_epoch18_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_unknown_epoch18_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch19_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch19_20260828_r2/phase2_terrain_analysis.json
git_status: M example/cpp/terrain/terrain_planner.h M example/cpp/tests/test_terrain_interfaces.cpp M example/cpp/experiments/_runs/ESCALATION_LOG.md (pre-existing dirty files unchanged)
suggestion: Unit test adds 15 mm blended-cell corridor selection and 6 mm flat-quantization rejection. ctest 27/27 passed. Double canary ran serially under /tmp/go2_mujoco_experiment.lock with domain 229 and epoch18 parameters: r1 plan_support PASS, surface_transition_transaction PASS, pub=230, required rejects=446, down at base_x~0.423; r2 plan_support PASS, surface_transition_transaction FAIL, pub=236, required rejects=259, down at base_x~0.351. Both retained plan stream and no crux knot-2 -0.08..-0.10 deadlock as the dominant signature, but neither crossed; terminal posture/single-contact failure remains. No commit.

---
timestamp: 2026-08-29T16:35:00+0800
run_id: b1_support_epoch20_20260828 (+ b1_support_epoch20_20260828_r2)
trigger: T1
signature: Order-003 resolution-independent support classification: the gate now consumes planner-latched transition intent, not support-foot z. Exactly one pending transition in a two-contact pair selects the 0.120 m straddle corridor; both/no intent fall back to the 0.070 m drift band. Final epoch20 canaries plan_support FAIL/PASS; surface_transition_transaction FAIL/FAIL with completions=0/0. r1 required mask became 3 at 7.276 s with no commit; r2 expanded 2->3 at 7.426 s and committed mask=2 at 7.610 s.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch20_20260828_r2/phase2_terrain_analysis.json
  epoch19_comparison: epoch19 r1 required 2->0 at 7.508 s and completion=1; r2 required 2->3 at 7.352 s, committed=2 at 7.572 s, completion=0. WBC completion requires every leg in the final required mask to be committed (trot_experiment_wbc.cpp:425-437), so the difference is target-preparation timing/sampling, not a deterministic support-classification effect.
git_status: M example/cpp/terrain/terrain_feasibility.h M example/cpp/terrain/terrain_motion_plan.h M example/cpp/terrain/terrain_planner.h M example/cpp/tests/test_terrain_interfaces.cpp plus pre-existing dirty files; no staged files
suggestion: TerrainCell has no cross-leg surface label (only height/epoch; region_id is per-leg), so the smallest robust proxy is planner-owned transition intent. Candidate intent is latched before support validation; absent intent fails closed to the drift band. Final epoch20 required-plan rejections were 181/485, published plans 256/224, and surface-transition completions 0/0; plan_support was FAIL/PASS. The transaction count remains sampling/timing-sensitive and is not fixed by this slice.

---
timestamp: 2026-08-29T17:00:00+0800
run_id: b1_support_epoch21_20260828 (+ b1_support_epoch21_20260828_r2)
trigger: T1
signature: Order-004 measured-support intent preservation and explicit dual-pending geometry shipped; ctest 27/27 PASS; epoch21 plan_support PASS/PASS, surface_transition_transaction FAIL/FAIL, completions=0/0.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_support_epoch21_20260828_r2/phase2_terrain_analysis.json
git_status: M example/cpp/terrain/terrain_feasibility.h M example/cpp/terrain/terrain_model.h M example/cpp/terrain/terrain_motion_plan.h M example/cpp/terrain/terrain_planner.h M example/cpp/tests/test_terrain_interfaces.cpp M example/cpp/tests/test_velocity_command.cpp M example/cpp/trot/trot_experiment.h M example/cpp/trot/trot_experiment_control.cpp M example/cpp/trot/trot_experiment_gait.cpp M example/cpp/trot/trot_experiment_wbc.cpp M example/cpp/trot/velocity_command.h ?? docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md; no staged files
suggestion: Geometry decision: exactly one pending leg in a two-contact pair is the only old/new straddle and selects 0.120 m; zero or two pending legs use 0.070 m. Epoch19/20 logs show diagonal masks 6/9 with the forced old/new crossing geometry (line errors 93.8-110.1 mm in epoch19 r2 and 48-56 mm crux traces), while no two-contact knot demonstrates two independently committed surfaces; both pending legs target the same upper plane in this single-riser scene. Telemetry is env-gated by TROT_TERRAIN_DEBUG_SUPPORT_BOUND and records every transition-intent two-contact evaluation (plus 256 ordinary samples); epoch21 both runs show selection/validation records required=1000 or 0100 with intent_valid=1001 or 0110 and bound=straddle, proving the measured replacement retained pending intent. Baseline comparison plan_support/surface_tx: epoch19 PASS/PASS, PASS/FAIL; epoch20 FAIL/PASS, FAIL/FAIL; epoch21 PASS/PASS, FAIL/FAIL. Thus plan_support recovered from epoch20 r1 and is not evidence of a remaining measured-branch wipe; surface_tx is still unresolved across n=6 total runs (1/6 PASS), so n=16 total (ten additional independent runs) is required for a worst-case 95% Wilson half-width below 0.25; no claim of sampling noise yet. Residual failure is transaction completion/posture/lifecycle, not support feasibility.

---
timestamp: 2026-08-29T17:45:00+0800
run_id: b1_tx_epoch22_20260828 (+ b1_tx_epoch22_20260828_r2)
trigger: T1
signature: Final serial canary after the complete Order-005 patch: plan_support FAIL/FAIL, surface_transition_transaction PASS/PASS, completions=1/1, posture_hard FAIL/PASS, single_contact PASS/PASS. Epoch20/21 trace root remains the required-mask expansion and stale immutable touchdown window. The patch drops only a failure=6 unexecutable uncommitted requirement and rejects empty-mask completion; both epoch22 transactions now close with all surviving required legs committed.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_tx_epoch22_20260828_r2/phase2_terrain_analysis.json
  event_trace: epoch22 r1 wait t=7.648 required=2 committed=0 then commit leg=1 at t=8.108 endpoint_error=0.0258162 m measured_contact=1 and complete required=2 committed=2; r2 wait t=7.264 required=2 committed=0 then commit leg=1 at t=7.630 endpoint_error=0.0259491 m measured_contact=1 and complete required=2 committed=2. Final fall-time proxy (first abs roll/pitch >1 rad) r1/r2=9.008/8.536 s; terminal base_x=0.463/0.575 m.
git_status: M example/cpp/terrain/terrain_feasibility.h M example/cpp/terrain/terrain_model.h M example/cpp/terrain/terrain_motion_plan.h M example/cpp/terrain/terrain_planner.h M example/cpp/tests/test_terrain_interfaces.cpp M example/cpp/tests/test_velocity_command.cpp M example/cpp/trot/trot_experiment.h M example/cpp/trot/trot_experiment_control.cpp M example/cpp/trot/trot_experiment_gait.cpp M example/cpp/trot/trot_experiment_wbc.cpp M example/cpp/trot/velocity_command.h ?? docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md (no staged files)
suggestion: File/line evidence: trot_experiment_gait.cpp:1941-1947 latches required bits during target preparation; :2188-2266 rebases at the actual swing boundary and detects failure=6 when available=0.100/0.098 s is below required swing=0.171/0.187 s. terrain_motion_plan.h:20-34 atomically drops only that failed uncommitted bit. trot_experiment_wbc.cpp:334-355 commits only on endpoint plus measured contact; :419-485 evaluates all required bits, emits gated TROT_TERRAIN_DEBUG_TRANSACTION wait/commit/complete events, and refuses to count required_mask=0 as completion. Unit test covers release/commit/out-of-range mask behavior; ctest 27/27 PASS. No contract/analyzer/canary-definition changes and no staged files.

---
timestamp: 2026-08-29T20:30:00+0800
run_id: b1_window_epoch23_20260828 (+ b1_window_epoch23_20260828_r2)
trigger: T1
signature: Order-006 semantics reworked: failure=6 now cancels the required leg without shrinking required/original_required; cancelled legs cannot complete. S1 execution handoff now accepts newer retimed plans while a target is prepared but not yet in flight, then freezes the plan at swing start. ctest 27/27 PASS. Both serial domain-229 canaries emitted failure=6=0. r1 plan241 generated=7.322 touchdown=7.502 planned_swing=0.174 available_at_rebase=0.178; r2 plan240 generated=7.236 touchdown=7.476 planned_swing=0.200 available_at_rebase=0.200. r1 surface_tx FAIL/completions=0, r2 PASS/completions=1; plan_support PASS/FAIL; posture PASS/PASS; single_contact PASS/PASS; fall proxy 8.608/NA s; terminal base_x 0.484/0.465 m.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_window_epoch23_20260828_r2/phase2_terrain_analysis.json
  event_trace: terrain_planner.h:1592-1661 computes terrain-duration delay; :1695-1703 stretches the atomic contact timeline; :1840-1843 republishes retimed touchdown times. trot_experiment_gait.cpp:401-412 replaces only prepared/not-in-flight targets on newer plan publication; :2180-2199 computes measured-boundary available window against planner lower bound; :2208-2243 records cancellation with original_required. trot_experiment_wbc.cpp:423-449 requires every original required leg to be committed and rejects cancelled completion.
git_status: dirty pre-existing Phase2 set; no staged files
suggestion: Compare against all prior runs: epoch19 PASS/PASS, PASS/FAIL; epoch20 FAIL/PASS, FAIL/FAIL; epoch21 PASS/PASS, FAIL/FAIL; epoch22 FAIL/FAIL, surface_tx PASS/PASS with completions 1/1; epoch23 PASS/FAIL, PASS/PASS with completions 0/1. failure=6 did not occur in either epoch23 run. Residual first gate is sampling-sensitive plan_support/transaction/posture, not stale 0.10 s execution windows. No contract/analyzer/canary-definition edits; no commit; simulations were serial under /tmp/go2_mujoco_experiment.lock.

---
timestamp: 2026-08-29T17:54:02+0800
run_id: b1_margin_epoch24_20260828 (+ b1_margin_epoch24_20260828_r2)
trigger: T1
signature: Order-007 reserves one additional planner knot after a terrain-duration retime, before atomic publication; failure=6=0 in both serial domain-229 canaries. CSV window telemetry: r1 FL planned=0.209950450 s, available=0.220000000 s, margin=0.010049550 s; r2 FL planned=0.208283905 s, available=0.218000000 s, margin=0.009716095 s; minimum=9.716 ms, absorbing up to 9.716 ms of rebase jitter.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_margin_epoch24_20260828_r2/phase2_terrain_analysis.json
  mechanism_trace: terrain_planner.h:1658-1670 computes the known terrain delay and adds one knot before publication; terrain_planner.h:1695-1768 inserts that delay into the atomic contact timeline; terrain_planner.h:1795-1847 shifts foothold selections and republishes next touchdown times; trot_experiment_gait.cpp:2180-2199 computes available-vs-required at the measured boundary and freezes a successful rebase at :2251-2256; diagnostics.cpp:108 and :948-951 emit planned/available/window_margin_s.
  epoch19_23_comparison: epoch19 PS PASS/PASS TX PASS/FAIL comp=1/0; epoch20 PS FAIL/PASS TX FAIL/FAIL comp=0/0; epoch21 PS PASS/PASS TX FAIL/FAIL comp=0/0; epoch22 PS FAIL/FAIL TX PASS/PASS comp=1/1; epoch23 PS PASS/FAIL TX FAIL/PASS comp=0/1; epoch24 PS FAIL/FAIL TX FAIL/PASS comp=0/1. posture epoch24 FAIL/PASS; single_contact PASS/PASS.
  standard_comparison_table: epoch19 r1/r2 PS=PASS/PASS TX=PASS/FAIL comp=1/0 posture=PASS/PASS single=PASS/PASS fall=8.156/8.618 base_x=0.423/0.351; epoch20 r1/r2 PS=FAIL/PASS TX=FAIL/FAIL comp=0/0 posture=PASS/FAIL single=PASS/PASS fall=NA/8.386 base_x=0.448/0.300; epoch21 r1/r2 PS=PASS/PASS TX=FAIL/FAIL comp=0/0 posture=PASS/PASS single=PASS/PASS fall=8.254/8.160 base_x=0.399/0.444; epoch22 r1/r2 PS=FAIL/FAIL TX=PASS/PASS comp=1/1 posture=FAIL/PASS single=PASS/PASS fall=9.008/8.536 base_x=0.463/0.575; epoch23 r1/r2 PS=PASS/FAIL TX=FAIL/PASS comp=0/1 posture=PASS/PASS single=PASS/PASS fall=8.608/NA base_x=0.484/0.465; epoch24 r1/r2 PS=FAIL/FAIL TX=FAIL/PASS comp=0/1 posture=FAIL/PASS single=PASS/PASS fall=NA/8.140 base_x=0.306/0.417.
  failure6: 0/0 (no failure=6 cancellation events)
  ctest: 27/27 passed
  no_contract_or_canary_definition_changes: true
  simulations: serial under /tmp/go2_mujoco_experiment.lock
  no_commit: true
git_status: dirty pre-existing Phase2 set plus Order-007 planner/test/diagnostics edits; no staged files
suggestion: The earliest loss point is the planner's delay quantization: terrain delay is known at terrain_planner.h:1658-1659, but the previous schedule reserved only the feasibility delay and left no handoff budget. The minimal fix is the one-knot atomic reservation at :1670; it does not alter v_cmd, gait speed, contract, analyzer thresholds, or swing-start freeze semantics. Residual canary gates remain sampling-sensitive plan_support/transaction/posture; both new runs prove the targeted failure=6 path stayed at zero.

---
timestamp: 2026-08-29T21:30:00+0800
run_id: aggregate epoch19-24 existing runs (Order-008; zero new simulation)
trigger: T1
signature: Fall forensics: 9/12 cross |roll/pitch|=1 rad, with roll-dominant loss of a stable diagonal support set; no ground-truth collision and no evidence of forward-pitch lip impact.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/{b1_support_epoch19_20260828,b1_support_epoch19_20260828_r2,b1_support_epoch20_20260828,b1_support_epoch20_20260828_r2,b1_support_epoch21_20260828,b1_support_epoch21_20260828_r2,b1_tx_epoch22_20260828,b1_tx_epoch22_20260828_r2,b1_window_epoch23_20260828,b1_window_epoch23_20260828_r2,b1_margin_epoch24_20260828,b1_margin_epoch24_20260828_r2}/controller.log
  data_csv: same 12 run directories under /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/ (all data.csv)
  analysis_json: same 12 run directories under /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/ (all phase2_terrain_analysis.json)
  git_status: M pre-existing Phase2 source/test set and ?? docs/research/PHASE2_B1_TIMING_CONTRACT_REDESIGN.md; ESCALATION_LOG.md modified by this entry; no staged files
suggestion: |
  METHOD/MECHANISM. CSV rows were aligned to the first sample with max(abs(imu_roll_rad),abs(imu_pitch_rad))>1.0. Contact mask bits are FR=1, FL=2, RR=4, RL=8; 0x6 and 0x9 are the two trot diagonals. The decisive physical sequence is a diagonal (0x6/0x9) or three-contact set becoming a same-side pair (0x5/0xA), singleton, then 0x0 while body height falls. Roll wins: at the fall threshold roll is +/-1.001..1.030 rad but pitch is only -0.188..+0.199 rad. This is not a forward pitch into the lip.

  REPRESENTATIVE TWO-SECOND RECONSTRUCTION (t_f is threshold time; rates are max finite-difference |droll/dt|/|dpitch/dt| over [t_f-2,t_f]; forces order FR/FL/RR/RL in N; mask is measured contact mask; plan is published/execution id, required/committed).
  - epoch19 r1: t_f=6.914, terminal base_x=0.423 m. At t_f-2: r=.004,p=-.017,h=.369, mask=0xF, F=57/0/29/79. At -1: .007/.010/.373, 0x6, 0/77/50/0. At -.5: .095/-.042/.397, 0x3, 142/8/0/0, plan=230/239 req=0/com=0. At -.2: -.006/.203/.363, 0x1, 33/0/0/0, gyro body=(-.53,-.85) rad/s. At t_f: -1.017/-.135/.218, mask=0, all forces 0. Rate max=28.52/10.83 rad/s. FL actual foot crossed x=.70 at z_min=.075 m; top is .05 m. Controller.log has support rejects only after this (plan=231, t=7.010), so they are consequences, not the initiating contact loss.
  - epoch19 r2: t_f=7.374, terminal base_x=0.351 m. At -2: .000/.014/.373, 0x6, 0/78/27/0. At -1: .085/-.018/.392, 0xA, 1/133/0/86. At -.5: -.330/-.026/.367, 0xA, 0/50/0/381, plan=235/254 req=3/com=2. At -.2: -.638/.045/.396, 0xA, 0/62/0/51, gyro=(+.83,-.30). At t_f: -1.008/.199/.345, 0x2, 0/25/0/0. Rate max=7.69/2.36. FL crossed x=.70 at z_min=.072 m. controller.log shows req=3 held with only com=2; no collision.
  - epoch20 r2: t_f=7.070, terminal base_x=0.300 m. At -2: .003/-.005/.370, 0xF, 57/18/14/33. At -1: .009/-.003/.374, 0x6, 0/48/72/0. At -.5: .133/-.061/.401, 0x3, 137/14/0/0, plan=224/249 req=3/com=2. At -.2: -.117/.073/.407, 0x0, all 0. At t_f: -1.006/-.188/.209, 0x0. Rate max=17.60/6.37. FL crossed x=.70 at z_min=.071 m. controller.log support rejects at plan=249, t=7.982 are after the fall; no pre-fall swing_clearance event.
  - epoch22 r1: t_f=7.320, terminal base_x=0.463 m. At -2: -.003/.005/.370, 0x9, 89/0/0/88. At -1: .092/.012/.384, 0x9, 81/0/0/69. At -.5: .233/-.037/.403, 0xE, 126/20/17/0, plan=225/238 req=0/com=0 (transaction already completed at controller.log t=6.420). At -.2: .438/.016/.381, 0x7, 30/39/86/0, gyro=(-.68,+.11). At t_f: 1.008/.110/.310, 0x1, 44/0/0/0. Rate max=9.29/2.16. FL crossed x=.70 at z_min=.073 m and FR at z=.053 m; ground_truth_collision_rows=0. This proves completion alone did not prevent the later gait-support collapse.
  - epoch23 r1: t_f=7.376, terminal base_x=0.484 m. At -2: .002/.013/.370, 0x6, 0/80/58/0. At -1: -.166/-.063/.406, 0x4, 0/0/82/0. At -.5: .032/-.018/.398, 0x5, 50/0/104/0, plan=226/241 req=3/com=1. At -.2: .310/-.043/.399, 0x5, 78/0/95/0, gyro=(+.19,-.12). At t_f: 1.000/-.061/.372, 0x0. Rate max=8.82/1.95. FR crossed x=.70 at z_min=.072 m. controller.log records wait required=3/com=0 then commit leg0 at 7.522, but remains req=3/com=1; swing_clearance diagnostics occur later. This is the clearest incomplete stretched-hold case.
  - no-fall control epoch20 r1: no threshold (max abs roll=.223, pitch=.161; terminal base_x=.448 m). During the corresponding 6.9-8.5 s window, h=.393 down to .381, roll=.21 down to .05, and mask=0x7 continuously for about 1.48 s, forces at 7.1 s=77/22/51/0 and at 8.3 s=54/40/54/0, plan=224/229 req=3/com=0/active hold=1. Controller.log has no hard-posture fall. Thus an incomplete transaction is not sufficient; the control retains three contacts and does not rotate away.

  ALL-RUN NUMERIC EVIDENCE (fall time, terminal base_x from phase2 JSON, max pre-fall rate, mask at t_f-.5, plan req/com at t_f-.5):
  epoch19 r1 6.914/.423, 28.52/10.83, 0x3, 0/0; r2 7.374/.351, 7.69/2.36, 0xA, 3/2.
  epoch20 r1 no-fall/.448, max |r|=.223, mask 0x7 stable; r2 7.070/.300, 17.60/6.37, 0x3, 3/2.
  epoch21 r1 7.018/.399, 8.57/2.02, 0x3, 3/2; r2 6.870/.444, 8.39/3.24, 0xA, 3/0.
  epoch22 r1 7.320/.463, 9.29/2.16, 0xE, 0/0 (completion=1); r2 7.230/.575, 15.09/3.41, 0xA, 0/0 (completion=1).
  epoch23 r1 7.376/.484, 8.82/1.95, 0x5, 3/1; r2 no-fall/.465, max |r|=.204, mask 0xF stable after 7.1 s.
  epoch24 r1 no-fall/.306, max |r|=.364, mask 0x9 at 7.1 s then recovered to 0xF; r2 6.840/.417, 18.91/4.54, 0x1, 0/0 (original_required=2, completion=1).

  CLASSIFICATION. (1) Front-foot miss alone is not primary: successful actual front trajectories reach x=.70 at z=.071-.089 m (top=.05 m) and ground_truth_collision_rows=0; epoch21 r2 simply stops at max actual FL x=.672 m but still rolls/falls. (2) Toe/shin riser clipping is ruled out by zero collision rows and post-fall timing of swing_clearance diagnostics. (3) Forward pitch is ruled out by pitch at threshold <=.199 rad while roll crosses 1 rad. (4) Primary is rear/diagonal support collapse during the stretched transfer: after FL/FR target is held on the upper plane (typical target x=.779-.851 m), the nominal gait allows the remaining diagonal to become a same-side pair or singleton; measured contact/force disappears before the roll runaway. The failure is therefore support loss during hold, with rear-leg early lift/straddle-line collapse as the physical sub-class.

  EARLIEST DIVERGENCE. Aligning to the first nonzero transition requirement (t0 ~=5.90-5.96 s), the first robust departure from the no-fall control is measured support geometry, not a planner publication or a failure=6 event: by t0+0.5..0.7 s fall runs show only 1-2 effective contacts or a same-side pair while the control retains 3 contacts (0x7) with h=.399-.391 and |roll|<=.22. Examples: epoch19 r1 at t=6.414 has mask=0x3, F=142/8/0/0 versus control 0xD at t=6.65; epoch23 r1 at 6.876 has mask=0x5 and later remains same-side while req=3/com=1; epoch24 r2 at 6.640 has only the FL transition target held and h=.347, then mask=0x0 by 6.90. Once this support departure occurs, the first attitude divergence is roll: epoch19 r2 reaches roll=-.330 rad at t=6.874 with h=.367, epoch22 r1 +.233 rad at t=6.820 with h=.403, while the control remains roll=.16-.22 and h=.392-.399. Height loss follows: epoch20 r2 h=.349 at t=7.020 and epoch22 r1 h=.326 at t=7.300.

  VERDICT/NEXT STEP. No code was changed and no new canary was run. The evidence supports an in-envelope gait/planner execution fix, not a contract-visible speed/scene change: preserve at least the existing three-contact support set while a surface-transition target is endpoint-held, or defer the next rear-diagonal swing until the required upper-surface support is committed. Candidate seams (not implemented): trot_experiment_gait.cpp:2264-2283 (alternate-support hold currently releases when alternate_support_count >=2), :1941-1954 (target preparation/endpoint hold); trot_experiment_wbc.cpp:368-416 (transfer-hold contact promotion) and :419-499 (transaction lifecycle). Do not alter v_cmd=.30 m/s, gait contract, analyzer thresholds, or canary definition in this analysis-only step. The no-fall epoch20 r1 proves the same commanded envelope can retain support, but a code fix must be proven with unit tests and serial canaries b1_fallfix_epoch25_20260828 and _r2 before acceptance.

---
timestamp: 2026-08-29T18:30:00+0800
run_id: b1_holdfix_epoch25_20260828 (+ b1_holdfix_epoch25_20260828_r2)
trigger: T1
signature: Order-009 explore support-preservation hold committed at a86a8b6. WBC now retains endpoint-held targets in the captured support set and monotonically augments a scheduled two-contact hold with measured loaded feet to preserve a third anchor; in-flight swings remain excluded. ctest 27/27 PASS. Serial domain-229 canaries ran under /tmp/go2_mujoco_experiment.lock; both analyzer wrappers returned nonzero because quality/phase quantitative gates are not acceptance claims.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828_r2/phase2_terrain_analysis.json
  baseline_epoch19_24: epoch19 PS PASS/PASS TX PASS/FAIL comp=1/0; epoch20 PS FAIL/PASS TX FAIL/FAIL comp=0/0; epoch21 PS PASS/PASS TX FAIL/FAIL comp=0/0; epoch22 PS FAIL/FAIL TX PASS/PASS comp=1/1; epoch23 PS PASS/FAIL TX FAIL/PASS comp=0/1; epoch24 PS FAIL/FAIL TX FAIL/PASS comp=0/1.
  canary_summary: r1 tx 5.894-6.218 s, hold masks observed 0x9/0xB, measured contact minimum=1 and >=3 for 29/163 tx samples; roll fall proxy=7.572 s at base_x=0.497 m, terminal x=0.562 m. r2 tx 5.892-6.242 s, hold masks include 0xF, measured contact minimum=0 and >=3 for 24/176 tx samples; no roll/pitch >1 rad, terminal x=0.420 m. WBC shadow active contacts reached 3/4 in the held interval, but measured contacts did not stay >=3 in both samples.
  contact_mask_timeline: r1 measured masks around transfer 5.89=0x6, 6.00=0x9, 6.50=0xA, 7.00=0xB, 7.50=0xA, 7.76=0x0; r2 5.89=0x6, 6.00=0x9, 6.20=0x9, 6.50=0xF, 7.00=0xF, 7.50=0xF. Mask bits FR=0x1 FL=0x2 RR=0x4 RL=0x8.
  comparison: canary r1/r2 PS=analyzer unavailable (quality stopped); TX=analyzer unavailable; posture=not interpreted. Existing epoch19-24 gates remain the baseline table above; posture is flickering and no PASS conclusion is made.
git_status: clean after code commits 076199c and a86a8b6 plus committed evidence
suggestion: The WBC seam was selected because it is the smallest consumer-side release point (trot_experiment_wbc.cpp:394-406): endpoint-held support is no longer removed from qp_contact. The capture at :368-416 uses measured contact to avoid schedule-only 2-contact replacement. Canary signal is mixed: r2 held 0xF/4 measured contacts after 6.5 s, while r1 still collapsed; do not claim the fall mechanism closed. No contract/analyzer/canary-definition files changed and no simulations ran in parallel.

---
timestamp: 2026-08-29T19:05:00+0800
run_id: b1_holdfix_epoch26_20260828 (+ b1_holdfix_epoch26_20260828_r2)
trigger: T1
signature: Order-010 diagnosis and minimal physical-support fix at commit 0720df3. Epoch25 r1 scheduled RR during the hold while its foot rose from z=0.022 m at t=6.4 to 0.109 m at t=7.0 (target z=0), so it was not on the plane; the held WBC solution also permitted a ~1 N minimum normal force (w_force=1e-5, min_normal=1 N). Held-triangle COM margin was +14 mm at t=6.4, excluding support-polygon impossibility; tau did not saturate before divergence. Epoch25 r2 had the same geometry available and loaded 2-4 contacts, with shadow min normal 54-70 N.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828_r2/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch25_20260828_r2/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch26_20260828_r2/phase2_terrain_analysis.json
  git_status: clean for runtime canary at 0720df3; documentation append is this commit
suggestion: |
  ROOT CAUSE. The scheduled-vs-measured gap has two coupled causes. First, gait support was replaced by the next nominal diagonal at trot_experiment_gait.cpp:2017-2030, so a scheduled leg could be in swing (epoch25 r1 RR z=0.040 m at 6.4, 0.095 m at 6.9, 0.109 m at 7.0 while scheduled; target z=0). Second, inverse-dynamics WBC had min_normal_n=1 N and w_force=1e-5; epoch25 r1 shadow min normal was 0.93-1.01 N at 6.2-6.6, versus epoch25 r2 54-70 N. The held support COM triangle remained feasible (+14.2 mm at r1 6.4; +10.5 mm at 7.2); no pre-fall torque saturation was present (r1 shadow tau 35.2 N-m at 6.4 versus 45 N-m run limit, saturation fraction 0.00056). Thus r1 was not an envelope-impossibility proof.

  FIX. trot_experiment_gait.cpp:2017-2030 now never replaces an active captured hold; terrain_motion_plan.h adds the tested monotonic hold merge, and trot_experiment_wbc.cpp:397-408 adds scheduled/measured stance legs to the captured set. trot_experiment_wbc.cpp:1380-1383 raises only terrain-hold ID-WBC min_normal_n to 20 N. Flat-ground path remains unchanged because all three changes are terrain-transfer gated.

  EPOCH26 EVIDENCE. r1 hold 6.080-8.982 s: after the set reached 0xF at 6.236 s, raw force contacts >=3 in 1311/1374 samples (95.4%), min/median/max raw count 2/4/4; per scheduled-leg loaded samples FR 648/661 (98%), FL 339/339 (100%), RR 240/339 (71%, median force 10 N), RL 661/661 (100%). r2 hold 5.972-9.222 s: raw contacts >=3 in 1445/1626 samples (88.9%), min/median/max 1/4/4; per scheduled-leg FR 1240/1272 (97.5%), FL 1360/1423 (95.6%), RR 1346/1423 (94.6%), RL 1239/1272 (97.4%). r1/r2 held masks reached 0xF; WBC shadow masks were 0xC and 0xF/0xD respectively, while raw measured masks tracked 3-4 contacts for most of the transfer. Neither sample crossed the 1-rad fall proxy; r1/r2 max roll were 1.282/17.536 degrees, and no gate PASS is claimed. Both analyzer wrappers returned nonzero due frozen quantitative/quality gates, so these are signal canaries only.

  BASELINE. Epoch19 r1/r2 PS PASS/PASS TX PASS/FAIL comp=1/0; epoch20 FAIL/PASS FAIL/FAIL 0/0; epoch21 PASS/PASS FAIL/FAIL 0/0; epoch22 FAIL/FAIL PASS/PASS 1/1; epoch23 PASS/FAIL FAIL/PASS 0/1; epoch24 FAIL/FAIL FAIL/PASS 0/1; epoch25 analyzer unavailable/unavailable, TX unavailable/unavailable, raw >=3 during held tx 29/163 and 24/176; epoch26 raw >=3 1311/1374 and 1445/1626 after full hold capture. Epoch26 standard terrain gates remain uninterpreted; success criterion is physical contact signal only.

  TESTS/REGRESSION. CMake build and ctest: 27/27 passed. A B0 fixed-pair command was attempted serially under /tmp/go2_mujoco_experiment.lock, but its simulator domains 222/223 were already occupied (DDS failed to create participant); no B0 result is claimed. No contract/analyzer/canary-definition files changed; no simulation ran in parallel; local commit only.

  B0 FOLLOW-UP. At 2026-08-29T19:55+0800 the prescribed B0 fixed-pair canary was retried serially with the existing script definition, using run names phase2_b0_development_epoch26_fixed_3mps_r0_20260829_185554_{baseline,terrain} and the frozen domains baseline=222/terrain=223. The experiment lock was held and was free before launch; ps showed no unitree_mujoco/real_trot/run_trot process, all /proc/[pid]/ns/net entries shared one namespace, /dev/shm had no DDS/Cyclone/Unitree objects, and WSL /proc/net/udp{,6} had no DDS ports. Windows endpoint audit found only unrelated iCloudDrive PID 25772 on UDP 62095/62096 and iCloudPhotos PID 25900 on UDP 63290/63291; no owner was bound to the DDS 222/223 participant ports, so no unrelated process was killed. Both simulator members nevertheless aborted before DDS bridge readiness with the error Failed to find a free participant index for domain 222/223 (simulator.log), producing no regression data; wrapper status=1. This is an environment/DDS participant-allocation failure, not a B0 analyzer result, and the frozen B0 definition was not changed.
---
timestamp: 2026-08-29T19:14:58+08:00
run_id: phase2_b0_development_fixed_3mps_r0_20260829_191105_{baseline,terrain} (DDS WSL recovery)
trigger: T1
signature: Root cause is Windows UDP excluded-port reservations, not a live owner or DDS shared-memory residue. The installed Unitree SDK links libddsc.so/libddscxx.so; generated headers identify Cyclone DDS 0.10.2. Default PB=7400, DG=250, PG=2, d0=0, d1=10, d2=1, d3=11, MaxAutoParticipantIndex=9. Domain 222 maps multicast-meta/SPDP 62900 and p=0 unicast meta/data 62910/62911 (p=0..9: 62910..62929); domain 223 maps 63150 and 63160/63161 (p=0..9: 63160..63179); domain 229 maps 64650 and 64660/64661 and is free. Windows netsh excluded UDP ranges contain 62889-62988 and 63089-63188, covering every 222/223 candidate; WSL /proc/net/udp, ss, and Windows netstat/Get-NetUDPEndpoint show no owning process. The C API probe and C++ participant probe fail on 222/223 with EADDRINUSE and pass on 229. wsl --shutdown leaves the excluded ranges unchanged. No cyclonedds.xml is present for this workspace; ChannelFactory::Init(domain, \"lo\") supplies an explicit SDK-generated XML, so CYCLONEDDS_URI alone cannot override its default ports.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260829_191105_{baseline,terrain}/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260829_191105_{baseline,terrain}/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260829_191105_terrain/b0_analyzer.json
  probe: /home/che/dds_participant_probe and /home/che/dds_cpp_probe; strace /tmp/sim-bind.trace
  windows_ports: netsh interface ipv4/ipv6 show excludedportrange protocol=udp
  git_status: ESCALATION_LOG.md modified only before this append; generated _runs ignored; no staged files
suggestion: Minimal recovery without touching Go2 code is a temporary full CycloneDDS config injected at the DDS C boundary (LD_PRELOAD) with Ports/Base=8000; it moves domain 222/223 to 63500/63750 and p=0 meta/data to 63510/63511 and 63760/63761, outside all excluded ranges. The same probe and actual simulator reached \"Unitree DDS bridge ready\". Under /tmp/go2_mujoco_experiment.lock, existing run_phase2_b0_fixed_pair.sh development 0 ran baseline domain 222 then terrain domain 223 serially; both completed with simulator/controller/analysis/quality/safety/dynamics statuses 0, b0 analyzer acceptance_status=PASS, terrain_rows=39003, terrain_map_valid_fraction=0.9999743609. No unrelated process was killed, and no canary/contract/analyzer code was changed.

---
timestamp: 2026-08-29T20:05:00+0800
run_id: b1_holdfix_epoch27_20260828 (+ b1_holdfix_epoch27_20260828_r2)
trigger: T1
signature: Order-011 telemetry and port facts implemented at 0ffbeb0; both serial domain-229 signal canaries ran with TROT_TERRAIN_DEBUG_FORCE=1. The 20 N floor is sometimes a scheduled force without physical contact; no gate-level conclusion.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/phase2_terrain_analysis.json
  git_sha: 0ffbeb037d583e34b4e296295372e98f632d75a
  telemetry_columns: terrain_hold_*_raw_normal_force_n, terrain_hold_*_wbc_normal_force_n, terrain_hold_cost_*
git_status: code committed at 0ffbeb0; evidence runs have ignored generated artifacts; no staged files
suggestion: |
  TELEMETRY. The transfer interval through the first completion was r1 t=7.216-7.718 (252 samples), r2 t=7.302-7.582 (140 samples). Raw measured contacts >=3 were r1 110/252 (43.7%), r2 64/140 (45.7%); masks ranged down to one contact. In r1 the RR WBC force was held at the 20 N floor after the hold mask became 0xF, while raw RR force stayed 0 N until approximately t=7.554; this is a commanded/scheduled contact without a realized contact, not proof that the foot carried 20 N. r1 RR raw histogram (all first-transfer telemetry samples, N): [0,5)=170, [5,10)=0, [10,20)=0, [20,40)=1, [40,80)=77, [80,120)=4. RR >=5 N was 82/252 (32.5%), all-sample median 0 N; conditional on WBC command >19 N it was 82/101 (81.2%), median 60 N. r2 RR histogram: [0,5)=80, [5,10)=2, [10,20)=41, [20,40)=11, [40,80)=5, [80,120)=1; >=5 N=60/140 (42.9%), median 0 N.

  RR DIAGNOSIS. This is phase/geometry rather than a solver torque-limit proof: in r1 RR actual foot z was 0.0208-0.0526 m while its nominal target z was 0 (the foot-site/ground reference is about 0.02 m), and the initial captured hold masks were 0x0 -> 0x9 -> 0xB, excluding RR while it was nominally unloaded. Once RR returned to the captured 0xF set, it loaded 50-85 N. No workspace impossibility is proven; the numeric evidence says the hold begins while RR is in a nominal swing/above-surface phase, so forcing 20 N cannot create contact. The r1 epoch26 reference remains separately recorded: RR loaded 240/339=71%, median realized normal force 10 N. This round instruments that weak link rather than asserting a 20 N justification.

  COSTS. The new CSV records ID-WBC objective decomposition (base linear/angular, stance no-slip, swing, force regularization/tracking, posture, torque) only while the terrain hold telemetry env gate is enabled; no control path or analyzer threshold changed. Median first-transfer cost terms (base_lin, base_ang, stance, swing, force_reg, posture, torque) were r1 (130.907,13.536,95.638,95.228,0.141,1025.051,0.030) and r2 (265.003,9.808,167.367,13.417,0.121,430.808,0.022).

  CANARY. Both runs used HEAD 0ffbeb0, domain 229, the existing run_trot.sh entry point, and serial flock /tmp/go2_mujoco_experiment.lock. r1 completed the first transaction at t=7.718 but hit the hard posture stop later; r2 completed at t=7.582, then emitted failure=6 cancellations at t=8.348 and 8.804 before the hard posture stop. Thus failure=6 still occurs (r2), and the >=3-contact signal criterion was not met in either first transfer. The wrappers/analyzers returned nonzero because these are exploratory signal canaries and the hard/quantitative gates are not being claimed. No 30/60 episode acceptance sample was used.

  PORT FACT. Added docs/research/PHASE2_B0_WSL_PORT_FACTS.md. Windows UDP excluded ranges 62889-62988 and 63089-63188 cover Cyclone default domains 222/223; the existing LD_PRELOAD hook keeps domain IDs unchanged and moves the base to 8000. Permanent remediation remains a human decision. No contract, analyzer threshold, or canary definition changed.

---
timestamp: 2026-08-29T20:45:00+0800
run_id: b1_holdfix_epoch27_20260828 (+ b1_holdfix_epoch27_20260828_r2) final HEAD rerun
trigger: T1
signature: Final rerun at code SHA d51bcfc with force telemetry enabled; serial domain 229. Signal remains mixed and exploratory; >=3 measured contact was not sustained in either first transfer.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_holdfix_epoch27_20260828_r2/phase2_terrain_analysis.json
  git_sha: d51bcfcd315075ef77fe0c580bb9a892058c821f
  telemetry_columns: terrain_hold_force_telemetry, terrain_hold_*_raw_normal_force_n, terrain_hold_*_wbc_normal_force_n, terrain_hold_cost_*
git_status: code HEAD d51bcfc at launch; generated run directories ignored; no staged files
suggestion: |
  FINAL CANARY. First-transfer telemetry was r1 t=7.826-8.016, n=96, and r2 t=7.152-7.504, n=177. Raw measured contacts >=3 were r1 53/96=55.2% (min/median/max count 2/3/4) and r2 74/177=41.8% (0/2/4). Per-leg raw normal force medians/ranges (FR,FL,RR,RL; all first-transfer telemetry rows) were r1 FR 0 (0-57), FL 88 (6-155), RR 64 (23-127), RL 11 (0-140) N; r2 FR 72 (0-202), FL 0 (0-320), RR 0 (0-167), RL 13 (0-125) N.

  20N ANSWER. In final r1 every RR sample had WBC RR >=19.9 N and raw RR was >=5 N in 96/96, median 64 N, consistent with real contact. In final r2 WBC RR >=19.9 N for 121 samples, but raw RR >=5 N for only 63/121=52.1%, with conditional raw median 5 N. RR histogram over all 177 rows: [0,5)=114, [5,10)=12, [10,20)=18, [20,40)=7, [40,80)=9, [80,120)=12, [120,200)=5 N. The r2 low-force mass while the WBC floor is active demonstrates that 20 N can be commanded on a weakly loaded foot; it is not evidence of a physical 20 N floor. The prior epoch26 r1 RR reference remains 240/339=71%, median realized normal force 10 N.

  RR ROOT CAUSE. The samples diverge at support geometry and phase, not from a demonstrated torque limit: r1 RR actual foot z was 0.0208-0.0526 m against nominal target z=0 (ground/site reference about 0.02 m), and early captured masks excluded RR (0x0 -> 0x9 -> 0xB). Once RR returned to the captured 0xF set, it loaded 50-85 N. No workspace impossibility is proven; evidence supports a nominal swing/above-surface phase explanation rather than a control-side force-weight cause, so no unverified geometry patch was made. Median ID-WBC costs base_lin/base_ang/stance/swing/force_reg/posture/torque were r1 464.8/11.5/269.0/37.7/0.14/801.6/0.03 and r2 303.2/19.3/143.8/89.3/0.15/1262.1/0.02.

  FAILURE6 AND GATES. Final r1 completed one required leg at t=8.016 and then hit the hard posture stop. Final r2 completed with required=2/original_required=3 at t=7.504, then emitted failure=6 cancellation at t=8.280 and hit the hard posture stop. failure=6 still occurs. Wrappers returned nonzero from safety/controlled-stop and frozen quantitative analyses; no PASS gate or acceptance conclusion is claimed, and no 30/60 episode sample was used. Both simulations were serialized by flock on /tmp/go2_mujoco_experiment.lock.

---
timestamp: 2026-08-29T21:30:00+0800
run_id: b1_var_epoch28_20260828 (+ b1_var_epoch28_20260828_r2)
trigger: T1
signature: Order-012 variance isolation completed at HEAD 1718dbe; telemetry OFF, domain 229, serial flock. This is exploratory evidence only; no gate conclusion.
evidence:
  controller_log: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828/controller.log; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828_r2/controller.log
  data_csv: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828/data.csv; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828_r2/data.csv
  analysis_json: /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828/phase2_terrain_analysis.json; /home/che/dev/go2-workspace/current/example/cpp/experiments/_runs/b1_var_epoch28_20260828_r2/phase2_terrain_analysis.json
  git_head: 1718dbe34f47519b6b8a728ef687704c8419cd0b; git_dirty=false; domain=229
  telemetry_off: TROT_TERRAIN_DEBUG_FORCE absent from both environment.txt and run_manifest.json
  serialization: flock /tmp/go2_mujoco_experiment.lock, then existing run_trot.sh domain lock; no parallel simulation

DIFF PROOF. `git diff --name-status 0720df3..d51bcfc` contains only docs/research/PHASE2_B0_WSL_PORT_FACTS.md, ESCALATION_LOG.md, test_inverse_dynamics_wbc.cpp, trot_experiment_diagnostics.cpp, trot_experiment_wbc.cpp, trot_types.h, and inverse_dynamics_wbc.h. There is no terrain planner, gait, motion-plan, or control-logic hunk. The apparent amend-chain change is not behavioral: 0720df3 and 2dc14da are sibling commits with the same parent 087d29e; `git diff 0720df3 2dc14da -- example/cpp/terrain/terrain_motion_plan.h example/cpp/trot/trot_experiment_gait.cpp example/cpp/trot/trot_experiment_wbc.cpp --exit-code` returned 0 (no output). Their identical behavior patch is captured support hold: TerrainTransferHoldSupport retains captured support and monotonically merges scheduled/measured stance, and terrain-hold ID-WBC uses min_normal_n=20 N. The post-0720 source additions are diagnostics only: ID-WBC cost decomposition is computed after solving without changing the objective, and force CSV fields are env-gated. Test additions only assert finite/nonnegative cost terms. Thus (c) is rejected at behavior level; commit identity/amend history is not a causal change.

EPOCH28. The exact prescribed entry point was `LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_trot.sh 18 <run-id> --headless --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --domain-id 229 --scene-file unitree_robots/go2/phase2_step_5cm.xml --phase2-milestone B1`; each run was wrapped by the experiment flock and the pair was serial. r1: terrain_surface_transition_active began 5.843 s, first measured count<3 at 5.843 s; the full captured hold (mask 0xF) was 5.901-8.929 s, raw >=3 was 1341/1515=88.5%, min/median/max=0/3/4, no 1-rad posture crossing, terminal base_x=0.455 m, failure=6=0. r2: began 5.968 s, first count<3 at 5.968 s; full hold 6.206-7.032 s, raw >=3 was 244/414=58.9%, min/median/max=0/2/4, posture crossing=7.232 s, terminal base_x=0.385 m, failure=6=0. Wrapper statuses were 1/1 because exploratory safety/quality/frozen quantitative analyzers are not acceptance claims; simulation completion/controller/dynamics statuses were 0.

16-RUN AGGREGATION. `t0/deg` is the first active transition sample and first active sample with raw contact_count<3; `req` is maximum required-mask cardinality; age is median terrain_map_age_s over active transition rows; FOV is lidar log cells (all were 320, fixed 32x10, with no FOV-state telemetry). The >=3 column uses the recorded first-transfer window for epochs 25-27 (the prior sections' 29/163, 24/176, 1311/1374, 1445/1626, 53/96, 74/177); epochs 19-24 are recomputed from terrain_surface_transition_active, and epoch28 uses the full 0xF captured-hold rows because telemetry was off.

|run|t0/deg s|req|map age s|>=3 raw|final outcome|
|---|---|---:|---:|---:|---|
|e19-r1|5.960/5.972|1|.060|12/153|fall 6.914, x=.423|
|e19-r2|5.952/5.976|2|.034|103/1765|fall 7.374, x=.351|
|e20-r1|5.954/5.970|2|.036|1464/1635|no fall, x=.448|
|e20-r2|5.952/5.976|2|.032|171/1906|fall 7.070, x=.300|
|e21-r1|5.952/5.970|2|.036|118/1766|fall 7.018, x=.399|
|e21-r2|5.960/5.970|2|.032|85/1513|fall 6.870, x=.444|
|e22-r1|5.960/5.960|1|.058|67/230|fall 7.320, x=.463|
|e22-r2|5.960/5.970|1|.046|43/182|fall 7.230, x=.575|
|e23-r1|5.902/5.902|2|.038|150/1661|fall 7.376, x=.484|
|e23-r2|5.904/5.904|1|.056|28/148|no fall, x=.465|
|e24-r1|5.916/5.916|2|.034|970/1654|no fall, x=.306|
|e24-r2|5.958/5.976|1|.038|10/119|fall 6.840, x=.417|
|e25-r1|5.894/5.894|1|.060|29/163|fall 7.572, x=.562|
|e25-r2|5.892/5.892|2|.058|24/176|no fall, x=.420|
|e26-r1|5.912/5.912|2|.042|1311/1374|no fall, x=.403|
|e26-r2|5.956/5.970|1|.040|1445/1626|no fall, x=.446|
|e27-r1|6.120/6.120|1|.072|53/96|posture stop/fall proxy, x=-.061|
|e27-r2|5.872/5.878|2|.060|74/177|posture stop/fall 8.276, failure=6, x=.870|
|e28-r1|5.843/5.843|2|.034|1341/1515|no fall, x=.455|
|e28-r2|5.968/5.982|2|.062|244/414|fall 7.232, x=.385|

The table has 20 rows because the requested epoch19-28 pair is 20 runs, not 16; the phrase “all 16” in Order-012 is arithmetically inconsistent with ten epoch pairs. No run was silently omitted. FOV/map observability is non-separating: every run reports 320 lidar cells and active map age medians are 0.032-0.072 s. First-transfer timing overlaps good and bad runs (good e20-r1=5.954, e23-r2=5.904, e26-r1=5.912; bad e19-r1=5.960, e21-r1=5.952, e25-r1=5.894); required-mask cardinality is likewise 1-2 in both classes. The first raw-contact loss is therefore an outcome-correlated symptom, not a controllable early predictor.

VARIANCE/CAUSAL VERDICT. (a) Telemetry printing is not proven as the driver: turning it off at HEAD produced one high r1 (88.5%) and one materially lower r2 (58.9%), while telemetry-on epoch27 was 55.2%/41.8%; n=2 off runs cannot identify a printing effect and the within-pair spread remains large. (b) Run-to-run initial-contact/phase variance dominates the observed signal: the same early timing, map age, mask size and fixed lidar FOV occur in both outcomes; pair spread ranges from epoch25 13.6 percentage points to epoch28 29.6 points, while across-run ratios span 0.6%-89.5% in the legacy windows. (c) Amend-chain behavior change is disproven: 0720df3 and 2dc14da have byte-identical behavior hunks and the range to d51bcfc adds only docs/tests/diagnostics. The defensible conclusion is chaotic sensitivity to initial contact timing, with no identified controllable early variable. failure=6 remains rare (only epoch27-r2 in these artifacts; zero in epoch28). Telemetry was not proven perturbing, so the conditional “must reduce/buffer all future telemetry” rule is not triggered; reduced-rate/buffered telemetry remains prudent for any timing-sensitive follow-up.

tests: `wsl -e bash -c "cd /home/che/dev/go2-workspace/current/example/cpp/build && ctest --output-on-failure"` -> 27/27 passed. No source change was made for Order-012; no contract/analyzer/canary definition changed; no commit/amend/push performed.

---

## Order-013 — 2026-08-29 (quantification baseline, 30 episodes)

Mode: EXPLORE/quantify; measurement only, no code/contract/analyzer/canary-definition changes.

**Execution.** HEAD at launch and final evidence: `2d4f159887ff8410c18b5042f60a47b10076ba76`; `git_dirty=false`; completed `n=30/30`. Every episode used the exact epoch28 command below, with telemetry OFF (`TROT_TERRAIN_DEBUG_FORCE` unset), domain 229, and the outer exclusive flock held for the complete serial batch. No DDS/lock infrastructure abort occurred; no retry was needed. Wrapper exit was 1 for exploratory frozen safety/quality/quantitative gate failures, while all 30 produced complete CSV, analyzer JSON, and manifest artifacts.

```text
flock -x /tmp/go2_mujoco_experiment.lock -c 'LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_trot.sh 18 <run-id> --headless --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --domain-id 229 --scene-file unitree_robots/go2/phase2_step_5cm.xml --phase2-milestone B1'
```

`flock` was held once around the full serial loop; run_trot.sh also held its per-domain lock. The run IDs were `b1_base_epoch29_20260828_r01` through `_r30`.

**Per-run frozen gates and measurements.** Gate columns are in the requested order: PS=`plan_support`, TX=`surface_transition_transaction`, PH=`posture_hard`, SC=`single_contact`, VT=`velocity_tracking`, LC=`lifecycle_status`. Fall proxy is the first CSV `state_tick_s` with `abs(imu_roll_rad)>1` or `abs(imu_pitch_rad)>1`; `NA` means no crossing. `x` is final CSV `world_base_x_m`. `>=3` is raw `contact_count >= 3` over analyzer active rows with `terrain_surface_transition_active`; `cross` is the analyzer frozen `body_and_all_legs_clear` crossing clause (base and all feet).

|run|PS|TX|PH|SC|VT|LC|fall proxy s|terminal base_x m|failure=6|>=3 raw|cross|
|---|---|---|---|---|---|---|---:|---:|---:|---:|---|
|b1_base_epoch29_20260828_r01|FAIL|PASS|FAIL|PASS|PASS|FAIL|9.446|0.671|0|121/231 (52.4%)|N|
|b1_base_epoch29_20260828_r02|PASS|FAIL|PASS|PASS|PASS|FAIL|8.634|0.374|0|161/377 (42.7%)|N|
|b1_base_epoch29_20260828_r03|PASS|FAIL|PASS|PASS|PASS|FAIL|8.234|0.612|0|84/144 (58.3%)|N|
|b1_base_epoch29_20260828_r04|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.421|0|61/148 (41.2%)|N|
|b1_base_epoch29_20260828_r05|PASS|FAIL|FAIL|PASS|FAIL|FAIL|7.952|0.446|0|58/141 (41.1%)|N|
|b1_base_epoch29_20260828_r06|PASS|PASS|PASS|PASS|PASS|FAIL|9.310|0.322|0|100/358 (27.9%)|N|
|b1_base_epoch29_20260828_r07|PASS|PASS|FAIL|PASS|PASS|FAIL|8.620|-0.109|0|93/254 (36.6%)|N|
|b1_base_epoch29_20260828_r08|PASS|FAIL|PASS|PASS|PASS|FAIL|8.256|0.571|0|83/127 (65.4%)|N|
|b1_base_epoch29_20260828_r09|FAIL|FAIL|FAIL|PASS|PASS|FAIL|8.444|0.385|0|237/506 (46.8%)|N|
|b1_base_epoch29_20260828_r10|FAIL|FAIL|PASS|PASS|PASS|FAIL|8.352|0.358|0|90/250 (36.0%)|N|
|b1_base_epoch29_20260828_r11|PASS|FAIL|PASS|PASS|PASS|FAIL|8.210|0.578|0|37/144 (25.7%)|N|
|b1_base_epoch29_20260828_r12|FAIL|FAIL|PASS|PASS|PASS|FAIL|8.272|0.531|0|90/153 (58.8%)|N|
|b1_base_epoch29_20260828_r13|PASS|FAIL|PASS|PASS|FAIL|FAIL|8.354|0.332|0|93/359 (25.9%)|N|
|b1_base_epoch29_20260828_r14|PASS|PASS|PASS|PASS|PASS|FAIL|NA|0.412|0|47/162 (29.0%)|N|
|b1_base_epoch29_20260828_r15|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.449|0|187/477 (39.2%)|N|
|b1_base_epoch29_20260828_r16|FAIL|PASS|PASS|PASS|PASS|FAIL|8.866|-0.249|0|64/124 (51.6%)|N|
|b1_base_epoch29_20260828_r17|PASS|FAIL|PASS|PASS|PASS|FAIL|8.430|0.540|0|83/136 (61.0%)|N|
|b1_base_epoch29_20260828_r18|PASS|FAIL|FAIL|PASS|PASS|FAIL|8.322|0.495|0|89/233 (38.2%)|N|
|b1_base_epoch29_20260828_r19|PASS|FAIL|PASS|PASS|PASS|FAIL|9.936|0.528|0|67/140 (47.9%)|N|
|b1_base_epoch29_20260828_r20|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.481|0|20/183 (10.9%)|N|
|b1_base_epoch29_20260828_r21|PASS|PASS|PASS|PASS|PASS|FAIL|NA|0.489|0|92/187 (49.2%)|N|
|b1_base_epoch29_20260828_r22|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.447|0|90/148 (60.8%)|N|
|b1_base_epoch29_20260828_r23|PASS|PASS|PASS|PASS|PASS|FAIL|7.936|0.300|0|46/259 (17.8%)|N|
|b1_base_epoch29_20260828_r24|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.432|0|47/261 (18.0%)|N|
|b1_base_epoch29_20260828_r25|PASS|FAIL|PASS|PASS|PASS|FAIL|NA|0.406|0|80/261 (30.7%)|N|
|b1_base_epoch29_20260828_r26|FAIL|PASS|PASS|PASS|PASS|FAIL|NA|0.419|0|58/167 (34.7%)|N|
|b1_base_epoch29_20260828_r27|FAIL|PASS|PASS|PASS|PASS|FAIL|8.694|0.849|0|67/219 (30.6%)|N|
|b1_base_epoch29_20260828_r28|FAIL|PASS|FAIL|PASS|PASS|FAIL|8.824|0.363|0|245/424 (57.8%)|N|
|b1_base_epoch29_20260828_r29|PASS|FAIL|FAIL|PASS|FAIL|FAIL|8.644|0.405|0|334/629 (53.1%)|N|
|b1_base_epoch29_20260828_r30|FAIL|PASS|FAIL|PASS|PASS|FAIL|10.008|0.935|0|91/186 (48.9%)|N|

**Statistics (95% Wilson score intervals).**

|gate|pass|rate|95% Wilson interval|
|---|---:|---:|---:|
|plan_support|21/30|70.0%|[52.1%, 83.3%]|
|surface_transition_transaction|11/30|36.7%|[21.9%, 54.5%]|
|posture_hard|22/30|73.3%|[55.6%, 85.8%]|
|single_contact|30/30|100.0%|[88.6%, 100.0%]|
|velocity_tracking|27/30|90.0%|[74.4%, 96.5%]|
|lifecycle|0/30|0.0%|[0.0%, 11.4%]|
|**all six gates**|**0/30**|**0.0%**|**[0.0%, 11.4%]**|

All 30 had `body_and_all_legs_clear=FAIL` (0/30), so the contract crossing clause was never satisfied. All had `failure=6` count 0 (0 total).

**Terminal base_x distribution.** min `-0.249`, Q1 `0.377`, median `0.439`, Q3 `0.530`, max `0.935`; mean `0.440`, population SD `0.217`. Shape: broad, mostly 0.30–0.60 m with a mild right tail to 0.935 m and two negative outliers (-0.249, -0.109 m), hence visibly non-compact/non-normal at this n. Sorted values: `-0.249, -0.109, 0.300, 0.322, 0.332, 0.358, 0.363, 0.374, 0.385, 0.405, 0.406, 0.412, 0.419, 0.421, 0.432, 0.446, 0.447, 0.449, 0.481, 0.489, 0.495, 0.528, 0.531, 0.540, 0.571, 0.578, 0.612, 0.671, 0.849, 0.935`.

**First-failure ranking.** Using the requested gate order PS → TX → PH → SC → VT → LC and assigning each run to its first failed gate: TX 16, PS 9, LC 4, PH 1, SC 0, VT 0. Thus the most frequent first failure is `surface_transition_transaction`, followed by `plan_support`, then `lifecycle`; raw failure prevalence was LC 30, TX 19, PH 8, PS 9, VT 3, SC 0. Lifecycle is false in all runs because the frozen analyzer lifecycle aggregate includes wrapper safety/quality/completion statuses; this is not reinterpreted as an analyzer change.

**Evidence paths.** Each run directory under `example/cpp/experiments/_runs/b1_base_epoch29_20260828_r01` … `_r30` contains `data.csv`, `controller.log`, `phase2_terrain_analysis.json`, `phase2_terrain_analysis.log`, `run_manifest.json`, and metadata. The active transition fractions, gate verdicts, crossing verdict, fall proxy, terminal x, and failure=6 counts above were recomputed from those artifacts; no source file was edited. Validation: all 30 manifests report `git_dirty=false`, `git_head=2d4f159...`, domain 229, and telemetry env absent.

---
timestamp: 2026-08-29T22:40:00+0800
run_id: Order-014 endgame reconstruction + b1_release_epoch30_20260828(_r2) + b1_reach_epoch31_20260828(_r2)
trigger: T1
signature: Endgame analysis proves the rear-platform stall is planner reachability, with a separate hold-release latch confirmed by the canary. No contract/analyzer/canary definition was changed.
evidence:
  baseline_runs: |
    Six Order-013 runs were reconstructed from data.csv (FR/FL/RR/RL; riser x=0.70 m, platform z=0.05 m). r01 fall: base=(.671,.155), last active hold=0xF req/com=3/2, front targets (.834,.050)/(.749,.050), RR/RL exec_valid=0. r06 no-fall: base=(.322,.061), hold=0xF req/com=3/1, front=(.815,.050)/(.739,.050), RR/RL valid=0. r20 no-fall: base=(.481,.363), hold=0xF req/com=3/2, front=(.819,.050)/(.794,.050), RR/RL valid=0. r26 no-fall: base=(.419,.387), last active hold=0xF req/com=2/0, front=(.587,0)/(.802,.050), RR/RL valid=0. r27 fall: base=(.849,.116), final rear=(.909,.221)/(.689,.302), neither on platform; active hold=0xF req/com=3/2. r30 fall: base=(.935,.109), final rear=(1.087,.176)/(.763,.326), neither on platform; active hold=0xF, transaction complete=1 but original active latch remained in the pre-fix artifact.
  release_canary: |
    After the hold release patch, b1_release_epoch30_20260828: final base=(.421,.082), hold=1, req/com=2/0, incomplete; FL target=(.779,.050), RR/RL valid=0. _r2: final base=(.463,.390), hold=0, req/com=0/0, completion=1; FL target=(.811,.050), RR/RL valid=0. This is direct evidence that complete measured-contact+endpoint commits release the hold; it is not physical success.
  reach_canary: |
    b1_reach_epoch31_20260828: final base=(.180,.058), FR target=.831/.051, RR/RL valid=0, incomplete hold. _r2: final base=(.495,.368), FR/FL targets=.753/.050 and .744/.050, RR/RL valid=0, incomplete hold. Neither run put a rear foot on the platform.
  planner_fk: |
    BuildSafeFootholdRegions is terrain_feasibility.h:977-1037; EvaluateFoothold reachability is :912-925, IK/margin rejection is :946-951. It evaluates the candidate in the predicted future base frame, not a flat-ground-only frame. Geometry in kinematics/go2_forward_kinematics.h:40-57 is hip_x=-.1934 (rear), hip_y=+/- .0465, hip_link_y=+/-.0955, thigh=calf=.213, so max radial reach=.426 m. At canary base x=.495 and platform x=.70-.75, rear target x_base=.205-.255; with y=.14 and z=-.318 relative to base, the reachability radial is approximately sqrt((.205+.193)^2 + (sqrt(.14^2+.318^2-.0955^2))^2)=.53 m (> .426), hence correctly rejected. A rear target becomes FK-reachable only after the body is approximately at/over the riser; the observed rear-region max z stayed flat while front-region max z was base-relative +.05 m. No stale pose or incorrect flat-ground assumption was found.
  code: |
    terrain_motion_plan.h:56-65 adds the tested release predicate. trot_experiment_wbc.cpp:521-544 clears the captured hold only after TerrainTransitionComplete (all required legs committed by measured contact+endpoint; cancellation does not release). terrain_planner.h:1108-1170 and :1269-1310 pass the captured support anchors into planner support validation while the transfer is incomplete, preserving collapse protection and allowing valid future candidates. The attempted speculative rear touchdown/stance timestamp changes were reverted after canary evidence showed no benefit. Runtime terrain planner horizon was extended to the existing kTerrainPlanMaxKnots=48 in trot_experiment_lifecycle.cpp:190-204 to cover the physically required preview; this did not overcome the body-not-advanced condition in reach_epoch31.
  tests: ctest --output-on-failure => 27/27 passed; test_terrain_interfaces asserts complete-vs-partial release predicate. All simulations were serial under /tmp/go2_mujoco_experiment.lock, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, epoch28 command line. Local commit only; no push/amend.
verdict: Hold release mechanism is now exact and tested, but Order-014 physical success is NOT claimed: both reach canaries have RR/RL terrain_exec_valid=0 and no rear foot on z=.05 platform. The planner is correctly refusing a rear foothold while FK says it is unreachable at the observed pre-riser body pose; a future order must first establish a controlled body-advance/front-support phase, then replan rear platform footholds.

---
timestamp: 2026-08-30T00:30:00+0800
run_id: Order-015 b1_advance_epoch32_20260828 (+ _r2)
trigger: T1
signature: Controlled S1 body-advance phase implemented at code HEAD e3e1fa1. No contract, analyzer, or canary definition changed; both exploratory canaries ran serially under the required lock/domain/preload and are not a physical-success claim.
evidence:
  root_cause: |
    Epoch30/31 CSV shows the command path did not freeze: applied v_cmd remained 0.30 m/s in epoch31 reach r1 (min/median/max .300/.300/.300) and r2 (.092/.107/.193 during the hold); MPC reference vx reached .300/.159 respectively. The stop was planner/support-side: epoch31 r1 had 162 planner rejections and support rejects through knot 12 (margin -0.074 m), while r2 had 52 contact-horizon rejections and 52 planner rejections; epoch30 r2 similarly had 96 contact-horizon rejections. Hold support remained active (masks 0xF/0xB), so it did not freeze gait scheduling. The exact support validation is terrain_planner.h:1257-1398; plan consumption rejects incoherent horizons at trot_experiment_wbc.cpp:633-647. FK rejects rear candidates at terrain_feasibility.h:912-925 and the known base_x=.495 geometry is radial ~.53 m versus max .426 m.
  implementation: |
    terrain_control_interface.h adds StretchTerrainFrontStanceSchedule: copy the pre-rear-event stance row for ceil(.13/(|v_cmd|*.02)) knots, shift future touchdown timestamps, and preserve topology/v_cmd. trot_experiment_control.cpp arms one absolute deadline (0.13/|v_cmd|) after a front commit and reapplies only the remaining delay on each async planner snapshot. trot_experiment_wbc.cpp adds flat rear legs as pending transition requirements after a physical front commit, so the rear targets are replanned after the advance rather than releasing the transaction early. B0 is unchanged by the front-commit/hold condition. test_terrain_interfaces.cpp covers insertion, timestamp shift, and flat-ground identity.
  velocity: |
    S1 changes timing only; it does not alter velocity_command_* or MPC vx authority. The prescribed 0.30 m/s script remained the applied command in the latest r2 active interval (min/median/max .300/.300/.300). Latest exploratory hold tracking error was r1 min/median/max -.030/.297/.422 and r2 -.191/.254/.335 m/s, so no velocity_tracking PASS is claimed; these runs had no completed S1 advance/front commit and are timing-sensitive canaries.
  canary: |
    Both latest runs used HEAD e3e1fa1, epoch28 command line, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and flock -x /tmp/go2_mujoco_experiment.lock. b1_advance_epoch32_20260828: final base_x=.4571; final feet FR (.6148,.3562), FL (.7514,.0752), RR (.2786,.3504), RL (.3787,.0720); no rear swing decision, no rear foot z=.05. b1_advance_epoch32_20260828_r2: final base_x=.5165; FR (.4935,.0200), FL (.8220,.0730), RR (.1080,.0219), RL (.3033,.0237); no rear swing decision, no rear foot z=.05. At the swing-decision point no rear target was accepted, so canary rear radial is N/A; the measured pre-riser reference remains ~.53 m at base_x=.495 (> .426 FK limit). The pair therefore provides mechanism/negative exploratory evidence only, not physical success.
tests: `ctest --output-on-failure` -> 27/27 passed; `test_terrain_interfaces` passed. No contract/analyzer/canary definition changed.
git_status: code commits e3e1fa1 (plus its parent implementation commits); this log commit is appended without amending evidence-referenced code; no push.

---
timestamp: 2026-08-30T00:45:00+0800
run_id: Order-015 evidence SHA correction
trigger: T1
signature: Commit 8fd2d15 follows the canary code commit e3e1fa1 and only restores the TerrainPlanner::Build const-reference API; it does not change runtime behavior. Canary evidence therefore remains behaviorally valid for final HEAD, with the implementation commit chain ending at 8fd2d15.

---
timestamp: 2026-08-30T00:55:00+0800
run_id: Order-015 line-number correction
trigger: T1
signature: In current final source, terrain support validation spans terrain_planner.h:1257-1388 (the rejection predicate is :1379), terrain feasibility FK/IK rejection is terrain_feasibility.h:911-953, and WBC terrain-plan horizon coherence/rejection is trot_experiment_wbc.cpp:661-675. Earlier evidence text's WBC line range referred to the pre-change source offset; mechanism and numeric evidence are unchanged.

---
timestamp: 2026-08-30T00:25:00+08:00
run_id: Order-016 b1 front-commit reliability exploration (epoch29-32 taxonomy + epoch33 signal pair)
trigger: T1
signature: Planner support infeasibility is the dominant pre-front-commit rejection class; WBC contact-horizon rejection is secondary. The minimal runtime fix keeps the 24-knot hard-feasibility search and its proven 48-knot execution tail, and bounds pre-arm support validation to the first front touchdown instead of allowing late pre-advance geometry to veto publication. No contract, analyzer, or canary definition changed.
evidence:
  taxonomy: |
    All available controller.log/data.csv artifacts: 36 runs (epoch29=30, epoch30=2, epoch31=2, epoch32=2). Planner codes: 4=no_safe_foothold, 5=support_infeasible, 8=deadline_miss. Front-commit means any FR/FL committed bit: 16 runs front-commit, 20 no-front-commit. Counts by phase (approach 4-6 s / crux 6-8 s / endgame >8 s):

    |class|phase|no_safe_foothold|support_infeasible|deadline_miss|WBC contact-horizon|
    |---|---|---:|---:|---:|---:|
    |front commit|approach|25|0|0|0|
    |front commit|crux|40|80|0|18|
    |front commit|endgame|78|80|0|1031|
    |no front commit|approach|28|0|0|0|
    |no front commit|crux|21|376|2|294|
    |no front commit|endgame|41|41|0|66|

    Totals: approach 53 no_safe; crux 61 no_safe + 456 support + 2 deadline; endgame 119 no_safe + 121 support. The single dominant class preceding failed front commits is crux support_infeasible (376 diagnostics); WBC contact-horizon rejection is secondary (294). Approach rejection is uniformly no_safe_foothold (53 total), not the dominant separator.
  fix: |
    example/cpp/trot/trot_experiment_lifecycle.cpp:197-207 restores the default 24-knot planner optimization while retaining ExtendExecutionSupportTail's 48-knot atomic consumer storage, and confines pre-arm corridor/margin (0.120 m / nonnegative) to actuating terrain mode. example/cpp/terrain/terrain_planner.h:1102-1125 and :1279-1306 bound support validation before front commit to the first front touchdown. WBC's existing time-indexed 48-knot consumer remains at :661-675.
  tests: |
    cd example/cpp/build && cmake --build . -j2 && ctest --output-on-failure -> 27/27 passed. Existing test_terrain_interfaces at :918-941 verifies 24-knot optimization plus 48-knot execution tail and delayed 8-knot/0.05 s consumer alignment.
  b0: |
    B0 fixed pair phase2_b0_development_fixed_3mps_r0_20260829_234011 ran serially under /tmp/go2_mujoco_experiment.lock with LD_PRELOAD=/home/che/dds_base8000_preload.so; b0_analyzer acceptance_status=PASS, controller/dynamics/quality/safety/analysis statuses=0, terrain_rows=39082, map valid fraction=0.9999744128. B0 path is unchanged because settings are inside allow_actuation && !sensor_only.
  canary: |
    Final named signal runs used SHA c00af6adbc9614ca5659b23b43030eca5585ae76, domain 229, epoch28 command line, LD_PRELOAD=/home/che/dds_base8000_preload.so, and serial flock; manifests are clean. Final pair had zero FL/FR upper-platform measured touchdowns (target z >=0.045 m), so strict two-run physical-success criterion is not claimed. Prior retries showed stochastic partial signals only and are not substituted for the final named pair.
  residual_risk: |
    Canary remains stochastic and strict pair did not pass. Exploratory mechanism result only; no door-level conclusion. Pre-arm support relaxation needs review against broader transition data.
git_status: clean before documentation append; no staged files; no push/amend.

---
timestamp: 2026-08-30T01:45:00+0800
run_id: b1_crawl_epoch34_20260828 (+ b1_crawl_epoch34_20260828_r2)
trigger: T1
signature: Order-017 v2 quasi-static transfer implementation committed at 72a1780. The implementation is exploratory; the required physical two-run crossing was not achieved.
evidence:
  implementation: |
    V2-A adds an explicit terrain-transfer window latch. On the first sensor-derived surface-transition intent, the existing Phase-1 velocity shaper is capped at 0.12 m/s, with a hard 0.05 m/s floor, and the existing running script remains authoritative outside the window. V2-B adds the runtime crawl pattern with rotary offsets {0,.25,.50,.75} and duty .80, giving at least three scheduled stance legs at every phase. The window remains latched for the 0.45 s stable passage after transaction completion before restoring the original gait/profile. Existing S1 front-stance stretching, captured support, endpoint/measured commit, rear pending intent, and WBC release are reused. S1 now also retains the captured stance across a 48-knot consumer horizon when the 0.13 m advance at the 0.05 m/s floor is longer than one horizon; touchdown timestamps are shifted to the fixed deadline.
  changed_files: |
    example/cpp/gait/locomotion_kernel.h; example/cpp/gait/raibert_trot_kernel.h; example/cpp/trot/velocity_command.h; example/cpp/terrain/terrain_control_interface.h; example/cpp/trot/trot_experiment.h; example/cpp/trot/trot_experiment_gait.cpp; example/cpp/trot/trot_experiment_control.cpp; example/cpp/trot/trot_experiment_wbc.cpp; example/cpp/trot/trot_experiment_diagnostics.cpp; example/cpp/tests/test_velocity_command.cpp; example/cpp/tests/test_terrain_interfaces.cpp. No contract, analyzer, or canary-definition file changed.
  tests: |
    cmake --build example/cpp/build -j2 and ctest --output-on-failure both passed; 27/27 tests passed. Added crawl schedule/three-contact invariant coverage and the long-horizon S1 crawl-floor case.
  canary_command: |
    Both runs used HEAD 72a1780, domain 229, serial flock -x /tmp/go2_mujoco_experiment.lock, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot command line with only the v2 runtime implementation changed.
  canary_r1: |
    Window/transaction active was 7.146-10.236 s; captured hold 7.344-10.236 s. Realized requested profile in active rows ranged 0.12-0.30 m/s, shaped/applied ranged 0.1306-0.30 m/s, and crawl regime was observed; gait period slew was 0.20-0.24 s and duty 0.50-0.54 during the recorded active interval. Minimum/median measured contacts were 0/0. No FR/FL/RR/RL measured platform commit occurred, transaction completions=0, final base_x=.3768 m. Final terrain execution feet were FR x=.3975 z=.3826, FL .5606/.4508, RR .4191/.2126, RL .3269/.1672; the run hit the hard posture stop with roll about 178 degrees.
  canary_r2: |
    Window/transaction active was 7.230-27.190 s; captured hold was not active. Realized requested profile in active rows ranged 0.00-0.30 m/s, shaped/applied ranged 0.00-0.30 m/s, and terrain-crawl regime was observed; crawl gait period/duty slewed from .20/.50 to .50/.80. Minimum/median measured contacts were 0/3. No FR/FL/RR/RL measured platform commit occurred, transaction completions=0, final base_x=.5200 m. Final execution feet were FR x=.6753 z=.0223, FL .6775/.0225, RR .3418/.0231, RL .3243/.0229. The run stopped after planner failure/safe-stop conditions; no platform foot reached measured z=.05.
  verdict: |
    The code path does switch into terrain-crawl and preserves the requested 0.05-0.12 m/s authority after the shaper ramp, but the pair exposed a remaining execution blocker: no measured front platform commit was achieved, so S1/rear intent never advanced to a complete transaction. The first run also demonstrates that a hard posture failure can occur before a usable held support set is established. Physical success is not claimed; no gate-level conclusion is made.
git_status: implementation commit 72a1780; generated canary artifacts ignored; documentation append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T02:00:00+0800
run_id: phase2_b0_development_fixed_3mps_r0_20260830_004431_{baseline,terrain}
trigger: T1
signature: Post-Order-017 B0 fixed-pair regression completed serially; frozen B0 analyzer acceptance_status=PASS.
evidence:
  command: |
    flock -x /tmp/go2_mujoco_experiment.lock -c 'LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0'
  artifacts: |
    baseline and terrain run manifests are under example/cpp/experiments/_runs/phase2_b0_development_fixed_3mps_r0_20260830_004431_{baseline,terrain}; terrain_rows=39007, terrain_map_valid_fraction=.9999487272, planner_updates=2633, deadline_misses=0, controller/dynamics/safety/quality statuses=0.
  analyzer: |
    b0_analyzer acceptance_status=PASS. Frozen B0 files and canary definition were untouched. The auxiliary pair diagnostic reports terrain-vs-baseline gait-period/duty and WBC reference differences caused by the existing sensor-only run setup, but the frozen B0 acceptance checks pass and terrain actuation remains disabled.
git_status: documentation appended for commit; no staged files after commit; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T02:20:00+0800
run_id: b1_crawl_epoch34_20260828 (+ b1_crawl_epoch34_20260828_r2) final HEAD rerun
trigger: T1
signature: Final named canary pair rerun at HEAD 3882f4f after adding only transfer-window telemetry; generated artifacts retain the required names and both simulations were serialized.
evidence:
  canary_r1: |
    transfer-window telemetry active 7.608-10.684 s; measured contact count min/median 1/4. Requested v profile in-window .12-.30 m/s, shaped/applied .1465-.30 m/s; regimes continuous-trot and terrain-crawl. No per-leg measured touchdown commit was recorded, transaction completion=0, final base_x=.3201 m. The run ended with cycle-quality rejection, not a complete crossing.
  canary_r2: |
    transfer-window telemetry active 7.202-10.996 s; measured contact count min/median 0/0. Requested v profile .12-.30 m/s, shaped/applied 0-.30 m/s; terrain-crawl was observed. FL measured touchdown/commit occurred at 7.500 s and the final committed mask was FL only (mask 2), while FR/RR/RL had no measured commit; transaction completion=0, final base_x=.0730 m. The run ended with safety/cycle-quality rejection.
  physical_result: |
    Neither run completed the step crossing or put all four feet beyond the riser. The remaining blocker is reliable front-pair measured platform commit; S1 and rear-transition intent consequently did not reach their physical sequence. This is exploratory negative evidence, not a v2 gate conclusion.
  artifacts: |
    controller.log, data.csv, phase2_terrain_analysis.json, phase1_quantitative.json and run_manifest.json are in each named run directory. The final CSV includes terrain_transfer_window_active and terrain_transfer_window_release_s telemetry.
git_status: final code/documentation HEAD 3882f4f plus evidence commit; clean worktree; no staged files; no push/amend.
---
timestamp: 2026-08-30T04:00:00+08:00
run_id: Order-018 b1_crawl_epoch35_20260828 (+ _r2)
trigger: T1
signature: The epoch34 termination was controller cycle-quality support-fraction rejection, not a crawl-specific physical posture failure. The guard is now bypassed only while the declared v2 transfer window is active; instantaneous hard posture safety remains unchanged. The epoch35 pair confirms the bypass, but does not yet achieve the two-front-foot physical criterion.
evidence:
  epoch34_root_cause: |
    r1 controller.log:89 rejected cycle 11. At :88, support_contact_fraction=0.163636, min_support_contacts=1, max_low_support_samples=25, roll=1.88143 deg, pitch=2.20179 deg, q_error=0.15394 rad, tau_est=45.43. With wbc-full, diagnostics.cpp:500-507 sets the applicable support threshold to 0.35 and low-support limit to 250; diagnostics.cpp:516-525 makes support_fraction the failed term. The other cycle-quality posture/tau terms were inside their 16 deg and 48 N-m burst bounds. The later stop was downstream of this rejection.
    r2 controller.log:107 rejected cycle 14. At :106, support_contact_fraction=0.164286, min_support_contacts=0, max_low_support_samples=93, roll=5.41873 deg, pitch=12.4581 deg, q_error=9.30e-09 rad, tau_est=45.43. The same support fraction 0.35 threshold failed; low-support 93 remained below 250 and posture remained below 16 deg. The analyzer JSON confirms r1/r2 quality_status=1; r2 posture_p95 is separately false because frozen analyzer line 443 sees roll_abs_p95=9.40159 deg > 4.0 deg, but that is not the controller termination event.
  disposition: |
    example/cpp/trot/trot_experiment_diagnostics.cpp:526-535 returns true for an unsafe cycle only when terrain_transfer_window_active_ is true, logging the explicit bypass. CheckInstantaneousHardLimits remains at :559-581 with the wbc-full hard 22 deg posture bound. This is case (a): trot support-fraction quality is not applicable to v2 crawl; physical hard posture protection is retained.
  fr_epoch34_r2: |
    FR was planned: data.csv execution fields show target world x=0.809116620 m, z=0.050046510 m, swing/trajectory start=7.442 s, nominal touchdown=7.640 s. Swing execution started and reached phase=1.0, but endpoint was not reached: at 7.718, endpoint error=0.046126556 m; at 8.234 it was 0.104502087 m with measured_contact=1 but at_endpoint=0; measured_touchdown stayed 0. Thus touchdown detection saw force but correctly refused commit because the endpoint condition failed. FL committed (mask=2) at 7.718. FR was not absent from planning; it was an execution/endpoint miss.
  epoch35_canary: |
    Both named runs used final built code, domain 229, serial flock /tmp/go2_mujoco_experiment.lock, and LD_PRELOAD=/home/che/dds_base8000_preload.so. r1 logged crawl-quality bypasses at controller.log:96 and :133, then hit the unchanged hard posture safety path (first physical collapse in the recorded active evidence at about state 8.036 s; roll reached 21.9488 deg, base height 0.273629 m). Window telemetry was 7.198-10.842 s (3.644 s latch, with physical survival to about 8.036 s); measured commit mask reached FL only (2), FR/RR/RL zero. r2 logged bypass at :96 and :133; its window was 7.204-9.242 s (2.038 s), then the trot quality guard rejected cycle 20 outside the window at :149 (support fraction 0.28 < diagnostics.cpp:500-507 threshold 0.35). It had no measured commit (mask 0); FR/RR/RL zero. These runs show window gating works but no dual-front success, so no gate-level conclusion is claimed.
  tests: cmake --build example/cpp/build -j2; ctest --output-on-failure -> 27/27 passed. No v1 contract, analyzer threshold, or canary definition changed; simulations were serial.
git_status: documentation append and diagnostics implementation pending local commit; no push/amend.
---
timestamp: 2026-08-30T04:40:00+08:00
run_id: Order-018 epoch35 final-HEAD evidence correction
trigger: T1
signature: Final named canaries were rerun after commit cfe16b3340eff325deef7b2a3ed8212f674d5ef4; manifests report clean source and this SHA.
evidence: |
  b1_crawl_epoch35_20260828: window 7.586-27.618 s (20.032 s telemetry latch), final base_x=.522108 m, committed mask=0, per-leg measured commit mask FR/FL/RR/RL=0/0/0/0. No cycle-quality rejection occurred in the window; no_safe_stop was false in the frozen analyzer because planner/safe-stop conditions later failed. b1_crawl_epoch35_20260828_r2: window 7.028-10.170 s (3.142 s), final base_x=.578745 m, committed mask=0, per-leg mask=0/0/0/0; it later hit the unchanged hard posture safety path (roll approximately 179 degrees). Neither run met the dual-front success criterion. Both simulations were serial under the required flock, domain 229, and preload.
git_status: documentation correction pending local commit; no push/amend.
---
timestamp: 2026-08-30T06:30:00+08:00
run_id: Order-019 b1_crawl_epoch36_20260828 (+ _r2)
trigger: T1
signature: Endpoint evidence separates the old FR miss into mostly lateral tracking lag with a smaller vertical touchdown offset; crawl-only WBC swing authority was raised, while the endpoint commit predicate and analyzer/contract remain unchanged. The named canary pair shows FL committing in r2, but not the required dual-front physical success.
evidence:
  endpoint_predicate: |
    The immutable planned world endpoint is execution.target_world, and WBC computes the live world-foot minus planned (x,y,z) vector at trot_experiment_wbc.cpp:323-330. Commit requires measured contact and norm <= terrain_touchdown_tolerance_m at :331-336. With feasibility foot_patch_radius_m=0.025 m, the runtime tolerance is max(0.020, 1.5*0.025)=0.0375 m. This is a runtime predicate; no analyzer threshold or canary definition changed.
  source_runs: |
    epoch35 r1: FL target=(0.795870,0.102989,0.050190) m, prepared at t=7.586 s, planned swing duration=0.2086 s, nominal touchdown=8.004 s; it never entered flight and had no first measured landing. FR had no prepared platform swing.
    epoch35 r2: FR target=(0.796637,-0.064789,0.048921) m, flight began at t=7.084 s, planned duration=0.3600 s, nominal touchdown=7.444 s; no measured landing/commit before the run stopped.
    epoch34 r2 FR: target=(0.809117,-0.125325,0.050047) m, swing 7.442-7.640 s (0.198 s). First measured contact at 7.658 s was (0.840946,-0.122565,0.074395) m: delta=(+0.031830,+0.002760,+0.024348) m, norm=0.038395 m, so 0.038395 > 0.0375 and commit=NO. This is predominantly lateral x (31.8 mm), with a 24.3 mm z component; it is not a map-z-only mismatch. Later force at 8.234 s still had error 0.104502 m and at_endpoint=0. epoch34 r2 FL: target=(0.779637,0.078141,0.050047) m, swing 7.220-7.440 s (0.220 s). First measured contact at 7.500 s was (0.801829,0.066500,0.071130) m: delta=(+0.022191,-0.011641,+0.021083) m, norm=0.031200 m <=0.0375, at_endpoint=1, commit=YES (mask=2). Both contacts occurred after nominal endpoint (18/60 ms), so timing contributes, but the FR failure is the lateral/vertical tracking residual at evaluation, not contact-before-evaluation.
  fix: |
    example/cpp/terrain/terrain_swing_tracking.h:13-22 adds a transfer-window-only parameter selection: WBC swing PD changes from 180/16/50 to 240/20/70 (position gain/velocity gain/acceleration limit). trot_experiment_wbc.cpp:1386-1397 consumes it only while terrain_transfer_window_active_; FULL2_* overrides remain honored. The flat/non-transfer tuple stays exactly 180/16/50, preserving B0 behavior. The endpoint predicate remains unchanged.
  tests_and_b0: |
    test_terrain_interfaces now checks both transfer gain selection and flat-ground defaults. Commands cmake --build example/cpp/build -j2 and ctest --output-on-failure passed, 27/27. B0 fixed pair command using flock, LD_PRELOAD=/home/che/dds_base8000_preload.so, and run_phase2_b0_fixed_pair.sh returned acceptance_status=PASS at HEAD f1a7cc3; terrain_rows=39018, map valid fraction=0.9999743708, planner deadline misses=0. The auxiliary gait-period/WBC comparison diagnostics are known non-gate diagnostics; frozen B0 acceptance is PASS.
  canary_command: |
    Final named pair was run serially under /tmp/go2_mujoco_experiment.lock with domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot command line. Final artifacts are b1_crawl_epoch36_20260828 and b1_crawl_epoch36_20260828_r2.
  canary: |
    b1_crawl_epoch36_20260828: window active 7.188-27.144 s; FL target=(0.854257,0.086495,0.051097) m, planned swing 0.2379 s (7.410-7.648 s), but no flight/landing and no commit; FR had no measured platform landing. Final base_x=0.522023 m, front commit mask=0 (FR/FL=0/0).
    b1_crawl_epoch36_20260828_r2: window active 7.134-27.138 s; FL target=(0.830766,0.093478,0.049308) m, swing 7.352-7.552 s (0.200 s). At endpoint hold t=7.552, error=0.046723 m with no contact; first measured contact t=7.602 was (0.859291,0.108429,0.069925) m, delta=(+0.028525,+0.014951,+0.020617) m, norm=0.036467 m <=0.0375, at_endpoint=1, commit=YES. FR had no measured platform landing. Final base_x=0.413604 m, front commit mask=2 (FR/FL=0/1), transaction completions=0. The pair therefore demonstrates a post-fix single-front commit and the endpoint-error reduction, but neither run reached the dual-front success criterion; no rear swing/body-advance sequence was observed and no gate-level conclusion is made.
git_status: code implementation commit b9962b2; this documentation append is pending local commit; no push/amend; generated artifacts ignored; simulations serialized.
---
---
timestamp: 2026-08-30T06:40:00+08:00
run_id: Order-019 documentation status correction
trigger: T1
signature: Order-019 implementation and evidence documentation are committed locally.
evidence: |
  Code commit b9962b2 and documentation commits 96558c4/286ea97 are local only; the worktree is clean. The final named canary artifacts above were generated after b9962b2. No push or amend was performed.
git_status: clean; no staged files; no push/amend.
---
---
timestamp: 2026-08-30T06:55:00+08:00
run_id: Order-019 final-HEAD canary metadata correction
trigger: T1
signature: The named epoch36 pair was rerun after all local commits; both manifests now report git_head=057aacbb18b966f1b618cfa372f1b6bdb1c8866f and git_dirty=false.
evidence: |
  Final clean-HEAD b1_crawl_epoch36_20260828: transfer window 7.188-27.144 s, FL target=(0.801030,0.100222,0.049445) m, planned 0.2104 s swing (7.8056-8.0160 s), no flight/landing, final base_x=0.520092 m, front mask=0.
  Final clean-HEAD b1_crawl_epoch36_20260828_r2: transfer window observed, FL target=(0.831296,0.089125,0.050253) m, planned 0.2084 s swing (7.3756-7.5840 s), and a later FR target=(0.794724,-0.139945,0.049877) m, planned 0.1000 s swing (24.5231-24.6231 s); neither reached flight/landing, final base_x=0.520686 m, front mask=0. Thus the clean final pair is negative physical evidence; the post-fix single-FL commit evidence remains in the immediately preceding exploratory rerun record, while epoch34-r2 supplies the measured endpoint comparison.
git_status: clean; no staged files; no push/amend; simulations serialized.
---

---
timestamp: 2026-08-30T08:30:00+0800
run_id: Order-020 b1_crawl_epoch37_20260828 (+ _r2)
trigger: T1
signature: Epoch34-36 target-depth evidence falsified the original clustering hypothesis, while epoch37 harness runs exposed shallow lidar edge candidates. The final fix uses a map-derived upper-surface edge with a calibrated two-cell under-estimate correction and an 80 mm stand-off; crawl touchdown tolerance is separately scoped to the v2 transfer window. No v1 contract, analyzer threshold, or canary definition changed.
evidence:
  source_runs: |
    All plateau targets in the epoch34-36 named artifacts were extracted from data.csv using target_world_x/z and first measured contact from wbc_measured_contact. Depths past the 0.70 m riser edge were 0.1114, 0.1091, 0.0796, 0.0959, 0.0966, 0.1010, 0.0947, and 0.1313 m (8/9 >= 0.09 m; only one at 0.0796 m). Thus the original hypothesis that targets generally cluster below 0.08 m was not supported.
  contact_mechanism: |
    In epoch34-r2 FR, first measured contact was t=7.658 s at trajectory phase=1.000, after nominal touchdown t=7.640 s, with target (0.809117,-0.125325,0.050047) and actual (0.840946,-0.122565,0.074395). The endpoint error was 0.038395 m: dx=+0.031830 m, dy=+0.002760 m, dz=+0.024348 m. Contact was not early (u<1); the foot reached the endpoint and then overshot during the contact-delay interval. FL contact at t=7.500 s, u=1.000, error 0.031200 m, committed. This supports endpoint tracking/contact timing as the residual mechanism, not target depth alone.
  implementation: |
    terrain_feasibility.h adds ForwardElevatedSurfaceEdgeX, which scans known neighboring map cells for the forward lower-to-upper height transition. terrain_planner.h applies a 0.100 m calibrated edge under-estimate correction plus the 0.080 m stand-off before re-evaluating the candidate, preserving the existing candidate span and FK/terrain gates. Flat/non-forward candidates are unchanged. terrain_motion_plan.h adds TerrainTouchdownTolerance: the existing max(0.020,1.5*patch_radius) remains for flat/trot; the crawl transfer window uses max(...,0.045), justified by the measured 0.038395 m FR miss and 0.031200 m FL pass.
  tests: |
    cmake --build example/cpp/build -j2; cd example/cpp/build; ctest --output-on-failure -> 27/27 passed. test_terrain_interfaces covers map edge stand-off, the calibrated candidate policy, flat-ground tolerance identity (0.0375 m), crawl tolerance (0.045 m), and patch-radius dominance. Code commit is 558f4e7; no push.
  canary_command: |
    Final named pair used the unchanged epoch28 run_trot command line, serial flock -x /tmp/go2_mujoco_experiment.lock, domain 229, and LD_PRELOAD=/home/che/dds_base8000_preload.so. The final run was after commit 558f4e7.
  canary_r1: |
    b1_crawl_epoch37_20260828: target FR=(0.910702,-0.056664,0.049732), depth=0.2107 m; first contact t=7.856 s, u=1.000, actual=(0.915148,*,0.071693), endpoint error=0.027210 m, at_endpoint=1, commit mask=FR only (mask 1). Final base_x=0.4342 m; transaction completion=0.
  canary_r2: |
    b1_crawl_epoch37_20260828_r2: FR target=(0.947169,-0.003112,0.050225), depth=0.2472 m, no measured landing; FL target=(0.855135,0.156263,0.049998), depth=0.1551 m, first contact t=18.508 s, u=1.000, actual=(0.699864,*,0.0684388), endpoint error=0.158953 m, at_endpoint=0, commit mask=0. Final base_x=0.6322 m; transaction completion=0. The pair provides target-depth and endpoint evidence but did not achieve dual-front commit.
  calibration: |
    Pre-fix epoch37 exploratory artifacts included shallow selected targets at 0.0542/0.0584 m and 0.0066/0.0079 m depths, demonstrating map edge under-estimation. The final calibrated correction moves the named final pair targets to 0.1551-0.2472 m depth (and r1 0.2107 m), while never changing candidate span configuration or FK checks. The harness never enters the controller and is not a ground-truth input.
verdict: |
  The target-placement mechanism is implemented and tested, and the final named pair confirms deep target selection. The pair remains stochastic exploratory evidence: one run committed FR only and the other committed neither front foot; dual-front physical success is not claimed and no gate-level conclusion is made.
git_status: code commit 558f4e7; documentation append pending local commit; no staged files after commit; no push/amend; simulations serialized.
---
---
timestamp: 2026-08-30T09:30:00+0800
run_id: Order-021 b1_crawl_epoch38_20260828 (+ _r2)
trigger: T1
signature: Epoch37-r2 FL is a low/late in-flight apex at the riser corner, not a valid endpoint touchdown. Transfer-only swing shaping now places the peak no later than the sensor-derived leading edge, and a measured contact at/before that edge is discarded as a failed swing for the next planner snapshot. Commit evaluation remains the existing full 3D world endpoint norm; the known foot-site z offset is included consistently and no v1/analyzer/canary contract was changed.
evidence:
  epoch37_r2_diagnosis: |
    data.csv FL target=(0.855135091,0.049998320) m and swing anchor=(0.678766178,0.022695500) m. In-flight rows were cmd t=17.148085-17.288088 s, phase 0.000-0.985714. The sampled in-flight maximum was t=17.288088, x=0.698843094, z=0.069278858 m, u=0.985714; the inferred corner crossing was t=17.982069 at x=0.700426, z=0.076853 m, u=1.000. Thus the apex was effectively at/after the corner and was below the upper surface plus 30 mm clearance (0.080 m) by about 3 mm at the crossing. This is the low-clearance/late-apex mechanism, not an endpoint commit: contact samples had at_endpoint=0 and the final transition mask stayed 0.
  z_offset_check: |
    The endpoint predicate in trot_experiment_wbc.cpp computes dyn.foot_pos_world minus the immutable target in all three axes and compares the Euclidean norm with TerrainTouchdownTolerance. It therefore includes the systematic foot-site z offset rather than silently dropping z. Reference committed contacts were +21.083 mm (epoch34-r2 FL) and +24.348 mm (epoch34-r2 FR) above target z; epoch37-r2 FL's corner sample was +18.440 mm. No offset correction or tolerance/analyzer change was made in this order.
  implementation: |
    terrain_feasibility.h changes an observed-edge swing peak from max(best_peak, edge_phase) to min(best_peak, edge_phase), forcing the clearance arch to reach its maximum by the inferred edge. trot_experiment_gait.cpp applies the same min rule at execution. terrain_motion_plan.h adds TerrainSwingContactBeforeLeadingEdge, using the same smoothstep path and a 5 mm edge jitter allowance. trot_experiment_wbc.cpp checks measured contact at the endpoint hold; a pre-edge contact clears only that execution, records failure reason 7, preserves the transition requirement, and allows the next planner snapshot to re-plan instead of committing or stalling.
  canary_command: |
    Both named canaries ran serially under flock -x /tmp/go2_mujoco_experiment.lock, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, headless, phase2_step_5cm.xml, and the unchanged epoch28 command line. Clean code SHA was 20e548e944a8510e2a89f949116c10495f24e66a; manifests report git_dirty=false.
  canary_r1: |
    b1_crawl_epoch38_20260828: FL target=(0.935309,0.049977) m, in-flight start state t=7.718 s. The first observed x>=0.70 crossing was state t=7.812 s, u=0.4273, x=0.703039, z=0.081470 m (above the 0.080 m upper-surface-plus-clearance gate). Sampled trajectory maximum was x=0.970299, z=0.107292 at u=0.9909 after crossing. First measured endpoint contact was x=0.945095, z=0.071821, 3D endpoint error=0.028143 m, at_endpoint=1, committed mask=FL (2); FR/RR/RL did not commit. Final base_x=0.374610 m, completion=0, no rear swing/body-advance sequence.
  canary_r2: |
    b1_crawl_epoch38_20260828_r2: FL target=(0.970772,0.049641) m. The sampled in-flight maximum before execution stopped was x=0.691028, z=0.052493 at u=0.4500; no x>=0.70 crossing or measured commit occurred. Final base_x=0.064894 m, committed mask=0, completion=0; FR/RR/RL had no terrain execution. This named run is stochastic negative evidence, not a physical-success or gate claim.
  tests: |
    cmake --build example/cpp/build -j2; cd example/cpp/build; ctest --output-on-failure -> 27/27 passed. B0 fixed pair acceptance_status=PASS, including no terrain actuation and no plan consumer/publication checks; auxiliary gait diagnostics remained non-gate. No contract, analyzer threshold, or canary definition changed.
 git_status: code commit 20e548e; documentation append pending local commit; no staged files after commit; no push/amend; simulations serialized.
verdict: |
  The mechanism and numeric diagnosis are established, and the transfer-only edge-gated path removes the epoch37-r2 corner condition in the successful r1 swing (z=0.08147 at x=0.70304). The named pair still has only one FL commit and no dual-front/rear progression, so physical success and gate-level conclusions are not claimed.
---
---
timestamp: 2026-08-30T10:15:00+0800
run_id: Order-021 final-HEAD canary metadata correction
trigger: T1
signature: The pre-edge contact check was moved ahead of the endpoint-held gate in b9d9a75, with redundant endpoint-only code removed in b486e87. This makes any measured corner contact fail immediately while preserving the transition requirement. Final named canaries were rerun at clean HEAD b486e87.
evidence: |
  b1_crawl_epoch38_20260828: manifest git_head=b486e87f8f0284c1f4b63b991a2ced09073eeaa8, git_dirty=false; final base_x=.991913 m, committed mask=0, completion=0. FL target=(.968304,.049934) was prepared but had only 19 rows at phase 0 and never entered in-flight execution; FR/RR/RL had no terrain swing. Therefore no apex/contact was observed in this final stochastic rerun.
  b1_crawl_epoch38_20260828_r2: manifest git_head=b486e87f8f0284c1f4b63b991a2ced09073eeaa8, git_dirty=false; final base_x=.437811 m, committed mask=0, completion=0. FL target=(.942118,.050135) had only 19 phase-0 rows and never entered in-flight execution; FR/RR/RL had no terrain swing. The immediately preceding clean-20e548 rerun remains the per-swing evidence: r1 FL crossed x=.70 at z=.081470 and committed mask=2; r2 FL apex=.052493 at x=.691028 with no crossing/commit. No dual-front or rear progression is claimed.
verdict: |
  Final code and clean-HEAD metadata are consistent. The named final pair is negative stochastic evidence; the earlier clean pair demonstrates the edge-height mechanism but only one-front commit. No gate-level conclusion.
git_status: clean; no staged files; no push/amend; simulations serialized.
---
---
timestamp: 2026-08-30T12:30:00+0800
run_id: Order-022 b1_crawl_epoch39_20260828 (+ _r2)
trigger: T1
signature: Epoch38 prepared targets were not blocked by the transaction mask or the V2 window; the WBC pre-edge contact guard fired during the first force-filtered support samples, immediately clearing the newly launched execution. The guard is now delayed until the commanded swing reaches the inferred leading-edge phase. Flat path and v1/analyzer/canary contracts are unchanged.
evidence:
  gate_location: |
    Target selection/timeline is trot_experiment_gait.cpp:1641-1688; preparation validates start anchor at :1829-1891, path timing at :1928-1937, and terrain_target_required at :1968-1974. The schedule handoff is :2082-2085, the touchdown deadline is :2304-2307, support capture requires >=2 scheduled supports at :2058-2075, and a target-leg hold requires >=2 alternate supports at :2310-2328. Only then is execution.in_flight set at :2330. The WBC gate that can clear that execution is trot_experiment_wbc.cpp:323-340, failure code 7.
  epoch38_blocker: |
    b1_crawl_epoch38_20260828 r1 prepared FL x=0.968304499 at t=6.340095, with 117 phase-0 rows; the first nonzero observed phase was 0.007740958 at t=6.574106 and WBC cleared it at t=6.578107 with failure=7. The planner schedule masks were 6 during preparation and 15 at the handoff; the window was active=1 and the hold became mask=15. r2 prepared FL x=0.942117942 at t=6.414105, with 17 phase-0 rows; first nonzero phase=0.003616340 at t=6.448130 and clear at t=6.452083, failure=7. Its schedule mask was 15 at handoff and hold mask 11 then 15. Required masks were 3 and 2 respectively, with committed mask 0. Thus schedule phase did eventually hand off, the window was active, and support/transaction masks were nonzero; the separating blocker was the pre-edge contact guard, not hold or mask starvation. Force-filtered contact was still present on FL at the first flight samples, so the old x<=edge+5 mm predicate classified ordinary lift-off as a corner catch.
  fix: |
    terrain_motion_plan.h:45-55 adds TerrainSwingLeadingEdgeReached. The WBC failure-7 predicate now requires measured contact and commanded phase >= the inferred leading-edge phase at trot_experiment_wbc.cpp:323-333. This prevents the initial support-force sample from cancelling a launch, while endpoint-held samples (phase clamped to 1) still retain the corner guard. The change is only in terrain execution; flat/trot has no TerrainSwingExecution and is bit-identical.
  latch_chain: |
    One crawl step-up currently needs these ordered latches: (1) terrain execution allowed; (2) usable plan and valid timeline; (3) valid touchdown candidate in its future window; (4) measured/planned start anchor; (5) feasible path-duration handoff; (6) terrain height-transition requirement and transition/window latch; (7) scheduled leg swing phase; (8) captured support set >=2; (9) >=2 alternate supports when the target leg is captured; (10) in-flight launch; (11) leading-edge phase reached without failure-7 corner contact; (12) endpoint-held; (13) measured contact plus 3D endpoint tolerance; (14) per-leg transition commit; (15) front commit arms rear requirements; (16) S1 body-advance deadline and retimed schedule; (17) rear target preparation/launch/hold/commit; (18) all required legs complete and hold release. The 49-run telemetry exposes latches 1-14 and the terminal completion; S1/rear latches have zero observed fires in this corpus because no crawl run reached a rear target.
  latch_rates: |
    Denominator is 49 on-record runs epoch29-38. Window entry 10/49=20.4%; usable plan 49/49=100%; target prepared 49/49=100%; hold active 45/49=91.8%; any execution valid 49/49=100%; any in-flight 43/49=87.8%; any endpoint-held 27/49=55.1%; any measured touchdown 24/49=49.0%; any committed mask 18/49=36.7%; completion 12/49=24.5%. For the 10 crawl-window runs epoch34-38, conditional crawl rates are: window 10/10=100%; plan 10/10=100%; prepared 10/10=100%; hold 6/10=60%; execution valid 10/10=100%; in-flight 4/10=40%; endpoint-held 3/10=30%; measured touchdown 2/10=20%; front commit 2/10=20%; rear requirement 0/10=0%; rear execution 0/10=0%; completion 0/10=0%. Per-leg crawl execution rates FR valid/in-flight/held/touchdown=5/10,3/10,2/10,1/10; FL=8/10,2/10,2/10,1/10; RR=0/10,0/10,0/10,0/10; RL=0/10,0/10,0/10,0/10. The product of the observed crawl launch and touchdown bottlenecks is already low; this is evidence for the explicit-state-machine decision, not a gate conclusion.
  canary_command: |
    Both named runs used commit 0d081c540f2d3a59069a48d843b112ce58ac0f5e, the unchanged epoch28 run_trot arguments, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and serial flock -x /tmp/go2_mujoco_experiment.lock.
  canary_r1: |
    b1_crawl_epoch39_20260828: window 6.390094-9.646134 s, FL launched at t=6.628105 (phase 0), remained in-flight through t=6.720100 (phase 0.422018), target=(0.984654,0.077845,0.050346), then failure=7 at t=6.722093; no commit, final base_x=0.637425.
  canary_r2: |
    b1_crawl_epoch39_20260828_r2: window 6.320108-15.828118 s, FL launched at t=11.656075 (phase 0), remained in-flight through t=11.702093 (phase 0.291139), target=(0.898661,0.133457,0.049945), then failure=7 at t=11.704073; no commit, final base_x=0.861413.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --output-on-failure passed, 27/27. B0 fixed-pair acceptance_status=PASS at the same commit; its auxiliary non-gate gait diagnostics remain outside the frozen acceptance checks. The epoch39 pair proves post-fix launch/in-flight evidence but did not dual-front commit; no physical-success or gate-level conclusion is claimed.
git_status: code commit 0d081c5 and documentation commit 71c2232; clean worktree; no staged files; no push/amend; simulations serialized.
---

---
timestamp: 2026-08-30T03:25:00+0800
run_id: Order-023 b1_sm_epoch40_20260828 (+ _r2)
trigger: T1
signature: Implemented a dedicated sensor-gated v2 crawl state machine. The named canary pair reached CRAWL_STEP and enforced FL-first sequencing, but both runs hit the unchanged hard posture stop before a measured endpoint commit; complete crossing is not claimed.
evidence:
  implementation: |
    example/cpp/terrain/terrain_crawl_state_machine.h defines INACTIVE, APPROACH, DECELERATE_TO_CREEP, CRAWL_STEP, ADVANCE_BODY, CLEAR, RESUME, and ABORT. The fixed leg order is FL,FR,RR,RL (indices 1,0,2,3). Preconditions consume measured contact count, lidar plan validity, measured speed >=0.05, measured endpoint commit, measured base/foot clear, and current FK/radial <=0.40 m. A failed step allows two retries, then controlled abort through the existing velocity shaper.
  integration: |
    trot_experiment_gait.cpp gates the transfer-only execution adapter to the sequenced leg and holds non-sequenced legs. Rear target reachability is checked against current measured pose with IK and radial <=0.40 m. WBC commit telemetry counts measured endpoint commits. CSV now reports state, active leg, retry count, state-entry time, minimum measured contacts, and step commits. Window-outside behavior does not read the state machine.
  canary_command: |
    Both runs used clean HEAD bd3eaab876f26e8828eb540fee067100d2b727c0, epoch28 arguments, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and serial flock -x /tmp/go2_mujoco_experiment.lock.
  canary_r1: |
    b1_sm_epoch40_20260828: window 6.314148-10.192143 s; trace APPROACH -> DECELERATE_TO_CREEP -> CRAWL_STEP(FL); no step commit, final world_base_x=0.317514 m, minimum measured contacts=0 after collapse, safety_status=1 from hard posture stop.
  canary_r2: |
    b1_sm_epoch40_20260828_r2: window 6.338073-9.932228 s; trace APPROACH -> DECELERATE_TO_CREEP -> CRAWL_STEP(FL); no step commit, final world_base_x=0.306343 m, minimum measured contacts=0 after collapse, safety_status=1 from hard posture stop.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --output-on-failure passed 27/27. Final B0 fixed pair at HEAD bd3eaab returned acceptance_status=PASS, with frozen B0 checks green. No v1 contract, analyzer threshold, or canary definition changed; simulations were serialized.
verdict: |
  The explicit state-machine path and unit coverage are present, but the required physical complete crossing and 0.45 s stable RESUME evidence were not achieved in either named canary. Residual blocker is physical posture collapse before first measured commit; no gate-level conclusion.
git_status: implementation commits bb15f02 and bd3eaab; this documentation append is pending local commit; no push/amend; generated canary artifacts ignored.
---

---
timestamp: 2026-08-30T03:35:00+0800
run_id: Order-023 final-HEAD metadata correction
trigger: T1
evidence: |
  After the measured CLEAR precondition integration, the named pair was rerun at clean HEAD 638386b98dc2c5988ee5fd33be5a6565353ca4e0. b1_sm_epoch40_20260828 window 6.390142-10.308085 s, trace APPROACH -> DECELERATE_TO_CREEP -> CRAWL_STEP(FL), final world_base_x=.538633 m, commits=0, min measured contacts=0 after hard-posture collapse. _r2 window 6.338124-10.114121 s, trace APPROACH only, final world_base_x=.962795 m, commits=0, min measured contacts=0, also hard-posture collapse. Both manifests git_dirty=false and safety_status=1; neither completed crossing or stable RESUME.
  final_b0: |
    Final B0 fixed pair at the same HEAD used serial flock and preload and returned acceptance_status=PASS; frozen acceptance checks were green. ctest remained 27/27 passed.
git_status: final-HEAD evidence correction pending local commit; no push/amend; generated canary artifacts ignored.
---

---
timestamp: 2026-08-30T03:45:00+0800
run_id: Order-023 final safety-invariant canary correction
trigger: T1
evidence: |
  Final named pair ran at clean HEAD 6fdc4bab810d0b715a6048450588c5a9a9dd9032. Both runs entered CRAWL_STEP(FL) only after APPROACH -> DECELERATE_TO_CREEP, then immediately entered ABORT when measured support fell below the >=3 invariant: r1 window 6.352123-10.074135 s, base_x=.635741 m; r2 6.360140-10.134162 s, base_x=.714441 m. Both had zero endpoint commits and minimum measured contacts=0 after collapse; complete crossing and stable RESUME were not achieved. This final rerun demonstrates controlled state-machine abort rather than allowing a crawl flight phase.
  validation: ctest 27/27 passed after the invariant guard; final B0 PASS was obtained at preceding clean HEAD 638386b and no B0-scoped files changed afterward.
git_status: final canary correction pending local commit; no push/amend; generated artifacts ignored.
---

---
timestamp: 2026-08-30T03:50:00+0800
run_id: Order-023 final B0 rerun correction
trigger: T1
evidence: |
  The first post-documentation B0 attempt was a stochastic paired-baseline lifecycle failure (acceptance_status=FAIL) while all terrain-disabled checks passed. A serial rerun at clean HEAD 30cfdbae099b6a699c2e78c443d948429ffa783a returned acceptance_status=PASS, controller/dynamics/quality/safety status 0, planner deadline misses 0, and terrain actuation disabled. No source files changed between these B0 runs.
git_status: final documentation correction pending local commit; no push/amend.
---

---
timestamp: 2026-08-30T05:20:00+0800
run_id: Order-024 b1_sm_epoch41_20260828 (+ _r2)
trigger: T1
signature: Added window-scoped COM pre-shift and support-triangle gating to the v2 crawl sequencer; no v1 contract, analyzer threshold, or canary definition changed.
evidence:
  epoch40_abort_triangle: |
    Recomputed from contact_ground_truth.csv at the first ABORT sample. r1 t=8.042 s: COM projection=(0.538951,0.002092) m; upcoming FL support triangle FR=(0.661686,-0.124885), RR=(0.212289,-0.064404), RL=(0.296026,0.044664) m. Oriented signed edge distances=(+0.218611,+0.109473,-0.063567) m, hence COM is 63.567 mm outside the RL-RR edge (signed margin=-0.063567 m). RR was already the first unloaded leg: its force was 0 N throughout 8.000-8.042 s; at 8.042 FR/RR/RL were 0/0/0 N while FL carried 392.7 N. r2 t=8.206 s: COM=(0.560226,-0.002613) m; triangle FR=(0.679314,-0.186522), RR=(0.302314,-0.036718), RL=(0.357733,0.162203) m; signed edge distances=(+0.239298,+0.126934,-0.037129) m, so 37.129 mm outside (signed margin=-0.037129 m). RR and RL were already unloaded; FR was the first additional leg to unload (96 N at 8.160 s to 3 N at 8.162 s), before CRAWL_STEP.
  implementation: |
    terrain_crawl_state_machine.h adds SHIFT_COM, sensor-only triangle construction, signed edge-distance margin, centroid COM target, and a 0.020 m readiness precondition. Every FL/FR/RR/RL step enters SHIFT_COM first; ADVANCE_BODY transitions to rear SHIFT_COM. The explicit crawl requires the three non-active measured supports. Existing MPC/WBC horizontal body-position reference is overridden only in the active v2 window; SHIFT_COM holds all four commanded contacts, while CRAWL_STEP removes only the sequenced active leg. During swing the COM target remains held by the existing WBC stance task. Diagnostics now record COM x/y, target, validity, and triangle margin per sample.
  tests: |
    test_terrain_interfaces covers triangle validity, outside/inside signed margins, 20 mm shift readiness, each sequenced SHIFT_COM transition, and retry/abort. ctest: 27/27 passed.
  canary_command: |
    Both named runs used serial flock -x /tmp/go2_mujoco_experiment.lock, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot arguments (18 s, headless, wall-clock, wbc-full, running-trot, raibert-trot, period 0.50, duty 0.75, step 0.15, lift 0.08, tau 45, velocity script, terrain planner, phase2_step_5cm.xml, B1).
  canary_r1: |
    b1_sm_epoch41_20260828: trace INACTIVE -> APPROACH (7.506 s) -> DECELERATE_TO_CREEP (7.766 s) -> SHIFT_COM (7.806 s) -> ABORT (7.808 s); min measured contacts=0, commits=0, final base_x=0.341446 m. The pre-shift contact signal was already 2, so the sensor-gated safety abort correctly prevented a lift; no completed CRAWL_STEP.
  canary_r2: |
    b1_sm_epoch41_20260828_r2: trace INACTIVE -> APPROACH (7.576 s) -> DECELERATE_TO_CREEP (7.846 s) -> SHIFT_COM/ABORT (8.044 s); min measured contacts=0, commits=0, final base_x=0.637526 m. The pre-shift contact signal was 1; no triangle was accepted and no lift was commanded.
  commits: |
    0a86a42 terrain: shift COM into crawl support triangle; 6fd47b5 terrain: keep all contacts during COM pre-shift; 2595c13 terrain: hold crawl body during COM shift; b188b5e terrain: strengthen scoped COM shift reference; 40d5c1f terrain: reset COM shift telemetry on entry.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --test-dir example/cpp/build --output-on-failure passed 27/27. Worktree is clean, no staged files, no push. The epoch41 pair is signal evidence only: both runs aborted before a measured CRAWL_STEP commit, so no physical-success or gate-level conclusion is claimed.
verdict: |
  The epoch40 data confirms the COM/support-triangle mechanism quantitatively. The implementation now makes the triangle margin a measured precondition and preserves the >=3-contact invariant; epoch41 stochastic contact loss occurred before the precondition could be met. Further tuning or canaries are required for a physical commit; no gate conclusion.
git_status: clean worktree; no staged files; no push/amend; simulations serialized.


---
timestamp: 2026-08-30T06:40:00+0800
run_id: Order-025 b1_sm_epoch42_20260828 (+ _r2)
trigger: T1
evidence:
  epoch41_verdict: |
    (a) is the primary mechanism; (b) is not supported at the abort instant.
    In epoch41 r1 the machine was SHIFT_COM at t=6.550118 s, running-trot
    phase=0.499972, measured contacts=2. The preceding 1.00 s changed from
    roll=+0.401 deg, pitch=+0.885 deg, z=0.3717 m to roll=+2.567 deg,
    pitch=+0.896 deg, z=0.3593 m; the contact drop coincided with a normal
    trot phase handoff and no prior attitude divergence. In r2 the machine
    was SHIFT_COM at t=6.812089 s, phase=0.473204, contacts=1; over the
    preceding 1.00 s roll changed -0.102 -> -2.860 deg, pitch +0.355 ->
    +1.457 deg, and z 0.3718 -> 0.3813 m. This was a modest transient,
    not a height collapse; the later hard-posture fall followed the abort.
    Thus the old guard was applied at the trot-to-crawl handoff, where 2/0
    contact phases are legal, rather than proving a deceleration fall.
  implementation: |
    The >=3 measured-contact invariant is now enforced only in SHIFT_COM,
    CRAWL_STEP, and ADVANCE_BODY. APPROACH and DECELERATE_TO_CREEP no longer
    require three contacts; DECELERATE hands off only at measured creep <=
    0.12 m/s. The window keeps the configured trot through approach and uses
    a 0.80 s 0.30 -> 0.12 m/s request ramp. Crawl gait is selected only at
    the measured creep handoff; quasi-static SHIFT_COM/CRAWL_STEP/ADVANCE
    commands hold the base at zero speed. A 0.10 s measured-contact recovery
    grace covers the one-tick gait handoff before the crawl invariant.
    Flat/out-of-window paths retain their prior scheduler and pattern.
  canary_command: |
    Both runs used serial flock -x /tmp/go2_mujoco_experiment.lock, domain
    229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged
    epoch28 run_trot arguments (18 s, headless, wall-clock, wbc-full,
    running-trot, raibert-trot, period 0.50, duty 0.75, step 0.15,
    lift 0.08, tau 45, velocity script, terrain planner, phase2_step_5cm.xml,
    B1). The pair was run serially.
  canary_r1: |
    b1_sm_epoch42_20260828 trace: INACTIVE (0.000) -> APPROACH (6.254)
    -> DECELERATE_TO_CREEP (6.256) -> SHIFT_COM (6.880) -> ABORT (6.980).
    Minimum measured contacts by state: INACTIVE=0, APPROACH=2,
    DECELERATE_TO_CREEP=1, SHIFT_COM=0, ABORT=0. SHIFT_COM lasted 50
    samples; no CRAWL_STEP and no measured commit. During deceleration,
    roll=-1.692..+0.268 deg, pitch=-0.699..+1.414 deg, z=0.3671..0.3780 m.
  canary_r2: |
    b1_sm_epoch42_20260828_r2 trace: INACTIVE (0.000) -> APPROACH (6.334)
    -> DECELERATE_TO_CREEP (6.336) -> SHIFT_COM (6.906) -> ABORT (7.008).
    Minimum measured contacts by state: INACTIVE=0, APPROACH=2,
    DECELERATE_TO_CREEP=0, SHIFT_COM=2, ABORT=0. SHIFT_COM lasted 51
    samples; no CRAWL_STEP and no measured commit. During deceleration,
    roll=-0.538..+1.824 deg, pitch=-0.786..+1.324 deg, z=0.3662..0.3785 m.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --test-dir
    example/cpp/build --output-on-failure passed 27/27. git diff --check
    passed. Existing v1 contract, analyzer thresholds, and canary
    definitions were untouched; no gate-level conclusion is claimed.
verdict: |
  Epoch41 distinguishes (a) from (b): the pre-CRAWL invariant was
  mis-scoped to a legal running-trot contact phase; deceleration did not
  show a true pre-contact fall in the decisive r1/r2 evidence. The epoch42
  pair confirms stable posture through deceleration, but neither run yet
  reaches CRAWL_STEP(FL) or a measured commit, so the physical success
  criterion remains open.
git_status: local commits complete; clean worktree; no staged files, no push/amend; simulations serialized.


---
timestamp: 2026-08-30T07:10:00+0800
run_id: Order-025 B0 fixed-pair verification
trigger: T1
evidence:
  command: |
    flock -x /tmp/go2_mujoco_experiment.lock -c
    'LD_PRELOAD=/home/che/dds_base8000_preload.so bash
    example/cpp/scripts/run_phase2_b0_fixed_pair.sh development 0'
  result: |
    Frozen B0 analyzer acceptance_status=PASS at HEAD
    5fa08585fb3a7a416ce546fdad64a9ae70636220. controller=0,
    dynamics=0, safety=0, quality=0, completion=0; terrain_rows=39013,
    map_valid_fraction=0.9999743675, planner_deadline_misses=0,
    planner_updates=2687. Frozen b0-contract-v1.2 hashes and no-terrain
    checks passed. The auxiliary paired comparison diagnostics for gait
    period/duty and WBC references are non-gate diagnostics and remain
    outside the frozen acceptance result.
git_status: clean worktree; no staged files; no push/amend; simulations serialized.


---
timestamp: 2026-08-30T07:45:00+0800
run_id: Order-025 clean-HEAD canary correction
trigger: T1
evidence: |
  After the local documentation commits, both named canaries were rerun
  serially with clean HEAD 47c7e0a7eb61d7dd8836969aa4ce642585fd2f37;
  both manifests report git_dirty=false. r1 trace INACTIVE (0.000) ->
  APPROACH (6.378) -> DECELERATE_TO_CREEP (6.380) -> SHIFT_COM (6.912)
  -> ABORT (7.016), with per-state minimum contacts
  INACTIVE=0, APPROACH=2, DECELERATE_TO_CREEP=1, SHIFT_COM=2, ABORT=0.
  r2 trace INACTIVE (0.000) -> APPROACH (6.318) ->
  DECELERATE_TO_CREEP (6.320) -> SHIFT_COM (6.924) -> ABORT (7.028),
  with per-state minimum contacts INACTIVE=0, APPROACH=1,
  DECELERATE_TO_CREEP=0, SHIFT_COM=0, ABORT=0. Neither reached
  CRAWL_STEP(FL) or measured commit. Deceleration posture ranges were
  r1 roll=-1.445..+0.225 deg, pitch=-0.689..+1.505 deg,
  z=.3675.. .3789 m; r2 roll=-3.031..+2.020 deg,
  pitch=-.709..+3.674 deg, z=.3662.. .3812 m. No pre-contact height
  collapse was observed; the remaining failure is post-handoff support/
  COM recovery, not a gate-level conclusion.
git_status: clean worktree; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T08:20:00+0800
run_id: Order-026 b1_sm_epoch43_20260828 (+ _r2)
trigger: T1
signature: Epoch42 reconstruction found an inconsistent support plant at SHIFT_COM. The WBC contact override was nested under terrain_transfer_has_target, so before any swing transaction existed the crawl state still fed the running-trot diagonal into WBC. The same interval then injected a centroid COM target as a step reference.
evidence:
  epoch42_reconstruction: |
    b1_sm_epoch42_20260828 SHIFT_COM had WBC shadow mask 9 and scheduled mask 9 at 6.912089 s, then mask 6/6 at 7.006147 s, while active leg was not yet selected and the state was required to be a four-foot stance. Measured contacts were already 3 then 2. The first valid COM target at 6.914146 s jumped from the previous logged target (0,0) to (0.436934,-0.057117) m while measured COM was (0.540636,-0.001062) m and margin was -0.034788 m. At the next MPC refresh, reference x became 0.435965 m and requested x acceleration was -1.809885 m/s2. In r2, the corresponding jump was to (0.441644,-0.048024) m from measured COM (0.567626,0.007488), margin -0.049843 m; WBC mask stayed 9 until 6.964052 s, then 15 briefly, then 6 at 6.982147 s. The first absurd quantity is the contact plant mask: a four-foot SHIFT_COM was solved as a two-foot running-trot plant. The reference jump is the second independent inconsistency.
  source_fix: |
    TerrainCrawlWbcContactOverride is now applied after transfer bookkeeping, including SHIFT_COM before a target exists. It forces all four WBC and MPC contact knots during SHIFT_COM; CRAWL_STEP removes only its selected leg. BuildGaitTargets freezes all four commanded feet at measured world anchors during SHIFT_COM, preventing the crawl scheduler from moving feet that WBC declares stance. COM targets now start at measured COM and linearly ramp to the support-triangle centroid over the window-scoped 0.40 s constant. The SHIFT_COM ID-WBC min normal floor is also enabled only in the active v2 window, matching the four-foot stance policy.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --test-dir example/cpp/build --output-on-failure passed 27/27. The final B0 fixed pair was run serially with the required lock and preload at HEAD 05fbdfb and returned acceptance_status=PASS, controller/dynamics/safety/quality=0 and planner deadline misses=0. Existing v1 contract, analyzer thresholds, and canary definitions were untouched.
  canary_command: |
    Both named runs used HEAD 05fbdfb462d4d1f3040f37eed2992bc76d600060, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, serial flock -x /tmp/go2_mujoco_experiment.lock, and the unchanged epoch28 run_trot command line.
  canary_r1: |
    b1_sm_epoch43_20260828 manifest is clean at 05fbdfb. Trace INACTIVE (0.000) -> APPROACH (6.372) -> DECELERATE_TO_CREEP (6.374) -> SHIFT_COM (6.928) -> ABORT (7.038). Per-state minimum measured contacts: INACTIVE=0, APPROACH=2, DECELERATE_TO_CREEP=1, SHIFT_COM=2, ABORT=0. SHIFT_COM WBC mask remained 15; COM margin ranged -0.034179 to -0.026848 m among valid samples; no CRAWL_STEP and commits=0.
  canary_r2: |
    b1_sm_epoch43_20260828_r2 manifest is clean at 05fbdfb. Trace INACTIVE (0.000) -> APPROACH (6.366) -> DECELERATE_TO_CREEP (6.368) -> SHIFT_COM (6.896) -> ABORT (7.068). Per-state minimum measured contacts: INACTIVE=0, APPROACH=2, DECELERATE_TO_CREEP=0, SHIFT_COM=1, ABORT=0. SHIFT_COM WBC mask remained 15; COM margin ranged -0.039608 to -0.034092 m among valid samples; no CRAWL_STEP and commits=0.
verdict: |
  The absurd quantity is the WBC support mask mismatch, numerically 9 then 6 instead of 15 during a nominal four-foot SHIFT_COM; the 100 mm class COM reference jump amplified it. The source fix makes stance scheduling coherent, anchors commanded stance feet, ramps the COM reference, and scopes the 20 N floor. Epoch43 proves the WBC mask and support policy are corrected, but both stochastic signal canaries still abort before COM readiness and before CRAWL_STEP(FL) measured commit. No gate-level conclusion is claimed.
git_status: commits 05b5526 and 05fbdfb; clean worktree; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T06:00:00+0800
run_id: Order-027 b1_sm_epoch44_20260828 (+ _r2)
trigger: T1
signature: Epoch43 COM stall is reference-tracking-limited, not support-triangle geometry or margin mathematics.
evidence:
  epoch43_ref_vs_measured: |
    b1_sm_epoch43_20260828: valid SHIFT_COM samples 6.952109-7.036157 s; measured COM x 0.550720 -> 0.566163 m while ramp reference x 0.550564 -> 0.525680 m; Euclidean ref error 0.00023 -> 0.04103 m; margin -0.026848..-0.034179 m. The r1 precondition aborts before the 0.40 s ramp can settle.
    b1_sm_epoch43_20260828_r2: valid samples 6.910162-7.066140 s; measured COM x 0.547584 -> 0.562763 m while ramp reference x 0.547602 -> 0.505716 m; error 0.00003 -> 0.05932 m; margin -0.034092..-0.039608 m. The measured COM initially follows the old forward solution and then lags the backward-moving ramp; it does not track the reference.
  geometry_and_math: |
    ComputeTerrainSupportTriangle receives measured_foot_world at terrain_crawl_state_machine.h:371-374, and source construction passes actual_world_feet from FK at trot_experiment_gait.cpp:1658-1662. The centroid target is generated from those three measured feet and MeasureTerrainSupportTriangle applies the same signed edge distance used by the +0.020 m readiness test; no scheduled-foot or inconsistent-margin path was found. Epoch44 r1 later reached +0.019971 m before the next tick, confirming the triangle is not degenerate/unreachable.
  source_fix: |
    SHIFT_COM/CRAWL_STEP/ADVANCE_BODY now refresh the existing SRBD MPC every 5 control ticks (20 ms) instead of the ordinary 100 ms period; scoped stance no-slip weight is 80.0 (flat/non-window remains 8.0) and the 20 N normal floor remains window-scoped. SHIFT_COM contact recovery grace is 0.80 s so the 0.40 s ramp can complete; crawl-step boundary gets a 0.10 s schedule handoff grace.
  validation: |
    cmake --build example/cpp/build -j2 and ctest --test-dir example/cpp/build --output-on-failure passed 27/27. Named canaries were run serially with flock -x /tmp/go2_mujoco_experiment.lock, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot command line. Final named pair had stochastic outcomes: r1 entered CRAWL_STEP at 7.680 s after SHIFT_COM but no FL commit; r2 did not reach CRAWL_STEP. Earlier same-code retries reached CRAWL_STEP with measured three-contact mask 13 and one retry reached FL in-flight, but no measured commit.
  canary_command: |
    flock -x /tmp/go2_mujoco_experiment.lock -c 'for id in b1_sm_epoch44_20260828 b1_sm_epoch44_20260828_r2; do LD_PRELOAD=/home/che/dds_base8000_preload.so bash example/cpp/scripts/run_trot.sh 18 $id --headless --wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75 --step-length 0.15 --foot-lift 0.08 --tau-limit 45 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase2_b1_velocity_0p3.csv --terrain-planner --domain-id 229 --scene-file unitree_robots/go2/phase2_step_5cm.xml --phase2-milestone B1; done'
verdict: |
  Root cause is tracking latency/stance authority: epoch43 applies a stale forward SRBD solution for up to 100 ms, then the four-foot stance is too weak to remove residual momentum before the readiness check. The fix is window-scoped MPC refresh plus stronger stance no-slip and longer recovery/settle allowance. No v1 contract, analyzer threshold, or canary definition changed. This is exploratory signal evidence only; the final pair does not claim the Order-027 physical-success criterion (FL measured commit).
git_status: local changes pending commit; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T18:00:00+0800
run_id: Order-028 b1_sm_epoch45_20260828 (+ _r2)
trigger: T1
signature: The epoch44 FL attempt never reached the measured commit gate because the explicit CRAWL_STEP leg was held in explicit CRAWL_STEP swing for the whole state. The endpoint transition was therefore unreachable; the state-machine wait condition at terrain_crawl_state_machine.h:306-308 also remained false because committed[FL] never became true.
evidence:
  epoch44_trace: |
    b1_sm_epoch44_20260828: state entered CRAWL_STEP at 8.918 s, active leg FL (1), measured mask 13 at the handoff; FL execution remained valid=0/in_flight=0/endpoint_held=0 through abort at 9.032 s, with FL prepare failure=4 (terrain target required false). The recorded FL world FK trajectory was x=0.6792 -> 0.6154 m and z=0.0230 -> 0.0732 m over the state interval; no FL command, leading-edge crossing, measured contact event, endpoint, or force-qualified commit exists in the CSV. r2 did not enter CRAWL_STEP.
    source_mechanism: |
      In the state-machine path, explicit_active_leg was unconditional while state==CRAWL_STEP (trot_experiment_gait.cpp:2294-2308), making effective_leg_in_swing true even after touchdown. The normal !effective_leg_in_swing branch at trot_experiment_gait.cpp:2357-2365, which changes in_flight to endpoint_held, could never run. Consequently the WBC commit predicates at trot_experiment_wbc.cpp:360-365 (measured contact && endpoint error <= window tolerance) were unreachable for an in-flight explicit swing.
  fix: |
    Added TerrainCrawlSwingStillInFlight() in terrain_motion_plan.h. The explicit leg is owned only until its immutable touchdown timestamp; at/after that timestamp the endpoint-held path executes. During CRAWL_STEP the selected leg now uses this predicate instead of the unconditional swing flag. During SHIFT_COM the selected upcoming leg is allowed to prepare from the retained last usable plan while all other legs remain measured anchors; a transient rejected planner snapshot no longer erases that snapshot inside the active window. Leading-edge guard, edge stand-off targets, and 0.045 m window tolerance are unchanged.
  canary_command: |
    Both named runs used serial flock -x /tmp/go2_mujoco_experiment.lock, domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot command line. b1_sm_epoch45_20260828: APPROACH 8.070 -> DECELERATE_TO_CREEP -> SHIFT_COM 8.616 -> CRAWL_STEP 9.314 -> ABORT 9.582; b1_sm_epoch45_20260828_r2: APPROACH 7.584 -> DECELERATE_TO_CREEP -> SHIFT_COM 8.142 -> CRAWL_STEP 8.720 -> ABORT 8.912. Both had zero measured commits; FL did not become in-flight because the available planner candidate was not surface-transition-required (failure=4) before the snapshot expired. Sequence progress: none beyond CRAWL_STEP; FR/ADVANCE_BODY/rear steps not reached.
verdict: |
  The source-level commit deadlock is fixed and covered by a pure helper test. The named stochastic pair did not produce a physical FL commit: both runs lost the valid FL transition candidate before launch and aborted on support. This is exploratory signal evidence only; no gate-level conclusion is claimed. The residual planner/state handoff issue is recorded rather than masked by treating a non-transition target as committed.
git_status: local changes pending commit; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T22:00:00+0800
run_id: Order-029 b1_sm_epoch46_20260828 (+ _r2)
trigger: T1
root_cause: |
  Epoch45 was not a lidar/FOV starvation. At the CRAWL_STEP handoffs the
  lidar model was fresh (map age 0.008--0.094 s in the CSV), complete
  (320/320 cells, 0.05 m relief), and FL had 19--80 static regions and 8
  swept candidates. The retained executable plan (r1 id 244 generated at
  8.508 s, r2 id 260 at 8.056 s) predated the state-machine requirement;
  its FL foothold therefore carried no transition intent. New snapshots
  were rejected by the support check while the robot was standing because
  planner input did not include the crawl hold support, so prepare failed
  at the existing failure=4 gate. The edge estimator itself was not the
  starvation source: candidate windows remained populated and upper-surface
  candidates were ranked whenever a fresh feasible plan was built.
fix: |
  TerrainCrawlStateMachine now exposes PendingTransitionLeg(). Control
  snapshots publish that state-owned intent through APPROACH/DECELERATE,
  SHIFT_COM, and CRAWL_STEP, instead of relying only on each snapshot's
  surface-height classification. During crawl execution the planner also
  receives the measured support hold before a target exists, allowing fresh
  plateau candidates to pass support planning. The explicit crawl leg treats
  a prepared target as owned until its immutable touchdown, which removes
  the pre-launch deadlock (execution_in_flight is set by the launch path).
  All changes are transfer-window gated; flat path and v1/analyzer/canary
  definitions are unchanged.
evidence: |
  Unit tests cover the latched FL intent and prepared-target launch;
  ctest: 27/27 passed. In the epoch46 r1 CSV, SHIFT_COM had 348 samples
  with FL candidate_required=1, swing_candidate_count=8, and FL execution
  valid; CRAWL_STEP had 110 samples and FL was in-flight for 109. FL then
  reached endpoint-held/measured touchdown at t=7.804133 s (target
  x=0.727480, z=0.049858; measured x=0.712069, z=0.073160; endpoint error
  0.028003 m), with committed mask 2. The run aborted on support before
  sequence continuation. _r2 reached CRAWL_STEP (52 samples) but did not
  launch FL. These are exploratory canaries, not gate conclusions.
canary_command: |
  Both named runs used serial flock -x /tmp/go2_mujoco_experiment.lock,
  domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the
  unchanged epoch28 run_trot command line.
git_status: local changes pending commit; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-30T23:30:00+0800
run_id: Order-030 b1_sm_epoch47_20260828 (+ _r2)
trigger: T1
root_cause: |
  Epoch46 r1's FL commit was recorded at t=9.058 in the CSV (the
  experiment-relative commit was t=7.804 in the prior report). The
  measured masks immediately before the abort were 15, then 6; with FL
  still the active leg, the old CRAWL_STEP invariant subtracted FL and
  saw only one remaining support. The state machine therefore aborted
  before it could enter FR SHIFT_COM. Epoch46 r2 had 52 CRAWL_STEP
  samples, seven preparations, and 3657 rejections; the stable terminal
  reason was failure=6: an immutable touchdown was too close/past by the
  time the prepared target reached the gait handoff, not missing intent.
fix: |
  The state machine now advances a measured commit before evaluating the
  prior active-leg support mask, and permits a bounded 0.40 s endpoint
  confirmation interval (0.10 s handoff + 0.30 s commit grace) when the
  active force sample is still present. CRAWL_STEP is gated on a live
  prepared target; stale targets return to SHIFT_COM without rewriting
  their immutable touchdown time. The 3-D support metric uses the true
  measured foot triangle plane, orthogonal COM projection, and a raised
  centroid z; exact equal-height triangles retain the prior flat arithmetic.
  Target preparation remains available in SHIFT_COM so fresh snapshots can
  replace a stale target. All behavior remains transfer-window scoped.
evidence: |
  Mixed-height unit geometry uses FL z=0.05 m and gives the support
  centroid z=0.0166666667 m with a positive >=0.02 m projected margin.
  Epoch46 evidence was FL endpoint error 0.028003 m, committed mask 2,
  r1 CRAWL_STEP measured masks 15 then 6, and r2 failure=6 after 52
  samples. Final rebuilt epoch47 canary traces were:
  r1 APPROACH 7.570 -> DECELERATE_TO_CREEP -> SHIFT_COM 8.132 ->
  CRAWL_STEP 8.742 -> ABORT 8.968; r2 APPROACH 7.506 ->
  DECELERATE_TO_CREEP -> SHIFT_COM 7.896 -> CRAWL_STEP 8.082 ->
  SHIFT_COM 8.084 -> ABORT 9.024. This final stochastic pair recorded
  no commit; one preceding rebuilt attempt recorded r2 FL commit and
  FR SHIFT_COM, but neither final named artifact reached FR commit.
  No ADVANCE_BODY/rear progress is claimed.
canary_command: |
  Both named runs used serial flock -x /tmp/go2_mujoco_experiment.lock,
  domain 229, LD_PRELOAD=/home/che/dds_base8000_preload.so, and the
  unchanged epoch28 run_trot command line.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed. No v1, analyzer threshold, or canary
  definition changed. Simulations were serialized; no push/amend.
git_status: local changes pending commit; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T00:30:00+0800
run_id: Order-031 autonomous grind b1_sm_epoch48..55
trigger: T1
objective: full v2 physical crossing; eight serial canary pairs consumed

cycle_1:
  diagnosis: epoch48 r2 reached FL commit (step_commits=1, committed mask=2) but FR SHIFT_COM aborted after 0.90 s when measured contacts fell to 2; r1 failed FL handoff with prepare failure=5.
  fix: 4a7f723 lets the next crawl leg bypass the old transfer-hold anchor deferral, so FR can be prepared while FL remains committed.
  evidence: ctest 27/27; epoch49 pair.
cycle_2:
  diagnosis: epoch49 both runs repeatedly hit prepare failure=5 at explicit crawl handoff; target IK was marginal at the measured pose although planner had checked its predicted handoff pose.
  fix: 4ae6af0 retains the immutable crawl endpoint and delegates marginal live-pose handling to clamped WBC instead of dropping the transaction.
  evidence: ctest 27/27; epoch50 r1 reached FL commit (step_commits=1), but committed mask telemetry was cleared before FR progress.
cycle_3:
  diagnosis: epoch50 r1 reached FL commit but FR handoff remained asynchronous; epoch50 r2 stopped before commit.
  fix: 70c60c9 permits SHIFT_COM to enter CRAWL_STEP on a valid plan/COM margin before the adapter publishes the same-tick target, then safely returns to SHIFT_COM if absent.
  evidence: ctest 27/27; epoch51 r2 reached FR CRAWL_STEP, but support collapsed before FR commit.
cycle_4:
  diagnosis: epoch51/52 showed crawl handoff being entered with fewer than three measured contacts; epoch52 r2 fell from base_z=0.4247 to 0.2040 m while support was 0--2.
  fix: 51ae8b3 requires three measured contacts before DECELERATE_TO_CREEP can enter SHIFT_COM; unit test updated for this invariant.
  evidence: ctest 27/27; epoch53 remained at FL/SHIFT_COM depth and did not commit.
cycle_5:
  diagnosis: epoch53 r2 reached CRAWL_STEP with only two measured contacts and aborted at t=8.378; no FR transaction was measured.
  fix: a42dfe9 removes the redundant consumer IK rejection during transfer-window handoff, retaining the live endpoint for the clamped WBC.
  evidence: ctest 27/27; epoch54 pair did not reach SHIFT_COM in either run.
cycle_6:
  diagnosis: epoch54 pair stopped before crawl (no canary state beyond DECELERATE_TO_CREEP).
  fix: 7d29909 tested the v2-allowed minimum 0.05 m/s crawl speed to reduce transfer momentum.
  evidence: ctest 27/27; epoch54 was the resulting pair; it produced no crawl progress.
cycle_7:
  diagnosis: minimum-speed trial removed the previously observed crawl entry in both runs.
  fix: 1bf31df restored the established 0.12 m/s handoff speed; the measured-support gate remains in place.
  evidence: ctest 27/27; epoch55 pair consumed the final budget pair.
cycle_8:
  diagnosis: epoch55 r1 remained in DECELERATE_TO_CREEP; r2 reached SHIFT_COM but no commit. r1/r2 base_z minima were 0.3494/0.2338 m, transition completions 0/0, committed mask 0/0, and max committed contacts 0/0.
  fix: no further budget remains; no speculative change made.

canary_command: |
  Every pair was serial under flock -x /tmp/go2_mujoco_experiment.lock, domain 229,
  LD_PRELOAD=/home/che/dds_base8000_preload.so, and the unchanged epoch28 run_trot
  command line (18 s, headless, wall-clock, wbc-full, running-trot, raibert-trot,
  period 0.50, duty 0.75, step 0.15, lift 0.08, tau 45, velocity script,
  terrain planner, phase2_step_5cm.xml, B1). No v1/analyzer/canary definition changed.
verdict: |
  Full physical crossing was NOT achieved. Deepest measured progress was epoch51 r2:
  FL committed (mask=2), then FR CRAWL_STEP was entered; no FR measured commit,
  ADVANCE_BODY, rear steps, CLEAR, RESUME, or 0.45 s stable passage occurred.
  Budget exhausted at 8/8 pairs (16 simulations). This is exploratory stuck evidence,
  not a gate conclusion.
git_status: local docs append pending commit; no staged files; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T01:20:00+0800
run_id: Order-032 scripted deterministic crawl b1_script_epoch56..58
trigger: T1
objective: fixed-timing, lidar-measured target crawl while retaining gait-SRBD-MPC-WBC

implementation:
  - terrain/terrain_crawl_script.h adds the direct lidar-map target measurement,
    deterministic FL,FR,RR,RL scheduler, fixed 0.40 s ramp + 0.20 s settle,
    fixed 0.60 s swing at 40% apex, endpoint commit, two retries, and abort.
  - TerrainMotionPlan carries the map-measured scripted target. The gait target
    adapter uses it only inside CRAWL_STEP; contact schedules and MPC/WBC
    consumption remain supplied by the validated plan.
  - The existing state skeleton is window-gated with the fixed shift/swing
    deadline and clears only an uncommitted endpoint on retry. CLEAR/RESUME and
    all flat/out-of-window paths are unchanged.

tests:
  ctest: 27/27 passed
  build: test_terrain_interfaces and real_trot_go2 passed
  unit_coverage: deterministic target ordering and edge stand-off; fixed timing;
    FL/FR/RR/RL order; endpoint retry budget; abort; clear/resume.

canary:
  - b1_script_epoch56_20260831: valid serial run; reached SHIFT_COM at 6.906 s,
    CRAWL_STEP at 7.532 s, no measured commit, then repeated fixed retry and
    ABORT at 13.604 s; max committed mask 0; no crossing.
  - b1_script_epoch56_20260831_r2: invalid simulator startup (DDS domain 227
    participant unavailable); not counted as physical evidence.
  - b1_script_epoch57_20260831: valid serial run; no crawl commit, safety stop;
    no crossing.
  - b1_script_epoch57_20260831_r2: valid serial run; entered CRAWL_STEP at
    7.296 s and aborted at 7.550 s on measured support; no commit/crossing.
  - b1_script_epoch58 pair: not counted; DDS participant allocation was
    unavailable on domains 220/221.

verdict: |
  Full physical crossing was not achieved. The valid runs remain stuck before
  the first measured FL commit (deepest rung CRAWL_STEP), with no ADVANCE_BODY,
  rear steps, CLEAR, RESUME, or 0.45 s stable passage. Further canary launch
  stopped after the repeated pre-commit/support signature and unavailable DDS
  participants; this is exploratory stuck evidence, not a gate conclusion.

git_status: local implementation and docs append pending commit; no staged files; no push/amend; simulations serialized.
---
timestamp: 2026-08-31T03:00:00+0800
run_id: Order-033 b1_script_epoch59_20260828 (+ _r2)
trigger: T1
root_cause: |
  Confirmed. The analytical FK/MuJoCo dynamics foot point is the center of
  the foot collision sphere, while the terrain map foothold z is the contact
  patch plane. unitree_robots/go2/go2.xml gives the foot sphere radius as
  0.022 m, and the prior FL touchdown (epoch46 r1) measured target z=0.049858
  against FK site z=0.073160: dz=+0.023302 m. No target-z/FK-site comparison
  had applied this conversion. Earlier measured misses preserve the same sign
  and magnitude: epoch56 FL dz=+0.024200 m (norm 0.029434), epoch57 r2
  dz=+0.028004 m (norm 0.033080). Epoch44/45 reports and epoch58 unavailable
  participant runs contain no newer measured endpoint; their recorded misses
  remain in the +22--24 mm cluster.
fix: |
  Added calibrated 0.022 m ContactPatchToFootSite and FootSiteToContactPatch
  conversion in the shared FK header. Terrain feasibility now evaluates IK
  reachability and swing clearance at the site target while retaining map
  patch elevations. Planner swing duration uses the site target. The terrain
  execution handoff raises planner contact-patch targets to the FK site, and
  transition source and committed surface bookkeeping converts measured FK
  sites back to contact-patch z. This is confined to terrain target handling;
  flat B0 touchdown tolerance and v1/analyzer/canary definitions are unchanged.

dds: |
  Windows measurement command: netsh interface ipv4 show
  excludedportrange protocol=udp. Observed complete exclusions are
  62889-62988 and 63089-63188. Base=8000 is replaced by Base=4000. Across
  harness domains 200-230, multicast is 54000-64500 and p=0..9 unicast is
  54010-61529, with no intersection. The reproducible source is
  example/cpp/scripts/dds_base4000_preload.c and the generated artifact is
  /home/che/dds_base4000_preload.so. Both epoch59 runs used serial flock,
  domain 229, and the new preload; controller logs report DDS domain=229 and
  no DDS participant or allocation error.
evidence: |
  ctest: 27/27 passed. Epoch46 r1 original endpoint decomposition was
  dx=-0.015411, dy=+0.003557, dz=+0.023302 m, norm=0.028003 m; after the
  calibrated z comparison the residual decomposition is
  (-0.015411,+0.003557,+0.001302) m, norm about 0.0159 m. Epoch56 FL nearest
  recorded endpoint was (-0.016753,-0.000296,+0.024200) m, norm 0.029434;
  epoch57 r2 was (-0.010226,-0.014337,+0.028004) m, norm 0.033080.
  Epoch59 r1/r2 did not reach CRAWL_STEP or measured FL touchdown (r1 stopped
  in SHIFT_COM on the hard posture limit; r2 stopped in DECELERATE_TO_CREEP),
  so no post-fix endpoint decomposition or commit can be claimed. DDS errors
  were absent from both logs.
verdict: |
  Offset mechanism is confirmed and implemented. The requested epoch59 pair
  is exploratory non-signal evidence because both runs stopped upstream of a
  swing; no gate-level conclusion is claimed.
git_status: local implementation commit 6ac88f4; canary artifacts ignored; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T04:00:00+0800
run_id: Order-033 canary rerun with Base=4000 b1_script_epoch59_20260828 (+ _r2)
trigger: T1
canary_update: |
  The first Base=7000 trial exposed a collision at domain 222, so the hardening
  base was reduced to 4000. Domain scan of harness representatives 200, 220,
  222, 223, 227, 229, and 230 started cleanly; the Base=4000 B0 fixed pair
  returned acceptance_status PASS with no DDS allocation failure. The named
  epoch59 pair was rerun serially under flock on domain 229 with
  /home/che/dds_base4000_preload.so.
endpoint_decomposition: |
  Neither epoch59 run reached CRAWL_STEP or a measured FL touchdown, so the
  requested post-fix endpoint decomposition is not available: r1 stopped in
  SHIFT_COM and r2 in DECELERATE_TO_CREEP on the hard posture limit. There is
  therefore no FL commit in this pair and no claim of the success criterion.
  The pre-fix comparison remains numeric: epoch46 r1 (-15.411,+3.557,+23.302)
  mm, norm 28.003 mm, becomes (-15.411,+3.557,+1.302) mm, norm about 15.9 mm
  when evaluated in site coordinates. Epoch56 and epoch57 r2 had dz +24.200
  mm and +28.004 mm respectively.
dds_result: |
  Both epoch59 controller logs show DDS domain=229 and no DDS participant,
  allocation, or discovery error. The Base=4000 B0 pair passed its analyzer
  acceptance report. This removes the Base=8000 collision class without
  changing the requested domain IDs.
verdict: |
  Offset is confirmed and source-fixed; epoch59 is upstream safety-stop
  evidence, not a successful signal canary and not a gate conclusion.
git_status: local commits 6ac88f4 (implementation) and 5a512d1 (DDS/docs); no push/amend; simulations serialized.

---
timestamp: 2026-08-31T09:00:00+0800
run_id: Order-034 b1_script_epoch60_20260828 (+ _r2)
trigger: T1
entry_analysis: |
  Aggregated existing telemetry from epoch34-59 (all available state-machine and
  scripted runs; epoch34-39 predates the state trace and was retained as the
  approach-only reference). The discriminating table is:

  cohort | entry outcome | DECEL/SHIFT time | measured v range | max |roll|/|pitch| | base-z range | map age
  40-43 (8 runs) | 2/8 reached CRAWL_STEP; 6 posture-stop | 6.32-7.07 s | 0.04-0.48 m/s before stop | stable 0.018-0.073 rad, stop then diverges | 0.360-0.396 m stable | 0.006-0.108 s
  44-51 (16 runs) | 16/16 reached CRAWL_STEP | 6.27-7.78 s | -0.29..0.40 m/s (filtered sign reversals) | 0.026-0.167 rad | 0.366-0.425 m | 0.004-0.106 s
  52-55 (8 runs) | 1/8 reached CRAWL_STEP; 7 posture-stop | 6.29-7.60 s | -1.02..0.72 m/s on failures | up to pi/1.08 rad | 0.071-0.454 m | 0.000-0.106 s
  56-59 (6 available runs) | 2/6 reached CRAWL_STEP; epoch59 0/2 | 6.21-7.08 s | -0.75..0.76 m/s on failures | up to 1.72/1.31 rad | 0.174-0.535 m | 0.004-0.102 s

  Survivor separator is posture/base-height stability, not map age: survivors
  remain near |roll|,|pitch|<0.12 rad and z=0.366-0.425 m through handoff;
  posture-stop traces leave that band before or during the 0.30->0.12 command
  ramp. Entry velocity is noisy and can reverse sign, while map ages overlap.
  Epoch59 r1 stopped in SHIFT_COM and r2 in DECELERATE, confirming the same
  upstream failure. The historical ramp was 0.80 s; the successful 44-51
  cohort still had transient gait handoff sensitivity.
fix: |
  In the terrain transfer window only, the command ramp is lengthened to 1.20 s
  and its target is lowered to 0.08 m/s. DECELERATE now uses the crawl support
  pattern while preserving the continuous ramp, and SHIFT_COM is gated by a
  0.24 s dwell with finite measured posture (|roll|,|pitch| <= 0.20 rad),
  measured contact count >=2, and the entry velocity guard <=0.50 m/s. The
  state-owned plan is no longer required to be valid on the exact handoff tick;
  planner/target validation remains at execution. No v1 path or analyzer/canary
  definition changed. Unit tests cover posture rejection and settle dwell.
canary: |
  Named pair was run serially under flock /tmp/go2_mujoco_experiment.lock,
  domain 229, LD_PRELOAD=/home/che/dds_base4000_preload.so, and the epoch28
  command line. Repeated evidence is exploratory: both runs repeatedly survived
  DECEL/SHIFT and reached CRAWL_STEP; FL committed in r2 in the successful
  attempts (offset-corrected endpoint error 7.9-16.4 mm class, dz removed),
  while r1's planner/target handoff remained flaky and did not produce a paired
  FL commit in the final overwrite. DDS logs had no participant/allocation error.
  The pair therefore does not meet the BOTH+FL signal success criterion yet;
  no gate conclusion is claimed.
b0: |
  run_phase2_b0_fixed_pair.sh development 0 with Base=4000 preload returned
  acceptance_status PASS. The paired diagnostic has expected terrain-vs-flat
  gait telemetry differences, but all B0 acceptance checks passed.
verdict: |
  Entry repair is implemented and materially moves the failure upstream rung:
  CRAWL_STEP is reached in both members across successful attempts, with one
  offset-corrected FL commit observed. Signal pair remains exploratory pending a
  repeat in which both named runs also commit FL. No gate conclusion.
git_status: local implementation/docs append pending commit; no push/amend; simulations serialized.


---
timestamp: 2026-08-31T12:00:00+0800
run_id: Order-035 b1_script_epoch61..69 (serial domain-229 canary pairs)
trigger: T1
implementation: |
  Ratchet-locked fixes were limited to the planner/script handoff and the
  post-handoff endpoint lifecycle. d512f89 applies the calibrated
  foot-site/contact-patch conversion to direct script target measurement and
  adds a regression test. 525aaf1 retains the captured support during the
  short endpoint/contact-filter handoff. a01186c retimes a prepared
  emergent endpoint to the fixed 0.60 s script deadline at CRAWL_STEP.
  d84ea35 prevents asynchronous plan refreshes from erasing an accepted
  scripted endpoint. 28ade28 keeps the captured support through 0.70 s,
  immediately before the fixed 0.80 s retry boundary; the commit predicate,
  tolerance, entry, COM shift, and edge-clearance logic are unchanged.

canary_command: |
  Every named run used the unchanged epoch28 command line, scene
  phase2_step_5cm.xml, domain 229, LD_PRELOAD=/home/che/dds_base4000_preload.so,
  and serial flock -x /tmp/go2_mujoco_experiment.lock. Epochs 61..69 were
  b1_script_epochNN_20260828 (+ _r2). Wrapper/analyzer status is exploratory
  FAIL because these runs safety-stop before the frozen B1 gate tail.

progress: |
  epoch61 r2 reached CRAWL_STEP at 7.460 s but stopped on support before
  commit; r1 stopped in DECELERATE. epoch62 r1 stopped in SHIFT_COM and r2
  in DECELERATE. epoch63 both reached CRAWL_STEP; r1 reached endpoint hold
  (55.1 mm residual) and r2 repeatedly hit the handoff rebase boundary.
  epoch64 r2 reached CRAWL_STEP but exhausted retries; r1 stopped in DECELERATE.
  epoch65 r2 reached CRAWL_STEP and hit the frozen leading-edge corner-catch
  rejection (failure=7); r1 stopped in DECELERATE. epoch66 r2 reached
  CRAWL_STEP once, but both members stopped before commit. epoch67 r2 reached
  CRAWL_STEP and held FL at 30.7 mm residual, then stopped at the support
  boundary; r1 stopped in DECELERATE. epoch68 r1 reached CRAWL_STEP and
  committed FL (committed mask=2, endpoint error 1.49 mm), then stopped in
  the following SHIFT_COM with measured support falling to 1; r2 stopped in
  DECELERATE. epoch69 r1 and r2 both reached CRAWL_STEP and each recorded a
  measured FL commit (r1 2.01 mm, r2 4.09 mm), but neither entered FR
  CRAWL_STEP before the following SHIFT_COM support/posture stop.

b0: |
  run_phase2_b0_fixed_pair.sh development 0 with Base=4000 returned
  acceptance_status PASS at HEAD 28ade28. The fixed analyzer reported the
  expected terrain-vs-flat diagnostic differences; frozen B0 acceptance
  checks passed.

verdict: |
  The direct script target flake was identified as a foot-site/contact-patch
  z-frame mismatch and fixed; the accepted target is now retained and retimed
  across planner refreshes. The deepest Order-035 evidence is FL commit in
  both members of epoch69 (and once in epoch68), followed by the next
  SHIFT_COM. FR commit, ADVANCE_BODY, rear steps, CLEAR, RESUME, and 0.45 s
  stable passage were not achieved. Full crossing and confirmation therefore
  remain NOT achieved. The exploratory budget is exhausted (the eight planned
  pairs are epochs 61..68; epoch69 was a final same-HEAD diagnostic pair),
  and this is not a gate conclusion.

git_status: implementation commits d512f89, 525aaf1, a01186c, d84ea35, 28ade28; docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T09:40:00+0800
run_id: Order-036 asymmetric-stance SHIFT_COM b1_script_epoch70_20260828 (+ _r2)
trigger: T1
implementation: |
  Added a window-scoped 3-D stance-plane attitude reference. The support
  triangle is the three feet excluding the pending crawl leg; its measured
  world normal is yaw-aligned and converted to Rz(yaw) Ry(pitch) Rx(roll)
  reference in terrain_crawl_state_machine.h:113-200. The COM ramp now also
  interpolates the triangle z (terrain_crawl_state_machine.h:704-708).
  During SHIFT_COM/CRAWL_STEP, WBC SRBD-MPC receives this roll/pitch and the
  auxiliary posture task uses deviation from the same reference. The hard
  posture stop uses the 0.20 rad deviation envelope only in this active
  window (trot_experiment_diagnostics.cpp:589-618); outside it the prior
  flat-ground limits are unchanged. Reference roll/pitch are CSV telemetry.

reconstruction: |
  Epoch68 r1 after FL commit: FL force was 17 -> 0 N; RR was the first
  remaining support leg to fall below 20 N (25 -> 18 N at the next sample),
  while FR/RL initially remained 54/53 N. The 3-D support was FL(z=0.05),
  RR/RL(z=0), with measured COM support margin reaching -0.0021 m.
  Epoch69 r1: FL 13 -> 0 N and RR 25 -> 12 N; margin reached -0.0855 m,
  actual roll/pitch about 0.012/-0.200 rad. Epoch69 r2: RR was already 0 N
  at the first post-commit sample, FL 9 -> 0 N, and the margin was -0.1015 m;
  actual roll/pitch was about 0.13/-0.05 rad. These are foot_force_* sensor
  columns; WBC logs expose the corresponding normal-force diagnostics.
  The old code set mpc_in.reference[0/1] = 0 at trot_experiment_wbc.cpp:807-809
  while MeasureTerrainSupportTriangle already used the mixed-z plane. Thus the
  controller held level torso against a tilted FL/RR/RL stance. The old hard
  stop was absolute 22 deg for wbc-full (diagnostics.cpp:587-606), so it
  treated deliberate tilt and fall identically. The mixed support triangle is
  the actual FL/RR/RL 3-D triangle, not an XY projection.

canary_command: |
  Both runs used serial flock -x /tmp/go2_mujoco_experiment.lock, domain 229,
  LD_PRELOAD=/home/che/dds_base4000_preload.so, and the unchanged epoch28
  command line with phase2_step_5cm.xml. b1_script_epoch70 reached FR
  CRAWL_STEP repeatedly (296 samples, 0.60 s fixed attempts) with reference
  roll/pitch approximately 0.03/-0.10 rad and measured forces 14-70 N, but
  no FR target commit before the run ended. _r2 reached FR CRAWL_STEP for
  301 samples with reference about -0.01/-0.09 rad, then diverged to the
  existing inverted-roll failure; no FR commit. The new telemetry is present
  in both data.csv files. Therefore the requested FR-commit success criterion
  is NOT achieved; this is exploratory evidence, not a gate conclusion.

validation: |
  ctest --output-on-failure: 27/27 passed. Full build produced real_trot_go2.
  run_phase2_b0_fixed_pair.sh development 0 with Base=4000 returned
  acceptance_status PASS (paired diagnostic differences are the expected
  terrain-vs-flat differences). No analyzer, v1 contract, or canary definition
  was changed. Epoch70 target preparation remained flaky (FR target_valid=0),
  so ADVANCE_BODY and rear progress were not reached.

git_status: local implementation/docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T12:30:00+0800
run_id: Order-037 FR target-validity + runtime budget b1_script_epoch71_20260828 (+ _r2)
trigger: T1
implementation: |
  FR target_validity=0 was a handoff race, not a right-side map starvation.
  MeasureTerrainScriptTarget supplied both sides from the lidar grid; epoch70
  planner candidate counts were FR 23-124 versus FL 21-128, and epoch71 were
  FR 0-124 versus FL 0-128 (swing candidates reached 8 on both). The target
  itself was prepared during entry/SHIFT_COM but was erased by each newer
  planner snapshot because only an in-flight/held transaction was retained;
  when CRAWL_STEP began, find_planned_foothold only accepted scripted_target
  in CRAWL_STEP, leaving target_valid=0. A nominal FR target could also enter
  the trot swing before the fixed handoff and be rejected at the leading edge.
  The fix keeps prepared targets for the active transfer window, recognizes
  the scripted target during the selected SHIFT_COM, suppresses pre-handoff
  nominal flight, and always retimes the prepared endpoint at CRAWL_STEP.
  The unit fixture confirms FR and FL receive deterministic edge-safe supply.

runtime: |
  Script canary duration was extended at harness invocation only from 20 s
  (epoch70 metadata) to controller-duration=30 with wall timeout 35 s. No
  analyzer threshold, contract, or canary definition changed. Both epoch71
  manifests record controller_duration_s=30; analyzer was invoked unchanged
  by run_trot.sh and emitted the normal JSON status. The B0 fixed-pair analyzer
  compatibility/hash checks passed with the same analyzer and contract hashes.

canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; LD_PRELOAD=
  /home/che/dds_base4000_preload.so; domain 229; phase2_step_5cm.xml;
  unchanged epoch28 controller arguments; --controller-duration 30.

canary_trace: |
  b1_script_epoch71_20260828: 5861 rows, last cmd_time 11.720 s before
  posture stop. APPROACH 6.310; DECELERATE_TO_CREEP 6.312-6.710;
  SHIFT_COM 6.712-11.720; CRAWL_STEP active FL(index 1) 7.350-7.590 and
  8.190-8.780; no FR(index 0) CRAWL_STEP. FR execution valid 2706 rows,
  candidate count 0-124, swing candidates reached 8; FL valid 2504 rows,
  candidate count 0-128, swing candidates reached 8. FL measured touchdown
  1471 rows and committed mask=2; FR measured/commit=0. No ADVANCE_BODY.
  b1_script_epoch71_20260828_r2: 5519 rows, last cmd_time 11.036 s before
  posture stop. APPROACH 6.274; DECELERATE_TO_CREEP 6.276-6.988;
  SHIFT_COM 6.990-7.840; ABORT 7.842-11.036; no CRAWL_STEP. FR valid
  2382 rows, candidate count 0-124, swing candidates reached 8; FL valid
  2024 rows, candidate count 0-128; no measured FR/FL commit. Runtime was
  sufficient to attempt the sequence; FR commit success criterion remains
  NOT achieved and no gate conclusion is made.

validation: |
  ctest --output-on-failure: 27/27 passed. B0 fixed pair at
  phase2_b0_development_fixed_3mps_r0_20260830_094801: acceptance_status PASS,
  analyzer_hash=true, contract_hash=true, no_terrain_actuation=true,
  planner_updated=true, planner_deadline_misses=0. Epoch71 analyzer reports
  completion false because the canaries stopped before FR, which is expected
  exploratory evidence rather than a threshold change.

git_status: |
  local commits f308674, 25b939a, 8561a7b, 996ffb4, 891aebb, 7285da3;
  no push/amend; simulations serialized; final tree clean.

---
timestamp: 2026-08-31T15:30:00+0800
run_id: Order-038 asymmetric FR-directed SHIFT_COM b1_script_epoch72_20260828 (+ _r2)
trigger: T1
reconstruction: |
  Epoch71 r1's requested post-FL interval (6.712-11.720 s) contains 2090
  SHIFT_COM rows. Before divergence (6.712-8.5 s), measured COM was
  x=0.480..0.522 m and y=-0.027..0.037 m; the COM reference ramp stayed
  near x=0.447..0.504 m/y=-0.047..0.032 m. The measured support margin
  moved from -0.019 m to +0.043 m, so the target is reachable in the
  horizontal support geometry while the machine remains upright. The
  planner's reachability margin was non-negative (0..0.068 m) in the
  same trace; no FK workspace impossibility was evidenced.

  The pre-failure support forces were FR/FL/RR/RL median approximately
  20/64/24/38 N in 7.8-8.5 s (earlier 6.712-7.8 s: 46/39/32/15 N).
  The support set for the FR shift is FL/RR/RL, with RL the weakest
  support; this explains the old 20 N hysteresis contact loss despite
  non-zero load. COM movement then became divergent: by 8.5-9.5 s,
  measured y=0.224..0.506 m and margin=-0.164..-0.110 m, while the
  stance-plane reference roll/pitch reached [-1.563,1.561]/[-0.160,0.251]
  rad. This is post-failure posture divergence, not a stable unreachable
  target. The 0.40 s + 0.8 s fixed timing was too short for the asymmetric
  force transfer, and the fixed +0.020 m readiness gate did not encode
  force/static stability.
implementation: |
  terrain_crawl_state_machine.h now keeps the flat 0.40 s ramp unchanged,
  but for mixed-height support triangles selects a displacement-driven
  0.40..1.20 s ramp. SHIFT_COM readiness may use a bounded -0.040 m margin
  only with three finite support loads >=10 N, total >=50 N, max/min <=4,
  and measured COM speed <=0.08 m/s for a 0.12 s dwell. The same force
  witness covers hysteretic contact-bit lag; non-window and flat arithmetic
  remain unchanged. A scripted SHIFT_COM timeout at 2.50 s restarts the
  measured ramp/re-square at most twice, then enters the existing bounded
  ABORT path. Diagnostics record selected ramp duration and recovery count.
  No foot micro-step was added because FK/reachability was not the cause.
canary_command: |
  Both named runs were serialized with flock -x /tmp/go2_mujoco_experiment.lock,
  domain 229, LD_PRELOAD=/home/che/dds_base4000_preload.so, phase2_step_5cm.xml,
  unchanged epoch28 arguments, --controller-duration 30 (wall timeout 35 s).
canary_trace: |
  Final-code b1_script_epoch72_20260828: entry reached SHIFT_COM at
  6.750 s, FL CRAWL_STEP at 8.186 s, and recorded one FL commit. Its
  bounded recovery is visible as SHIFT_COM recovery_count=1 at 8.436 s;
  it retried FL CRAWL_STEP at 9.814 s and then returned to SHIFT_COM at
  10.404 s without FR commit before safety termination. _r2 reached
  SHIFT_COM at 6.790 s, FL CRAWL_STEP at 8.110 s, then one bounded
  recovery at 8.354 s and stopped before a commit. Both runs were
  controller-duration=30 attempts, serialized on domain 229. The required
  FR commit/ADVANCE_BODY success criterion remains unmet; these are
  exploratory stochastic results, not a door-level conclusion.
validation: |
  ctest --test-dir example/cpp/build --output-on-failure: 27/27 passed.
  test_terrain_interfaces includes the displacement-scaled ramp, force/static
  readiness, and two-recovery-then-abort timeout cases. Full real_trot_go2
  build passed. B0 fixed pair at
  phase2_b0_development_fixed_3mps_r0_20260830_102122 returned
  acceptance_status=PASS with analyzer_hash=true, contract_hash=true,
  no_terrain_actuation=true, planner_deadline_misses=0, terrain_rows=18593.
  No v1 contract, analyzer threshold, or canary definition changed; no
  gate-level conclusion is made.
git_status: local implementation commits 51c1b67, ffeeff5 plus docs append; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T18:00:00+0800
run_id: Order-039 commit-ledger epoch73 b1_script_epoch73_20260828 (+ _r2)
trigger: T1
implementation: |
  e28ad60 kept the measured transition commit ledger alive through transaction
  completion and bounded recovery; it is cleared only when the post-crossing
  transfer window releases. The crawl sequencer also latches commit facts and
  the unit test covers a refreshed snapshot with the active mask cleared.
canary_trace: |
  Both runs used controller-duration=30, wall timeout=35, domain 229,
  Base=4000 preload, and serial flock /tmp/go2_mujoco_experiment.lock.
  r1 reached CRAWL_STEP twice but no measured commit before posture stop at
  cmd_time 14.088 s. r2 committed FL at 11.904 s (endpoint error 1.4 mm)
  and FR at 14.516 s (14.7 mm), with committed masks 2 -> 3; no rear step
  or ADVANCE_BODY was recorded before posture stop at cmd_time 16.098 s.
  The old FL retry signature did not recur after the durable mask reached 2.
validation: |
  test_terrain_interfaces and ctest 27/27 passed before the canary. The
  exploratory canary analyzers reported incomplete crossing because both
  runs hit the unchanged posture safety path; this is not a gate conclusion.
git_status: local commits e28ad60 and 4a1cdc7; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T19:00:00+0800
run_id: Order-039 commit-ledger epoch74 b1_script_epoch74_20260828 (+ _r2)
trigger: T1
implementation: |
  4a1cdc7 consumes a commit latched during SHIFT_COM before choosing another
  crawl target, so a commit arriving after recovery cannot regress the pointer.
canary_trace: |
  Both runs used controller-duration=30, wall timeout=35, domain 229,
  Base=4000 preload, and serial flock /tmp/go2_mujoco_experiment.lock.
  r1 stopped in SHIFT_COM at cmd_time 9.564 s with no commit. r2 entered
  CRAWL_STEP and committed FL at 10.826 s (44.6 mm endpoint error), leaving
  durable mask 2; it did not retry FL, but did not reach FR before the
  posture stop at cmd_time 13.494 s. No ADVANCE_BODY/rear/CLEAR/RESUME.
validation: |
  ctest 27/27 passed and both artifacts record git_head 4a1cdc7. The
  incomplete-crossing analyzer failures are expected exploratory evidence;
  no analyzer threshold or contract was changed.
git_status: local commits only; no push/amend; simulations serialized.

B0 regression for the ledger diff: phase2_b0_development_fixed_3mps_r0_20260830_103636 returned acceptance_status=PASS with controller_status=0, quality_status=0, safety_status=0, no_terrain_actuation=true, planner_deadline_misses=0, analyzer_hash=true, contract_hash=true, and terrain_rows=39086. The paired diagnostic report contains its existing non-gating command-profile comparison warnings; fixed analyzer acceptance remained PASS.

---
timestamp: 2026-08-31T20:30:00+0800
run_id: Order-039 commit-ledger epoch75 b1_script_epoch75_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Serial domain-229 runs used Base=4000 preload, controller-duration=30,
  wall timeout=35, and flock /tmp/go2_mujoco_experiment.lock. r1 reached
  SHIFT_COM at 6.960 s, exhausted two bounded shift recoveries, and entered
  ABORT at 9.844 s with no commit. r2 reached SHIFT_COM at 6.580 s but
  stopped before CRAWL_STEP at cmd_time 9.736 s; neither run reached FR,
  ADVANCE_BODY, rear legs, CLEAR, or RESUME.
validation: ctest remained green (27/27); failures are unchanged posture/safety exploratory outcomes, with no analyzer or contract edits.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T22:00:00+0800
run_id: Order-039 commit-ledger epoch76 b1_script_epoch76_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Serial domain-229 runs used Base=4000 preload and controller-duration=30.
  r1 reached SHIFT_COM at 6.604 s and stopped before CRAWL_STEP at 9.734 s.
  r2 reached FL CRAWL_STEP at 8.840 s, recovered to SHIFT_COM at 9.084 s
  with required mask reduced to 1, then stopped at 13.432 s. No measured
  commit, FR, ADVANCE_BODY, rear legs, CLEAR, or RESUME occurred.
validation: ctest 27/27 passed before this canary; no red-line thresholds or contract definitions changed.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-08-31T23:30:00+0800
run_id: Order-039 commit-ledger epoch77 b1_script_epoch77_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Serial domain-229 runs used Base=4000 preload and controller-duration=30.
  r1 reached SHIFT_COM at 6.628 s, exhausted two bounded recoveries, and
  entered ABORT at 9.368 s with no commit. r2 reached SHIFT_COM at 6.964 s
  (required mask 1 then 3) and stopped at 12.064 s without CRAWL_STEP or a
  commit. Neither reached FR, ADVANCE_BODY, rear legs, CLEAR, or RESUME.
validation: ctest 27/27 passed; this cycle made no source change.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T00:30:00+0800
run_id: Order-039 commit-ledger epoch78 b1_script_epoch78_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Serial domain-229/Base=4000/controller-duration=30 pair. r1 reached FL
  CRAWL_STEP at 7.864 s and committed FL at 9.714 s (2.5 mm), durable mask
  2; it stopped at 11.264 s before FR. r2 stopped after two shift recoveries
  at 12.596 s with no commit. No ADVANCE_BODY/rear/CLEAR/RESUME.
validation: ctest 27/27 passed; no analyzer thresholds or contracts changed.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T01:45:00+0800
run_id: Order-039 commit-ledger epoch79 b1_script_epoch79_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Serial domain-229/Base=4000/controller-duration=30 pair. r1 reached FL
  CRAWL_STEP at 9.144 s, recovered, retried FL CRAWL_STEP at 9.986 s, and
  stopped at 12.874 s without commit. r2 reached FL CRAWL_STEP at 8.242 s
  and stopped at 11.592 s without commit. No FR/ADVANCE_BODY/rear/CLEAR/RESUME.
validation: ctest 27/27 passed; no source change this cycle.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T03:00:00+0800
run_id: Order-039 commit-ledger epoch80 b1_script_epoch80_20260828 (+ _r2)
trigger: T1
canary_trace: |
  Final budget pair, serial domain-229/Base=4000/controller-duration=30. r1
  reached FL CRAWL_STEP at 8.730 s, recovered to SHIFT_COM at 8.972 s, and
  stopped at 11.838 s with mask 0. r2 reached FL CRAWL_STEP at 8.932 s,
  recovered at 9.188 s, and stopped at 12.876 s with mask 0. Neither run
  committed FL or reached FR/ADVANCE_BODY/rear/CLEAR/RESUME.
validation: |
  ctest 27/27 passed; B0 fixed pair PASS is recorded above. Budget exhausted
  at 8/8 pairs. Exact stuck report: deepest observed rung is FR CRAWL_STEP
  only as a post-FL target in epoch73 r2, with masks 2 -> 3 and no stable
  ADVANCE_BODY; final repeated failure is FL CRAWL_STEP/recovery with zero
  commit in epoch79-80. No gate-level conclusion.
git_status: local docs commit pending; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T05:00:00+0800
run_id: Order-040 cycle-1 ADVANCE_BODY attack b1_script_epoch81_20260828 (+ _r2)
trigger: T1
reliability_table: |
  Recomputed from all 82 on-record b1_sm/b1_script data.csv runs epoch40+
  (including retries), using state transitions and the measured surface
  transition committed mask (FR=1, FL=2, RR=4, RL=8). Window entry is
  DECELERATE_TO_CREEP; each later denominator is the preceding rung.
  window entry 82/82=100.0% (failure 0.0%); survive DECEL|entry
  67/82=81.7% (18.3%); SHIFT converge|DECEL 46/67=68.7% (31.3%);
  FL commit|SHIFT 12/46=26.1% (73.9%); FR SHIFT converge|FL
  4/12=33.3% (66.7%); FR commit|FR SHIFT 1/4=25.0% (75.0%);
  ADVANCE|FR 0/1=0.0% (100.0%); RR|ADVANCE N/A (0/0);
  RL|RR N/A (0/0); CLEAR+RESUME|RL N/A (0/0). Ranked measurable
  conditional failures: ADVANCE 100.0%, FR commit 75.0%, FL commit
  73.9%, FR SHIFT 66.7%, SHIFT 31.3%, DECEL 18.3%, entry 0.0%.
  Rear/CLEAR rates are unestimable, not zero-rate claims.
implementation: |
  8a3c607 changes only v2 terrain-window ADVANCE_BODY behavior. The
  previous branch commanded 0.0 m/s while waiting for rear_targets_fk_reachable,
  making measured FK reachability self-blocking. It now commands the bounded
  0.12 m/s crawl during ADVANCE_BODY; SHIFT_COM/CRAWL_STEP remain stopped.
  No alternate leg order was prototyped because ADVANCE_BODY, not alternating
  COM shift, is the worst measured rung.
canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; domain 229;
  LD_PRELOAD=/home/che/dds_base4000_preload.so; run_trot.sh 35;
  unchanged epoch28 arguments, --controller-duration 30, phase2_step_5cm.xml.
canary_trace: |
  b1_script_epoch81_20260828 entered APPROACH 6.328, DECEL 6.330,
  SHIFT_COM 6.574, CRAWL_STEP(FL) 8.214; no commit before last
  cmd_time 11.482, base_x=0.641. _r2 entered APPROACH 6.270, DECEL
  6.272, SHIFT_COM 6.964, CRAWL_STEP(FL) 9.228, retried FL at 9.990,
  then committed FL at 10.582 (mask=2, step_commits=1); no FR or
  ADVANCE_BODY before last cmd_time 14.644, base_x=0.424. Neither
  reached the attacked rung. These are exploratory outcomes, not a gate.
validation: |
  cmake --build example/cpp/build -j2 and ctest --test-dir
  example/cpp/build --output-on-failure: 27/27 passed. No analyzer,
  v1 contract, or canary definition changed; source/test commit is 8a3c607.
git_status: implementation committed; docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T07:30:00+0800
run_id: Order-040 cycle-2 ADVANCE_BODY attack b1_script_epoch82_20260828 (+ _r2)
trigger: T1
implementation: 8a3c607 unchanged; no source change because cycle-1 did not reach ADVANCE_BODY.
canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; domain 229;
  LD_PRELOAD=/home/che/dds_base4000_preload.so; unchanged epoch28
  run_trot command with --controller-duration 30 and wall timeout 35.
canary_trace: |
  r1: APPROACH 6.354, DECEL 6.356, SHIFT_COM 6.598,
  CRAWL_STEP(FL) 9.000; no FL/FR commit, ADVANCE_BODY absent;
  last cmd_time 12.184, base_x=0.503. r2: APPROACH 6.290, DECEL
  6.292, SHIFT_COM 6.710, CRAWL_STEP(FL) 7.870; no commit or
  ADVANCE_BODY; last cmd_time 10.716, base_x=0.355. No attacked-rung
  observation; exploratory only.
validation: ctest 27/27 passed at 8a3c607; no analyzer/v1/canary changes.
git_status: implementation committed; docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T10:00:00+0800
run_id: Order-040 cycle-3 ADVANCE_BODY attack b1_script_epoch83_20260828 (+ _r2)
trigger: T1
implementation: 8a3c607 unchanged; no source change because ADVANCE_BODY remained unobserved.
canary_command: Serial flock/domain 229, Base=4000 preload, --controller-duration 30, wall timeout 35, unchanged B1 command.
canary_trace: |
  r1 reached APPROACH 6.350, DECEL 6.352, SHIFT_COM 6.594 and
  CRAWL_STEP(FL) 9.730; no commit or ADVANCE_BODY, last cmd_time
  12.984, base_x=0.500. r2 reached APPROACH 6.382, DECEL 6.384,
  SHIFT_COM 6.626 and stopped before CRAWL_STEP at cmd_time 10.172,
  base_x=0.507. The attack was not exercised; exploratory only.
validation: ctest 27/27 passed at 8a3c607; no analyzer/v1/canary changes.
git_status: implementation committed; docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T12:30:00+0800
run_id: Order-040 cycle-4 ADVANCE_BODY attack b1_script_epoch84_20260828 (+ _r2)
trigger: T1
implementation: 8a3c607 unchanged; no source change because the predecessor FR rung did not commit.
canary_command: Serial flock/domain 229, Base=4000 preload, --controller-duration 30, wall timeout 35, unchanged B1 command.
canary_trace: |
  r1 reached APPROACH 6.322, DECEL 6.324, SHIFT_COM 6.568,
  FL CRAWL_STEP 7.916, retried into SHIFT_COM 8.160 and FL
  CRAWL_STEP 9.700, with no FL commit/FR/ADVANCE; last cmd_time
  13.004, base_x=0.492. r2 reached APPROACH 6.292, DECEL 6.294,
  SHIFT_COM 7.100 but no CRAWL_STEP; last cmd_time 10.630,
  base_x=0.425. Attack not exercised; exploratory only.
validation: ctest 27/27 passed at 8a3c607; no analyzer/v1/canary changes.
git_status: implementation committed; docs append pending commit; no push/amend; simulations serialized.

b0: |
  Required B0 fixed pair was attempted serially after 8a3c607. Both
  baseline domain 222 and terrain domain 223 aborted before DDS bridge
  readiness with 'Failed to find a free participant index'; no analyzer
  result exists. This is an environment/DDS allocation failure, not a
  B0 regression result. Existing recorded B0 PASS remains the prior
  fixed-pair evidence; no B0 conclusion is inferred from this attempt.

---
timestamp: 2026-09-01T14:00:00+0800
run_id: Order-040 stuck report after cycle-4 (3 same-signature cycles)
trigger: T1
verdict: |
  Stopped under the three-cycle same-signature rule. The worst measured
  conditional rung is ADVANCE_BODY|FR: 0/1=0.0%, failure 100.0%; only
  one FR-commit predecessor reached it, so the estimate is sparse. The
  attacked change 8a3c607 makes ADVANCE_BODY issue a bounded 0.12 m/s
  creep instead of the previous 0.0 m/s self-blocking command, but all
  four new pairs stopped before ADVANCE_BODY (three at FL/no commit or
  retry, one before CRAWL_STEP). The next measured failures are FR
  commit|FR SHIFT 1/4=25.0% (75.0%) and FL commit|SHIFT 13/52=25.0%
  (75.0%) over the updated 90-run table. RR/RL/CLEAR+RESUME remain
  N/A (zero reached denominators), not zero-rate claims. No complete
  crossing or confirmation was achieved; no gate conclusion.
git_status: implementation commit 8a3c607; docs append pending commit; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T16:00:00+0800
run_id: Order-041 cycle-1 deterministic STAGE b1_script_epoch85
trigger: T1
implementation: |
  Added scripted-only STAGE between DECELERATE_TO_CREEP and SHIFT_COM.
  The lidar-derived planner snapshot carries a world-frame body target: the
  inferred rising edge minus the 0.25 m canonical standoff and nominal front
  foot offset. STAGE commands bounded signed creep, then requires position,
  force/contact, posture, and velocity dwell before entering SHIFT_COM. No
  terrain endpoint is applied during pre-step staging.
  STAGE is represented in the state trace and diagnostics as a distinct rung.
  Non-scripted callers retain the previous DECELERATE_TO_CREEP transition.
determinism: |
  Historical pre-STAGE epoch84 pair entry base-x values were 0.492 m and
  0.425 m (spread 0.067 m). Order-041 exploratory stage traces entered at
  0.560 m and 0.474 m in the two comparable retries (spread 0.086 m);
  this pair did not demonstrate improvement because both detections were
  already beyond the canonical target. The diagnostics now record
  terrain_staging_target_valid, terrain_staging_error_m, and
  terrain_staging_target_world_x_m for independent recomputation.
canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; domain 229;
  LD_PRELOAD=/home/che/dds_base4000_preload.so; run_trot.sh 35;
  --controller-duration 30; phase2_step_5cm.xml; epochs 85 exploratory
  retries; all runs remained serialized.
canary_trace: |
  STAGE was entered reproducibly in the recorded attempts. No run passed
  the STAGE dwell into SHIFT_COM: the canonical world target was behind
  the body by approximately 0.25-0.43 m in the diagnostic trace, while
  measured support/contact never sustained the required witness. Runs
  stopped at STAGE/ABORT on the existing hard safety path; no crawl step,
  commit, ADVANCE_BODY, or crossing was observed. This is an exploratory
  stuck result, not a gate conclusion.
b0: |
  Cleaned/inspected /dev/shm before retry; it was empty and no stale DDS
  processes were present. Rebuilt Base=4000 preload with
  MaxAutoParticipantIndex=31. The fixed B0 pair PASS was restored at
  _runs/phase2_b0_development_fixed_3mps_r0_20260830_114752_{baseline,terrain};
  analyzer acceptance_status=PASS, no terrain actuation, 0 deadline misses.
validation: |
  cmake --build example/cpp/build -j2 and ctest --test-dir
  example/cpp/build --output-on-failure: 27/27 passed; git diff --check
  passed. No v1 contract, analyzer threshold, or canary definition changed.
verdict: |
  Stuck at the new STAGE rung: 0/2 comparable exploratory attempts reached
  canonical staging dwell, so downstream conditional rates FROM canonical
  staging are N/A. Budget was not treated as exhausted and no crossing or
  confirmation was achieved. No gate-level conclusion.
git_status: docs/source changes pending local review; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T18:30:00+0800
run_id: Order-042 cycle-1 clean event-driven crawl b1_freegait_epoch90 (+ _r2)
trigger: T1
implementation: |
  b40a898 adds terrain/terrain_crawl_sequencer.h. The window-gated owner
  samples the live lidar TerrainModel, generates measured-state-to-map
  targets and a smooth swing trajectory, publishes explicit contact topology,
  and computes the support-polygon COM reference. Transfer activation uses a
  map edge distance threshold before trot transition intent. Legacy Phase 1
  code and v1/analyzer/canary definitions are unchanged.
canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; domain 229;
  LD_PRELOAD=/home/che/dds_base4000_preload.so; run_trot.sh 35;
  --headless --wall-clock-motion --controller-duration 30 --wbc-full;
  running-trot/raibert-trot, period 0.50, duty 0.75, step 0.15, lift 0.08;
  phase2_b1_velocity_0p3.csv; phase2_step_5cm.xml; epochs 90 and 90_r2.
canary_trace: |
  Both serialized runs armed the transfer path but stopped on the existing
  safety path before a crawl commit. The final CSV rows report old diagnostic
  state ABORT, active leg 4, and event-sequencer state STAGE. Sequencer
  measured-contact counts were 0 (r1) and 1 (r2), below the required 3-foot
  staging witness; no SHIFT/SWING/COMMIT/ADVANCE/CLEAR/RESUME event occurred.
  The safety log reports roll approximately -179.73 deg and the existing
  0.20-rad deviation hard stop. No complete crossing or confirmation.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir
  example/cpp/build --output-on-failure: 27/27 passed. Both canaries ran
  serially under the required lock/domain/preload and ended with nonzero
  harness analysis because the controller safety stop prevented controlled
  completion; this is exploratory evidence, not a gate conclusion.
verdict: |
  Precise stuck report: STAGE precondition (>=3 measured contacts) was not
  met before the inherited posture safety stop in 2/2 runs. The rewrite's
  downstream rungs are therefore unmeasured; budget remains 9 pairs and no
  three-cycle same-signature stop applies.
git_status: implementation committed as b40a898; docs append pending; no push/amend; simulations serialized.


---
timestamp: 2026-09-01T22:30:00+0800
run_id: Order-043 sequencer activation flip + canary cycles (epoch91-106)
trigger: T1
forensics: |
  Epoch90 r1 armed at state_tick 5.520 s and r2 at 5.580 s. The first
  armed rows had sequencer state STAGE, measured contacts 4, and the
  diagnostic WBC scheduled mask was 15 (four contacts); no SHIFT/SWING
  event had occurred. Roll stayed near zero through the armed interval,
  then first crossed |roll|=0.08 rad at 9.954 s (r1) and 9.926 s (r2),
  well before the stop at approximately 15.0/15.9 s. The old integration
  switched the terrain-crawl path immediately on arming, so a running-trot
  phase was replaced by a four-contact declaration while the plant was not
  guaranteed to be at a four-contact boundary. Measured COM moved r1 from
  (-0.087,0.000) m at arm to (0.084,-0.025) m before divergence and then
  (-0.723,-0.245) m after inversion; r2 ended at (-0.646,-0.592) m. No
  commanded COM target jump was recorded: old target fields stayed invalid
  or zero and MPC reference telemetry stayed 0. This is a contact and
  authority handoff failure, not a deliberate COM step.
implementation: |
  terrain_crawl_sequencer now separates armed STAGE from
  control_authority_active. Before seizure it publishes no crawl topology,
  swing target, or COM reference and consumers retain the trot path. Seizure
  requires the live running-trot schedule to be four-contact-able, at least
  three measured contacts, settled speed <=0.04 m/s, and roll and pitch <=
  0.08 rad. On seizure COM is latched from measured COM. A phase-respecting
  stand-transition request is exposed while armed; kernel native stance hold
  is enabled only after seizure in STAGE. WBC contact, MPC, stance policy,
  terrain target hold, and legacy crawl state are gated by authority. Added
  authority, stand, and COM telemetry to the CSV.
canary_cycles: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock, domain 229,
  LD_PRELOAD=/home/che/dds_base4000_preload.so, run_trot.sh 35,
  --controller-duration 30, running-trot/raibert-trot, step .15/lift .08.
  B0 fixed pair rerun with dds_base8000_preload.so returned PASS at HEAD
  97b6097; all frozen checks were green and paired non-gate diagnostics
  retained their known terrain-vs-flat differences. Epoch103 r1 reached
  controlled 30 s with max roll 0.121 rad; authority reached SHIFT/SWING
  but commit=0. Epoch103 r2 remained STAGE and inverted (max roll=pi).
  Epoch104 pair and epoch105 pair reached controlled 30 s, max roll
  0.179/0.130 and 0.117/0.154 rad; each reached SHIFT/SWING but commit=0.
  Epoch106 pair exposed a STAGE-only safety failure (max roll about
  1.81/3.14 rad). Deepest measured rung is SHIFT/SWING entry: no measured
  commit, ADVANCE, CLEAR, RESUME, or complete crossing. Mixed signatures do
  not qualify for a three-cycle same-signature stop; three of nine pair
  budget slots remain under loop accounting.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir
  example/cpp/build --output-on-failure: 27/27 passed after final code.
  No v1 contract, analyzer threshold, or canary definition changed; all
  simulations were serialized and no push or amend was performed.
verdict: |
  Flip mechanism is confirmed by epoch90 timing and masks: authority was
  effectively granted at arming and the old terrain path changed control
  before a safe measured boundary. The fix removes pre-seizure interference
  and prevents inversion in several controlled runs, but first swing and
  commit remain unreliable and crossing is not achieved.
git_status: implementation and this evidence append are unstaged; no push or amend; simulations serialized.

---
timestamp: 2026-09-01T23:25:00+0800
run_id: Order-043 final canary cycle epoch108 (+ _r2)
trigger: T1
canary_trace: |
  After the final stand-boundary latch and authority-gated WBC changes,
  epoch108 r1 completed the full 30 s controller duration with max
  |roll|=0.118 rad, reached SHIFT/SWING, and recorded zero commits.
  Epoch108 r2 hit the inherited safety stop in STAGE with max |roll|=3.134
  rad and zero commits. This pair confirms that inversion is no longer
  deterministic but the STAGE-to-SWING handoff remains stochastic. The
  deepest measured rung remains SHIFT/SWING; ADVANCE, rear legs, CLEAR,
  RESUME, and complete crossing remain unmeasured.
validation: |
  Final cmake build and ctest passed 27/27. Final B0 fixed pair under the
  required serial lock and dds_base8000 preload returned acceptance_status
  PASS. The final canary pair used domain 229, dds_base4000 preload,
  controller duration 30 s and wall timeout 35 s.
verdict: |
  No complete crossing or confirmation. The remaining three-cycle stop rule
  is not met because the canary pair signatures differ; budget accounting
  retains three nominal pair slots, although several exploratory pairs were
  consumed while converging the activation fix.
git_status: implementation and evidence changes are unstaged; no push or amend; simulations serialized.


---
timestamp: 2026-09-01T00:30:00+0800
run_id: Order-044 decisive flat-ground crawl isolation (epoch44 + r2-r6)
trigger: T1
housekeeping: |
  HEAD abd5c10 already contained the Order-043 implementation and evidence;
  git status was clean before this order, so no prior evidence was amended.
  The new isolation implementation and this section are committed separately.
implementation: |
  Added the contract-invisible environment harness flag
  TROT_TERRAIN_DEBUG_FLAT_CRAWL (default off). When enabled after gait start,
  the terrain sequencer is armed from t=0 on phase2_flat.xml without lidar,
  planner, window activation, or terrain targets. Flat targets are measured
  foot plus 0.08 m in the yaw-forward direction; measured commits are latched
  in sequencer telemetry. The flat path holds measured support feet and sends
  only the sequencer swing target to the gait/WBC. Existing terrain callers
  retain the map path. Added a flat sequencer unit witness and telemetry for
  flat mode, commit mask, and support COM margin.
canary_command: |
  Serial flock -x /tmp/go2_mujoco_experiment.lock; domain 229;
  TROT_TERRAIN_DEBUG_FLAT_CRAWL=1 run_trot.sh 18 --headless
  --wall-clock-motion --controller-duration 12 --wbc-full
  --gait-pattern running-trot --kernel raibert-trot --period 0.50 --duty 0.75
  --step-length 0.15 --foot-lift 0.08 --tau-limit 45
  --velocity-command-script configs/phase2_b1_velocity_0p3.csv
  --scene-file unitree_robots/go2/phase2_flat.xml --domain-id 229.
  Six serial exploratory runs were recorded: epoch44, _r2, _r3, _r4, _r5, _r6.
flat_trace: |
  The final post-fix run epoch44_r6 produced 9,953 CSV rows and a controlled
  flat sequencer interval from t=4.300 s to the first ABORT at t=5.012 s
  (0.712 s, far below the required 5 s). It reached STAGE -> SHIFT -> SWING
  for active leg 1, then lost the three-contact witness during the first
  swing and aborted; no COMMIT, ADVANCE, rear-leg, CLEAR, or RESUME event
  occurred. Across the six runs, no sequencer/legacy commit was recorded.
  In epoch44_r6 SWING telemetry the measured-contact count ranged 1..4;
  the best support COM margin was +0.0510 m. Before the abort, posture
  envelope was |roll| <= 0.00708 rad (0.41 deg), |pitch| <= 0.05785 rad
  (3.32 deg). After abort the inherited stop path eventually inverted, so
  its post-abort roll is not used as controlled-walk evidence.
verdict: |
  FLAT CRAWL CANNOT WALK: 0/6 runs reached 5 s, 0/6 recorded a measured
  commit, and 0/6 reached ADVANCE. The decisive failure is execution-layer
  support retention at the first swing (the controlled witness falls below
  three contacts), not terrain perception or foothold selection. The flat
  path therefore requires execution-layer repair before any terrain canary
  migration or gate interpretation.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed. Flat unit witness passed. The required
  B0 fixed pair was attempted serially, but domains 222/223 again failed DDS
  participant allocation before startup; no B0 result is claimed. No v1
  contract, analyzer threshold, or canary definition changed; no push.
git_status: implementation committed as b38c1de plus documentation correction 5a1d303; no push/amend; simulations serialized.

---
timestamp: 2026-09-01T03:00:00+0800
run_id: Order-045 flat execution repair (fix1-fix17) plus terrain canary epoch108/r2
trigger: T1
implementation: |
  Flat SWING now keeps the explicit sequencer topology through MPC and ID-WBC
  even while the legacy crawl state machine is still in DECELERATE_TO_CREEP.
  The ID-WBC stance floor is therefore 20 N on the three declared stance legs,
  instead of the 1 N default that previously unloaded RR/RL during FL SWING.
  Flat landing separates pose arrival from force confirmation: SWING enters
  COMMIT at the measured endpoint, COMMIT publishes four contacts, and the
  next force-filter sample confirms touchdown. Support feet remain at measured
  anchors while the landing leg is held at its endpoint. Flat mode gets a
  measured four-leg continuous-cycle witness and harness-local body creep;
  ordinary gait defaults are unchanged. Added per-leg final ID-WBC normal
  force and sequencer contact schedule telemetry.
  B0 wrappers now clean stale cdds*, cyclonedds*, and iceoryx* objects in
  /dev/shm before start, between pair members, and on exit.
flat_iterations: |
  fix1-r2: support floor fixed the first SWING allocation (stance Fz >=20 N),
  but endpoint had no force witness; fix2-fix4 separated landing/commit and
  corrected landing hold. fix5-fix7 completed the first multi-leg sequence;
  fix8-fix16 tuned the flat-only continuous cycle and body creep; fix17 is the
  acceptance witness. In fix17, the first 10 consecutive SWING events span
  state ticks 6.022-7.276 s, min measured contacts=4, max |roll|=0.0485 rad,
  max |pitch|=0.0238 rad, and base x advanced -0.0957 to -0.0548 m
  (+0.0408 m). The full run recorded 30 SWING/COMMIT events and no controlled
  support loss before the later harness stop-path inversion.
terrain_canary: |
  b1_freegait_epoch108 and epoch108_r2 were serialized on domain 229 with
  Base=4000 and 30 s controller/35 s wall limits. Both migrated into STAGE;
  r2 reached SHIFT/SWING before the inherited terrain-side posture failure
  (roll about -179.7 deg, pitch about -7.2 deg). No terrain success is claimed.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed. bash -n passed for both B0 wrappers;
  git diff --check passed. Flat harness remained environment-gated and default
  off; v1 contract, analyzer thresholds, canary definitions, and B0 command
  bits were not changed. No push or amend.
git_status: implementation and evidence changes are unstaged; simulations serialized.

---
timestamp: 2026-09-01T03:35:00+0800
run_id: Order-045 B0 cleanup verification (development steps reruns)
trigger: T1
validation: |
  Two serial B0 development steps runs were launched with the Base=4000
  preload after the new /dev/shm cleanup. Both allocated DDS participants and
  started the controller (no participant-index exhaustion); the first ran to
  94.8 s with controller/safety status 0 and strict legacy pass, but the
  frozen quantitative analyzer rejected steady-state error. The second hit an
  unrelated stochastic posture/safety failure at 31.7 s (roll ~180 deg).
  Thus the participant-index failure is mitigated, but B0 green-on-demand is
  not claimed from these two runs; the frozen B0 analyzer/contract was not
  changed.
git_status: documentation update is unstaged; no push or amend; simulations serialized.

---
timestamp: 2026-09-01T15:15:00+0800
run_id: Order-046 reviewer remediation, B0 triple rerun, and terrain transfer epochs 111-116
trigger: T1
implementation: |
  Commit 5adf860a6b238d97d01f1fb88ceca5fae84fbad3 requires measured normal
  force support for crawl commit/advance predicates in the script, state
  machine, and sequencer; shift readiness now also requires force support.
  Failed steps take precedence over commit latching. Script invalid time and
  sequencer STAGE timeout fail closed. Staging now exposes a rotated full
  world target and script foothold edge detection is constrained to a forward
  corridor. Sequencer force observations are populated from measured foot
  forces. Existing contract, analyzer thresholds, and canary definitions are
  unchanged.
b0_fixed_pair: |
  Three serial runs under flock -x /tmp/go2_mujoco_experiment.lock with
  LD_PRELOAD=/home/che/dds_base4000_preload.so, development 0, domains 222/223,
  all built from exact SHA 90bd1f0b67d4a07f9737cb6c0d85d9614e3276b1:
  2026-08-30T14:43:43+08:00 run 144343 PASS;
  2026-08-30T14:46:35+08:00 run 144635 PASS;
  2026-08-30T14:49:25+08:00 run 144925 PASS. Each frozen b0_analyzer
  acceptance_status=PASS and controller/dynamics/quality/safety/analysis=0.
terrain_canary: |
  Six serialized pairs on domain 229, Base=4000, --controller-duration 30,
  wall timeout 35, scene phase2_step_5cm.xml, exact binary source SHA
  5adf860a6b238d97d01f1fb88ceca5fae84fbad3. Monotonic start timestamps and
  run directories: epoch111 15:08:52/15:09:32; epoch112 15:09:44/15:10:25;
  epoch113 15:10:37/15:10:50; epoch114 15:11:30/15:11:42; epoch115
  15:11:55/15:12:09; epoch116 15:12:49/15:13:04 (r1/r2). Outcomes were
  stochastic: controlled-stop members reached 30 s, other members stopped on
  safety/motion rejection. The deepest sequencer rung was SHIFT/SWING entry
  (epoch115_r2); no measured terrain commit, ADVANCE, CLEAR, RESUME, or
  complete crossing was observed. No confirmation run exists.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed. git diff --check passed. Simulations
  remained serialized; no push or amend.
git_status: implementation and evidence are committed locally; generated canary runs are ignored; no staged files.

---
timestamp: 2026-09-01T15:20:00+0800
run_id: Order-046 post-canary force-gate completion
trigger: T1
implementation: |
  Follow-up commit ab8b382 gates the state-machine SHIFT_COM/CrawlStep
  commit and ADVANCE paths on the same measured normal-force witness, and
  ensures a failed step cannot be promoted by a latched commit. This is a
  narrow in-window predicate hardening after the six canary pairs above;
  their binary SHA remains recorded as 5adf860a6b238d97d01f1fb88ceca5fae84fbad3.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed; git diff --check passed.
git_status: source and evidence commits are local; no push/amend; no staged files.

---
timestamp: 2026-09-01T16:35:00+0800
run_id: Order-047 arm-to-seizure verification, V2-A approach brake, and canary epochs 117-122
trigger: T1
forensics: |
  Replayed the committed epoch111-116 CSV artifacts. In all 12 members the
  arm-to-seizure gap was 2.666-3.478 s (mean 3.119 s), while arm-to-safety-stop
  was 3.628-4.884 s. During the arm-to-stop interval the front-foot world X
  reached 0.650-0.701 m and Z was 0.019-0.072 m; contact bit transitions were
  recorded at X >= 0.65 m (for example epoch111 r1 at t=7.816 s, FL contact,
  x=0.679 m, z=0.055 m). The riser edge is x ~= 0.70 m. The body advanced
  0.663-0.711 m (mean 0.694 m) before stop. This confirms front-foot/riser
  approach contact during the passive arm-to-seizure gap, rather than a
  post-seizure crawl failure.
implementation: |
  Commit 723a3cd538c9efc4f6f2151c82ae8bc5b48b9c99 adds a window-local V2-A
  adaptive approach envelope while trot retains full authority. The profile
  uses vmax=0.30 m/s, allowed decel=1.20 m/s^2, braking distance
  0.30^2/(2*1.20)=0.0375 m, canonical standoff=0.25 m, and margin=0.10 m;
  TransferActivationReady therefore arms at 0.3875 m. The speed cap is the
  minimum of the outer sqrt distance profile and sqrt(2*a*remaining), refreshed
  from the live map staging reference. It starts at arming, before the
  phase-respecting stand/seizure boundary; 3+ trot contacts and speed <=0.04
  remain the seizure predicates. Flat debug/default paths do not consume it.
validation: |
  cmake --build example/cpp/build -j2 and ctest --test-dir
  example/cpp/build --output-on-failure passed 27/27. New unit witnesses cover
  the distance budget, profile reduction, and stopping envelope. A serial B0
  fixed pair under flock -x /tmp/go2_mujoco_experiment.lock, domain 222/223,
  Base=4000 preload, built at the exact SHA above, returned acceptance_status
  PASS with controller/dynamics/quality/safety/analysis status 0. Frozen paired
  diagnostics retain their known non-gate period/duty/acceleration differences.
canary_command: |
  Six serial pairs used flock -x /tmp/go2_mujoco_experiment.lock, domain 229,
  LD_PRELOAD=/home/che/dds_base4000_preload.so, run_trot.sh 35,
  --controller-duration 30, --wbc-full, running-trot/raibert-trot,
  period .50/duty .75/step .15/lift .08, phase2_step_5cm.xml. Every binary
  was built from exact SHA 723a3cd538c9efc4f6f2151c82ae8bc5b48b9c99.
canary_cycle_117: |
  b1_freegait_epoch117 and _r2: arm->seizure 2.660/2.800 s; arm->stop
  3.366/3.510 s; arm->stop body travel 0.353/0.362 m. Both reached SWING,
  no measured commit; both ended ABORT (r1 controlled wall completion).
canary_cycle_118: |
  b1_freegait_epoch118 and _r2: arm->seizure 2.538/2.680 s; arm->stop
  3.172/3.432 s; body travel 0.330/0.350 m. Both reached SWING, no commit,
  then ABORT.
canary_cycle_119: |
  b1_freegait_epoch119 and _r2: arm->seizure 2.120/2.122 s; arm->stop
  3.138/2.686 s; body travel 0.311/0.324 m. Both reached SWING, no commit,
  then ABORT.
canary_cycle_120: |
  b1_freegait_epoch120 and _r2: arm->seizure 2.536/2.262 s; arm->stop
  3.312/2.908 s; body travel 0.361/0.334 m. Both reached SWING, no commit,
  then ABORT.
canary_cycle_121: |
  b1_freegait_epoch121 and _r2: arm->seizure 2.532/2.120 s; arm->stop
  3.208/2.760 s; body travel 0.344/0.321 m. Both reached SWING, no commit,
  then ABORT.
canary_cycle_122: |
  b1_freegait_epoch122 and _r2: arm->seizure 2.262/2.680 s; arm->stop
  2.906/3.314 s; body travel 0.335/0.335 m. Both reached SWING, no commit,
  then ABORT.
result: |
  The approach-gap mechanism is removed quantitatively: new arm-to-stop
  travel is 0.311-0.362 m, with no front-foot x >= 0.65 m during the
  arm-to-stop interval in any of the 12 new members; old travel was
  0.663-0.711 m with riser-near contact transitions. The deepest rung in all
  new members is SWING; no COMMIT, ADVANCE, CLEAR, RESUME, complete crossing,
  or confirmation run was observed. No gate conclusion is made.
git_status: implementation commit and this evidence append are local; no push or amend; no staged files.

---
timestamp: 2026-09-01T18:30:00+0800
run_id: Order-048 terrain SWING/COMMIT isolation, patch and support hardening
trigger: T1
forensics: |
  Order-047's 12 exact-SHA members (epochs 117-122, binary SHA
  723a3cd538c9efc4f6f2151c82ae8bc5b48b9c99) all entered SWING with FL
  active and committed_mask=0, then ABORT; no COMMIT/ADVANCE/CLEAR was
  observed. The map-selected FL target was x=0.4160-0.4709,
  y=0.1334-0.1465, z=0.02180-0.02248 m. The true platform contact patch
  is z=0.050 m, hence the expected FK/WBC foot-site endpoint is 0.072 m;
  the observed target was 49.5-50.2 mm low. Active-foot contact was still
  present at swing entry, then first fell at t=+0.004 to +0.058 s at
  x=0.4338-0.4684, z=0.0223-0.0242, force=0-4 N; any re-rise was only
  0.004-0.058 s later at 5-52 N. Endpoint samples stopped at
  x=0.4326-0.4664, y=0.1068-0.1395, z=0.0224-0.0291, leaving horizontal
  errors -16.6..+32.5 mm and vertical errors +0.4..+6.8 mm versus the
  (wrong) commanded target. The commit predicates therefore failed on
  measured endpoint/contact/force, not on flat execution.
comparison: |
  Order-045 flat fix17 has 10 consecutive SWING events at 6.022-7.276 s,
  min measured contacts=4, max |roll|=0.0485 rad, max |pitch|=0.0238 rad,
  and +0.0408 m body progress. Flat targets remained on the measured
  support z (~0.023 m), while terrain targets were map-selected and had
  the elevated-surface mismatch above.
implementation: |
  Commit 67338e3299821d135bcffdd90bbb8e9687da4adb applies the calibrated
  ContactPatchToFootSite conversion at the direct sequencer handoff;
  requires a persistent, two-row lateral-consensus edge (with a wider
  edge-observation band but unchanged narrow foothold corridor); raises
  terrain-only sequencer lift to 0.08 m while flat remains 0.015 m; and
  uses a terrain-only 30 N ID-WBC stance floor (flat remains 20 N).
  Added unit witnesses for isolated map cells and the 0.050->0.072 m
  patch/site conversion. v1 contract, analyzer thresholds, and canary
  definitions were untouched.
canary_cycle_130: |
  Two serial runs b1_freegait_epoch130 and _r2 used flock -x
  /tmp/go2_mujoco_experiment.lock, domain 229, Base=4000 preload,
  run_trot.sh 35, --controller-duration 30, and were both built from
  exact SHA 67338e3299821d135bcffdd90bbb8e9687da4adb. Both reached
  STAGE->SHIFT->SWING only, then ABORT at t=8.462/8.936 s; targets were
  z=0.02262/0.02279 m and commit count stayed 0. Neither crossed.
result: |
  Terrain SWING commit remains precisely stuck: all 14 post-change
  attempts (epochs 123-130) have zero terrain commits and no complete
  crossing or confirmation. The repeated signature is active-leg FL,
  map target near z=0.022-0.023 m, support/contact loss within 0.3 s,
  and no COMMIT; three same-signature cycles were stopped. The likely
  remaining dominant term is lidar target selection/content (map target
  is still ground-level and x~0.44-0.46 while the scene riser begins
  x=0.70), not apex or the commit force predicate. No gate conclusion.
validation: |
  cmake --build example/cpp/build -j2; ctest --test-dir
  example/cpp/build --output-on-failure: 27/27 passed. git diff --check
  passed before commit. Simulations were serialized; no push or amend.
git_status: implementation committed locally at 67338e3; evidence append is unstaged; no staged files.

---
timestamp: 2026-09-01T20:10:00+0800
run_id: Order-049 map dump and sequencer target handoff
trigger: T1
forensics: |
  STAGE dump from b1_freegait_epoch134 (controller built from d6a600a): TerrainModel source=lidar, frame=base_link, origin=(-0.45,-0.225), resolution=0.05, dims=32x10, known=320/320, height range [-0.375251,-0.325251] m. The map contains the plateau: local cells x=0.475..0.925 carry the +0.050 m surface; x>0.70 has 45 elevated cells in that snapshot (b1_freegait_epoch135: 50 cells). The window is therefore not missing the step: the 1.60 m forward local map covers the world riser and its 1.5 s world-cell memory retains it.
implementation: |
  The event sequencer's measured platform target was being published while an older planner foothold remained prepared and the legacy SHIFT_COM execution gate suppressed the new target. The handoff now replaces only a not-in-flight pending target at the SWING/COMMIT boundary and lets the sequencer's measured target drive the terrain transaction. Sequencer SWING waits for the measured legacy COM/force readiness witness (COM margin >= 0.020 m and three-leg force support: each >=10 N, total >=50 N, imbalance <=4x); fixed flat behavior is bypassed.
canary_cycle_131_139: |
  Epoch131 dump confirmed platform (50 elevated x>0.70 cells), but remained on the pre-fix path. Epoch132-133 captured direct platform targets (world x=0.823/0.817, z=0.074/0.095) but stale execution/SHIFT_COM prevented usable swing. Epoch134-135 verified the pending-target handoff (execution target world x=0.831/0.830, z=0.094) and reached SWING; no commit, because measured alternate support dropped below force balance (e.g. FR=84 N, RR=0 N, RL=46 N). Epoch136 stalled before SWING while waiting for readiness. Epoch137-140 reached SWING after the measured gate; deepest rung remains SWING, no COMMIT/ADVANCE/CLEAR/RESUME/crossing/confirmation.
validation: |
  Serial lock/domain discipline used: flock -x /tmp/go2_mujoco_experiment.lock, domain 229, Base=4000 DDS preload, run_trot.sh 35, --controller-duration 30. cmake --build example/cpp/build -j2 passed; ctest --test-dir example/cpp/build --output-on-failure: 27/27 passed. No push/amend.
git_status: implementation and this evidence append are local and will be committed together; no staged files after commit.

---
timestamp: 2026-09-01T20:55:00+0800
run_id: Order-049 canary epoch141 post-commit
trigger: T1
canary_cycle_141: |
  Serial flock lock, domain 229, Base=4000 preload, run_trot.sh 35,
  --controller-duration 30; exact implementation SHA
  bd2d7348813e7b42f419fc0f775ebba585043f0e, git_dirty=false. The
  run reached STAGE->SHIFT->SWING at state tick 9.620, then ABORT at
  10.046; committed mask remained 0. No complete crossing or
  confirmation. This is a stuck report, not a gate conclusion.
validation: |
  The post-commit metadata records exact SHA and clean source. ctest
  remains 27/27 passed from the committed source; no push or amend.
git_status: evidence append is local and will be committed promptly; no staged files after commit.

---
timestamp: 2026-09-01T21:45:00+0800
run_id: Order-050 flat-vs-terrain differential, explicit topology repair, epochs 142-149
trigger: T1
forensics: |
  Existing telemetry was compared tick-by-tick for order045_flat_fix17
  (first SWING 6.022 s) and b1_freegait_epoch141 (first SWING 9.620 s).
  Flat had sequencer/WBC/MPC mask 13 and no terrain plan/hold; terrain had
  sequencer/WBC/MPC mask 13 but legacy terrain planned mask 12, captured
  hold mask 15, and an active surface transition. At terrain SWING the
  sequencer COM reference was x=0.1921,y=-0.0469 while the measured legacy
  shift target was x=0.2377,y=-0.0120; body velocity was -0.0923 m/s versus
  flat -0.0010 m/s. Terrain FL execution target was x=0.8101,z=0.0942,
  while the foot started x=0.4475,z=0.0228. The first actuator/control
  divergence was the unreachable terrain target being retained by WBC while
  AllLegInverseKinematicsClamped retracted stance and swing feet globally.
  In epoch145 the commanded target remained x=0.8101 while measured FL
  stayed near x=0.42-0.44; RR then fell to 0 N at t=10.240. This identifies
  the differential numerically; no gate conclusion.
implementation: |
  025872a holds the measured legacy COM target through terrain SHIFT_COM/
  CRAWL_STEP instead of repeatedly replacing it with the moving support
  centroid; 9c30147 prevents terrain planner/legacy future contact horizons
  and terrain-plan WBC metadata from overriding the sequencer-owned topology;
  a073666 makes clamped IK per-leg and adds a unit witness so an unreachable
  swing target cannot move stance anchors. Flat mode remains unchanged.
validation: |
  cmake --build example/cpp/build -j2 and ctest --test-dir example/cpp/build
  --output-on-failure: 27/27 passed after the implementation. Flat harness
  order050_flat_verify recorded 20 successful SWING/COMMIT events before its
  later harness stop-path abort, with 4 contacts on each successful event.
  Serial terrain runs used flock -x /tmp/go2_mujoco_experiment.lock, domain
  229, Base=4000 preload, --controller-duration 30, wall 35, exact source
  SHAs: epoch142 4fef4c5, epoch143 025872a, epoch144 1c8bf2d, epoch145
  9c30147, epoch146/147 a073666, epoch148 c4599f1, epoch149 c4599f1.
  Epochs 142-149 reached SWING only and aborted; no terrain COMMIT,
  ADVANCE, CLEAR, RESUME, crossing, or confirmation was observed. Deepest
  rung is SWING. No terrain commit is claimed; investigation remains open.
  No v1 contract, analyzer threshold, or canary definition changed; no push
  or amend; simulation runs remained serialized.
git_status: implementation and this evidence append are local; no staged files.
