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
