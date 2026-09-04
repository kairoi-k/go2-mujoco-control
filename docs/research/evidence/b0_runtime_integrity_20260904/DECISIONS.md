# B0 runtime-integrity decision log

Date: 2026-09-04. Status: open. This record explains the research forks on
`fix/phase2-b0-runtime-integrity`; it does not grant acceptance. Frozen Phase 2
thresholds and profiles were not changed. Raw artifacts remain immutable under
`example/cpp/experiments/_runs/`.

## Question

Why does current `main` fail the full B0 development suite although the
historical Phase 1 dynamic suite (15/15 across steps, acceleration, braking,
ramp, and varying commands) and the separate Order-109b lockstep slice passed,
and what is the smallest coherent repair that restores that behavior without
introducing a forbidden terrain route?

## Forks and decisions

### F0 — Current-main reproduction

- Revision: `8f4a58176abcae6ed94a20f8230e236f2990f18d`, clean.
- Intervention: none.
- Evidence: `phase2_b0_development_steps_r0_20260904_200114_terrain` and
  `phase2_b0_development_steps_r0_20260904_200728_{baseline,terrain}`.
- Result: one terrain member fell; the completed pair still failed ID-WBC
  validity (about 0.9946) and steady-state error. This establishes a current
  runtime regression, not that historical B0 never passed.
- Decision: diagnose contact/WBC and zero-command coherence before Stage C or
  B1 actuation.

### F1 — Reject isolated measured support

- Revisions: tested `91d7fcc36c266a5696bb5bc61467ea396f817a30`;
  clean-line equivalent `12a34aa`.
- Intervention: in running trot, reject a measured/scheduled union that would
  give ID-WBC exactly one support foot; retain scheduled diagonal support.
- Evidence: `phase2_b0_development_steps_r0_20260904_201531_{baseline,terrain}`.
- Result: steps completed on both members and ID-WBC validity rose from about
  0.9946 to about 0.9998, but the zero-speed plateau was still not exact.
- Decision: retain. It repairs an internally inconsistent contact handoff
  without changing gait topology or adding terrain authority.

### F2 — Scale foot lift to zero

- Revision: `2b5648a0721ac0208124df5b525c8cf06ae9529b`;
  archive ref `archive/b0-zero-lift-failure-20260904`.
- Intervention: scale low-speed foot lift continuously to zero while leaving
  the alternating contact schedule unchanged.
- Evidence: `phase2_b0_development_steps_r0_20260904_202234_{baseline,terrain}`.
- Result: both members fell at about 9 s; the schedule continued switching
  support with no matching foot motion.
- Decision: reject and revert. A zero command must be one coherent state, not
  a local trajectory-amplitude patch.

### F3 — Raise ID-WBC internal torque limit

- Revision: `fe1ef9917db9e9b201b835046eb99e03c9ebff6a`;
  archive ref `archive/b0-tau-limit-failure-20260904`.
- Intervention: align the internal QP limit with the runner's 45 Nm command
  limit.
- Evidence: `phase2_b0_development_steps_r0_20260904_202517_{baseline,terrain}`.
- Result: both members completed, but ID-WBC validity remained below 1.0 and
  the terrain member regressed. Raising authority did not remove the defect.
- Decision: reject and revert; keep the 35 Nm internal safety constraint.

### F4 — Freeze a coherent zero-speed stance

- Failed revision: `a485010fc77c82c262c215c7863a12e336bbd9c4`;
  archive ref `archive/b0-stance-wbc-failure-20260904`.
- Final revisions: tested `8b079702df79febd54670a5dd69918dfd8c2ebde`;
  clean-line equivalent `95d1677`.
- Intervention: atomically freeze phase, gait trajectory, contact consumers,
  MPC and WBC at a measured multi-foot stance. The first version left the
  locomotion ID-WBC as plant authority and fell at about 8 s; the final version
  keeps ID-WBC diagnostic and uses the existing stand hold until a nonzero
  command releases the same running-trot controller.
- Evidence: `phase2_b0_diag_steps_r1_20260904_2046_baseline` (failed) and
  `phase2_b0_diag_steps_r2_20260904_2050_baseline` (completed, ID-WBC 1.0).
- Decision: retain the final version. It defines Phase 1 zero-velocity
  semantics and is not a terrain trigger, crawl, preload gate, or stop-to-arm
  transfer.

### F5 — Bound sustained overspeed through existing authority

- Revisions: tested `9f97f6457ad86481054058476405bd9fa3f4142a`;
  clean-line equivalent `803e7f6`; archive ref
  `archive/b0-steps-pass-20260904` points to the exact tested revision.
- Intervention: strengthen only the existing applied-velocity governor; the
  Phase 1 shaper remains the sole horizontal velocity authority.
- Evidence: `phase2_b0_diag_steps_r3_20260904_2057_{baseline,terrain}`.
- Result: both steps members passed the Phase 1 quantitative analyzer with
  ID-WBC validity 1.0. This is development evidence, not full B0 acceptance.
- Decision: retain and rebuild a clean candidate line.

### F6 — Exact-candidate full B0 attempt

- Revision: `803e7f6f37286a173031ca2438c55041242e7acf`, clean; build and CTest
  32/32 passed.
- Acceleration evidence:
  `phase2_b0_development_accel_1_to_3_r0_20260904_205900_{baseline,terrain}`.
  Both completed with ID-WBC validity 1.0; terrain passed. The paired baseline
  exceeded only its diagnostic torque-saturation metric (0.003197917 versus
  0.003).
- Brake evidence:
  `phase2_b0_development_brake_3_to_0_r0_20260904_210106_{baseline,terrain}`.
  Baseline flipped at about 10.22 s. Terrain completed and passed motion gates
  but had two rejected ID-WBC ticks (validity 0.9999019704), so the frozen gate
  failed.
- Decision: stop the suite at the first information-bearing failure. Do not
  claim B0 or start B1 actuation. Diagnose the first rejected QP/contact-time
  divergence; do not widen thresholds, raise torque, or rerun blindly.

### F7 — Rejected: bounded ADMM recovery

- Question: are the isolated brake failures caused by the 120-iteration ADMM
  cap rather than by an infeasible torque/contact problem?
- Evidence motivating the fork: every rejected candidate exhausted 120
  iterations with an equality-accurate finite solution. The two terrain
  failures were isolated at diagonal-support transitions; their torque-limit
  excesses were 22.670340392 Nm and 0.084780863 Nm, and the next tick returned
  to a feasible solution. The largest event followed a 0/4 ms state delivery
  interval but did not destabilize the plant.
- Single intervention: retain the same objective, constraints, 35 Nm internal
  limit, contact mask, gait and acceptance gates; retry only a rejected finite
  solve with a bounded larger iteration budget. Record whether recovery ran
  and its total work.
- Criterion: focused solver/WBC tests pass; one exact-SHA brake development
  pair has lifecycle and frozen B0 PASS, ID-WBC validity 1.0, and solver budget
  fraction at least 0.80. Stop and reject this fork at the first failure.
- Interpretation limit: a pass supports bounded numerical recovery only. It
  does not validate B0, B1, contact timing changes, or DDS scheduling.
- Revision: `faa432c8f4d46f618d28ec61f8a34c89a5141bca`; archive ref
  `archive/b0-admm-recovery-failure-20260904`; reverted by `c8761bb`.
- Evidence: `phase2_b0_development_brake_3_to_0_r0_20260904_211440_{baseline,terrain}`.
- Result: baseline passed with no recovery calls. Terrain fell at about
  12.67 s; 606 recovery calls occurred, 59 candidates remained invalid,
  ID-WBC validity was 0.9906896008, and solver-budget validity was 0.8365157014.
  The first recovery appeared only after the trajectory had developed a new
  failure cluster; a larger iteration cap was therefore not a sufficient or
  isolated repair.
- Decision: reject and revert. Do not spend another fork on iteration count;
  next isolate why a nominal contact schedule can remain disconnected from
  measured support and why stale fallback torque crosses contact epochs.

### F8 — Development-supported: equality-nullspace feasibility recovery

- Question: are isolated transition failures caused by the mixed-scale KKT
  formulation rather than physical infeasibility?
- Single intervention: when the ordinary finite candidate violates a hard
  inequality, solve the identical objective and constraints in the nullspace
  of the floating-base equality, with inequality-row normalization. This is a
  different formulation of the same QP; torque cap, gait, contacts, command,
  thresholds and the ordinary successful path remain unchanged. Record use,
  correction size, equality residual and final constraint violation.
- Criterion: deterministic unit cases prove equality preservation and hard
  feasibility; one exact-SHA brake pair has lifecycle and frozen B0 PASS,
  ID-WBC validity 1.0, and solver-budget fraction at least 0.80. Stop at the
  first failure. A pass still requires later full-B0 regression.
- Interpretation limit: this can establish numerical formulation robustness;
  it cannot justify stale cross-contact fallback, contact fabrication, B0 or
  B1 acceptance.
- Revision: `b74552fb018a4d7308fcb3d3d03212af8c58fe41`, clean.
- Evidence: `phase2_b0_development_brake_3_to_0_r0_20260904_212336_{baseline,terrain}`.
- Result: paired brake B0 PASS. Both members completed with ID-WBC validity
  1.0; solver-budget validity was 0.9197137536 and 0.9198647258. Recovery was
  exercised twice in baseline and once in terrain. All three recovered solves
  became equality/inequality feasible; maximum final recorded inequality
  violation was 0.000004820, with recovery corrections of 0.8793 to 2.4255.
- Exact-candidate regression: `8a1924855cc167a6ee971086bc86deaea71b859e`
  built cleanly and passed CTest 32/32. The steps pair
  `phase2_b0_development_steps_r0_20260904_212752_{baseline,terrain}` passed
  with ID-WBC validity 1.0 and no recovery calls. In the next ordered profile,
  `phase2_b0_development_accel_1_to_3_r0_20260904_213214_{baseline,terrain}`,
  baseline passed but terrain failed: measured zero-contact fraction was
  0.2553872306 versus the frozen 0.25 limit, settling time was undefined, and
  diagnostic torque saturation was 0.0032036859 versus 0.003. ID-WBC validity
  remained 1.0 and recovery was never used in either member.
- Decision: retain the numerically isolated recovery, but reject this candidate
  as B0. Stop the suite before ramp, varying, and fixed-3-m/s. The failure is
  physical touchdown/tracking sensitivity exposed by the terrain sensor path,
  not the recovery formulation; diagnose it without changing frozen gates or
  contact labels.

### F9 — Preregistered diagnosis: commanded versus measured touchdown

- Question: does high-speed physical contact loss come from commanded swing
  geometry, vertical tracking lag, or contact sensing after the nominal
  touchdown boundary?
- Intervention: none. Reconstruct commanded and measured foot position and
  velocity from the immutable F8 acceleration pair, align them to gait phase,
  and compare baseline with terrain for every leg around touchdown.
- Criterion: identify a repeatable, leg/phase-localized discrepancy before any
  source change. If the evidence is ambiguous, add telemetry only and rerun the
  acceleration pair. Do not change duty, period, swing timing, contact masks,
  thresholds, or velocity authority.
- Interpretation limit: this diagnosis may select the next hypothesis; it
  cannot establish B0 or authorize a trajectory change.
- Method: for all samples with shaped command at least 2.8 m/s, apply the
  repository FK to each leg's logged target and measured joint angles. Align
  them with the logged gait phase and the running-trot offsets; for each of 150
  nominal touchdown boundaries per leg, measure first force-backed contact.
- Result: the target swing-height range was 199.5--199.6 mm while the measured
  range was only 103.6--112.7 mm. At nominal touchdown, target contact-patch
  height was already -13.3 to -7.4 mm but the measured patch remained 29.8 to
  38.3 mm above ground: 41.1--47.4 mm of vertical lag. Mean force-contact delay
  was 22.1--28.4 ms in baseline and 23.3--29.8 ms in terrain; terrain's maximum
  reached 68 ms. Finite-difference target descent reached 11.2--11.8 m/s while
  measured descent reached only 5.4--7.7 m/s. The target requests 200 mm of
  flat-ground clearance inside a 78.4 ms swing window, so the delayed support
  is a reproducible reference-feasibility mismatch.
- Decision: F9 is conclusive; do not add telemetry or change contact sensing.

### F10 — Preregistered: feasible flat high-speed swing amplitude

- Question: can a physically trackable flat-ground vertical reference remove
  the touchdown delay without changing gait/contact timing?
- Single intervention: reduce only the continuous velocity scheduler's 3 m/s
  flat-ground swing lift from 0.200 m to 0.080 m. The latter is the existing
  high-speed gait clearance floor elsewhere in this controller; it reduces
  peak reference descent to about 4.7 m/s, below the measured 5.4--7.7 m/s
  capability. Preserve period, duty, phase, touchdown time, horizontal target,
  contact masks, velocity authority, solver, profiles, and gates. Terrain-
  specific clearance remains future Stage C planner output, not this flat
  schedule.
- Criterion: focused scheduler/kernel tests pass; one exact-SHA acceleration
  pair has lifecycle and frozen B0 PASS on both members, measured contact-loss
  fraction at most 0.25, finite settling, ID-WBC validity 1.0, and no worse
  diagnostic torque saturation. Stop at the first failure; do not sweep lift
  constants. A pass requires the complete exact-SHA B0 suite afterward.
- Interpretation limit: a pass repairs Phase 1 reference feasibility only. It
  does not establish terrain clearance, B0, B1, or authorize swing retiming.
- Invalid preliminary attempt: `phase2_b0_development_accel_1_to_3_r0_20260904_214453_{baseline,terrain}`
  used the old controller binary (`c8c91330...`) after the source commit and
  logged a 0.1999 m lift. Its result is retained only as a process-error
  record; it does not test F10.
- Result on exact clean `68ddc67061bdbd23ee3b154b09658392a9c171f0` after a
  complete rebuild: both members used the intended 0.080 m lift but failed
  before the controlled stop. Baseline stopped at 27.94 s and terrain at
  28.58 s after hard posture safety, with pitch crossing 20 degrees at about
  25.12 s and 25.76 s respectively. Contact loss before stopping was only
  0.1231 and 0.1463, but ID-WBC validity fell to 0.9894 and 0.9955; the
  logged contact pattern showed a swing leg striking early, then loss of the
  scheduled diagonal. The paired gate therefore failed despite improved
  contact-loss fraction.
- Decision: reject F10 and restore the 0.200 m high-speed reference. Amplitude
  alone trades delayed touchdown for premature swing-foot collision; do not
  sweep lift constants or alter contact labels.

### F11 — Preregistered: increase swing acceleration authority

- Question: with the collision-avoiding 0.200 m reference restored, is the
  remaining touchdown lag caused by the 50 m/s2 swing-acceleration clamp?
- Single intervention: set `FULL2_SWING_ACC=80` for the exact acceleration pair;
  keep `FULL2_SWING_KP=180`, `FULL2_SWING_KD=16`, all gait geometry/timing,
  contact logic, torque limits, solver and frozen gates unchanged. This raises
  only the Cartesian swing request cap and does not retime a swing.
- Criterion: controller and analyzer both report clean lifecycle; baseline and
  terrain each meet the frozen Phase 1 quantitative gate, ID-WBC validity is
  1.0, and terrain contact loss is at most 0.25. Stop at the first failure;
  no gain/cap sweep. A pass still requires a fresh full exact-SHA B0.
- Interpretation limit: a pass supports swing tracking authority only; it does
  not establish B0, obstacle clearance, B1, or Stage C actuation.
- Result on exact clean `f95349cc0928ee4ca62cc71d2f160ab5612803ab` with the
  recorded `FULL2_SWING_ACC=80`: both members completed and kept ID-WBC and
  solver validity at 1.0. Baseline contact loss was 0.2281 and terrain was
  0.2407, both within the contact gate. Baseline nevertheless exceeded the
  frozen 45-Nm saturation fraction at 0.0030845; terrain stayed below that
  gate at 0.0029670 but exceeded the positive speed excursion at 0.5052 m/s.
  Thus neither member supplied a complete quantitative pass, although the
  observed contact-loss improvement is useful evidence.
- Decision: do not promote the environment override or rerun it. F11 does not
  isolate a sufficient repair; the next review must address coupled runtime
  margin (swing authority, speed lead, and torque saturation) as one coherent
  control-path issue rather than tuning one cap at a time.

### F12 — Architecture review required before another probe

- Trigger: F9 localized the reference-feasibility mismatch; F10 rejected
  amplitude-only repair by early swing-foot collision; F11 improved contact
  loss but still failed independent frozen margins on the exact pair.
- Review question: can the current short-period running-trot reference and
  wall-clock execution path satisfy the frozen B0 margins with one shared,
  physically feasible plan, or must Phase 1 expose a bounded, time-indexed
  reference/authority contract before more tuning?
- Required output before source changes: reconcile target foot geometry,
  measured support, applied velocity, Cartesian swing request, torque margin,
  and scheduler timing on the same tick index; identify whether the failure is
  deterministic control feasibility or realtime delivery variance. No new
  canary, threshold change, contact fabrication, or local swing retiming is
  authorized until this review is recorded.
- Review evidence: F11 had identical source, controller, simulator, scene and
  profile hashes, but the wall-clock harness deliberately assigned different
  CPU topologies. Baseline used controller `3,4`, simulator `2`, and terrain
  worker `4`; terrain used controller `4`, simulator `2,5`, lidar `5`, physics
  and bridge `2`, and terrain worker `6`. Their state-gap p95 was 4 ms in both
  members, yet same-index paired diagnostics diverged by up to 2.748 m/s in
  applied/WBC target velocity, 0.0549 m in step length, and 4.3013 m/s2 in
  requested acceleration. This is sufficient realtime-path variance to make a
  near-boundary torque/overshoot result non-attributable to terrain logic.
- Review conclusion: the control-path hypothesis is not yet identifiable from
  asymmetric wall-clock pairs. First hold the F11 reference and solver fixed,
  make both members use the same explicit CPU placement, and repeat one accel
  pair. This is a harness-validity probe, not a gate relaxation.

### F13 — Preregistered: symmetric wall-clock pair

- Question: does explicit identical CPU placement remove the F11 paired
  command/timing divergence and make the remaining B0 margins attributable?
- Single intervention: retain exact source `f95349c`, `FULL2_SWING_ACC=80`,
  200-mm high-speed lift, and every controller setting; set both members to
  `TROT_CPU_AFFINITY_CTRL=4`, `TROT_CPU_AFFINITY_SIM=2,5`,
  `TROT_CPU_AFFINITY_WRITER=3`, `TROT_CPU_AFFINITY_TERRAIN=6`,
  `TROT_SIM_LIDAR_CPU=5`, `TROT_SIM_PHYSICS_CPU=2`, and
  `TROT_SIM_BRIDGE_CPU=2`. No source, profile, scene, threshold, contact or
  timing change is allowed.
- Criterion: both members complete with lifecycle zero; paired applied/WBC
  velocity, gait duty, step length and lift remain within the analyzer's
  declared tolerances; each member meets the frozen accel Phase-1 gate. Stop
  at the first failure. A pass only validates the harness/control comparison;
  it still requires the complete exact-SHA B0 suite.
- Interpretation limit: this cannot establish B0, B1, or realtime hardware
  performance; it only determines whether the previous wall-clock pair was
  experimentally identifiable.

### F13 result

- Run: `phase2_b0_development_accel_1_to_3_r0_20260904_220538_{baseline,terrain}`
  with the preregistered identical CPU placement and unchanged F11 source.
- Both members completed with lifecycle and safety status zero. Baseline
  settling time was 11.702 s and terrain was 10.724 s, both above the frozen
  10-s bound; every other Phase-1 quantitative check was true.
- The paired analyzer still reported material same-index differences: applied
  velocity 0.232 m/s, gait duty 0.0105, lift 0.00987 m, step length 0.0369 m,
  requested acceleration 1.693 m/s2, and WBC velocity target 2.625 m/s.
- Decision: reject F13. Identical CPU placement did not make the wall-clock
  pair attributable; stop parameter probes and review the runtime time-index
  contract before another canary.

### F14 — Active: runtime time-index contract review

- Trigger: F13 completed safely but missed settling on both members and
  retained large same-index command/trajectory differences.
- Scope: trace simulation tick, controller tick, state timestamp, command
  application, and analyzer sample index as one causal chain; test a shared
  simulation-tick barrier or a recorded/replayed time-indexed schedule.

- Reference check: fresh standalone `varying` passed on old `524680be`
  (run-manifest SHA `2ac305dc8a6cd931dc17473a8f277af2e669b579ff904f58c54f3381d1217178`)
  and also on the current integrated head `a305de3` (run-manifest SHA
  `43a5b2bf0c7cf7fab5da19ad2fb085d950aacb6535ab6ce7cc2d1d29affb1b40`).
  Old artifact root: `/home/che/dev/go2-workspace/reference/phase1-20260904/example/cpp/experiments/_runs/phase1_baseline_repro_20260904/varying_20260904_233010`;
  current artifact root: `example/cpp/experiments/_runs/phase2_current_regression_varying_20260904/varying_20260904_233256`.
- Interpretation: this does not show a blanket Phase-1 variable-speed
  regression; the blocker is localized to the current B0 paired
  terrain-sensor/acceleration path and its wall-clock attribution.
- Decision: preserve all frozen B0 thresholds and profiles; no further
  parameter probe or B1 canary until this contract is evidenced.

## Current choice

Do not execute another parameter probe or B1 canary yet. Complete F14's
runtime time-index review and focused contract test first; only a fresh full
exact-SHA B0 pass permits the Stage C shadow path and smallest dynamic B1 slice
to resume.

The raw run-manifest hashes used by this record are frozen in `MANIFEST.json`.
