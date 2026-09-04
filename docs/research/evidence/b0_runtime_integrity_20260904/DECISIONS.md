# B0 runtime-integrity decision log

Date: 2026-09-04. Status: open. This record explains the research forks on
`fix/phase2-b0-runtime-integrity`; it does not grant acceptance. Frozen Phase 2
thresholds and profiles were not changed. Raw artifacts remain immutable under
`example/cpp/experiments/_runs/`.

## Question

Why does current `main` fail the full B0 development suite although the
historical Order-109b lockstep slice passed, and what is the smallest coherent
repair that restores Phase 1 without introducing a forbidden terrain route?

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
- Decision: retain provisionally and run the complete B0 suite on one new
  exact clean SHA. This is evidence for the formulation, not yet acceptance.

## Current choice

Run the complete B0 suite on the exact clean candidate after this record is
committed. It must preserve steps and acceleration while repeating the brake
result and passing ramp, varying, and fixed 3 m/s. Stop at the first failure.
Only a full exact-SHA pass permits the Stage C shadow path and smallest dynamic
B1 slice to resume.

The raw run-manifest hashes used by this record are frozen in `MANIFEST.json`.
