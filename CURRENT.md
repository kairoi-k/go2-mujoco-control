# Go2 current research checkpoint
Updated2026-09-08. Only live route/status/next-action entrypoint.
## Outcome and exact boundary
Resume base0349725c90cf5e3a15addfc8715a32f21a3a98bd on feat/stage-c-joint-planner,
native /home/che/dev/go2-workspace/feat-stage-c-joint-planner.
**Genuine5cm dynamic running-trot B1 remains NOT_CERTIFIED;10cm not started.**
The NEW full-body backend now sustains initialized flat20cycle feedback and the
two registered small perturbations within physical limits. These are privileged
model/state replays, NOT the live controller, observed-terrain planning or B1.
The actual integrated centroidal/WBC path last ran at0f6ec6526f4fcde77f2b737b5a65d9855ed5365b
(attempt0012), stopping at21.204s for roll28.80deg. No newer live controller run.
Full-body diagnostics have not replaced that production path or its acceptance.
## Native horizon continuation (not live acceptance)
Latest executed clean54d750eb4b1730db9041abd384d126dbe1f7a028:
fixed12ms committed prefix +20ms optimized tail, single20cyclevy replay0002.
All1400steps and233candidates complete, no late/rejection; generation median3.169ms,
max11.234ms. Actual and candidate independent full-state/force replay errors0;
committed law algebra error<=4.44e-16. Force179.99999999999952N,torque28.35590Nm,
minheight0.36753384m, nonfootcontacts0;19/19complete running-contact cycles.
Frozen V1 retains FAIL time1.55209e-12 and dynamics3.98340e-7,19other checks pass.
This is synchronous initialized privileged flat; no live realtime certificate,
nonprivileged production seed, observed-model rollout or B1 acceptance.
Source packets and original full40MB run are losslessly archived (xz compression)
in whole_body_native_20260908/rolling_0002. Earlier20ms dirty-source probe stays
separate. CMake tests cover native torque contract, byte-bound research packet,
and observed collision descriptors;23focused Python tests pass.
New worker-local observed model compiles robot-only XML plus cell-prisms and
rejects imported scene floor. Tests retain inertial/actuator/collision parameters.
Robot XML/mesh hashes remain a trusted caller responsibility. Next is real
recorded snapshot full-body coverage and reconstruction, then live shadow/owner.
See WHOLE_BODY_NATIVE_HORIZON_V1.md. No production owner edits.
## Latest executed evidence
- Clean0349725 single-cycle170N-objective solve0002: force170.120N<180N,
  torque28.153Nm, terminalbody0.883mm; exact saved-state replay. Original V1
  FAILS dynamics2.287774e-7>1e-7. Whole finite differences used99.67s.
- Contact-boundary audit identifies native1e-6 axis perturbations crossing FR
  (geom32) inclusion atstep32/time21.086. Whole-model manual FD agrees with
  native; propagated warmstart is not the main cause. Checked local derivatives
  reject unresolved points;3directions x3scales yield1.01e-5..2.92e-5 whole-chain
  discrepancy.69steps choose1e-7;step32 chooses1e-9 after adjacent-scale checks.
- Clean6a996ce ordinary TVLQR:20periods,19/19complete running-contact cycles,
  force171.989N, no torque clipping. Registered+vy0.05m/s converges but one RR
  sample reaches188.715N. A0.337mm foot-height shift misses prior contact
  deceleration; same-state removal of feedback increases195.916N. This failure
  is preserved. Nominal gain/reference/certificates do not imply hard constraints.
- Clean714bf3 constrained feedback:20period nominal,+vy0.05m/s,+roll0.02rad
  all complete1400steps,19/19complete running-contact cycles, physical/terminal
  checks pass. vy needs one correction, peak179.999994498747N; roll173.229N.
  Independent4200step pre/post-force replay has zero discrepancy/violation.
  One-step constraints are not horizon viability. Original V1 still FAILS
  dynamics and accumulated floating-clock tolerance; no threshold was changed.
- Latest executed clean4023ff87732236b629a59c80f886bb2f22126248, constrained0004:
  full20cyclevy repeat with integration-state scratch restoration. All1400
  controls and every recorded time/q/v/force/torque row equal714bf3 attempt0002
  EXACTLY. Full-law p50/p95/max789.307/1041.3901/17863.507us. Thus ordinary
  ticks improve, but the constraint solve still exceeds2ms. Five fixed-state
  paired benchmarks independently reduce median solve134.075->18.108ms with
  identical commands/forces. No realtime guarantee or production readiness.
## Code, tests and durable packets
Research code lives in example/cpp/tools/research. Native default shooting
behavior is preserved; checked transition mode is explicit. Periodic feedback
uses checked derivatives and optional --constrained direct torque optimization.
All model/initial integration state/command ordering remains unchanged. Plugins
are rejected by scratch restoration; na>0 is rejected by checked derivatives.
30 focused Python tests pass: derivatives8, independent certificate8, periodic
feedback7, constrained tracking4, complete-cycle contact windows3. No new C++
controller code/build/B0 campaign was performed in this resumed session.
Durable evidence under docs/research/evidence:
- whole_body_cycle_20260908/attempt_0002
- whole_body_derivative_audit_20260908 (attempts0001..0004)
- whole_body_feedback_20260908 (ordinary feedback, command law, force cause/oracle)
- whole_body_constrained_feedback_20260908 (attempts0001..0004, pre-force audit,
  scratch benchmark; portable result JSON plus lossless original JSON gzip).
Each packet binds sources, commands/inputs, hashes and independent verdicts.
Raw _runs directories are immutable. Curated physical replays match originals.
## Next required work toward5cm, then10cm
Do not activate the18ms constrained solve inside a2ms consumer. The next design
must budget computation and enforce force constraints across an actual horizon,
then independently demonstrate deadline/state-error handling. Avoid another gain
or force-threshold sweep: current evidence localizes contact inclusion sensitivity
and shows single-state feasible control but not realtime horizon viability.
Retain ONE atomic execution owner with a versioned WholeBodyTrajectoryV1 payload
(model/observation/terrain identity, absoluteq/v/tau plus feedback, validity,
events/commitments/certificates). Do not forge centroidal selected-problem or
ID-WBC force certificates for this backend. Owner must sample its trajectory
without regenerating conflicting foot cubics/stance bridges.
Build worker-local model collision geometry from the same immutable OBSERVED
terrain snapshot with explicit unknown coverage, not scene XML/GT at runtime.
Validate shadow/admission/command semantics, then actual live flat with unchanged
Phase-1 velocity authority; only afterward real5cm and independent10cm under
separately bound empirical contracts. Existing privileged seed is not an allowed
future controller input. No new backend should claim production/B1 readiness
from these initialized flat results.
## Reproduction and operating rules
Use pinned localhost SSH helper, not hanging wsl.exe launch. From Windows:
PowerShell Bash here-string piped to python
C:/Users/w1881/Documents/Codex/2026-09-06/feat-stage-c-joint-planner-fetch/wsl_exec.py.
MuJoCo3.3.6 and SciPy in native WSL; OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
MKL_NUM_THREADS=1. Hold /tmp/go2_mujoco_experiment.lock for physics/timed work,
avoid competing builds, preserve other worktrees/stashes/archive branches.
Replay optimized curated result with its copied runtime verifier:
python3 docs/research/evidence/whole_body_constrained_feedback_20260908/attempt_0004/runtime_sources/verify_whole_body_shooting.py docs/research/evidence/whole_body_constrained_feedback_20260908/attempt_0004/result.json --out NEW_FILE.json
Expected exit1 preserves original V1 numerical failure, not execution failure.
Research protocols: WHOLE_BODY_SHOOTING_V1.md, WHOLE_BODY_FEEDBACK_DIAGNOSTIC_V1.md,
WHOLE_BODY_CONSTRAINED_FEEDBACK_V1.md. New current source HEAD comes from Git;
4023ff8 is the latest executed full feedback source, not the final docs HEAD.
## Authority and retained history
Current user instructions and CURRENT; AGENTS.md; frozen PHASE2_ACCEPTANCE.md
and PHASE2_HOLDOUT_MANIFEST.json for their campaigns; versioned B1_DYNAMIC_TRAVERSAL_V3.md
and B1_REGISTERED_INTERVALS_V2.md with exact analyzers/evidence. Retain historical
V1 numerical failures, T13 aerial conflict and15mm GEOMETRIC diagnostic separately.
Unknown is not safe; measured/planned/applied/collision contacts stay distinct.
Keep normal running-trot/two-contact diagonals, no quasi-static/crawl/local recovery
or local retiming. No remote push was requested. Existing original handoff and
historical source/physics evidence remain in SESSION_HANDOFF_20260908.md and the
named packets. Do not restart completed workers or rerun old audits without cause.
