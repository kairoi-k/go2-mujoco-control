# B1 dynamic traversal V3: stable approach, not a green legacy analyzer

Pre-registered 2026-09-07, before any run of the V3 scene/profile. Historical
PHASE2_ACCEPTANCE, holdout manifest, scenes, thresholds and analyzers remain
unchanged and separately reportable. This protocol strengthens the running
claim; it is not a reinterpretation of earlier PASS results.

## Why this version exists

Clean 764a21c first step run physically crossed without nonfoot contact, but
interaction 17.454--19.876 s largely occurred while effective period/duty were
changing .36/.66 toward .14/.44. Stable nominal running began only near exit.
The same-SHA repeat failed IK. Thus the earlier physical subclaim is neither a
stable-running traversal claim nor a reproducible candidate.

## Fixed development conditions

- Same box dimensions: 0.05 m high, 0.50 m long, 1.50 m wide, on level floor.
- New distinct scene `b1_v3_running_step_5cm.xml` places its leading edge at
  world X=5.0 m (center 5.25 m), retaining zero initial pose and providing a
  settling approach. No controller may read this scene position.
- `b1_v3_running_1mps.csv`: speed 0 through 8 s, linear ramp to 1 m/s at
  16 s, hold through 24 s, ramp to zero at 26 s, zero through 30 s.
  Controller duration is 32 s, including a full requested stop tail.
- Actual runtime period/duty must stabilize near .14 s/.44; the configuration
  enum alone does not establish running. Same-SHA flat control uses this same
  profile and all actuator/planner flags except the physical obstacle.
- Initial development seed/domain/pinning are inherited and recorded by the
  research harness. Repeated same-SHA runs are required. Existing holdouts are
  not tuning data; a separate V3 holdout campaign must be fixed before use.

## Physical empirical gates, fixed before testing this scene

Retain V2 truth/controller coverage, four top-support witnesses, conservative
full collision-geometry and foot exit, posture <=15 deg and base height >=.28 m.
Add all of the following (thresholds are protocol definitions, not solver
feasibility claims):

1. Before first step contact, continuous .8 s approach coverage has effective
   period .14 +/- .005 s, duty .44 +/- .01, requested speed 1 +/- .02 m/s.
   At least 95% of these samples have measured forward speed .75--1.25 m/s.
2. The first-to-last step interaction spans at least four complete gait cycles.
   In each sliding five-cycle window, at least three cycles contain both
   diagonal support episodes (each >=10 ms) AND total-robot GRF norm below
   10 N for >=4 ms. If only four complete cycles exist, at least three qualify.
   A cycle with no force-bearing feet is not necessarily aerial: use TOTAL
   external robot contact force, not the terrain-only foot mask.
3. During interaction forward-speed P05 >=.5 m/s, median .8--1.2 m/s.
4. No nonfoot step contact and no foot-riser contact: exact nontop foot force
   above 1e-6 N is recorded as contact. Do not infer top support from height or
   aggregate force. A successful traversal with riser strikes remains a
   separate diagnostic, not this collision-free candidate.
5. The entire requested profile and stop tail must be covered, with no IK,
   fatal controller message, safety intervention or early termination. Process
   status zero is insufficient. V2 passage gates alone do not establish this.

Report raw force/momentum residuals and timing distributions, exact source and
binary/run binding, repetitions and unchanged legacy verdict alongside V3.
No dynamics/whole-body feasibility certificate is implied by empirical PASS.
Insufficient field/time/cycle coverage is NOT_CERTIFIED, never filtered out.
A later protocol change must create a separately identifiable version and
retain this one; do not silently tune these gates against V3 failures.
