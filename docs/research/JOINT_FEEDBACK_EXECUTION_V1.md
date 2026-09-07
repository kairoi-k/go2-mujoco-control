# Joint feedback execution V1
Research decision, 2026-09-07. This implements the model-based backend of
LOCOMOTION_ARCHITECTURE_V1; it does not alter historical acceptance or certify
5/10 cm traversal. Current explicit user goal requires real closed-loop
validation, not an indefinitely expanding shadow-only pipeline.
## Execution interpretation
A nominal centroidal trajectory is a feedback reference, not a promise that
MuJoCo reproduces its contact force sample exactly. Whole-combination selection
binds COM/velocity/momentum, event identities, footholds, terrain surfaces and
foot curves. The existing articulated WBC evaluates current measured state and
tracks these references under the actual declared contact model. Force tracking
may be a soft objective. Its actual optimized and applied commands are checked
separately; nominal reduced feasibility cannot certify actuator execution.
The strict body-reconstruction fixture assumes stationary stance points. Actual
running baseline has nonzero contact-material velocity and contact timing
mismatch. Do not project measured q/dq onto that fixture or zero its velocities.
Reference foot stance targets may be stationary while measured feet differ;
that tracking error must remain explicit. Preserve exact-source full-model
sample tests as conditional model evidence, rather than using them as a
universal admission claim for feedback control.
## One owner and migration gates
The worker produces an immutable proposal containing the selected existing
CentroidalProblem/result plus event-specific foot references and observation
provenance. A single owner at LowCmdWrite adopts once per tick, preserves
already-in-flight targets/curves and their event identities, and supplies the
same accepted version to gait, body/force tracking and actuator output. No
consumer independently accepts a newer plan or changes a touchdown deadline.
Nominal scheduled contact, measured force contact and WBC contact decisions
remain separately recorded. Missing measured support is not silently invented.
Before command enable, independently replay actual-state proposals, check
reference coverage and sampled geometry/kinematic reach, commitment continuity,
clock/epoch expiry, and final actuator composition. An absent/expired proposal
must have an explicitly chosen shared fallback; no stale certificate reuse.
The first closed-loop admission is opt-in flat with complete command lineage,
then one 5 cm canary. Failures feed the same architecture rather than patches
to the old per-leg greedy planner. 10 cm follows credible 5 cm evidence.
Orientation feedback must remain explicit: COM/momentum tracking alone does
not regulate absolute body attitude. Reuse the articulated model to translate
posture feedback into a compatible objective. Terrain-adaptive height and
multi-event horizon references must derive from observed candidate surfaces,
not simulator scene labels or future ground-truth contacts.
Actuator verification must include requested joint PD plus feedforward and
plant saturation. A certificate for feedforward torque alone is insufficient.
Do not require exact nominal force equality when feedback needs redistribution;
do require physical constraints and truthful requested/applied lineage.
