# WBC physical certificate: registered shadow experiment
Registered before simulation, from baseline 06f002d (runtime 7a8ffc6).
Architecture decision: ../../LOCOMOTION_ARCHITECTURE_V1.md.
## Question and intervention
Do current solver-accepted or reused WBC candidates satisfy the physical
constraints of their own current rigid-body model? Existing QP convergence,
floating-base residual and torque diagnostics do not answer this question.
Add an independent certificate and record both the attempted and selected
candidate against the current state/contact constraints. Do not change solver,
force limits, acceptance, fallback, gait, terrain plan or actuator command.
A selected candidate means after cache selection and WBC overlays, before later
feedforward arbitration and joint PD. It is NOT the applied plant command.
Coverage is explicit. A nonexistent attempt is unchecked, not a zero-force
solution. Invalid/nonfinite declared inputs fail closed. A missing surface normal
uses the named legacy flat assumption; that assumption is not measured terrain
truth. Circular cone feasibility is not conservative QP-pyramid feasibility.
Certificate tolerances are defined in the versioned header and bound to the
runtime source before data collection; no tolerance will be tuned on these runs.
## Registered execution and interpretation
After focused tests and independent review, run a clean exact-SHA flat diagnostic
with the unchanged V3 profile (32 s), research interval map V2, .14 s period,
zero research lift floor, seed 11/domain 231 and terrain execution enabled.
This is a development diagnostic, not a full B0 acceptance campaign.
Inspect runtime completion, measured running cycles, certificate coverage,
legacy-accepted violations grouped by convergence, selected/reused proposal
violations and validator timing. Preserve nonconvergence versus infeasibility.
If flat runtime integrity is intact and the observed signal warrants a terrain
comparison, one identical-source/configuration 5 cm step diagnostic is permitted.
No holdout, gain search or threshold change belongs to this instrumentation
experiment. A useful failure leads to analysis and a separately identified
implementation decision, not repeated blind runs. The original V3 and legacy
analyzers remain unchanged. No physical PASS can prove realistic perception
while the privileged observation path remains unresolved.
All raw files remain under unique ignored `_runs` names. Curated results will
bind source, binary, environment, commands, analyzers and raw SHA256 digests.
A read-only replay must reproduce the summaries. The certificate is evidence
about one proposed rigid-body solution, not a B1 or whole-robot capability claim.
