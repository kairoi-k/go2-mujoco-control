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

## Observed result: clean runtime 71d242a
Exact runtime: `71d242a80e84356cf9dd786830a01ff425a1a838`.
Raw: `example/cpp/experiments/_runs/wbc_cert_flat_71d242a_20260907_0001`.
Flat completed with safety/completion/quality/dynamics/truth status zero.
The legacy velocity-profile KeyError and dependent Phase-2 failure remain;
wrapper exit one is not the physical verdict. No step run was collected.
There are 16403 active certificate rows: 12202 feasible and 4201 infeasible,
for both attempted and selected proposals. Every active row was legacy accepted.
Failure masks: 128=4027, 160=17, 192=152, 224=1, 32=2, 64=2.
The largest swing violation is 4.467892419 N beyond the historical 0.05 N
per-axis allowance: approximately 4.5179 N allocated to an inactive foot.
Worst row state time 22.862 s, gait time 16.561971062 s, measured mask 6,
QP not converged after 120 primary iterations. This is a proposed force,
not measured plant force. Friction/normal maxima are 0.079908839/0.004096225 N.
Maximum force/moment/joint dynamics residuals are 3.968e-6 N,
0.000164654 Nm and zero Nm. Online paired validator p50/p95/max:
1.331/1.623/23.816 microseconds. Running-cycle diagnostic: 47/49 good.
This single flat run is not a controlled performance improvement or B1 result.
## Decision
The observer exposed a physical modeling path worth fixing before extending
planning: inactive contact forces must be absent from optimization variables.
A separately registered follow-up will eliminate them, retaining public force
layout, gains, thresholds, contact selection and independent certificate.
Remaining normal/friction failures and final actuator command validation stay
open. The diagnostic does not establish that all inequality constraints hold.

## Reproduction
Run `python3 docs/research/evidence/wbc_certificate_20260907/reproduce.py
--out-dir /tmp/CHOOSE_NEW_DIRECTORY` as one command from the repository.
The output results.json matches this packet byte-for-byte on the original
workspace; hashes, exact clean runtime and binary identity are checked.
Root independently replayed the final script and compared the complete JSON.
All raw files are hashed in results.json; packet files are in SHA256SUMS.
