# Active-contact-only WBC forces: registered experiment
Baseline: clean runtime 71d242a80e84356cf9dd786830a01ff425a1a838.
The independent observer found inactive-foot forces up to approximately
4.5179 N in an accepted flat-running QP candidate. Keep the raw baseline.
## Intervention and hypothesis
Optimize qdd and only active-contact forces. Scatter to the existing twelve
force outputs, filling inactive legs with exact zero. Use the same scatter in
floating-base equality and joint torque affine map. Aerial mode has no force
variables. Inactive force references cannot generate contact force.
Keep objective weights, contact modes, friction construction, torque limits,
solver iteration budget, command selection and acceptance thresholds unchanged.
The old 0.05 N certificate allowance remains a historical diagnostic tolerance;
it is not an intended permission to use swing contact force.
Prediction: swing violations disappear structurally. Normal/friction and
nonconvergence may remain. This is not a full inequality acceptance repair.
## Validation before simulation
Four MuJoCo fixtures (aerial/mixed support, zero/moving velocity) with nonzero
inactive force references fail the old solver and require inactive force <=1e-12 N.
Existing physical swing-bias, rigid-body, torque and support tests remain.
Full controller CTest and independent source review precede a clean commit.
## Registered execution
One 32 s flat diagnostic, same V3 profile, interval-map V2, running period .14 s,
lift floor zero, seed 11/domain 231, terrain execution on, no gain changes.
Use run_b1_research_probe.sh with exact clean source and unique raw name.
Compare certificates, completion/safety, running topology and timing against
71d242a; timing-sensitive single runs do not establish statistical improvement.
Only after inspecting flat integrity decide whether a step comparison is useful.
Preserve old analyzers and all raw files. Bind executable hashes and source,
then report independently replayable evidence without a B1 claim.

## Independent review and pre-run result
Luna read-only review confirmed zero-force aerial dimension (18 variables,
24 torque inequalities), sparse contact scatter and equality/torque consistency.
The reported force-tracking cost retains inactive-reference constant error;
this constant does not affect the optimization or output.
All 45 controller tests pass after full rebuild. Existing swing-bias
finite-difference error is 2.13037e-8 m/s^2; equivalent-QP qdd differences
are 6.59269e-8 (mixed) and 1.2748e-7 (aerial). Four exact-zero fixtures
pass after failing on the old solver. See red.txt, green.txt and controller_tests.txt.

## Collected result and limitation
Executed clean source: `43c5f5a91e5c075a69170bc8af7d5582d1031474`.
Two flat runs `wbc_active_flat_43c5f5a_20260907_0001` and `_0002`.
Both complete with controller/safety/completion/quality/dynamics/truth status zero.
Legacy velocity-profile KeyError and dependent Phase-2 failure remain separate.
Run 0001 has 16243/16403 physically certified proposals, 160 failures
(23 normal-only, 131 friction-only, 6 both). Swing violations are zero.
Maximum force/moment residuals: 4.756e-6 N / 0.000209812 Nm.
Maximum friction/normal violations: 0.092163896 N / 0.001734946 N.
Running topology is 40/49 good cycles, compared with prior observer 47/49.
Root archived prior evidence concurrently for approximately 8.6 seconds during
this run. Preserve this scheduling confound; do not use it for timing claims.
Run 0002 is the same clean source/configuration, repeated without concurrent
build/archive. It has 16175/16402 certified proposals and 227 failures
(31 normal-only, 190 friction-only, 6 both), zero swing violations, 37/49
good cycles. Paired validator p50/p95/max: 1.332/1.643/22.152 microseconds.
No proposal reuse occurred in 0001; replay records per-run lineage coverage.
Neither run is B0/B1 acceptance or evidence of improved running performance.

## Research judgment
The exact-zero structural claim is confirmed. The locomotion improvement
hypothesis is not: isolated repeat still has fewer good running cycles.
A physically incorrect force path may have influenced historical behavior;
that does not justify restoring it or labeling the corrected backend a release.
Normal/friction failures remain and require correct solver/acceptance semantics,
but these small residuals alone do not establish the cause of the B1 impacts.
Continue contact/command-lineage audit and shared geometry/future-event work.
Do not retune gains to hide this result or claim that a green certificate alone
solves traversal. The older no-executable-plan-before-impact evidence remains
the most direct terrain failure witness; no new step run was collected here.

## Read-only reproduction
From the repo, run `python3 docs/research/evidence/wbc_active_forces_20260907/reproduce.py`
with `--out-dir /tmp/CHOOSE_NEW_DIRECTORY` and both
`--run-dir example/cpp/experiments/_runs/wbc_active_flat_43c5f5a_20260907_0001`
and `--run-dir example/cpp/experiments/_runs/wbc_active_flat_43c5f5a_20260907_0002`.
Use those arguments as one command. The resulting results.json binds both
runs, imported replay dependency, source/binaries, raw files and analyzers.

Root independently replayed both runs and compared complete results.json: identical.
