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
