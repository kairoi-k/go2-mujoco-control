# Whole-body feedback diagnostic V1
Registered before first multi-cycle feedback replay,2026-09-08.

## Hypothesis and fixed experiment
Attempt0002 provides a hard-force-feasible initialized full-model0.14s cycle but fails the original V1 absolute dynamics residual. Preserve that failure. A five-period (0.70s) periodic TVLQR diagnostic now tests whether its nominal wrap defect and contact sensitivity permit bounded feedback repetition. This is a research probe, not promotion past V1 or live runtime integration. Use unchanged MuJoCo3.3.6 model/timestep/contact solver, original source state, and r-weight0.01. Apply feedback every2ms with final35Nm clip, without state reset or reference retiming. Record Riccati convergence, repeated linear-map spectral radius (not proof of nonlinear stability), wrap defect, saturation, actual contact patterns, full saved controls and latency.

## Independent judgments
Run the unchanged V1 verifier on saved actual controls, retaining ALL failing checks, then independent total-robot GRF/contact diagnostics. Original V1 terminal target closes the actual initialized state; for perturbed runs also report separately the unperturbed nominal recovery target and tangent error. Nominal metadata never replaces original terminal checks. Missing/changed hashes, incomplete nominal replay, nonconverged Riccati or malformed state stop the run.
For the five-cycle nominal probe, require physical torque/force/joint/attitude/height/nonfoot checks and original terminal bounds before extending repetition or introducing perturbations. Dynamics numerical error remains separately reported against frozen V1. Each complete gait cycle must be reported for both>=10ms diagonal support episodes and>=4ms total GRF<10N, preserving all-four/other contacts. No claim of terrain readiness from survival.
If nominal succeeds physically, test20periods with the same settings, then separately initialvy=+0.05m/s and initialroll=+0.02rad for20periods. These fixed diagnostic perturbations are not holdout trials. Stop at first useful physical failure and locate it before further variants.

## Command
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 example/cpp/tools/research/periodic_whole_body_feedback.py docs/research/evidence/whole_body_cycle_20260908/attempt_0002/result.json --periods 5 --r-weight 0.01 --out UNIQUE.json. The script holds /tmp/go2_mujoco_experiment.lock; preserve every raw output under _runs and curate with hashes.

Pre-run derivative prerequisite: attempt0002 initial chain check fails at nominal step32. Validate local transitions on the selected nominal before using those derivatives for TVLQR; do not interpret convergence of an inaccurate local model as reliable feedback.
