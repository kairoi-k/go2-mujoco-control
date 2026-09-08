# Coupled horizon diagnostic, 2026-09-08
B1 remains NOT_CERTIFIED. These are privileged initialized flat, sampled32ms
research solves, not terrain planning, full-cycle stability or runtime control.
0001: clean29b64aa source,120 free torque variables,60.0025337s budget exhausted,
5250 evaluations, no candidate published. This is not an infeasibility proof.
0002: clean d33c47e source, two linear torque-correction nodes (24 variables)
over the same10 free2ms stages, first6 stages immutable. SLSQP reached30 iterations
in17.2769295s/1512 evaluations. It retained a freshly evaluated feasible witness;
solver convergence/global optimality were NOT established.
Independent replay: state/force/motor residual0, clock1.7736e-14s; force159.89679N,
torque28.35590Nm. Terminal vy0.00662523m/s versus baseline0.04849031m/s.
All13 declared independent checks pass. However roll/pitch angular rates approach
-0.3rad/s and forward speed drops0.843832 to0.819843m/s. The terminal cost can
trade lateral velocity for rotational momentum; this is not improved gait quality.
A viability/continuation objective and longer event-spanning horizon must precede
runtime use. A17s solve cannot meet the existing12ms publication deadline.
No latency distribution is claimed from one solve per formulation.
The analytic double-integrator test independently uses HiGHS LP and the closed-form
minimum-energy solution (.9,.3), proving true temporal coupling versus a greedy
first input. Eight focused tests pass. This does not establish terrain/B1 capability.
## Reproduction
Use each result's exact source_sha in a separate native WSL clean worktree.
MuJoCo3.3.6; installed NumPy/SciPy. Set OPENBLAS_NUM_THREADS=1, OMP_NUM_THREADS=1,
MKL_NUM_THREADS=1. The probe holds /tmp/go2_mujoco_experiment.lock itself.
Run from repository root (OUTPUT must not exist):
python3 example/cpp/tools/research/probe_coupled_mujoco_horizon.py \
 --rolling-result docs/research/evidence/whole_body_native_20260908/rolling_0002/run.json.xz \
 --scene unitree_robots/go2/phase2_flat.xml --out OUTPUT \
 --privileged-scene-oracle --wall-budget-s 60 --control-knots 2
For0001 omit --control-knots2. Run the independent verifier on freshly generated
OUTPUT: python3 example/cpp/tools/research/verify_coupled_mujoco_horizon.py OUTPUT
--out NEW_VERIFICATION. Saved raw manifests retain original absolute paths and
SHA256 hashes; reproduction from another checkout creates its own correct paths.
No raw evidence was rewritten for portability. Hardware/time-sensitive failures
need not occur at the same evaluation count. Model assets and seed packet remain
bound by each result.hashes. Original raw files remain in ignored _runs.

## Post-run review correction
The generic budget guard now checks after each evaluator returns as well as before
it begins. A deterministic mocked-clock regression proves a final late evaluation
cannot publish. Nine focused tests pass. The two timed results and archived solver
copy deliberately retain their original source identity; their measured completion
times do not exercise this final-deadline corner. No rerun was used to hide0001.
