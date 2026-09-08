# Native coupled evaluator checkpoint
Source46824afd8ef1422963e9ada5fb6692866d0c8c39, clean native WSL, MuJoCo3.3.6.
This closes one bounded speed experiment. No more extension of the initialized
32ms flat toy is the next milestone. Genuine5cm B1 remains NOT_CERTIFIED.
Same complete integration state, warmstart, pre/work/post forward/step ordering,
all1540 constraints, fixed12ms prefix, model parameters and SLSQP settings.
10control cases match Python within1e-12cost/1e-9constraints and have identical
strict feasibility. Deterministic repeated evaluation and5 input/coverage/closed
failure checks pass. Raw audit records each discrepancy and all timings.
30evaluation observations: Python p50/p95/max13.022765/14.242381/15.102304ms;
native1.767376/2.232617/2.354567ms. Evaluator-only, not controller latency.
One identical2-knot solve:30iterations1512evaluations in2.766913s versus prior
17.276930s; feasible witness retained, iteration limit, no optimality assertion.
Independent Python replay: all13checks pass, state/force/motor residual0,
clock1.77358e-14s, force159.896800N, torque28.355897Nm. This remains much too slow
for12ms publication. No liveRT, event-spanning or foothold-selection claim.
Initial native0001 construction rejected the host-installed profiling timer;
source93905f5 and its local raw binary/build output remain. Final code retains
this timer but rejects force/control/sensor/actuation callbacks. No dynamics
callback or simulator option was disabled to obtain agreement. Root review also
corrected the worker's actuator-address loop bound before first compilation.
Rebuild using build_command.txt (output into a new unique directory) and run
 audit_coupled_native.py coupled_horizon_20260908/0002/result.json --library LIB --out NEW
Then probe_coupled_mujoco_horizon.py with original rolling seed, original flat
scene, --control-knots2 --native-library LIB --privileged-scene-oracle, followed
by verify_coupled_mujoco_horizon.py. Exact executed command options are defaults
30iterations/60seconds, OPENBLAS_NUM_THREADS=OMP_NUM_THREADS=MKL_NUM_THREADS=1.
Scripts self-lock the experiment lock. Binary gzip preserves original bytes;
absolute paths in original evidence remain unmodified. Scope is research only.
