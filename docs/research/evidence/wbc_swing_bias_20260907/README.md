# Production swing-acceleration audit F02
## Registered experiment
Baseline: clean f5b5155fc4750b358c39516c70a4e76014cf98b9, runtime-equivalent
to the retained eeb5d75 flat/step runs in ../b1_checkpoint_20260907.
The user supplied architecture audit bundle identifies F02 analytically.
This packet tests the actual MuJoCo Go2 model and production WBC solver.
Single hypothesis: the swing task must minimize
`||J qdd + Jdot qvel - a_des||_W^2`, matching the existing stance semantics.
The caller already supplies world-frame physical PD acceleration without
bias subtraction. Only the swing gradient and corresponding cost diagnostic
change. No gains, contact schedule, friction rows, solver acceptance,
terrain observation, execution policy, analyzer or threshold changes.
The regression obtains acceleration bias from second differences of actual
foot collision-geometry positions along a constant-joint-velocity path.
It compares the real QP against an equivalent target compensated using that
independent finite difference (zeroing swing Jdot only in the oracle input).
It also checks physical cost, swing-force allowance and floating-base residual
in two-contact and aerial fixtures with anisotropic swing weighting.
`red.log` is the unmodified baseline solver with the new test (expected FAIL).
`green.log` and `ctest.log` are the corrected implementation.
These fixtures are not closed-loop or full-body feasibility certificates.
After focused/full tests, create a clean runtime commit before simulation.
Run the unchanged research harness, interval map V2 enabled, period .14 s,
lift floor 0, 32 s V3 profile, seed 11/domain 231. First run flat with terrain
execution enabled; inspect safety/completion and measured running-cycle output.
If no useful failure blocks further exposure, run the same-source 5 cm scene.
Retain every raw run. Report unchanged legacy and V3 analyzer outcomes, clocks,
contact topology, riser impacts, tracking and planner availability. The retained
historical baseline is a reference, not a contemporaneous randomized control;
one before/after difference cannot establish robustness or causal improvement.
Stop this single-variable experiment at the first useful failure, investigate
it, and preserve a clean reproducible checkpoint rather than retuning gains.
B1 is not accepted by a mathematical correction or by a visually good crossing.
