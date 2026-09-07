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
## Observed result (runtime 7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6)
The formula correction is confirmed, but B1 is NOT_CERTIFIED. The full 32 s
flat and step runs have clean exact source/binary bindings. Flat has 43/49 good
steady running cycles (minimum 3 good in each five-cycle window), versus the
historical 46/49. The step achieves full exit, all four top-support witnesses,
no nonfoot collision and normal runtime completion. All four feet still have
non-top contacts. Only 2/7 complete interaction cycles qualify, and interaction
speed p05/median is 0.559852/0.718292 m/s (historical 0.673269/0.850097).
The corrected single run is not an empirical improvement claim. Keep it as a
mathematical correctness baseline, not a validated locomotion release.
State/profile drift is 16.009 ms flat and 3.915 ms step. Both satisfy the old
20 ms gate in these runs; this does not prove clock architecture was fixed.
At first FR impact 23.216 s and FL impact 23.262 s, every row in the preceding
inclusive 0.2 s (100/101 rows) still reports no-safe-foothold, no usable plan,
no applied mask and no in-flight target. Thus F02 does not remove the decisive
planning/execution absence. Unknown coverage and future-liftoff geometry remain
separate research work; no scene-specific shortcut or relaxed margin was added.
The independent finite-difference bias error is 2.13037e-8 m/s2. Baseline
physical/oracle qdd differences 9.11707 and 22.5286 drop to 6.59281e-8 and
1.26547e-7 after the fix. These are mixed generalized acceleration vector norms,
not exclusively linear m/s2 errors. Physical-cost errors are 1.03709e-8 and
7.43444e-7; fixture floating-base residuals are 6.0288e-9 and 6.82749e-6.
Production step attempted equality residual p50/p95/max is
3.1e-8/2.85825e-6/1.33534e-4. No attempted torque limit violation is logged.
However, 726/16402 step attempts and 692/16402 flat attempts report QP not
converged while the output remains usable. Unlogged full inequality residuals
cannot be inferred from equality or torque checks: audit F04 remains open.
An independent Luna review confirms gradient sign, caller non-subtraction and
MuJoCo qvel semantics. It identifies the existing collision-geom versus foot-site
reference mismatch (about 2 mm local X in MJCF) as separate from this correction.
The current fixture holds body orientation fixed and tests moving joints; it
does not claim a full state/frame or actuator-envelope audit.
Legacy Phase-1 analysis still throws KeyError for b1_v3_running_1mps; dependent
legacy Phase-2 analysis fails. Both wrapper exits are 1 for that reason, as in
the historical runs; status fields and original logs are retained. The step
wrapper also records an early grep of simulator.log before its creation; final
simulator/truth logs, runtime status and time coverage are present. No legacy
contract/analyzer was changed to turn these statuses green. The V3 physical
sub-analyzer's estimated_state_and_lidar flag is not independent proof of
onboard sensing: the privileged observation issue in the external audit is open.
## Reproduce and verify
From native repository root, with the original immutable raw directories:
```
python3 docs/research/evidence/wbc_swing_bias_20260907/reproduce.py --out /tmp/wbc-bias-replay-UNIQUE
cmp /tmp/wbc-bias-replay-UNIQUE/results.json docs/research/evidence/wbc_swing_bias_20260907/results.json
```
The output directory must be new. Replay invokes the unchanged analyzers,
checks recorded clean source and binary binding, hashes every raw file and
recalculates clocks, WBC diagnostics and pre-impact plan absence. It does not
run physics or modify `_runs`. `pre_run_binding.json` binds the runtime/test
sources, binaries, frozen contracts and compiler before simulation. Rebuilds
may legitimately change binary bytes; only the recorded binaries were run.
To reproduce physics at the exact runtime commit in an authorized native
worktree, rebuild as documented above, export the registered settings below,
and choose new raw names. This is development evidence, not B0/holdout admission.
```
export TROT_RESEARCH_MAP_INTERVALS_V2=1 TROT_RESEARCH_RUNNING_PERIOD_S=0.14 TROT_RESEARCH_RUNNING_LIFT_FLOOR_M=0
unset TROT_RESEARCH_NOMINAL_COM_HEIGHT TROT_TERRAIN_EXECUTION_CONSISTENCY_SHADOW TROT_TERRAIN_DEBUG_PLANNER TROT_TERRAIN_DEBUG_SWING
bash example/cpp/scripts/run_b1_research_probe.sh 7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6 terrain-b1-execution UNIQUE_FLAT phase2_flat.xml 32 b1_v3_running_1mps.csv
bash example/cpp/scripts/run_b1_research_probe.sh 7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6 terrain-b1-execution UNIQUE_STEP b1_v3_running_step_5cm.xml 32
```
The shared experiment lock is acquired by the harness. Failed runs remain
immutable. No repeats/holdouts were run after this useful single-variable
failure; no robustness distribution or B1 candidate is claimed.
