# Bounded V4 near5cm validation,20260909
User resumed validation only; no controller/solver tuning in this run.
Source: edb54411741a0efb8ef5958851366fe20ed6cbea. Scope remains known-scene, recorded
initialization, synchronous whole-body MPC; NOT actual-sensor/runtime/B1 acceptance.
Command: tools/research/whole_body_mpc_oracle_probe.py, --library
example/cpp/experiments/_runs/whole_body_mpc_oracle_20260908_0005/libwm.so,
--scene unitree_robots/go2/whole_body_oracle_step_5cm_near.xml, exclusive --out,
--max-chunks200 --solver-wall-budget-s30 --solver-max-iterations20.
Run with Python3, MuJoCo3.3.6, OPENBLAS/OMP/MKL_NUM_THREADS=1; harness self-locks.
Model/code/input/library hashes and full commands/states/references are retained.
Result: FAILED,16completed chunks/80steps/0.16s. No actual step contact.
Final bodyx2.7133565m, vx0.8655391m/s. Actual peak footforce175.757N and motor
28.153Nm. State/force/motor replay discrepancies0. Normwise dynamics5.601e-10;
old absolute2.626e-7 fails and remains visible. Old absolute gate was not changed.
Failedchunk16 reached20iteration limit, not deadline or numerical exception.
The baseline's retained65controls exactly match the shifted previous accepted
trajectory (maxdelta0). Violations begin in new suffix atstep66: forbidden contact
force, then RR182.9449N>180 atstep67. No feasible witness within this search budget;
not a global infeasibility certificate, and no proof merely raising budget fixes it.
Solver p50/p95/max16.8143/17.4515/17.6331s. No realtime claim.
Saved-reference diagnostic: peak finite-difference vertical reference speed FR
3.9344m/s, FL2.7465m/s. These are objective references, NOT actual foot speeds;
they suggest further review of reference/foothold/landing feasibility but do not
by themselves prove a cause. No new search or parameter change followed failure.
The explicit next research issue is producing a continuously extendible traversal
trajectory under actual support and force limits. Transport correction helps but
is insufficient. Real5cm B1 and10cm remain unachieved.
