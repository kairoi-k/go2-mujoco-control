# Full-cycle whole-body MPC research
Not B1 certified. Protocol: docs/research/WHOLE_BODY_MPC_ORACLE_V1.md.
Source99d256dd02c811a83f62f76dcd14fe500be1685f enumerated all25 first simultaneous
in-flight touchdown pairs at recorded0012 source21.020. All centroidal feasible,
all fail shared initial articulated torque check. Best(2,2)58.3664345292Nm versus
35Nm limit, nominal(0,0)312.044048852Nm. Remaining six future events unchanged.
This rules out just searching these existing footholds with the same stitched
trajectory at this initialization; not global trajectory or5cm infeasibility.
Command: build replay_joint_shadow_snapshot, then pass recorded initial_source.txt
and --initial-pair-audit. Raw stdout is losslessly preserved here.
Static target decomposition independently reproduces FL/RR displacements30.533/
31.196mm. Legacy target is already25.715/26.384mm ahead of actualgeom; geom/site
correction about-1.6mm, future COMtranslation6.414mm. Old actual commanded curve
is absent, so bootstrap inheritance fault is NOT established by this evidence.
Source89040b13185794177d9353f3f4d709425537ecfc native full-cycle evaluator audit:
actual5cm scene but originalx2.577 initialstate (no obstacle encounter),1/70steps,
nominal/deterministic perturbedcontrols. Native/Python constraint delta0, maxcost
delta3.47e-18, exact repeatedoutputs, ten failclosed checks pass. One evaluation
70steps8.37..8.56ms, not a latency distribution or optimization/runtime guarantee.
All four controls feasible in sampled inequalities. Full-cycle movement/terrain
capability is not established by an evaluator check.
Reproduce build using g++ -std=c++17 -O3 -fPIC -shared -Wall -Wextra,
MuJoCo3.3.6 include/library/rpath at
/home/che/.local/lib/python3.10/site-packages/mujoco; source
example/cpp/tools/research/whole_body_mpc_native.cpp, link libmujoco.so.3.3.6.
Run audit_whole_body_mpc_native.py with --library built.so --packet
 docs/research/evidence/whole_body_native_20260908/trajectory_packet_0002
 --scene unitree_robots/go2/b1_v3_running_step_5cm.xml --out new.json.
Use OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1. Script owns lock.
