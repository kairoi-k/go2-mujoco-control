# Go2 current research checkpoint
Updated2026-09-08. **PAUSED by explicit user quota-stop request.**
Do not start research, simulations or agents until the user explicitly resumes.
This file remains the only live route/status entrypoint; older instructions and
run.json status fields do not override this stop.
## Actual outcome
Genuine5cm dynamic running-trot B1 remains NOT_CERTIFIED.10cm not started.
Canonical /home/che/dev/go2-workspace/feat-stage-c-joint-planner,
branch feat/stage-c-joint-planner. Latest implementation source:
503324035880a70d40530337d1e348872fa1bfb9; final checkpoint is its documentation
and evidence descendant. No production controller or frozen analyzer was changed.
The latest actual production-controller experiment is still0f6ec65 flat0012,
roll28.80deg at21.204s. The old visually good5cm video is not this new planner.
## New whole-body route and evidence
Read docs/research/WHOLE_BODY_MPC_ORACLE_V1.md and
 docs/research/evidence/whole_body_mpc_oracle_20260908/README.md.
Current research harness is whole_body_mpc_oracle_probe.py: same full MuJoCo
articulated dynamics,70step/140ms horizon,5step exact command prefix, joint torque
corrections induce body/feet/contact forces together. No zero touchdown velocity
stitching. Synchronous known terrain and recorded full-state/periodic nominal
initialization are privileged; NOT sensor/runtime/B1 authority.
Near5cm scene only moves original plateau center5.25->3.25m. Original scene kept.
0003 four-iteration run stopped after100steps;20iteration same-state solve found
a sampled feasible witness. Code/protocol V1 numerical admission mismatch is
preserved explicitly; do not promote this to a conforming certificate.
0004 V2/20iterations executed175steps but BOTH front feet pressed on the step wall:
31step-contact samples, zero top contacts. State/force/motor replay differences0;
normwise balance1.357e-9; old absolute4.335e-7 fails. Peak force179.30N,torque31.62Nm.
Later failed horizon approached FL calf upper limit. Solver p50/p95/max
13.753/15.383/15.607s, nowhere near realtime. This is not successful traversal.
V3 adds explicit top-support affordance: known horizontal planes/boxes only;
side/corner/underside support is forbidden, unknown support fails closed. New
native ABI preserves legacy evaluator. Counterexample and Python agree1.42e-14.
Geometry-derived C1 swing surface envelope reaches elevated surface before the
sphere footprint overlaps the step;7synthetic geometry tests pass.
0006 stopped after65steps before actual collision. Its seed incorrectly discarded
the accepted future beyond5committed commands, contrary to the transport description.
No prefix or physical certificate discrepancy; it lost the warm start and could
reintroduce force violations in previously covered time.
V4/source5033240 now transports all65remaining accepted controls and appends only
5nominal-feedback steps; all uncommitted controls remain jointly optimizable.
Four correction knots[5,35,64,69] permit new-suffix adjustment without changing the
whole prior half-horizon. Exact failed-state test: retained65steps feasible,
full horizon strict witness after15.667s, Python min inequality0, exact prefix.
## Stopped鐜板満
0007 was started from original initialization under V4 and USER-INTERRUPTED after
2chunks/10steps. Its run.json status started is historical, not a live process.
PID3319231 received SIGINT; session98697 ended KeyboardInterrupt, PID absent,
experiment lock free. Raw stop snapshot/request/receipt and partial run preserved
and archived losslessly. No active child work; runtime pending handle interrupted.
Do not automatically restart0007 or treat user stop as an algorithmic failure.
## When explicitly resumed
First inspect git status and stopped receipts; no reset/clean or evidence overwrite.
The next useful experiment is one bounded V4 near5cm canary from original initial
state, not another flat toy. Existing executable library:
example/cpp/experiments/_runs/whole_body_mpc_oracle_20260908_0005/libwm.so.
Run whole_body_mpc_oracle_probe.py with that --library, --scene
unitree_robots/go2/whole_body_oracle_step_5cm_near.xml, a NEW exclusive --out,
--max-chunks100 --solver-wall-budget-s30 --solver-max-iterations20.
It self-locks. Audit actual step top contacts with audit_whole_body_mpc_run.py;
never count foot-wall contact as touchdown success. Full traversal, running
contact quality, nonprivileged observed terrain and realtime remain open.
Only after real5cm candidate, independently evaluate10cm. No success inferred
from evaluator tests, feasibility of one initialized horizon or old videos.
## Environment and retained context
Use pinned localhost SSH helper wsl_exec.py from Windows task directory; do not
use previously hanging wsl.exe. MuJoCo3.3.6 native WSL. Set OPENBLAS_NUM_THREADS=1
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1. Serialize physics/timing with
/tmp/go2_mujoco_experiment.lock. Preserve all _runs, worktrees, stashes and baselines.
Older context: SESSION_HANDOFF_20260908.md, COUPLED_HORIZON_RESEARCH_V1.md and
whole_body_native/observed_seam evidence. Actual sensor snapshot881 coverage and
observed collision model remain unresolved; known-scene success cannot bypass them.
