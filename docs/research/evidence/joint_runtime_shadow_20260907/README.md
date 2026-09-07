# Joint runtime shadow checkpoint
B1 remains NOT_CERTIFIED. This packet first freezes the source/test basis for
an actual-state, sensor-map, event-indexed centroidal joint-planning shadow.
See ../../JOINT_RUNTIME_SHADOW_V1.md for the registered probe and clock branch.
61 controller tests pass, including unbound candidate combinations, actual
articulated centroidal derivatives, non-projected moving-state feedback samples,
full absolute schedules, clock boundaries and heading-map world queries.
The prior contact-point pair is in ../joint_point_pair_20260907; its runtime is
e1e68de and it does not establish a point-model improvement.
The seven-row counterfactual sample experiment uses retained historical raw
`wbc_active_flat_43c5f5a_20260907_0001`, not a new closed-loop result. Extraction
joins exact recorded times and uses simulator truth for orientation/velocity
and scene geometry as an explicitly privileged offline oracle. It is NOT an
online planner input or a replay of the historical command. Centroidal/foot
accelerations and force-reference values below are explicitly supplied zero
targets; they were absent from historical telemetry. All seven sample QP/model
certificates pass without resetting initial q/dq, while contact evolution and
execution_ready remain false. The extractor metadata records source files and
hashes. A first CLI validation returned all UNKNOWN due to a tool status-state
bug; the parser-success path is now set before surface checking. This is not a
change to dynamics acceptance.
Reproduce this counterfactual (use a fresh output path):
```
example/cpp/build/replay_stage_c_feedback \
 --samples docs/research/evidence/joint_runtime_shadow_20260907/counterfactual_samples.csv \
 --model unitree_robots/go2/go2.xml --out /tmp/joint_feedback_counterfactual_review.json \
 --centroidal-target 0,0,0,0,0,0 \
 --foot-acceleration 0,0,0,0,0,0,0,0,0,0,0,0 \
 --force-reference 0,0,0,0,0,0,0,0,0,0,0,0 --use-scene-surface \
 --surface-friction-mu 0.8 --surface-min-normal-n 0 --surface-max-normal-n 180
```
`reproduce.py RUN --runtime-sha SHA --point-model 0 --out FRESH_DIRECTORY`
reuses the established source/binary/raw audit core and adds structured
JointShadow coverage, failure and pipeline latency statistics. Solver-attempt
latency is reported separately from captures rejected before optimization.
Runtime truth and exact SHA will be appended after the registered live probe.
