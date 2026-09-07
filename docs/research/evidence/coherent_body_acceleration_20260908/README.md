# Coherent body acceleration: fixed-source counterfactual
Source ceed2971ebe66ebc8283ed248c7551fe0c4f2403. Default behavior is unchanged.
Research config coherent_body_acceleration derives body angular acceleration
from the existing actual-model acceleration lift, using the SAME COM/Ldot and
feedback-corrected four-foot acceleration targets supplied to ID-WBC. No source
q/dq projection, independent attitude PD addition, or post-lift clipping occurs.
The original force, friction and torque constraints/certificates remain active.
A separate COM/body-angular/foot linear system checks angular-momentum task
compatibility in the focused test. Compatibility does not imply exact tracking
under physical bounds: the existing WBC still has soft motion objectives.
This is not a finished attitude controller. Absolute orientation regulation and
initial contact-motion realization remain unresolved. At ceed297 there was no runtime
activation flag; CLI --closed-loop-coherent enables only the short diagnostic.
## Registered comparison and results
Both modes start from the exact attempt0006 STATE21.018 snapshot, on the same
phase2_flat.xml plant, for0.2s. The core selected proposal is fixed, no replanning.
Exclusive experiment lock; clean source; raw run joint_coherent_accel_20260908_0001.
Each mode completes100 applied2ms steps plus the terminal state row.
Baseline/coherent peak foot norm:58.1248/24.1157mm; COM13.7873/7.8856mm.
Roll1.1907/2.5365deg; pitch1.3328/5.1927deg. No non-foot contact or predicted
motor saturation. Planned/geom-contact mask mismatches remain30/25 applied rows.
This is one source-state result, not a gait-cycle stability or traversal claim.
The larger periodic body excursion is neither automatically acceptable nor a
failure under an invented threshold; longer feedback evidence must decide it.
Independent Python MuJoCo replays the saved torque commands: maximum recorded
base-pose/joint-position/joint-velocity discrepancies2.48e-13/6.22e-14;
actuator-torque and clock discrepancies0. The CSV does not record base velocity,
so this check must not be described as a complete generalized-state comparison.
It verifies the plant response to saved commands, not planner or WBC optimality.
Four focused CTests pass. Initial test incorrectly demanded exact soft-task
tracking under constraints; replaced by the independent task-compatibility
oracle and separate physical certificate checks, without relaxing thresholds.
Reproduce commands in attempt_0001/{baseline,coherent}/manifest.json with fresh
output paths. Run analyze_pair.py ATTEMPT --out NEW_JSON for paired metrics,
and the existing joint_feedback_replay_20260907/verify_plant.py for command replay.
All results are privileged scene/model counterfactuals; no B0/B1 or10cm claim.
Next: test different captured phases and bounded execution, while introducing
horizon-level body orientation/contact-motion consistency rather than weight
sweeps or resetting the initial state. The strict full-body reconstruction
initial_condition_conflict remains unresolved by this acceleration-only mode.

## Additional source registered before replay
Use actual0005 source (opposite diagonal phase to0006), extracted with the
existing exact-clean-runtime snapshot tool. Repeat the same0.2s baseline versus
coherent comparison; no gain changes, no selection based on the new results.
The shared run_replay.py accepts --coherent-body-acceleration while retaining
its clean-source, complete input-hash and exclusive-lock protocol.

Opposite-phase result (attempt_0002): both100 steps complete, no non-foot contact
or motor saturation; baseline/coherent foot59.109/18.361mm, COM10.436/5.391mm,
pitch3.062/4.502deg, contact mask mismatches32/22. Independent plant verification
passes for both. The run wrapper bound all source/model/library files before
execution and confirmed no changes. This supports one bounded actual flat
canary, not promotion. Register joint_execution_flat_20260908_0008 with the
prior fixed-start21s protocol, coherent-body flag1 and soft-orientation flag0;
no other gains/thresholds change. Stop at the first useful failure.
