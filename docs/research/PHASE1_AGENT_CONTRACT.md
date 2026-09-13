# Phase1 experiment contract

## Mission

Finish Phase1 arbitrary continuous forward-velocity control well enough to unblock the terrain-aware locomotion research path. Do not turn Phase1 into an open-ended controller rewrite or parameter-search project.

The current research question is narrow: explain and reduce the remaining low/mid/high-speed tracking bias in the B-semantic controller while preserving the already demonstrated dynamic-trot stability.

## Canonical starting point

This branch was created from checkpoint:

`1fc68a34233551ac9ed1e57f68cc86357fd560ad`

That checkpoint contains the correctly aligned targeted ID-dynamics audit. Its controller lineage is the B-semantic Phase1 controller:

- `step_length = applied_velocity * period`
- effective-speed convention disabled
- period `0.14 s`
- duty factor `0.44`
- existing shaper/governor/Raibert/preview/SRBD/WBC/contact logic unchanged unless a task explicitly authorizes one conceptual variable.

Do not use failed experiment checkpoint `e836edc4458759e15a7a0b33539c072427ac42ac` as an implementation base. It is evidence only. Its simulator-wide `tau_ff-only` switch removed PD from stand-up/preflight as well as gait and therefore did not test endpoint overspeed causality.

## Established evidence

Treat these as current evidence, not immutable truth:

1. The original Phase1 speed-semantic mapping was inconsistent across layers. Switching to B semantic materially improved several low/mid-speed tracking cases, but did not eliminate residual overspeed.
2. Period `0.18 s` caused serious stability regressions and is not an accepted solution.
3. Duty `0.50` did not solve the mid/high-speed tracking bias.
4. In the main `varying 1.4→2.3` failure window, WBC desired x acceleration, SRBD x acceleration, and ID-WBC `qdd[0]` all request braking while realized body acceleration remains near zero.
5. Exact same-tick ID dynamics closure at the real failure window is numerically tight: the ID-WBC solution itself is dynamically self-consistent.
6. Solver torque → final torque → LowCmd `tau_ff` mapping is numerically intact in that window.
7. The simulator bridge then applies joint PD on top of `tau_ff`; this changes effective actuator commands by tens of Nm. This is the first directly observed post-ID deviation, but it has NOT yet been shown to cause the residual overspeed.
8. Removing PD globally from simulator startup prevents normal preflight/gait entry. Therefore `tau_ff-only from t=0` is invalid for deciding the endpoint-overspeed hypothesis.

## Experimental discipline

Use the lean loop:

`one hypothesis → one controlled experiment → one auditable checkpoint → stop`

For each experiment:

- Change exactly one conceptual variable.
- Keep controller/model/profile/acceptance behavior fixed unless the task explicitly names a variable.
- Prefer same binary, same seed, same instrumentation, and paired A/B runs.
- Record exact source SHA, simulator/controller binary SHA256, scene/profile SHA256, command line, run IDs, and raw file SHA256.
- Never silently substitute a different benchmark, threshold, gait mode, profile, or seed.
- Do not convert a failed or missing endpoint into evidence for endpoint tracking.
- Do not infer body-axis propulsion/braking directly from individual joint torque signs.
- Do not treat simple summed world `Fx` as a substitute for full floating-base dynamics.
- Do not claim a physical root cause when the evidence only identifies an upstream candidate.

## Safety and stopping rules

Stop the current task and report instead of improvising when any of the following occurs:

- the requested single-variable isolation cannot be implemented without changing another control behavior;
- the A baseline no longer reproduces the expected continuous-trot regime;
- instrumentation changes controller timing/behavior materially;
- a safety/posture failure makes the requested causal window unavailable;
- more than the task-authorized number of runs would be required;
- the evidence rejects the task hypothesis;
- a controller architecture change, gain scan, gait-regime change, benchmark change, or acceptance-criterion change appears necessary.

Do not automatically fix the next thing after a result. End with one recommended next step only.

## Scope boundaries

Do not expand this work into terrain/Phase2, RL, a new gait state machine, a broad WBC rewrite, or global parameter optimization. The long-term terrain project remains separate and should resume only after Phase1 is sufficiently understood and stable.

## Reporting standard

A useful checkpoint contains only what another reviewer needs to reproduce and judge the experiment:

- hypothesis and unique variable;
- exact A/B provenance;
- primary causal-window measurements;
- safety/stability observations;
- conclusion with calibrated confidence;
- one next step, not executed.

Keep raw evidence locally if large, but commit derived data sufficient to audit the stated conclusion and record hashes for all raw artifacts.