# Task: delayed post-ID PD pulse A/B

Read `docs/research/PHASE1_AGENT_CONTRACT.md` first. Its constraints are part of this task.

## Question

Does the simulator-side/post-ID joint PD contribution causally oppose the braking requested by WBC/ID-WBC during the known `varying 1.4→2.3` residual-overspeed window?

The previous global `tau_ff-only` experiment is not a valid endpoint test because it removed PD from stand-up/preflight and never reached continuous trot. Do not repeat that design.

## Hypothesis

During the established overspeed/braking window, temporarily removing only the post-ID PD contribution will make realized body-x acceleration more negative and/or reduce the short-horizon velocity excess relative to an otherwise identical baseline.

This is a causal perturbation test, not a proposed final controller fix.

## Unique conceptual variable

A and B must be identical until active-relative time `32.10 s`.

- **A:** normal actuator composition for the entire run.
- **B:** normal composition everywhere except active-relative `[32.10, 32.40) s`, where joint PD contribution is suppressed so the simulator receives effectively `tau_ff` only. At `32.40 s`, restore normal PD composition.

The intervention is therefore a **300 ms delayed pulse after stable continuous trot has already been established**.

Do not use or cherry-pick the global-from-start bridge implementation from `e836edc4458759e15a7a0b33539c072427ac42ac`.

## Preferred implementation

Use the controller's existing velocity-command active-relative clock so the intervention is tied to the same time basis as the Phase1 profile/analyzer.

The simplest acceptable implementation is a default-off experimental flag that, only inside the B pulse window, leaves `q_des`, `dq_des`, and `tau_ff` unchanged but zeros the LowCmd `kp`/`kd` sent to the simulator. Outside the window, LowCmd must be bit-for-bit/logic-equivalent to baseline behavior.

An equivalent bridge-side implementation is acceptable only if it can consume the exact same active-relative clock without changing preflight or earlier gait behavior.

Requirements:

- default flag off = baseline behavior;
- no change before active `32.10 s`;
- no change after active `32.40 s`;
- no change to ID/WBC mathematics;
- no change to q/dq targets or tau_ff;
- no gain tuning; this is binary PD-present vs PD-suppressed during the declared pulse only.

Suggested flag name:

`TROT_PD_PULSE_AB=1`

The exact name may differ, but record it.

## Fixed controller/benchmark state

Start from this branch, whose base is `1fc68a34233551ac9ed1e57f68cc86357fd560ad`.

Keep fixed:

- B semantic: `step = applied * period`, effective-speed convention false;
- period `0.14 s`;
- duty `0.44`;
- shaper/governor;
- Raibert/preview;
- SRBD/MPC;
- WBC weights/gains;
- contact logic;
- model/scene;
- torque limits;
- `varying` profile;
- acceptance thresholds.

Retain the targeted closure/diagnostic capture around the failure window. Capture at least active `[31.90, 32.60] s` and keep the existing full window if simpler.

## Run plan

Build one source state/binary containing the default-off pulse implementation and diagnostics.

Use the **same binary and same fresh seed** for both A and B. Use seed `241` unless infrastructure requires another unused seed; if changed, use the same replacement seed for both and record why.

Planned runs:

1. **A control:** pulse flag OFF.
2. **B intervention:** pulse flag ON.

Run A first.

A must reach continuous trot and reproduce the known qualitative state around active `[32,33)`:

- measured speed above applied/target;
- WBC/ID braking request present;
- no new safety failure.

If A does not reproduce that regime, STOP. Do not run B and do not improvise another parameter change.

If A is valid, run B once.

Maximum total runs: 3. A third run is allowed only to replace an infrastructure/instrumentation-invalid run that never produced usable intervention data. Do not retry a genuine B instability/safety response just to obtain a nicer result.

## Required verification before interpreting B

Prove from logs that:

- A and B are identical in actuator composition before `32.10 s`;
- B's PD contribution is zero only on `[32.10,32.40)`;
- B restores the normal PD path at `32.40 s`;
- B `tau_ff`, q/dq targets, WBC/SRBD/ID outputs remain generated normally;
- the intervention does not accidentally alter stand-up/preflight or the earlier continuous-trot trajectory.

If any of these fail, classify the experiment as invalid rather than interpreting tracking.

## Primary causal measurements

Use A/B time-aligned on active-relative time.

At minimum report windows:

- pre-pulse: `[31.90,32.10)`
- early pulse: `[32.10,32.20)`
- late pulse: `[32.20,32.40)`
- recovery: `[32.40,32.60)`

For each window compare:

- requested/shaped/applied/kernel nominal velocity;
- measured velocity;
- measured minus applied/target;
- realized body-x acceleration using the same 100 ms local linear fit as prior audits;
- WBC desired ax;
- SRBD ax;
- ID `qdd[0]`;
- ID solver tau / LowCmd tau_ff;
- PD torque contribution;
- effective actuator command;
- tau_est if available;
- internal contact mask and physical/logger contact mask;
- roll/pitch and safety state.

Primary effect sizes:

1. `realized_ax_B - realized_ax_A` during early and late pulse;
2. change in measured velocity from pulse onset: `v(t)-v(32.10)` for A vs B;
3. recovery after PD restoration;
4. whether contact topology or posture changes immediately enough to confound the acceleration effect.

Do not infer body braking from joint torque signs alone.

## Interpretation

**Supports the hypothesis** if the B pulse produces a clear, time-locked increase in braking (more negative realized ax / smaller short-horizon velocity excess) compared with A while WBC/ID braking remains comparable and without a simultaneous contact/posture event that better explains the effect.

**Does not support the hypothesis** if removing PD makes realized braking weaker, velocity rise faster, or stability/contact quality clearly deteriorate.

**Little/no causal effect** if A and B physical acceleration/velocity remain close despite the large actuator-composition difference.

**Inconclusive** if the B pulse triggers an immediate topology/safety change that prevents separating PD torque effect from gait/contact collapse, or if A/B are not comparable before the pulse.

Do not call PD the sole root cause from one pulse even if the hypothesis is supported. The result should determine whether PD/ID coordination deserves the next experiment.

## Output

Commit only a compact checkpoint containing:

`docs/validation/phase1_pd_pulse_ab_20260913/RESULTS.md`

`docs/validation/phase1_pd_pulse_ab_20260913/ab.csv`

plus the minimal source diff implementing the default-off pulse behavior.

Raw logs may remain local; record SHA256 and run IDs.

`RESULTS.md` must state:

- exact source and binaries;
- exact A/B commands and seed;
- proof that pre-pulse behavior is comparable;
- proof of pulse on/off timing;
- the four time-window measurements;
- stability/contact observations;
- hypothesis outcome: `SUPPORTED`, `NOT SUPPORTED`, or `INCONCLUSIVE` (use `PARTIALLY SUPPORTED` only if the exact supported and unsupported subclaims are separately stated);
- one recommended next step, not executed.

Push one experimental checkpoint and stop. Do not automatically tune PD, change gains, modify WBC, or run another architecture experiment.