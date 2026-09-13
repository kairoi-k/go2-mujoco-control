# Task: Phase1 same-state PD counterfactual replay

Read `docs/research/PHASE1_AGENT_CONTRACT.md` first. Its constraints remain binding.

## Starting point

Branch: `research/phase1-pd-counterfactual-replay-20260913`

Base checkpoint: `cf2a8892038c356ca6c4427855de9913b0a929ca`

The valid A baseline from the previous checkpoint is:

- run: `varying_20260913_204709`
- pulse flag OFF
- active-relative time reached ~80 s
- no new safety failure
- in `[32,33)`, measured-minus-applied median ≈ `+0.246 m/s`
- WBC/SRBD/ID all requested braking while realized body-x acceleration remained near zero on average

The previous live PD-off pulse is useful only as a stability observation. It is **not** a clean body-x causal measurement because removing PD caused rapid posture/contact collapse.

## Question

At the **same physical MuJoCo state**, with the same pose, velocity, contact geometry, controller outputs, WBC/ID solution, and no feedback of the counterfactual branch into the real trajectory:

**What is the instantaneous body-world-x acceleration effect of the simulator-side joint PD contribution?**

Equivalently, compare from an identical snapshot:

- actual actuator composition: `tau_ff + kp*(q_des-q) + kd*(dq_des-dq)`
- counterfactual actuator composition: `tau_ff` only

The purpose is to determine whether the PD term, at the known residual-overspeed state, instantaneously contributes forward acceleration, braking, or little net body-x effect.

This is a **shadow/offline counterfactual dynamics experiment**, not a controller modification and not another live PD-off rollout.

## Core principle

Do **not** let the counterfactual state feed back into the controller or become the next real simulator state.

Both branches must start from the exact same MuJoCo state. The only changed variable in the counterfactual branch is actuator `ctrl` composition.

No gain tuning. No partial-PD scan. No live pulse. No change to WBC/SRBD/contact logic/gait/profile/model.

## Preferred method

Prefer an offline or shadow forward-dynamics replay using exact MuJoCo state snapshots.

For each selected A-baseline snapshot:

1. restore/copy the exact same MuJoCo state into two private `mjData` instances;
2. branch ACTUAL: apply the exact actuator control that the bridge would use with normal PD;
3. branch CF: apply `tau_ff` only;
4. evaluate forward dynamics independently with the same `mjModel`;
5. compare world-x base acceleration and, optionally, one-step velocity change.

Primary measurement should be **instantaneous forward dynamics**, ideally `qacc[0]` after `mj_forward` (or the equivalent correct MuJoCo forward-dynamics call) from the identical state.

A single `mj_step` from each identical snapshot may be used as a secondary consistency check for `Δv_x`, but do not substitute a multi-step open-loop rollout for the same-state experiment.

If using `mj_step`, both branches must start from independently restored identical state and must advance exactly one model integration step.

## Snapshot fidelity

Do not reconstruct an “exact state” approximately from RPY, body speed, joint positions, etc.

A valid replay snapshot must contain enough MuJoCo integration state to reproduce forward dynamics faithfully. Use the official MuJoCo state APIs (`mj_getState`/`mj_setState` with an appropriate state specification) or an equivalent exact `mjData` copy/serialization.

Capture/restore all state needed by this model, including as applicable:

- `qpos`
- `qvel`
- actuator state (`act`) if present
- time / plugin or integration state required by the model
- warm-start state if needed for deterministic solver reproduction
- actuator control vector

The actual and counterfactual copies must receive identical non-control state.

If exact replay requires additional fields, include them. Do not silently omit them.

## Data source and allowed new run

First inspect the local raw evidence for `varying_20260913_204709`.

If it already contains an exact restorable MuJoCo state for the target ticks, reuse it and do **not** run a new simulation.

If the existing raw does **not** contain sufficient exact state, one new baseline diagnostic run is allowed solely to capture restorable snapshots.

That new run must:

- use normal PD for the entire real trajectory;
- keep `TROT_PD_PULSE_AB` OFF/unset;
- reproduce the current A baseline;
- add only default-off snapshot diagnostics;
- not perform shadow `mj_forward`/`mj_step` in the real-time control/simulation loop if doing so could perturb wall-clock behavior.

Preferred safe pattern if a new run is needed:

1. real run only serializes exact snapshots and bridge/controller command components around the target window;
2. after the run completes, a standalone replay tool loads those snapshots and performs the two-branch counterfactual dynamics offline.

An asynchronous/private shadow-data implementation is acceptable only if it is demonstrably non-invasive to the live trajectory.

Maximum new baseline runs: **1 valid run**, plus at most one replacement for a pure infrastructure failure that produces no usable data. No repeated runs to obtain nicer results.

## Time alignment

Target active-relative region from the valid A gait:

- broad capture: at least `[31.90, 32.60] s`
- also report `[32.60, 33.00) s` if snapshots are naturally available

Required analysis windows:

- W0: `[31.90,32.10)`
- W1: `[32.10,32.20)`
- W2: `[32.20,32.40)`
- W3: `[32.40,32.60)`

These are now **analysis windows on the normal A trajectory**, not live intervention windows.

Align simulator snapshots to controller active-relative time using an explicit common key, preferably the logged simulator/state tick. Do not assume a fixed offset between wall-clock, MuJoCo time, and active-relative time.

Record the exact join rule and maximum timing mismatch. If matching is ambiguous, stop rather than guessing.

## Actuator controls

For every snapshot, reconstruct exactly:

`pd_i = kp_i*(q_des_i-q_i) + kd_i*(dq_des_i-dq_i)`

`ctrl_actual_i = tau_ff_i + pd_i`

`ctrl_cf_i = tau_ff_i`

Confirm from source that this is the bridge semantics for the captured baseline.

Before interpreting body acceleration, prove numerically that `ctrl_actual` reconstructed by the replay matches the actuator control from the real baseline snapshot/log to a tight tolerance.

If actuator saturation, clipping, gearbox mapping, actuator gain, or other simulator-side transform occurs after `mj_data->ctrl`, identify it and ensure the replay follows the same MuJoCo path. Do not compare pre-transform quantities as though they were final generalized torques.

## Replay validation gate

This gate is mandatory.

Before using the counterfactual branch, the ACTUAL replay branch must reproduce the recorded live dynamics closely enough to establish replay fidelity.

At minimum compare, where available:

- replay ACTUAL `qacc[0]` vs recorded live MuJoCo `qacc[0]` at the same snapshot;
- if live `qacc[0]` is not currently logged, capture it in the diagnostic snapshot run;
- one-step ACTUAL `qvel[0]`/`Δv_x` vs the corresponding live next-step value if exact next-step pairing is available;
- contact count/constraint state and actuator control.

Define a reasonable numerical tolerance from MuJoCo precision/timestep, state it before interpreting results, and report actual residuals.

If ACTUAL replay cannot reproduce the live forward dynamics, classify the experiment `INCONCLUSIVE` and stop. Do not interpret the tau_ff-only branch.

## Primary counterfactual quantities

For each valid snapshot compute:

- `ax_actual = qacc_actual[0]`
- `ax_cf = qacc_tau_ff_only[0]`
- `delta_ax = ax_cf - ax_actual`

Interpretation of `delta_ax`:

- `delta_ax < 0`: removing PD makes world-x acceleration more negative; therefore the PD contribution was instantaneously more forward / less braking at that state.
- `delta_ax > 0`: removing PD makes acceleration more positive; therefore the PD contribution was instantaneously braking at that state.
- near zero: little instantaneous body-x effect.

If doing the optional one-step check:

- `delta_dvx = (qvel_cf_next[0]-qvel_snapshot[0]) - (qvel_actual_next[0]-qvel_snapshot[0])`

The sign should be consistent with `delta_ax` within integration/contact-solver effects. Report disagreement rather than hiding it.

Also record per snapshot:

- active-relative time
- MuJoCo/state tick
- measured/applied/target velocity
- gait phase
- roll/pitch
- physical contact mask/count
- internal controller contact mask if available via joined diagnostics
- `tau_ff`
- PD torque vector
- actual ctrl vector
- counterfactual ctrl vector
- `qacc_actual[0]`
- `qacc_cf[0]`
- `delta_ax`
- actual and counterfactual contact/constraint summary after forward dynamics if MuJoCo exposes a meaningful comparable quantity

Do not infer body-x acceleration from joint torque signs alone.

## Required summaries

For each W0-W3 window report:

- snapshot count
- median / p05 / p95 `delta_ax`
- fraction `delta_ax < 0`
- fraction `delta_ax > 0`
- median `|delta_ax|`
- baseline measured-minus-applied velocity error
- contact-mask distribution
- gait-phase coverage

Also report the full target interval `[31.90,32.60)`.

Because the PD effect can change sign with gait phase/contact pair, additionally stratify `delta_ax` by the major physical contact masks and/or diagonal support phase if sample count is sufficient. This is not parameter fishing; it is necessary to avoid an average hiding phase-dependent effects.

Do not over-fragment into tiny groups. Use only the dominant contact/phase groups.

## Hypothesis outcome

The hypothesis under test is:

**At the known residual-overspeed state, the post-ID PD contribution instantaneously pushes body-x dynamics in the forward direction enough to plausibly contribute to overspeed.**

Classify:

### SUPPORTED

Only if same-state replay is validated and removing PD produces a clear, consistently negative `delta_ax` of meaningful magnitude across the target overspeed states, not just a few isolated contact transitions.

### NOT SUPPORTED

If validated replay shows `delta_ax` is mostly positive (PD is braking) or near zero relative to the observed acceleration mismatch.

### MIXED / PHASE-DEPENDENT

If the effect is robustly forward in some support phases and braking in others with substantial cancellation. State which phases do what.

### INCONCLUSIVE

If exact snapshot/replay validation fails, state alignment is ambiguous, or numerical/contact-solver artifacts are too large.

Do not call PD the sole root cause even if SUPPORTED. This experiment measures the instantaneous same-state actuator contribution only; it does not capture long-horizon stabilization feedback.

## Important distinction from previous experiments

Do not run another live `tau_ff-only` gait and do not run another 300 ms PD-off pulse.

The previous B safety failure already showed that long enough PD suppression destabilizes the gait. This task explicitly avoids that confound by never allowing the counterfactual state to become the real trajectory.

## Controller/model invariants

Do not change:

- B semantic mapping
- period/duty
- gait pattern
- velocity shaper/governor
- Raibert/preview
- SRBD/MPC
- WBC weights/gains
- contact logic
- model/scene
- torque limits
- Phase1 profile
- safety/acceptance thresholds

No automatic follow-up experiment.

## Output

Commit one compact checkpoint containing:

- `docs/validation/phase1_pd_counterfactual_replay_20260913/RESULTS.md`
- `docs/validation/phase1_pd_counterfactual_replay_20260913/counterfactual.csv`
- minimal source/tool changes required for exact snapshot capture and replay

Raw exact-state snapshots may remain local if large; record their SHA256, format/version, source run ID, model/scene hash, and replay-tool hash.

`RESULTS.md` must state:

1. whether existing raw was sufficient or one new A diagnostic run was needed;
2. exact snapshot state representation;
3. exact active-time↔state-tick alignment method;
4. ACTUAL replay validation residuals;
5. W0-W3 and full-window `delta_ax` statistics;
6. dominant phase/contact stratification;
7. hypothesis result: `SUPPORTED`, `NOT SUPPORTED`, `MIXED/PHASE-DEPENDENT`, or `INCONCLUSIVE`;
8. one recommended next step, not executed.

Push exactly one experimental checkpoint after the analysis is complete and stop. Do not tune anything and do not automatically run the next experiment.