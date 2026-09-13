# Task: Phase1 baseline reproduction gate before PD pulse A/B

Read `docs/research/PHASE1_AGENT_CONTRACT.md` first. This task replaces the previous immediate A/B attempt until the baseline gate below passes.

## Why this task exists

The previous delayed-PD pulse experiment stopped correctly because its A control failed at active time about 19.36 s, before the pulse window. That A used an explicit fresh seed (`TROT_SEED=241`) and therefore was not a strict reproduction of the known successful targeted-closure run `varying_20260913_190838`.

Do not interpret that A failure as evidence about PD. The next job is only to determine whether the current experimental source can reproduce the known successful Phase1 regime before any B intervention is allowed.

Known successful reference:

- run: `varying_20260913_190838`
- checkpoint/report lineage: `1fc68a34233551ac9ed1e57f68cc86357fd560ad`
- source used there: `5288b13abeba687963613893bf309ddc6fc55fd5`
- command semantics: `TROT_DIAG_ID_CLOSURE=1 TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying ...`
- importantly: no explicit `TROT_SEED` was set in that successful run
- that run reached active `[32,33)` and reproduced the known residual overspeed with no new safety failure.

Current branch contains the default-off delayed PD pulse implementation from the previous attempt. Preserve the failed checkpoint as audit evidence; do not delete or rewrite it.

## Goal

Establish a valid A baseline from the current source/binary that reaches active time 32 s and reproduces the qualitative `varying 1.4→2.3` residual-overspeed regime.

Only after that gate passes may the existing delayed PD pulse B be run.

## Step 1 — make flag plumbing baseline-safe

Audit the current pulse diff against `1fc68a34233551ac9ed1e57f68cc86357fd560ad`.

The experimental flag must not perform environment parsing inside the 500 Hz `WriteMotorCommands` hot loop.

Refactor only the experiment plumbing so that:

- the pulse enable flag is read/cached once during initialization/setup;
- the hot loop only reads a cached boolean and checks the active-relative time window;
- flag OFF preserves the original baseline command path;
- no controller mathematics, gains, timing, gait, WBC/SRBD, contact logic, q/dq targets, tau_ff, model, scene, or benchmark thresholds change;
- the pulse window remains exactly active-relative `[32.10,32.40)`;
- flag ON still suppresses only LowCmd `kp/kd` in that window and restores them immediately afterward.

Do not optimize or refactor unrelated code.

## Step 2 — reproduce the known baseline semantics

Run A with the pulse flag OFF using the successful `190838` invocation semantics as closely as possible:

- `varying` profile;
- `TROT_DIAG_ID_CLOSURE=1`;
- `TROT_CPU_AUTOPIN=1`;
- no explicit `TROT_SEED`, unless you can prove from the old metadata that one was implicitly fixed and recover the exact value;
- same controller parameters, B semantic, period `0.14`, duty `0.44`, governor, Raibert, preview, WBC/SRBD, model/scene, acceptance thresholds;
- use a fresh DDS domain/output directory as infrastructure requires.

Do not add experimental environment variables beyond what is needed for the existing diagnostics. In particular, `TROT_PD_PULSE_AB` must be unset for A.

Record the complete effective environment/command and binary/source hashes.

## A baseline gate

A passes only if all of the following are true:

1. It reaches continuous trot and active-relative time at least `33.0 s`.
2. It reaches the intended targeted capture window around active `[31.9,33.1]`.
3. No new hard-posture/safety failure occurs before or during that window.
4. Around active `[32,33)`, it reproduces the known qualitative state:
   - measured speed remains above applied/target by a material amount;
   - WBC desired x acceleration is negative;
   - ID `qdd[0]` is negative;
   - realized body x acceleration is much less negative than model-side braking request / remains near the prior mismatch pattern.

Do not require exact numerical identity with `190838`; this is a reproducibility gate, not a bitwise replay. But report the main differences numerically.

If A fails before 32 s again, STOP. Do not run B. Classify the result as a baseline reproducibility/robustness problem and report the first divergence from the known-good run that can be supported by evidence. Do not tune anything.

One replacement A is allowed only for an infrastructure-invalid launch (DDS/startup/instrumentation failure). A genuine posture/gait failure is not an infrastructure invalidation and must not be retried just to obtain a passing run.

## Step 3 — only if A passes, run B

Use the exact same built source/binary and the same run semantics as A, with the single difference:

`TROT_PD_PULSE_AB=1`

Do not set a seed for B if A did not set one. Do not introduce any other difference.

B must remain identical to A before active `32.10 s`. Verify from logs that the PD suppression is active only on `[32.10,32.40)` and normal PD resumes at `32.40 s`.

Run B once. Do not retry a genuine B instability/safety response.

## Measurements if B is reached

Compare A/B on:

- pre-pulse `[31.90,32.10)`;
- early pulse `[32.10,32.20)`;
- late pulse `[32.20,32.40)`;
- recovery `[32.40,32.60)`.

Report:

- requested/shaped/applied/kernel nominal velocity;
- measured velocity and measured-minus-applied;
- realized body-x acceleration using the same 100 ms local-fit method;
- WBC desired ax, SRBD ax, ID `qdd[0]`;
- ID tau / LowCmd tau_ff;
- PD contribution and effective actuator command;
- tau_est if available;
- internal vs physical contact masks;
- roll/pitch and safety state.

Primary causal effects:

1. `realized_ax_B - realized_ax_A` during the pulse;
2. `v(t)-v(32.10)` A vs B;
3. whether posture/contact topology changes immediately and confounds the result;
4. recovery after PD restoration.

Do not infer body braking from joint torque signs alone.

## Interpretation

If A fails before the intervention window: `BASELINE GATE FAILED`; PD causality remains INCONCLUSIVE.

If A passes and B produces more braking / less velocity excess with comparable model-side braking and no immediate contact/posture collapse: supports a causal PD contribution.

If A passes and B weakens braking or destabilizes the gait: does not support “PD causes the overspeed”; instead PD is required for this closed-loop gait and PD/ID coordination may still need study.

If A and B remain close: little/no causal effect.

If B causes an immediate contact/posture topology change that prevents clean separation: INCONCLUSIVE.

Never call PD the sole root cause from this experiment.

## Output

Use a new compact checkpoint under:

`docs/validation/phase1_pd_pulse_baseline_gate_20260913/RESULTS.md`

`docs/validation/phase1_pd_pulse_baseline_gate_20260913/ab.csv`

Include only the minimal source diff for cached flag plumbing plus experimental evidence. Raw logs may remain local with SHA256/run IDs recorded.

Push one checkpoint and stop. Do not tune PD, change gains, modify WBC/SRBD/contact logic, alter the profile, or start terrain/Phase2 work.
