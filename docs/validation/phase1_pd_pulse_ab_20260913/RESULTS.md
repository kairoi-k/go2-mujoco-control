# Phase1 delayed post-ID PD pulse A/B checkpoint

## Outcome

`INCONCLUSIVE`. The checkpoint stops before B because the A baseline did not
reach the required active-relative causal window. No PD causal conclusion is
drawn.

## Hypothesis and unique variable

The hypothesis was that suppressing only simulator-side/post-ID joint PD
during active-relative `[32.10,32.40) s` would increase realized braking or
reduce short-horizon velocity excess while WBC/ID-WBC outputs stayed normal.
The only experimental variable was `TROT_PD_PULSE_AB=1`; the implementation
uses `gait_elapsed_s` and writes LowCmd `kp=kd=0` only in that half-open
window. The default is off. q/dq targets, tau_ff, ID/WBC mathematics, model,
profile, gait, and thresholds were unchanged.

## Provenance

- Checkout: `research/phase1-pd-pulse-ab-20260913`.
- Source base: `1fc68a34233551ac9ed1e57f68cc86357fd560ad`; task checkout before
  the source diff: `e818face603fc4d8bf1ef5a78c535d2e3164f364`.
- Modified source SHA-256: `56525af3941419b96fe6261e72445a8b5def522505238696854421707fec9350`
  (`example/cpp/trot/trot_experiment_control.cpp`).
- Simulator SHA-256: `2c82dc3b3f67efed51a4287fbcf584ef345559a6d0ce763ac16a648c927814be`.
- Controller SHA-256: `32fcd476a7b008e3a4eb114a6862e277455ebdb1d875979fa9c45d19a9379c9c`.
- Scene SHA-256: `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`.
- Profile SHA-256: `9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74`.
- Seed: `241`; `TROT_DIAG_ID_CLOSURE=1`; headless wall-clock motion.

## Runs

The first A launch (`varying_20260913_202643`, domain 220) was an
infrastructure-invalid replacement: the simulator aborted with DDS “Failed
to find a free participant index”. Its simulator log SHA-256 is
`de075eecc5c5325a985731d35fbcaa619b318db6135472a05c13e38744e97529`.

The permitted replacement A control was
`varying_20260913_202733`, domain 230, with `TROT_PD_PULSE_AB` unset. Its
effective controller arguments were:

```text
TROT_SEED=241 GO2_PROFILE_PATH=example/cpp/configs/phase1_velocity_varying.csv TROT_DIAG_ID_CLOSURE=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_pd_pulse_ab_20260913/A 230
```

B was not launched. The planned B invocation would have been the same command
with `TROT_PD_PULSE_AB=1`; it was deliberately not executed after A failed.

```text
--wall-clock-motion --wbc-full --gait-pattern running-trot --kernel raibert-trot --period 0.14 --duty 0.44 --step-length 0.50 --foot-lift 0.20 --tau-limit 45 --raibert-velocity-gain 0.010 --raibert-max-adjustment 0.06 --preview-horizon 4 --support-anchor-feedback --support-anchor-gain 0.35 --velocity-max-accel 0.80 --velocity-max-decel 1.20 --velocity-max-jerk 4.0 --velocity-command-script example/cpp/configs/phase1_velocity_varying.csv --velocity-max-tracking-lead 0.20 --domain-id 230
```

A reached continuous low-speed trot but failed before the required window:
`cmd_time_s` max `26.462881284`, diagnostic active-relative time max
`19.362063632`, `safety_status=1`, `completion_status=1`. The first
`|roll|>1 rad` occurred at `cmd_time_s=24.682132649` with
`roll=-1.004745960 rad`; the last logged roll was `-3.140872478 rad`.
The closure CSV contains only its header, so no closure-window measurement
exists. B was not run, as required by the stopping rule.

## Required windows and comparability

All four requested windows are marked `not_reached` in `ab.csv`. A/B
pre-pulse comparability cannot be assessed because B does not exist and A
did not reach `[31.90,32.10) s`. Source inspection proves the intended pulse
boundary and A's default-off setting, but there is no runtime B proof of
zero/restore timing. There are therefore no valid velocity, acceleration,
PD-torque, effective-command, contact, or posture comparisons.

## Raw evidence

For A `varying_20260913_202733`: `data.csv`
`0574cd971b708b21776aff39394c47898067a375ff80c6126295e2607db6def1`;
`data.csv.id_closure.csv`
`2628ea3ea003472e1fa3ea02ac01eef83d051c01e0729abac3ac6cb65180cf14`;
`controller.log`
`ad09f219995cbb8506698cfc25e1fee3c50e4e02b4d9201daa53ddce9ce5d703`;
`simulator.log`
`94d289b586914003faea4026f779f2e16704068ee5ec39e914801163d79de2f6`;
`run_metadata.txt`
`04edfe679c98fb40c136357ae744f10eea73861716839d007e353158b960afa3`;
`run_manifest.json`
`7be44882eeed504e52d4416198f3f694b90089d2b014014ae5d7abeceb2a909f`;
`contact_ground_truth.csv`
`67964f85939f1bf04c6221042abcd531cde8da7a01972b8c4c5846831edae8ff`.

## Recommended next step

Audit and re-establish a valid fixed-state A baseline that reaches active 32 s; only after that gate passes should the exact declared A/B pulse be run. This next step was not executed.
