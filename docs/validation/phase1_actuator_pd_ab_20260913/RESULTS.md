# Phase1 actuator-composition A/B

## Scope and unique variable

This checkpoint changes only the simulator bridge actuator composition. A uses the existing bridge expression

```text
ctrl = tau_ff + kp*(q_des-q) + kd*(dq_des-dq)
```

B selects the default-off `TROT_BRIDGE_TAU_FF_ONLY=1` branch:

```text
ctrl = tau_ff
```

Controller output, LowCmd `q_des/dq_des/kp/kd/tau_ff`, B semantic, period `0.14`, duty `0.44`, profile, contact logic, WBC/SRBD, model, torque limits, and acceptance thresholds were unchanged. The bridge change is in commit `5719d4fde483a1ce8752ff562f60157d84c2b786`; with the flag unset the original expression remains the default.

No controller or parameter tuning was performed. B was run twice, the maximum allowed: the first attempt lacked `TROT_DIAG_ID_CLOSURE=1` and is instrumentation-incomplete; the second had the correct diagnostic flag but never entered continuous trot. No third run was started.

## Provenance and commands

A reuses `varying_20260913_190838`, captured from source `5288b13abeba687963613893bf309ddc6fc55fd5`, with the prior targeted closure instrumentation. It completed with the prior endpoint `[32,33)` containing 100 rows.

B attempts used source `5719d4fde483a1ce8752ff562f60157d84c2b786`, controller SHA256 `20a2018373e9542f3011edf3df049cb65a0d505c84b807f456a41ff1a1602523`, tau-ff-only simulator SHA256 `682c073cc871374946fd8c4671fdffbbe41c2f3a1b8c9e908d291a23fb2ff2d7`, scene SHA256 `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`, and varying profile SHA256 `9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74`.

```text
flock -n /tmp/go2_mujoco_experiment.lock env TROT_BRIDGE_TAU_FF_ONLY=1 TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_actuator_pd_ab_20260913 232
flock -n /tmp/go2_mujoco_experiment.lock env TROT_BRIDGE_TAU_FF_ONLY=1 TROT_DIAG_ID_CLOSURE=1 TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_actuator_pd_ab_20260913 232
```

The first B run was manually stopped after it failed to produce a continuous-trot evaluation and is recorded as invalid for endpoint analysis. The second was manually stopped after the same preflight stall; its closure file is header-only. Runner metadata status fields were zero, but this does not override the absent gait/evaluation rows.

Raw files remain local. Hashes are recorded here and the CSV contains the derived quantities needed for the conclusions:

```text
A varying_20260913_190838
  data.csv              200e8e92ab0a96a6d42c1be78be30d77e0b423ccff97e36fa44b8d78b1015c41
  data.csv.id_closure   811e2b27e64684b75f4637adf949f5ed70c8596d68fd48c86cd913e59c3f9fae
  run_metadata.txt      c7c9cb768aea42232828592556adfa7f4e53b2749eb91b4883167a06fc90a382
  run_manifest.json     66102a7560b9ea716b1cfefdb317d931df54e944b5f04442c67f97112b39c772
B varying_20260913_192546 (diagnostic flag omitted)
  data.csv              53dc8973b23dbd78176f4d9fefa73a400ae928e1cec31eb27052b7da6ed6d2fb
  run_metadata.txt      096dc6a521ad5dad2572ebe0b4101c06474d6d687ab10cf823f91c5f30a16dcf
  run_manifest.json     370029249e1efb00e28b906dd6e517bad419e79c068939925b00f5ff299f2a59
  environment.txt       7d790e074da1a7691ffbd8526e0359c88800eeb01b040744b81300cd8dc142dc
B varying_20260913_193514 (correct diagnostic flags)
  data.csv              15137d9c43087e85c3cb75defcf0ba584e43156fff1a17fadeed8c1a89b5d22d
  data.csv.id_closure   2628ea3ea003472e1fa3ea02ac01eef83d051c01e0729abac3ac6cb65180cf14
  run_metadata.txt      35f84bacb51d73311e198e7721626dff2030f0764e7c226335c212432b3a3436
  run_manifest.json     001c583cb4764fdab43ef6d1e6c357e92243af3beef87d9b5439c9e077fbba17
  environment.txt       8357ab922945a8435c318ee5f68a2b27b0b5ab474e4a1bceaff1a52058396dd6
```

## Endpoint comparison

Only A reached the requested endpoint, so B endpoint tracking, settling latency, overspeed peak, realized acceleration, and A/B model-side equality are **not evaluable**. A's exact `[32,33)` values are:

| quantity | A baseline |
|---|---:|
| measured median / p05 / p95 (m/s) | 2.523670 / 2.504776 / 2.546212 |
| measured - applied median (m/s) | 0.247340 |
| measured - target median / peak (m/s) | 0.223670 / 0.252182 |
| realized ax median / p05 / p95 (m/s²) | 0.01671 / -0.34468 / 0.29154 |
| 1 s settling for 1.4→2.3 | FAIL |
| applied < shaped fraction; shaped-applied median (m/s) | 99% / 0.02367 |
| WBC/SRBD/ID solver status | 100/100 valid |
| ID qdd x median (m/s²) | -2.206401 |
| roll/pitch absolute peak (deg) | 1.53 / 1.85 |

A's prior same-tick closure remains numerical: full residual max `3.04e-5`, base-x residual max `5.86e-7`. This is the comparison anchor, not a new A run.

## Torque composition

In A's endpoint window, ID solver tau and LowCmd `tau_ff` mapping are unchanged, while the bridge PD term is large: absolute per-joint PD median `5.85 Nm`, p95 `39.82 Nm`, peak `68.74 Nm`; per-row maximum PD median `32.51 Nm`, p95 `67.45 Nm`, peak `72.74 Nm`. The resulting effective-command peak is `72.74 Nm` versus ID/LowCmd peak `30.41 Nm`; effective command minus `tau_est` peaks at `51.56 Nm`.

Using `PD * ID_tau` for the requested sign diagnostic, PD opposes ID tau on `42.3%` of joint-ticks and assists on `57.7%`; median product is positive. This is not a body-axis force attribution, but it shows that large PD magnitude alone does not establish systematic braking cancellation. B's source branch makes effective command equal to `tau_ff` by construction; no B endpoint sample exists in which to validate that identity numerically.

## B stability result and attribution

Both B attempts remained in the stand/high-speed preflight path and produced zero active-relative diagnostic rows and zero continuous-trot evaluation rows. The correctly instrumented second run had motion stages `0/1/3` only, stage 1 from about `3.00` to `165.11 s`, contact count minimum `2` (preflight requires `3`), and pitch absolute peak `0.14946 rad` (`8.56°`, above the `0.08 rad` preflight angle). It was manually stopped; no controller hard-posture log was observed, so this is a preflight stall/stability regression rather than a classified hard-safety stop.

Thus tau-ff-only is not a viable drop-in replacement: it cannot reach the 1.4→2.3 endpoint under the otherwise unchanged closed loop. The experiment does not prove whether residual endpoint overspeed would improve, because the endpoint was never reached. The requested hypothesis is **PARTIALLY SUPPORTED**: B strongly supports that the post-ID PD composition is causally important to maintaining this gait/preflight behavior, while residual-overspeed causality at the endpoint remains **INCONCLUSIVE**. PD/ID coordination remains plausible; PD is not shown to be the sole root cause.

## One next step (not executed)

After human approval, design one preflight-safe actuator-composition isolation that preserves the observed stabilization path while exposing the ID/PD interaction; do not change gains or run it automatically.
