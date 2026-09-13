# Phase1 targeted ID dynamics closure

## Scope and provenance

This is one diagnostic-only `varying` run. No controller mathematics, gait parameters, WBC/SRBD weights, contact handling, model, or acceptance thresholds were changed. The source used for the run was `5288b13abeba687963613893bf309ddc6fc55fd5`, based on B semantic source `f87cf929ddbb124a85464ce2863a4584b4270e2c`. The source diff from `2c7a3b23635c05204e0fc4ebda8b56877365b8e1` contains only active-relative capture binding and diagnostic serialization of `M`, `h`, and per-leg `J`.

Command:

```text
TROT_DIAG_ID_CLOSURE=1 TROT_CPU_AUTOPIN=1 bash example/cpp/scripts/run_phase1_velocity_benchmark.sh varying _runs/phase1_id_dynamics_closure_target_20260913 231
```

The run is `varying_20260913_190838`. `controller_status=0`, `safety_status=0`, `completion_status=0`, `strict_pass=true`; no diagnostic-induced safety stop occurred. Raw files remain local and unmodified:

```text
data.csv                    200e8e92ab0a96a6d42c1be78be30d77e0b423ccff97e36fa44b8d78b1015c41
data.csv.id_closure.csv     811e2b27e64684b75f4637adf949f5ed70c8596d68fd48c86cd913e59c3f9fae
run_metadata.txt            c7c9cb768aea42232828592556adfa7f4e53b2749eb91b4883167a06fc90a382
run_manifest.json           66102a7560b9ea716b1cfefdb317d931df54e944b5f04442c67f97112b39c772
controller binary           20a2018373e9542f3011edf3df049cb65a0d505c84b807f456a41ff1a1602523
simulator binary            4b311183c0817aab82aee909d3f1a4d82fdec9ec8237eea5e8f3ba707475f137
scene                       12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8
profile                     9efcc3b2d89fb349a12990ace1cf6ceb45e0d731deb470bdf2af084d82449d74
```

The captured window is active-relative `31.902015716–33.091995436 s` (120 ticks); the requested endpoint window `[32,33)` contains 100 ticks. State ticks are `37.922–39.112 s`, confirming that capture is no longer keyed to the old absolute `30–34 s` condition. `closure_target.csv` contains every captured row, full `qdd`, `f`, solver/final `tau`, `M`, `h`, every leg's world linear `J`, masks, LowCmd fields, exact bridge PD torque, and independently recomputed closure terms.

## Coordinates and equation

The B-source `Go2RigidBody` uses `nv=18`, with the first three generalized velocities as world-frame base linear velocity, the next three as body angular velocity, followed by the MuJoCo joint DoFs. `M`, `h`, and `foot_jac_world` are evaluated from the same MuJoCo model state. ID-WBC uses world-frame linear contact force and the same stacked leg Jacobian. The offline closure is recomputed as

```text
lhs = M*qdd + h
rhs = S^T*tau + J^T*f
residual = lhs - rhs
```

The first six rows are the floating-base equation; actuator torque contributes only to the joint rows in `S^T*tau`.

## Endpoint `[32,33)` evidence

| quantity | median | p05 | p95 |
|---|---:|---:|---:|
| measured velocity (m/s) | 2.52367 | 2.50478 | 2.54621 |
| applied = kernel nominal (m/s) | 2.27633 | 2.25379 | 2.29522 |
| WBC desired ax (m/s²) | -2.47340 | -2.92424 | -2.09552 |
| SRBD ax (m/s²) | -1.62125 | -4.48985 | 0.00000 |
| ID `qdd[0]` (m/s²) | -2.20640 | -3.33336 | -0.74625 |
| exact base `J^T f` x (N) | 2.06957 | -17.99576 | 18.21974 |
| simple sum world `Fx` (N) | 2.06957 | -17.99576 | 18.21974 |
| realized ax, 100 ms local fit (m/s²) | 0.01671 | -0.34468 | 0.29154 |

The positive median velocity error is therefore present while WBC, SRBD, and ID `qdd[0]` are negative. A simple force sign is not a valid acceleration explanation: the base-x decomposition is median `M00*qdd[0]=-33.5514`, remaining inertial coupling `ΣM0j*qdd[j]=+14.8470`, `h_x=+24.2758`, and `J^T f|x=+2.0696`. These terms close the equation to numerical precision, so negative `qdd[0]` with positive simple `ΣFx` is dynamically legal.

Per-leg base-x `J^T f` medians are FR `+0.0540 N`, FL `+0.0511 N`, RR `+0.0498 N`, RL `+0.0487 N`; their signs vary with contact exchange rather than showing one leg with a persistent anomalous forward generalized force. In this model the exact base-x sum and simple world `Fx` agree within `1.1e-9 N` in the captured rows, but the full closure—not the simple sum—was used for attribution.

## Closure and output chain

For `[32,33)`, recomputed full residual has median `1.39e-6`, p95 `1.82e-5`, max `3.04e-5` in generalized-force units. Base-six residual has the same max bound; base-x residual has median `2.59e-7`, p95 `5.17e-7`, max `6.72e-7`. `solver_returned`, SRBD, ID-WBC, and logger solver status are all `100/100`; no stale fallback is present.

Solver `tau` and final `tau` are identical in the capture. MuJoCo-DoF-to-motor remap followed by final `tau` to LowCmd `tau_ff` differs by at most `9.5e-7 Nm` (median per-row maximum `2.37e-7 Nm`). The run has no enabled direct-force or Cartesian force overlay, and the captured force/tau post-solver deltas are zero; no post-QP clamp or force overlay is evidenced in this window.

The simulator bridge then applies, per motor,

```text
ctrl = tau_ff + kp*(q_des-q) + kd*(dq_des-dq)
```

This is not the ID `tau` closure command. In `[32,33)`, the exact effective actuator command differs from final mapped `tau` by a per-row maximum median `29.35 Nm`, p95 `64.49 Nm`, max `68.74 Nm` (vector-norm median `49.74 Nm`, p95 `98.19 Nm`). The first captured tick already has a nonzero PD delta. Thus the first directly evidenced post-ID deviation is the actuator/PD composition, not solver closure or `tau` serialization. `tau_est` also differs from the computed effective command (per-row maximum median `14.06 Nm`, p95 `44.11 Nm`), so the plant-side torque realization remains an observed secondary gap rather than a solver closure failure.

Internal ID contact masks and physical/logger threshold masks differ on `39/100` endpoint rows. Internal masks are `{0:7, 6:44, 9:42, 15:7}` and physical masks are `{0:24, 4:9, 6:32, 8:6, 9:29}`. This is a real topology discrepancy to retain for follow-up, but it is not by itself evidence of an ID equation error: the ID equation closes using its own mask and force solution, while the physical mask is a separate logger threshold. Its causal contribution cannot be separated from the PD composition in this single run.

## Time chain

Within the observable capture, WBC desired ax, SRBD ax, and ID `qdd[0]` are already negative at `31.9020 s`; the data cannot establish which became negative before the capture boundary. No `100 ms` continuous negative interval was found for exact base `J^T f|x` in the capture. Realized ax first becomes continuously negative at approximately `32.2220 s`, but remains centered near zero over the endpoint window. The time ordering supports “requested/model-side braking exists, physical acceleration does not follow” while also showing that the command actually delivered to the simulator is PD-modified.

## Attribution

* **A — ID optimization/dynamics closure failure: NOT SUPPORTED, strong.** The same-tick full equation closes to `3.04e-5` max, with base-x residual below `6.8e-7`.
* **B — ID output mapping/post-processing failure: SUPPORTED, strong.** The solver/final/LowCmd `tau_ff` mapping is numerically intact, but the simulator's post-ID PD composition changes the effective actuator command by tens of Nm. The precise attribution is **post-ID actuator/PD interaction**.
* **C — model-to-plant/contact realization failure: INCONCLUSIVE, medium.** Realized ax does not follow ID `qdd[0]`, and masks disagree, but the effective actuator command is not the ID torque, so plant/contact failure is not isolated.
* **D — diagnostic semantic error: NOT SUPPORTED, strong.** The qdd frame, force frame, Jacobian, DoF ordering, and bridge composition were confirmed from source and checked by closure.
* **E — distributed coupling: PARTIALLY SUPPORTED, medium.** PD interaction is the first evidenced deviation; contact-topology/plant realization remains a simultaneous secondary contributor that this one run cannot isolate.

Final answer: in the correctly aligned B window, the first stable, directly evidenced loss is after ID-WBC, at the actuator command composition (`tau_ff` plus simulator PD). The ID solution itself is dynamically self-consistent. This does not prove that PD interaction is the sole physical root cause.

## One next step (not executed)

Run one diagnostic-only actuator-composition A/B on the same B varying profile: keep all controller outputs, LowCmd `q/dq/kp/kd`, contact logic, model, and timing unchanged, but compare the bridge's current `tau_ff+PD` composition against a `tau_ff`-only actuator input. This isolates the newly evidenced post-ID interaction without changing ID/WBC mathematics or gait parameters.
