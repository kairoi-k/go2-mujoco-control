# Reactive transition matrix protocol v1.1

## Purpose

Verify whether one continuous-reference transition layer and one WBC/MPC plant can handle directed event changes without pair-specific action stitching.

## Matrix

Seven nonterminal source events (`obstacle_left`, `obstacle_right`, `turn_left`, `turn_right`, `slip`, `low_friction`, `impact`) transition to each of the eight event states, excluding self-transitions: 7 × 7 + 7 = 49 directed pairs. `emergency_stop` is absorbing, so outgoing transitions are intentionally not required.

## Fixed protocol

- One controller binary, scene, gait parameters, WBC/MPC path, and configuration fingerprint for every pair.
- Two adjacent 2.0 s scheduled events at 1.5 s and 3.5 s after gait start; 7.5 s controller duration and 1.5 s post window.
- Automatic sensor-event detection is disabled for this matrix so the event sequence is deterministic and attributable to the protocol script. Sensor-triggered adaptation is a separate experiment.
- At most two attempts are allowed for infrastructure startup failures, with a 5 s cooldown and alternate DDS domains; attempts are retained in each manifest.

## Acceptance gates

CSV completeness and nondecreasing time; exact event sequence (or absorbing emergency termination); WBC/MPC stage continuity; bounded reference rates; finite solver/status fields; Ground Truth support from `total_contact_grf_world_z_N` with no more than five consecutive 2 ms unloading samples; velocity jump, roll, pitch, target-sign, and event-response checks.

## Reproduction

From the repository root:

```bash
python3 example/cpp/tools/run_reactive_transition_matrix.py \
  --root example/cpp/experiments/go2_reactive_transition_matrix_2026-08-20 \
  --start-index 1 --count 49 --domain-base 100 --domain-stride 49 --max-attempts 2
python3 example/cpp/tools/analyze_reactive_transition_matrix.py \
  example/cpp/experiments/go2_reactive_transition_matrix_2026-08-20
```

The final report and metrics are generated under `reports/`; raw CSV, logs, scripts, and manifests remain beside them for auditability. This proves the scripted common transition path, not autonomous perception or local obstacle planning.
