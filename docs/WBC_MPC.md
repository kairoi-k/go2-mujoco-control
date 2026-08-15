# WBC / MPC line

Goal: on `--wbc-full`, the walk-phase command is a centroidal whole-body QP plus a receding-horizon foothold MPC. `real_trot_go2` without the flag stays the 0.15 m/s baseline.

## What `--wbc-primary` does today

Stance legs get an extra `M a` torque; gravity/bias `h` is left to the position servo; contact tasks often drop the moment when a foot is in the air. That is incremental feedforward, not a whole-body controller.

## What `--wbc-full` is

- Desired centroidal wrench `W* = M a* + h` (`centroidal_wbc.h`).
- All six wrench axes stay in the task.
- Contact forces are the solution of a dense inequality QP (`contact_wrench_qp.h` / `dense_qp.h`): friction pyramid, unilaterality, inactive feet at zero. The pyramid is inscribed in the cone (`μ/√2`). If the QP is infeasible it falls back to the projected allocator.
- Stance PD is `kWbcFullStanceKp/Kd` (25 / 2.0). Swing legs stay IK + position control. Joint torque is `J^T f`.
- Footholds come from an N-step receding-horizon QP (`footstep_mpc.h`): all preview adjustments are solved together; only the first is applied. The first-step planned `a_x` is the centroidal acceleration task.

This is still centroidal (6-DoF base `M` from the simulator), not a full 18-DoF inverse-dynamics WBC, and not OSQP/qpOASES. It is a complete centroidal WBC + foothold MPC on the `--wbc-full` path.

## How to run

```bash
bash example/cpp/scripts/go2sim walk --wbc-full
```

Or trot-only:

```bash
./example/cpp/build/real_trot_go2 lo 20 /tmp/wbc_full.csv --wbc-full --kernel raibert-trot
```

`--preview-horizon 0` after `--wbc-full` keeps the centroidal QP and turns the foothold MPC off.

## Acceptance

The gated 0.15 m/s trot is the comparison. This path is ahead of that baseline only if it walks faster without failing the same cycle-quality / torque gates. That measurement is not claimed yet.
