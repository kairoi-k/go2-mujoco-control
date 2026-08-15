# WBC / MPC line

Goal: on `--wbc-full`, the walk-phase command is a centroidal whole-body QP plus a receding-horizon foothold MPC. `real_trot_go2` without the flag stays the 0.15 m/s baseline.

## What `--wbc-primary` does today

Stance legs get an extra `M a` torque; gravity/bias `h` is left to the position servo; contact tasks often drop the moment when a foot is in the air. That is incremental feedforward, not a whole-body controller.

## What `--wbc-full` is

- Desired centroidal wrench `W* = M a* + h` (`centroidal_wbc.h`).
- All six wrench axes stay in the task.
- Contact forces are the solution of a dense inequality QP (`contact_wrench_qp.h` / `dense_qp.h`): friction pyramid, unilaterality, inactive feet at zero. The pyramid is inscribed in the cone (`μ/√2`). If the QP is infeasible it falls back to the projected allocator.
- Stance PD is `kWbcFullStanceKp/Kd` (25 / 2.0). Swing legs stay IK + position control. Joint torque is `J^T f`.
- Footholds come from an N-step receding-horizon QP (`footstep_mpc.h`): all preview adjustments in x and y are solved together; only the first is applied. The first-step planned `a_x` is the centroidal acceleration task.

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

Headless `run_trot.sh` must not wait on a MuJoCo GLFW window. An xdotool search there blocks up to 20 s while physics runs without LowCmd, which is not part of the gait comparison.

## Same-gate comparison (2026-08-15)

Protocol (nominal `0.091/0.60 = 0.151667` m/s):

```text
--period 0.60 --duty 0.75 --foot-lift 0.020 --kp 63 --kd 2.8
--kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010
--world-feedback-max 0.060 --world-feedback-slew 0.004 --wbc-primary
--step-length 0.091 --headless
```

`walk` is that command. `full` adds `--wbc-full`. Speed is `analyze_locomotion_progress.py` on `motion_stage==2`. Raw CSVs stay in gitignored `_runs/`.

| Arm | Gate | Cycles until reject | measured m/s | ratio vs 0.151667 | lateral drift m | Notes |
|---|---|---|---|---|---|---|
| `go2sim walk` | FAIL `q_error=0.293` at cycle 3 | 3 | 0.0866 | 0.57 | 0.0084 | 500 Hz wall; projected wrench satisfied 35/1202 ticks |
| `go2sim full` | FAIL `q_error=0.292` at cycle 1 | 1 | 0.0499 | 0.33 | 0.0064 | QP wrench satisfied 0/600 ticks; residual ~8.7 |

`--wbc-full` did not beat the 0.15 baseline under these gates. It rejected earlier and moved slower on the walking samples it produced.

The same-host Aug 9 `go2sim walk` (same sim binary and scene hash, 64 cycles, quality pass) is not reproduced in this session: the Aug 13 local `real_trot_go2` also rejected at cycle 3 with the same first-cycle pitch (~3.8° vs historical ~0.7°) and world-ref `x≈-0.091` (historical `x≈-0.050`). That confounder is recorded; it is not used to claim a public-tree gait regression, and it is not used to claim that `--wbc-full` is faster.
