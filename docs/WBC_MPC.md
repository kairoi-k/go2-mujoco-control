# WBC / MPC line

`--wbc-full` is a controller-side 18-DoF inverse-dynamics WBC plus receding-horizon SRBD MPC. `go2sim walk` / `real_trot_go2` without the flag stays the 0.15 m/s position-control baseline.

## Stack

1. **Model.** The controller loads the same Go2 MJCF the simulator uses (`go2_rigid_body.h`). Every tick it evaluates `M(q)`, `h(q,qd)`, foot Jacobians, and CoM inertia from `LowState` / `SportModeState`. Packed LowState spare-slot `M` is not used on this path.
2. **MPC (~20 Hz).** Single-rigid-body receding horizon (`srbd_mpc.h`): CoM pose/velocity, orientation, contact forces, friction pyramid, swing forces at zero. Horizon 6 × 0.05 s. The CoM x/y reference integrates commanded velocity; world `y` is held at 0. First-knot force implies the CoM linear/angular acceleration the WBC tracks.
3. **WBC (500 Hz).** Hierarchical inverse-dynamics QP (`inverse_dynamics_wbc.h`):
   - equality: floating-base rows of `M qdd + h = J^T f` (KKT ADMM, not two-sided inequalities)
   - hard: friction pyramid, unilaterality, swing `f ≈ 0`, `|τ| ≤ 35 N·m`
   - tasks: CoM/orientation acc from MPC, swing-foot PD, stance no-slip, posture
   - `τ* = M_j qdd + h_j − J_j^T f`
4. **Motors.** Stance: `τ*` plus `kp=25`. Swing: IK + position PD. During trot the QP uses the gait contact schedule (force sensors stay high through lift-off). Diagonal trot no longer halves `τ*`.

Gait timing still comes from the Raibert kernel. The kernel is a warm start / swing target, not the force planner.

## Bumpless stand-to-walk handoff

`--wbc-full` now uses the same WBC/MPC plant during settle, locomotion, and
return-to-stand. Stand-up and lie-down remain joint-PD phases because the
near-floor posture is outside the SRBD support model. During settle the QP
uses measured support; the CoM reference is slewed from the measured CoM to
`kWbcPrimaryBaseHeightM`, and the total motor command (torque, `kp`, and `kd`)
is blended over bounded rise/fall durations. The gait velocity reference is
then quintically blended over `kGaitBlendDuration`; the scheduled diagonal
contact mask is held back briefly and its resulting WBC torque is interpolated
when the mask changes.

The CSV handoff evidence is `wbc_primary_blend`,
`wbc_gait_reference_blend`, `wbc_contact_schedule_blend`,
`wbc_contact_transition_blend`, `wbc_com_z_m`, and `wbc_com_ref_z_m`.

## Cartesian world (`--cartesian-world`)

`go2sim full2` turns on world-frame stance hold and Cartesian swing (`cartesian_world_trot.h`). Stance feet are IK'd to a captured world anchor; swing is a world quintic to a Raibert foothold with feedforward velocity; ID-WBC tracks `J qdd + Ĵ q̇ = a_des`. After the first cycles the gait is a short-stance running trot (`T_st ≈ 0.11 s`, duty ~0.5), not the 0.75 walking trot. MPC holds the captured heading; the speed governor follows measured body `v_x` with a small lead.

This is the Mini Cheetah representation, not a 2 m/s result. Headless **280/280 complete**, last-8s **0.50 m/s** (`full2_185811`), return to stand. Peak cycle-mean about **0.79 m/s**. Variance is high; a later 280-cycle repeat can quality-reject. Hard stance equalities and large `J^T` cartesian pulls caused roll kills and were not kept. `go2sim walk` / `full` stay the 0.15 gate.

## How to run

```bash
bash example/cpp/scripts/go2sim full
```

Requires `simulate/mujoco` (the controller links `libmujoco` and loads `unitree_robots/go2/go2.xml`). `go2sim full` sets `--tau-limit 35` to match the ID-WBC motor envelope. `go2sim walk` stays at the 18 N·m position-control gate.

## Unit tests

- `test_go2_rigid_body`: mass, `h_z ≈ mg`, RNEA residual
- `test_srbd_mpc`: 4-contact and 2-contact gravity
- `test_inverse_dynamics_wbc`: stand residual ~1e-9 N; diagonal 2-contact feasible
- `test_dense_qp`: inequality ADMM plus equality KKT
- `test_wbc_transition`: stage gates, bounded plant blend, and reference/contact handoff endpoints

## Same-gate comparison (2026-08-15)

Protocol (nominal `0.091/0.60 = 0.151667` m/s):

```text
--period 0.60 --duty 0.75 --foot-lift 0.020 --kp 63 --kd 2.8
--kernel raibert-trot --raibert-velocity-gain 0.05 --raibert-max-adjustment 0.010
--world-feedback-max 0.060 --world-feedback-slew 0.004 --wbc-primary
--step-length 0.091 --headless --max-cycles 64
```

`walk` is that command. `full` adds `--wbc-full --tau-limit 35`. Speed is `analyze_locomotion_progress.py` on `motion_stage==2`. Raw CSVs stay in gitignored `_runs/`.

| Arm | Gate | Cycles | measured m/s | ratio vs 0.151667 | lateral m | Notes |
|---|---|---|---|---|---|---|
| `go2sim walk` | FAIL `q_error=0.296` at cycle 3 | 3 | 0.096 | 0.63 | 0.010 | this-session walk still dies; Aug 9 64-cycle pass not reproduced |
| `go2sim full` | PASS 64/64, return to stand | 64 | **0.1494** | **0.985** | 0.012 | ID 100%; eq residual med 1.8e-7; cruise roll/pitch 0.34°/0.18° |

`--wbc-full` completed the gated 0.15 m/s trot. Commanded speed was matched to 1.5%. That is the cruise the walk arm is supposed to hold; in this session only `--wbc-full` actually held it.

Stand ID-WBC unit-test residual is ~1e-9 N. On the 64-cycle walk, floating-base residual median 1.8e-7 N (max 2.3e-6). ID-WBC returned a feasible `τ*` on **100%** of walking ticks; SRBD was feasible every walking tick; feedforward applied every walking tick. Median WBC 176 µs; 9 / 19201 ticks exceeded 1 ms (MPC).

A 32-cycle probe at commanded 0.25 m/s (`--period 0.50 --step-length 0.125`) completed with ratio 0.94. Commanded 0.30 m/s under-tracked (ratio 0.84) and sat on the q_error quality envelope. Faster cruise is not claimed.

## Still not the hybrid-OCP global optimum

- Contact *timing* is not optimized (duty/period stay on the kernel). That is the observed ceiling above ~0.25 m/s.
- MPC is SRBD, not full-body DDP.
- QP is dense ADMM, not HPIPM/OSQP.
- 18-DoF ID is in the WBC, not in the horizon.

Those are the next theoretical steps. They are not required to claim that `--wbc-full` now walks the 0.15 gate with a real dynamics model, a real ID QP, and near-zero RNEA residual.
