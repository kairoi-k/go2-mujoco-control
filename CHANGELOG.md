# Changelog

## 2026-08-19

- `go2sim task` / `full` on main are `--wbc-full --tau-limit 35` (`2b82dae`). Indexed cruise is the 2026-08-18 repeat: `full` n=5 **0.130 ± 0.011 m/s**, `task` n=3 **0.139 ± 0.004 m/s**. Record: `example/cpp/experiments/go2_wbc_full_mainline_repeat_2026-08-18`.
- Homepage clip is `docs/media/stand_walk_lie_wbcfull.gif` / `.mp4`. The older `--wbc-primary` `stand_walk_lie.gif` is kept.
- 2026-08-15 single-run 0.149 m/s stays in `docs/WBC_MPC.md` as historical. `go2sim walk` (`--wbc-primary`) is not claimed on this tree.

## 2026-08-15

- Checkout vendors Go2 MJCF only; unused Unitree robot meshes are not in this tree.
- `ctest` registers the CPU unit tests (FK still needs the local MuJoCo library).
- `example/cpp/MODULES.md` points at `docs/CODE_GUIDE.md`.
- `--wbc-full` centroidal wrench (`W = M a + h`); N-step foothold preview now selects the next Raibert touchdown; stance PD is lowered on that path.
- Preview terminal velocity error feeds the centroidal `a_x` task on `--wbc-full`.
- `--wbc-full` contact forces come from a dense inequality QP; footholds come from a receding-horizon MPC that jointly plans the preview.
- Foothold MPC also plans lateral (y) touchdowns from body-frame `v_y`.
- Headless `run_trot.sh` no longer waits up to 20 s for a MuJoCo window before starting the controller.
- `--wbc-full` is controller-side 18-DoF ID-WBC + SRBD MPC (`go2_rigid_body.h`, `srbd_mpc.h`, `inverse_dynamics_wbc.h`). Equality QP uses a KKT ADMM. Same-gate 64-cycle: 0.149 m/s, ratio 0.985, ID/SRBD 100%, eq residual ~1e-7 N. `go2sim walk` is unchanged.
- `--cartesian-world` (`go2sim full2`): world-frame stance + Cartesian swing, running-trot schedule, heading-hold MPC, swing velocity PD. Headless 280/280 complete, last-8s **0.50 m/s**, peak cycle ~0.79 m/s. Not 2 m/s. `go2sim walk` / `full` unchanged.

## 2026-08-14

- README clip for stand-walk-lie.
- Isaac Lab velocity-RL snapshot and `model_54950` were published; that track
  is now maintained in [`kairoi-k/go2-isaaclab-rl`](https://github.com/kairoi-k/go2-isaaclab-rl).

## 2026-08-13

C++ stand-walk-lie and slow trot as recorded in `docs/RESEARCH_INDEX.md`.
