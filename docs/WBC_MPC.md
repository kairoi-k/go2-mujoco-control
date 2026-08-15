# WBC / MPC line

Goal: replace the walk-phase incremental feedforward with a centroidal wrench that owns gravity, plus a foothold preview that actually selects the next touchdown. `real_trot_go2` stays the 0.15 m/s baseline. `--wbc-full` is the new path.

## What `--wbc-primary` does today

Stance legs get an extra `M a` torque; gravity/bias `h` is left to the position servo; contact tasks often drop the moment when a foot is in the air. That is why it is documented as incremental components, not a whole-body controller.

## What `--wbc-full` changes

- wrench is `W = M a + h` (`centroidal_wbc.h`);
- all six wrench axes stay in the task (`{1,1,1,1,1,1}`);
- contact forces come from the lexicographic slack allocator (force first, moment second);
- stance PD drops to `kWbcFullStanceKp/Kd` (25 / 2.0) so gravity is not double-counted by the position servo;
- swing legs remain IK + position control;
- the Raibert kernel takes the first foothold from an N-step preview (`--preview-horizon`, default 4). Remaining steps stay greedy Raibert. Velocity is propagated with a first-order capture model (rearward placement raises forward speed). This is not a receding-horizon QP.
- remaining terminal velocity error over that horizon sets the centroidal `a_x` task, so the wrench looks ahead instead of only reacting to the current speed error.

## How to run

Same as the sequenced task, with the new flag:

```bash
bash example/cpp/scripts/go2sim walk --wbc-full
```

Or trot-only:

```bash
./example/cpp/build/real_trot_go2 lo 20 /tmp/wbc_full.csv --wbc-full --kernel raibert-trot
```

`--preview-horizon 0` after `--wbc-full` keeps the centroidal wrench and turns the foothold preview off.

## Acceptance

Keep the current 0.15 m/s gated trot as the comparison. This path is ahead of the baseline only if it walks faster without failing the same cycle-quality / torque gates. That measurement is not claimed yet.

## Next

A QP that owns the wrench (OSQP/qpOASES or equivalent) instead of the lexicographic slack allocator, then a sim comparison against the 0.15 m/s gates.
