# Reactive acceptance — verified v4 results

The eight runs below were generated after the emergency-stop and obstacle
corrections. `PASS` means all metadata status codes are zero, the event and
recovery windows are complete, and the event-specific response gate passes.

| case | source | yaw Δ (rad) | lateral Δy (m) | vx drop (m/s) | max jump (m/s) | gate |
|---|---|---:|---:|---:|---:|:---:|
| baseline | nominal | — | — | — | — | PASS |
| emergency_stop | scripted reference | +0.002 | +0.002 | 0.198 | 0.009 | PASS |
| turn_left | scripted reference | +0.366 | +0.059 | 0.162 | 0.015 | PASS |
| turn_right | scripted reference | −0.338 | −0.045 | 0.099 | 0.025 | PASS |
| obstacle_right | visible proxy + scripted reference | −0.191 | −0.069 | 0.173 | 0.038 | PASS |
| low_friction | floor μ=0.05 for 1 s | +0.003 | +0.002 | 0.041 | 0.018 | PASS |
| impact | push Δv≈0.80 m/s | −0.083 | −0.002 | 0.447 | 0.801 | PASS |
| slip | scripted protective slowdown | −0.003 | +0.009 | 0.158 | 0.022 | PASS |

## Quality checks

- Each CSV has about 11.5k rows and about 23.1 s of controller time.
- Each run has zero controller, safety, quality, CSV, ground-truth, dynamics,
  and completion status errors.
- Scheduled cases have exactly `none → event → none`; external cases have one
  synchronized simulator event and a complete post-event window.
- The emergency gate checks the last 0.75 s of the active window: target vx is
  zero, stance hold is active, and measured |vx| max is 0.015 m/s.
- The turn gates check direction and at least 0.12 rad accumulated yaw. The
  obstacle gate additionally requires the signed lateral shift (−0.069 m).
- The videos were reviewed with 1 fps contact sheets; the obstacle entity is
  visible in its active clip and the event bands align with the trace plots.

## Interpretation

The result demonstrates that one WBC/MPC plant can accept changed velocity,
yaw, and protective references without hard-coded action concatenation. It is
not yet an automatic perception result: event detection is still scripted.
Low friction is expected to be visually subtle; use the upper trace (cyan
measured vx versus yellow target vx), the red μ-change window, and the
simulator.log active/restored lines as the evidence.
