# WBC/MPC 1 m/s speed acceptance

This is an experimental result on branch `speed/1mps-2026-08-21`; it is not a
claim about `main` until the branch is reviewed and promoted.

The earlier nominal `0.5 m/s` probe is rejected.  Its body was crouched with
tucked legs and the feet dragged near the floor, so its speed was not a valid
normal-gait result.  A valid run must keep the normal standing body height,
show swing-foot clearance on every leg, and must not spend most of cruise with
all four feet dragging.

The current candidate is the standard 18-DoF ID-WBC + SRBD-MPC plant:

```text
period=0.18 s, duty=0.45, step-length=0.216 m, foot-lift=0.100 m
WBC velocity gain=8, tau-limit=35 Nm, Raibert gain=0.015, max adjustment=0.060 m
```

At the requested cycle limit the controller first performs a 0.80 s gait-level
pre-stop brake: the same WBC/MPC plant slews the step reference down to 45% of
the cruise step, then enters the four-contact WBC stop hold.  The WBC velocity
wrench follows the kernel's current nominal velocity, and stale preview
acceleration/bounce terms are disabled during the stop hold.

Acceptance command (from the repository root):

```bash
bash example/cpp/scripts/run_trot.sh 100 speed_accept_v120_duty45_gain8_repN \
  --headless --wbc-full --wbc-velocity-gain 8 --tau-limit 35 \
  --period 0.18 --duty 0.45 --step-length 0.216 --foot-lift 0.100 \
  --kernel raibert-trot --raibert-velocity-gain 0.015 \
  --raibert-max-adjustment 0.060 --max-cycles 40 \
  --controller-duration 25 --domain-id 213
python3 example/cpp/tools/analysis/analyze_speed_acceptance.py \
  example/cpp/experiments/_runs/speed_accept_v120_duty45_gain8_repN
```

The analyzer trims the first 1.0 s and last 1.5 s of the WBC locomotion
stage, then checks all status codes, at least 40 healthy cycles, true MuJoCo
base speed median >=1.0 m/s, cruise p05 >=0.85 m/s, base height 0.33--0.40 m,
per-leg swing clearance, no continuous all-feet drag, and a settled stop.

Three independent release-binary runs passed on 2026-08-21.  The release
controller SHA-256 is
`71a45e2ac711d07ba96f12976ec3eec7221c7476accdf9f0215306ef6530637b`.

| run | true cruise p05 / median / p95 (m/s) | base-z median (m) | result |
|---|---:|---:|---|
| `speed_accept_release_rep1_2026-08-21` | 0.875 / 1.010 / 1.129 | 0.3642 | PASS |
| `speed_accept_release_rep2_2026-08-21` | 0.986 / 1.105 / 1.198 | 0.3647 | PASS |
| `speed_accept_release_rep3_2026-08-21` | 0.896 / 1.043 / 1.162 | 0.3651 | PASS |

All three runs recorded `controller_status=0`, `safety_status=0`,
`quality_status=0`, and `completion_status=0`.  Cruise foot-height p95 was
0.060--0.070 m per leg; the all-feet-low fraction was 0.285--0.339, which is
incompatible with the earlier tucked/dragging failure mode.  The p05 spread
shows remaining speed variability; this is a valid 1 m/s median cruise, not a
claim of perfectly constant 1.000 m/s instantaneous velocity.
