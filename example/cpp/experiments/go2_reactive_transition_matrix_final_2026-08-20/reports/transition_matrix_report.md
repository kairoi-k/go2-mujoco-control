# Reactive transition matrix — validation report

Generated: 2026-08-20T22:47:56+08:00

## Overall assessment

**Ready to share** — all analyzed directed transitions passed the same-controller acceptance gates.

Coverage: 49/49 analyzed pairs passed; protocol matrix contains 49 pairs.
Unique controller configuration fingerprints: 1 (pair-specific tuning detected: False).

## Question tested

Can one bounded continuous-reference transition layer and one WBC/MPC plant handle directed event changes without hand-written pairwise action stitching? Each nonterminal run contains `none -> A -> B -> none`; an emergency target ends in the absorbing WBC stance hold. Only the two-line event script changes between runs.

## Protocol

- Events: emergency_stop, obstacle_left, obstacle_right, turn_left, turn_right, slip, low_friction, impact
- Sources: obstacle_left, obstacle_right, turn_left, turn_right, slip, low_friction, impact
- Event windows: start=1.5 s, duration=2.0 s, adjacent A→B
- Controller duration: 8.0 s; post window: 1.5 s
- Event source: scheduled scripts only (automatic sensor events enabled: False)
- Infrastructure policy: up to 2 attempts with 1.0 s cooldown and alternate DDS domains
- Safety policy: `emergency_stop` is absorbing; incoming transitions are tested, outgoing transitions are intentionally not required.

## Gates

1. CSV completeness and monotonic time; both event windows contain data.
2. Observed event sequence exactly matches `none -> A -> B -> none`.
3. WBC/MPC remains in gait stage 2 during both events; no controller reset.
4. Reference rates stay within shared limits; target jumps are not mistaken for reference discontinuities.
5. Solver/status gates, Ground Truth contact support, velocity jumps, roll and pitch pass; up to 5 consecutive 2 ms contact-unloading samples are tolerated.
6. Event-specific target signs and response checks pass; emergency includes the terminal WBC stance-hold marker.

## Pair results

| pair | pass | ref-rate | posture | dynamic | sequence |
|---|---:|---:|---:|---:|---:|
| obstacle_left → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → turn_left | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → turn_right | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → slip | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → low_friction | PASS | PASS | PASS | PASS | PASS |
| obstacle_left → impact | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → turn_left | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → turn_right | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → slip | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → low_friction | PASS | PASS | PASS | PASS | PASS |
| obstacle_right → impact | PASS | PASS | PASS | PASS | PASS |
| turn_left → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| turn_left → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| turn_left → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| turn_left → turn_right | PASS | PASS | PASS | PASS | PASS |
| turn_left → slip | PASS | PASS | PASS | PASS | PASS |
| turn_left → low_friction | PASS | PASS | PASS | PASS | PASS |
| turn_left → impact | PASS | PASS | PASS | PASS | PASS |
| turn_right → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| turn_right → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| turn_right → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| turn_right → turn_left | PASS | PASS | PASS | PASS | PASS |
| turn_right → slip | PASS | PASS | PASS | PASS | PASS |
| turn_right → low_friction | PASS | PASS | PASS | PASS | PASS |
| turn_right → impact | PASS | PASS | PASS | PASS | PASS |
| slip → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| slip → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| slip → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| slip → turn_left | PASS | PASS | PASS | PASS | PASS |
| slip → turn_right | PASS | PASS | PASS | PASS | PASS |
| slip → low_friction | PASS | PASS | PASS | PASS | PASS |
| slip → impact | PASS | PASS | PASS | PASS | PASS |
| low_friction → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| low_friction → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| low_friction → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| low_friction → turn_left | PASS | PASS | PASS | PASS | PASS |
| low_friction → turn_right | PASS | PASS | PASS | PASS | PASS |
| low_friction → slip | PASS | PASS | PASS | PASS | PASS |
| low_friction → impact | PASS | PASS | PASS | PASS | PASS |
| impact → emergency_stop | PASS | PASS | PASS | PASS | PASS |
| impact → obstacle_left | PASS | PASS | PASS | PASS | PASS |
| impact → obstacle_right | PASS | PASS | PASS | PASS | PASS |
| impact → turn_left | PASS | PASS | PASS | PASS | PASS |
| impact → turn_right | PASS | PASS | PASS | PASS | PASS |
| impact → slip | PASS | PASS | PASS | PASS | PASS |
| impact → low_friction | PASS | PASS | PASS | PASS | PASS |

## Failed runs

None.

## Scope and caveats

This matrix demonstrates the shared reference/transition/WBC-MPC path under scripted events. It does not prove autonomous perception, local obstacle planning, or every possible physical disturbance. The physical obstacle acceptance run remains a separate scene-level test.

Raw CSV, simulator/controller logs, per-pair event scripts, and manifests are retained under this experiment directory for reproduction. Duplicate CSV timestamps are preserved; rate gates use positive time intervals only.
