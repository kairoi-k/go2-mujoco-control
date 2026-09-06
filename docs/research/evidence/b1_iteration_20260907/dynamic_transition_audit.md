# B1 stance-ablation dynamic running quality audit

This report is a read-only audit of the retained raw run
`b1_stance_ablation_step_764a21c_20260907_0001`. It is diagnostic evidence for
B1 empirical traversal v2 review; it does not change an acceptance threshold,
replace the frozen analyzer, or claim this run is a candidate.

## Reproducible method

The companion script is `audit_stance_ablation_dynamic.py`. Run it from the
Linux checkout with:

```sh
python3 docs/research/evidence/b1_iteration_20260907/audit_stance_ablation_dynamic.py \
  example/cpp/experiments/_runs/b1_stance_ablation_step_764a21c_20260907_0001 \
  --json-out /tmp/b1_stance_dynamic_new_output.json
```

The script reads only `data.csv` and `contact_ground_truth.csv`. It associates
controller rows to ground truth by exact `state_tick_s == time_s`; it does not
join by row number or interpolate. Actual contact topology is the descriptive
existing 5 N convention applied to each leg's aggregate
`<LEG>_foot_contact_grf_world_z_N`. Aerial is evaluated from the norm of the
all-robot `total_contact_grf_world_{x,y,z}_N`, so no-foot support is not called
aerial when another robot geom is contacting the environment. Exact terrain top
and non-top force fields are reported separately. The 5 N and 0.10 m/s bands in
the script are measurement conventions for this audit, not proposed v2 gates.

## Clock and coverage integrity

| stream | rows | time range | sampling evidence |
|---|---:|---:|---|
| `data.csv` | 16,731 | `cmd_time_s=0.000..33.460 s` | 12,616 unique state ticks; 4,115 rows reuse the latest tick |
| `contact_ground_truth.csv` | 17,678 | `time_s=0.002..35.356 s` | median `dt=0.002 s` |

All 12,616 unique controller state ticks are present as exact ground-truth
times; maximum association error is `0 s`, with no missing tick. The streams
have different lifecycles, so the audit uses the truth-owned geometry window
and matches controller evidence inside it.

## What the source fields mean

`contact_ground_truth.csv` is logged after each MuJoCo step
(`simulate/src/main.cc:536-545,612-614,714`). The exact per-leg terrain fields
are computed in `main.cc:484-533`; top classification is the simulator's
terrain-box normal test (terrain-to-robot normal dotted with local +Z at least
0.99, `simulate/src/terrain_contact_audit.h:11-24`). Every other foot/terrain
contact contributes to that leg's non-top force. The aggregate foot GRF fields
are not surface-classified (`main.cc:367-443`).

`terrain_execution_planned_contact_mask` is the terrain execution schedule
(`trot_experiment_diagnostics.cpp:729-740,875-891`).
`wbc_scheduled_contact_mask` is the merged WBC/QP contact state, not a pure
unmodified gait request (`trot_experiment_wbc.cpp:179-212`), so the tables keep
both fields distinct. The configured CLI/manifest says `running-trot`, while
the runtime CSV scheduler says `continuous-trot`; the latter is the scheduler's
normal runtime label (`locomotion_kernel.h:29-62`, `velocity_command.h:178-185`,
`trot_experiment_gait.cpp:116-202`). Numerical period, duty, speed and measured
contact topology are therefore required to establish the motion actually
executed.

## Crossing versus stable running

The step box occupies x=`0.70..1.20 m` (`phase2_step_5cm.xml:9-10`). Ground
truth gives:

| window | truth time | controller cycles | period / duty in window | GT forward speed p05 / p50 / p95 | stall band |
|---|---:|---:|---:|---:|---:|
| terrain interaction (any exact top/non-top force) | `17.454..19.876 s` | 11..21 | not yet fixed; transition `.36..14 s` / `.66..44` | `.067/.388/.877 m/s` | 13.3% |
| base enters box to all feet beyond rear edge | `18.044..19.928 s` | 13..21 | `.30..14 s` / `.60..44` | `.047/.452/.982 m/s` | 17.1% |
| modal stable scheduler pair | `19.856..31.054 s` | 21..100 | `.14 s` / `.44` | `.699/.932/1.179 m/s` | 0% |

The modal pair is selected from the observed numeric scheduler values, not from
an acceptance threshold. Cycles 11..20 contain the obstacle interaction while
period and duty are still shrinking from `.36/.66` to `.16/.46`; cycle 21 first
has the `.14/.44` pair and overlaps only the last `0.072 s` of the all-feet
rear-edge crossing. Terrain contact interaction overlaps the stable pair for
only `0.020 s`. Thus this run crossed the step mainly during scheduler
transition, not during a complete stable `.14 s/.44` running-trot cycle.

The complete per-cycle plan/actual masks, speed, aerial fraction, diagonal
fraction, and non-top force are in the JSON emitted by the script. The obstacle
cycles are summarized below. Diagonal masks are 6=`FL+RR` and 9=`FR+RL`.

| cycle | time (s) | period/duty | GT speed p50 | diag 6 / 9 | diag switches / samples | aerial | single | terrain plan = actual | max non-top N |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 11 | 17.354..17.692 | .36/.66 | .304 | .276/.306 | 1/99 | .000 | .000 | .641 | 0 |
| 12 | 17.696..18.014 | .32/.62 | .299 | .231/.244 | 2/76 | .000 | .094 | .405 | 0 |
| 13 | 18.014..18.312 | .30/.60 | .079 | .347/.293 | 1/96 | .000 | .000 | .711 | 0 |
| 14 | 18.314..18.592 | .28/.58 | .107 | .350/.371 | 1/101 | .000 | .000 | .750 | 0 |
| 15 | 18.594..18.854 | .26/.56 | .431 | .237/.397 | 1/83 | .000 | .115 | .598 | 116.4 (RL) |
| 16 | 18.854..19.094 | .24/.54 | .563 | .157/.182 | 1/41 | .000 | .157 | .314 | 315.8 (RR) |
| 17 | 19.094..19.312 | .22/.52 | .503 | .082/.027 | 1/12 | .109 | .291 | .095 | 0 |
| 18 | 19.314..19.512 | .20/.50 | .451 | .030/.200 | 1/23 | .020 | .390 | .043 | 0 |
| 19 | 19.514..19.692 | .18/.48 | .572 | .311/.344 | 1/59 | .000 | .122 | .554 | 0 |
| 20 | 19.694..19.852 | .16/.46 | .882 | .413/.313 | 1/58 | .000 | .038 | .569 | 0 |
| 21 | 19.856..19.992 | .14/.44 | .973 | .000/.116 | 0/8 | .087 | .638 | .132 | 0 |

The longest all-robot-contact-free interval inside the exact terrain-interaction
window is `8 ms` at `19.150..19.158 s`, late in the interaction but before the
all-feet rear edge at `19.928 s`. It is not the only aerial behavior: over the
stable tail, the longest all-robot-contact-free aerial interval is `42 ms`, with per-leg
longest swing durations `FL 108 ms`, `FR 110 ms`, `RL 260 ms`, `RR 240 ms`.

## Stable-tail topology

The stable tail contains 80 complete modal-pair cycles (`19.856..31.054 s`),
5,600 matched controller rows and 5,600 truth rows. Its forward displacement is
`10.403 m`; GT speed is `.699/.932/1.179 m/s` at p05/p50/p95, with no sample in
the descriptive `|v_x| <= .10 m/s` stall band.

Actual stable contact fractions by leg are `FL .521`, `FR .517`, `RL .417`,
`RR .412`. Exact diagonal masks occupy `27.7%` (mask 6) and `29.4%` (mask 9), with 147 orientation switches among 3,200 adjacent diagonal-labelled samples (4.59%);
all-robot-contact-free is `4.14%`, single-contact is `22.3%`, and terrain-plan versus
actual exact-mask agreement is `44.6%`. Across individual stable cycles, the
median diagonal fractions are only `.300/.321` and the 5th percentiles are
`.097/.111`; the maximum cycle aerial fraction is `.324` and maximum
single-contact fraction is `.757`. The stable tail is moving at the target
speed, but its realized topology is variable and should not be described as a
clean stable trot without a separate topology policy.

## Riser and non-foot evidence

During the exact terrain-interaction window, every leg has a top-support
witness. Top upward-force maxima and integrated force are:

| leg | top max N | top impulse Ns | longest top support |
|---|---:|---:|---:|
| FR | 465.5 | 69.96 | 206 ms |
| FL | 397.9 | 67.64 | 226 ms |
| RR | 428.1 | 31.30 | 112 ms |
| RL | 303.9 | 27.70 | 120 ms |

The exact foot non-top/riser fields are positive for all four legs. Maxima and
impulses (integrated with actual adjacent ground-truth `dt`) are:

| leg | max non-top N | impulse Ns | positive physics samples |
|---|---:|---:|---:|
| RR | 315.8 | 4.374 | 16 |
| RL | 116.4 | 2.327 | 17 |
| FR | 78.6 | 1.400 | 10 |
| FL | 47.8 | .248 | 4 |
| **sum** | — | **8.349** | 47 |

Ground truth reports zero `phase2_terrain_nonfoot_contact_count`, zero
non-foot force, and zero reactive-obstacle contacts over the whole run. Hence
this raw run has clear evidence of foot/step non-top interaction, while the
non-foot collision channel is clean. The physical-traversal and collision-free
claims must remain separate: successful body/feet exit can coexist with foot
riser contact and cannot establish collision-free traversal.

## Audit conclusion and recommendation

The raw run is a useful closed-loop physical probe with exact clock coverage,
per-leg top support, forward stable running after the obstacle, and no non-foot
collision. It is not evidence of a stable running-trot step crossing: the
obstacle is crossed during the period/duty transition, and the later stable
segment has materially variable actual topology. The observed late 8 ms
all-robot-contact-free interval should be reported as an interaction-window
event; the stable tail also contains a 42 ms interval.

For the research objective, add a predeclared stable-speed approach/hold and
require the body/feet crossing and complete exit to lie inside that stable
window, while retaining the existing step run as a transition baseline. Keep
foot-riser non-top force visible in the physical-traversal report and gate
collision-free independently on the non-foot and non-top evidence. Do not tune
limits from this run.

## Source and artifact index

- Raw run: `example/cpp/experiments/_runs/b1_stance_ablation_step_764a21c_20260907_0001`
- Reproducible script: `acceptance_v2_draft_0001/audit_stance_ablation_dynamic.py`
- Machine-readable output used for this report: `/tmp/b1_stance_dynamic_new_output.json`
- Source semantics: `simulate/src/main.cc:274-312,367-443,484-533,536-545,612-614,714`; `simulate/src/terrain_contact_audit.h:11-24`; `example/cpp/trot/trot_experiment_diagnostics.cpp:729-740,875-891`; `example/cpp/trot/trot_experiment_wbc.cpp:179-212`; `example/cpp/trot/trot_experiment_gait.cpp:116-202`; `example/cpp/trot/velocity_command.h:178-185`; `example/cpp/gait/locomotion_kernel.h:29-62`.


Root review: exact timestamp matching does not remove the logger's one-step
derived-kinematics offset after mj_step. This is a descriptive historical-run
audit with the original box bounds, not a general V3 analyzer. Root packaged
script writes strict JSON (nonfinite diagnostics become null) and refuses to
overwrite an output.
