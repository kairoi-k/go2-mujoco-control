# B1 traversal v2 schema and clock audit

This is a read-only schema audit of the current simulator/controller sources
and the retained run
`example/cpp/experiments/_runs/b1_lift_step_8bd8eb2_20260907_0001`.
It defines field meaning and clock provenance only. It does not set or tune
acceptance thresholds.

## Artifacts and clocks

| artifact | clock / row event | source and implication |
|---|---|---|
| `data.csv` | `cmd_time_s` is controller `running_time_`; `state_tick_s` is the latest LowState `tick()*1e-3` | `trot_experiment_diagnostics.cpp:620-621,747-749`; `trot_experiment_control.cpp:1301-1302` |
| `data.csv` | one row is written after the controller computes and publishes one LowCmd; it can reuse the latest state snapshot | `trot_experiment_control.cpp:574-600`; `trot_experiment_control.cpp:1140-1214` |
| `contact_ground_truth.csv` | `time_s` is MuJoCo `data->time`; `step_index` increments once per logged physics step | `simulate/src/main.cc:536-545,612-614,714`; `time_s` is therefore simulator physics time |
| ground truth logging | `Log()` follows each `mj_step`; the wall-clock loop may perform multiple physics steps in one iteration | `simulate/src/main.cc:1242-1274,1324-1378` |
| wall telemetry | `*_wall_time_s` is steady-clock time from controller initialization; lidar/high-state stamps and arrival times are separate fields | `trot_experiment.h:237-243`; `trot_experiment_lifecycle.cpp:42-133`; `trot_experiment_diagnostics.cpp:176-202,1051-1087` |

The CSVs are asynchronous streams. Do not join them by row number or require
equal start/end timestamps. Associate truth to controller evidence by its
physical clock after explicitly recording the association error and uncovered
ranges. A valid crossing window must be formed from the stream that owns the
measurement: geometry/contact crossing is truth-time; command/stage coverage is
controller-time.

Observed in the real 2026-09-07 step run: `data.csv` has 16,732 rows over
`cmd_time_s=0.000000..33.462079 s`; truth has 17,656 rows over
`time_s=0.002..35.312 s`. Controller sample gaps have median about 2.002 ms
and maximum about 5.954 ms; truth is 2 ms. `state_tick_s` repeats for 1,436
controller intervals (15,296 unique ticks), so a row is not proof of a new
state sample. Stage-2 plus active-command rows span `cmd_time_s=4.300058..
31.460049 s` in this run.

## Scene and physical crossing coordinates

`unitree_robots/go2/phase2_step_5cm.xml:9-10` defines the floor and
`phase2_step_5cm` box with `pos="0.95 0 0.025"` and
`size="0.25 0.75 0.025"`. Thus the step box occupies x=0.70..1.20 m and
its top is z=0.05 m. In the retained run, truth `base_pos_world_x_m` enters
that x interval at 18.810 s and reaches its rear edge at 20.854 s. The
controller `world_base_x_m` reports the corresponding entries at controller
`cmd_time_s` 17.050 s and 19.094 s; this observed offset is why row-position or equal-clock assumptions
are invalid. These are observed crossing coordinates/times, not a v2 gate.

## Terrain contact fields

The truth header is emitted in `simulate/src/main.cc:274-312`. It contains
aggregate contact fields, `phase2_terrain_foot_contact_mask`,
`phase2_terrain_nonfoot_contact_count/force_N`, and per-leg sensor/contact/site
fields. Exact per-leg audit fields are
`FR,FL,RR,RL_terrain_top_grf_world_z_N` and
`FR,FL,RR,RL_terrain_nontop_contact_force_N` (`main.cc:293-312`); their leg
order is FR bit 0, FL bit 1, RR bit 2, RL bit 3.

`ComputePhase2TerrainContact()` (`simulate/src/main.cc:484-533`) first limits
terrain to geoms whose names start with `phase2_step`, then requires the other
geom to belong to a non-world robot body. It sets the mask only when that
other geom is exactly the registered foot geom for a leg (`main.cc:506-515`).
Therefore `phase2_terrain_foot_contact_mask` means “registered foot geom is in
contact with a phase2 step geom” at this physics row. It is not all terrain
contact, not an aerial-state label, and not proof of support or full-body
clearance. Non-foot count/force covers the same step-geoms with every other
robot geom (`main.cc:526-532`), so it is the direct non-foot terrain evidence
but does not identify which non-foot geom.

The aggregate `<LEG>_foot_contact_grf_world_*` fields are accumulated by
`ComputeContactGrf()` (`main.cc:367-443`) and are not surface-classified.
Top/riser attribution must use the exact audit fields or remain unknown.
`AuditTerrainContact()` (`simulate/src/terrain_contact_audit.h:11-24`) uses the
terrain geom's world rotation and contact-frame normal; top is only a box whose
terrain-to-robot normal has dot product at least 0.99 with the box local +Z.
The top channel accumulates upward world-z force; every other foot/terrain
contact accumulates the non-top force norm. This is logger classification, not
an acceptance threshold. Non-foot and foot-riser observations remain visible;
a physical traversal claim cannot imply collision-free traversal.

Truth rows are written after `mj_step` (`main.cc:1267-1269,1333-1336,
1375-1378`), so a positive contact value is a physics-step observation. In the
retained step run, top-force observations occur for FR 17.646..20.594 s, FL
17.818..20.666 s, RR 20.470..21.090 s, and RL 20.082..21.150 s. The aggregate
non-foot count is positive at 18.848..19.192 s. These facts demonstrate why
the report must separately preserve top support, foot-riser interaction, and
non-foot collision evidence.

## Estimated state, lidar, and plan provenance

`SnapshotState()` copies the latest LowState/HighState and records arrival and
consumption counters (`trot_experiment_control.cpp:1140-1214`).
`telemetry_lowstate_consumed_new_tick` and its delta/repeated/jumped/reordered
fields expose whether the copied LowState is new; `has_state` alone does not.
`state_tick_s` and `terrain_model_com_state_stamp_s` are both derived from a
LowState tick. WBC computes model COM from that state/high-state snapshot and
sets the model COM stamp at `trot_experiment_wbc.cpp:115-133`.

Terrain snapshots use the state stamp while the worker copies the latest lidar
height map (`trot_experiment_control.cpp:201-268`). The worker builds a model
with `TerrainSource::kLidar` and the same state stamp
(`trot_experiment_control.cpp:327-338`). Thus `terrain_map_source=lidar`, map
epoch/age, model-COM validity/stamp, and plan state stamps are provenance
fields; they do not make a row synchronous with a truth row. Retained step
data has `terrain_model_com_valid=1` on every row, `terrain_map_source=lidar`
on 16,730 of 16,732 rows, and repeated LowState ticks as noted above.

## Gait and command semantics

`--gait-pattern running-trot` selects the configuration enum and is printed in
the CLI summary (`locomotion_kernel.h:29-62`; `trot_cli.cpp:87-97,474-489`).
It is a configuration/manifest fact. The CSV column
`velocity_command_gait_regime` is a runtime scheduler state written from
`runtime_gait_regime_` (`trot_experiment_diagnostics.cpp:40,746-787`). Its
source sets `inactive` before runtime command, `stop-brake` during braking, or
the velocity scheduler's `schedule.regime`
(`trot_experiment_gait.cpp:116-202`). The scheduler currently names its normal
runtime regime `continuous-trot` (`velocity_command.h:178-185,227-279`).
Consequently an analyzer must not require the literal runtime CSV value
`running-trot` as proof of the configured gait, nor infer the configured gait
from `continuous-trot`; bind configuration to manifest/argv and assess actual
motion from numeric speed and contact topology.

The retained run confirms this distinction: its manifest/controller log says
`gait-pattern running-trot`, while the runtime column contains only
`continuous-trot`, `inactive`, and `stop-brake`.

## Coverage interpretation for v2 implementation

Required evidence should be fail-closed when a required column, provenance
stamp, or exact top/non-top classifier is absent. Coverage means the matched
controller/truth time window contains the intended dynamic crossing and a
post-crossing body/feet exit interval; it does not mean the two files share
the same lifecycle. A report should expose missing spans, matching error,
truth sample gaps, state-tick reuse, and field availability separately from
physical outcomes. The old analyzer and its report remain parallel artifacts;
their PASS/FAIL cannot substitute for this independent evidence mapping.

Main-agent verification: at the first controller base_x >= 0.70 row,
cmd_time_s=17.050093396, state_tick_s=18.812, model COM stamp=18.812,
and lidar stamp=18.796. Truth first entry is 18.810. Thus state_tick_s and
truth time share physics-time semantics within publication/sampling latency;
the 1.76 s offset above concerns cmd_time_s, not a new physics clock offset.
