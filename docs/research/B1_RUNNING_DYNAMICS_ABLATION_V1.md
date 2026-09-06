# B1 running dynamics ablation V1 — preregistered development experiments

Date: 2026-09-07. Preregistered after clean d355a2f world-swing development
probe, before any changed-period or lift-floor MuJoCo probe. This is not a
revision of the frozen Phase-2 or B1 V3 acceptance contract. No holdout is used.

## Research question and existing evidence

The d355a2f V3 probe completely exited the 5 cm platform, supported every foot on
its top and completed the requested profile without nonfoot collision or safety
termination. Nevertheless, foot-riser forces occurred, only 1/8 contained
interaction cycles had both exact diagonals and total-GRF aerial intervals,
and interaction speed median was 0.75621 m/s. It is NOT_CERTIFIED by V3.
At the preceding stable 1 m/s approach, the running scheduler requested period
0.14 s, duty 0.44 and effective lift 0.051851852 m. The lift is produced by
0.2*smoothstep(applied_speed/3), not by the normally disabled Raibert
speed-adaptive 0.058 m cap.

A 0.28 s period at unchanged duty doubles swing time from 78.4 to 156.8 ms.
For unchanged lift it halves ideal vertical swing velocity and quarters ideal
vertical acceleration; actual actuator/contact response remains an experiment.
The existing same-speed step-length equation must use the changed period, so
step length doubles too. This is a globally consistent gait change, not a
per-leg contact retiming or a stop/creep terrain policy.

## Predeclared development configurations

All use the existing V3 32 s velocity profile, 1 m/s requested traversal speed,
running-trot topology, duty 0.44, existing velocity shaper, nominal COM mode,
seed 11 and clean exact-SHA source. Low-speed qualification/stance hold remain
unchanged. Old defaults and old tests remain identifiable and unchanged.

A: historical global high-speed period 0.14 s and lift floor 0.
B: high-speed period 0.28 s and lift floor 0; isolates period and its required
same-speed stride scaling.
C: period 0.28 s and lift floor 0.10 m; tests additional clearance after B.
D: period 0.14 s and lift floor 0.10 m; optional interaction/control contrast if
B/C do not identify a physically plausible route.

For each new configuration first run phase2_flat.xml with the same V3 profile,
then b1_v3_running_step_5cm.xml if the flat physics/lifecycle is viable. Use
terrain-sensor-only for these dynamics-capacity probes, explicitly excluding a
claim that the terrain planner's future horizon was fixed. Sensor data still
flows, but changing period does not secretly authorize terrain MPC actuation.
Promising configurations require identical-source repeats and then a separately
preregistered validation campaign. A failed flat probe stops that configuration
for diagnosis rather than proceeding blindly to the obstacle.

## Evidence and decision rules

Preserve raw GT, controller/simulator logs, manifests, complete profile and
braking tail, requested/effective schedule, real command/source/binary hashes,
CPU placement and residual/latency evidence. Re-run old V3 unchanged and report
its fixed-period rejection as expected for B/C, without relabeling it a PASS.
Separately report actual contained cycle boundaries, exact GRF-supported
9/6 diagonals (each at least 10 ms), total-GRF aerial duration (at least 4 ms),
step-riser and nonfoot forces, per-foot top support, whole-body/feet exit,
posture/height, full profile, speed and runtime integrity. Planned masks are
not evidence of aerial or measured diagonal support.

The 0.5 m platform and longer period can geometrically allow fewer than V3's
four contained interaction cycles. This does not justify hiding bad cycles or
changing old results. A future V4 acceptance design must require directly
observed running topology over sufficient pre/interaction/post windows and
predeclare its finite-window rule before validation runs. The present probes
are diagnostics and cannot by themselves establish a B1 candidate.

## Known limits

TerrainPlanner ordinary actuation defaults to 8 knots at 0.020 s and validity
0.15 s; the 24-knot/0.46 s option is a distinct consistency-shadow route.
MPC currently chooses dt from period/8 clamped to [0.020,0.050], so increasing
period without a separate horizon/contact-event repair would make terrain
future lookup inconsistent. Current ID-WBC uses actual measured foot geometry;
its present lever arms must not be replaced with unreached planned footholds.
The world-swing checker uses fixed-body sampled geometry, not a force or joint
tracking certificate. These limitations are recorded rather than masked by a
geometric 15 mm diagnostic or by analyzer status.
