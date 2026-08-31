# Order-102 C-006b Phase-A diagnosis

Date: 2026-09-01. Source under diagnosis: `e422a53305f38101bf584982c40c71fdd5d49d53` (clean before this evidence commit). This is read-only diagnosis of the exact Order-101 artifacts; no rerun was used to discover or wash out the failure.

## Frozen identity and setup

All four completed members (pair-1 baseline/terrain and pair-2 baseline/terrain) report source `7861bf98cd32f454b3da6783a09b5571f4cfe037`, clean tree, controller SHA256 `46033a1ca918ab5e143aa64124b93a2637f4fa7c4060cdcc6c5e508cde363f39`, simulator SHA256 `7f3c29e3c684207beadf3d260e7f347f72147e349ba39012b1cacb3ce493864c`, scene SHA256 `12286418247d0e240ae131b5ae5c60f3a7a481d4754aefe4517476e937aa05b8`, and domains 222/223. The effective argv, scene, analyzer/contract hashes, DDS Base=4000 preload (`5cfc8c514cdf0e99bd7db8837a3f031d93aa9be00a0de16017ebf2fee535bd`), period 0.14, duty 0.44, step 0.50, foot lift 0.20, tau 45, running-trot, wall-clock motion, and 40 s/75 s limits match the frozen Order-101 manifest. Seed/profile/event hashes are empty by the frozen fixed-pair entrypoint. Terrain differs only by sensor-only lidar and domain/CPU assignments; Stage-C execution is false and all consumers/publish/actuation counters are zero.

## Aligned lifecycle and controller evidence

The initial state is the same settled lying state in every member: measured mask 3 (FR+FL), base x about -0.091081 m, z about 0.085426 m, roll about 0.000012 rad, pitch about -0.15024 rad. Pair-2 baseline starts at controller t=0 with simulator state stamp 1.732 s; its terrain mate starts at 1.678 s. Pair-1 starts at 1.682/1.692 s. The stand transition is at controller t=3.000 s (motion stage 1), and gait starts at t=4.300 s (stage 2), with no runtime command or event arrival (`runtime_velocity_command=off`, event type 0). DDS bridge ready is present in all simulator logs before control; there is no missing-ready or start-order error in the artifacts.

The failure member has a distinct state-delivery disturbance before and during the failing gait: state_tick_gap_s reaches 0.030 s at controller t=3.926 s, 0.024 s at t=5.948 s, 0.012 s at t=7.966 s, and 0.040 s at t=13.404 s. Its paired terrain member has maximum state gap 0.008 s; pair-1 baseline maximum is 0.006 s. Controller motion dt remains near 0.002 s, so the 40 ms simulator-state gap is hidden from the controller's wall-clock loop rather than handled as an explicit stop. This is a WSL/DDS wall-clock sensitivity signal, not a planner deadline miss (pair-2 terrain planner max 4358.228 us against 5000 us, zero misses).

At the first large pre-gait gap, pair-2 baseline has already advanced through a different simulator-state schedule; at gait start its first requested acceleration is -0.177994 m/s2, unlike pair-1 baseline and both terrain members at +11.716 m/s2. The first gait contact sequence remains nominal for several cycles, then at controller t=13.388 s pair-2 baseline has measured mask 0 while the terrain mate still has mask 9; at t=13.404 s baseline repeats mask 0 and consumes a 40 ms state gap. The baseline then reaches roll -1.009 rad at t=13.830 s, crosses the hard posture stop, and ends with roll 178.554 deg, safety=1/completion=1. This is the first causal divergence: loss of all measured support and state/phase desynchronization, followed by posture growth; the later hard stop is an effect, not a planner action.

Before the divergence, SRBD/ID-WBC status is valid and residuals are finite; after mask loss, SRBD/ID-WBC continue to report solver fields but no measured support exists. Terrain pair-2 remains safety/controller/quality/dynamics/analysis/completion zero and fixed analyzer PASS. Planned/raw/filtered/fused masks remain separated and no plan is published or consumed. Thus no terrain controller output can be the cause.

## Classification and probe authorization

Classification: **inherited stochastic Phase-1 controller failure, triggered/exposed by wall-clock/state_tick_gap jitter**. This is not a startup-readiness race: DDS ready, stand/gait transitions, and command profile are present and ordered in every member. It is not a C-000..C-005 flag-off regression: execution and all consumers are off, and the same fixed 3 m/s baseline has authoritative failures before C-000 (source `30cfdbae`, max state gap 56 ms; source `2f6935c7`, max 102 ms; source `9433cab9`, max 26 ms), interspersed with passes. Pair-2 terrain passing at the same source further rejects terrain causality. No threshold, analyzer, contract, config, binary, or controller code fix is justified.

Per supervisor review, proceed only with a fresh pre-registered three-pair fixed sample at the exact frozen setup. The original Order-101 sample remains permanent: 1/2 completed PASS, Wilson diagnostic [0.094531, 0.905469]. The new sample is independent and cannot exclude or replace any run because of state gaps or outcome. Any new authoritative failure stops immediately; a 3/3 result can support only the narrow claim that Stage-C sensor/planner plumbing is flag-off non-regressing, while jitter-triggered inherited instability remains a residual risk.

## Rollback

No runtime change was made. The existing rollback is Stage-C flags off / prior verified source. No B1 run, contract/analyzer edit, threshold edit, or startup workaround is permitted by this order.
