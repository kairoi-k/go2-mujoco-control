# B1 autonomous iteration, 2026-09-07

Status: development, no candidate. Prior failures remain in
`../b1_research_audit_20260907/`. Old frozen acceptance and holdouts are unchanged.
The current mandate authorizes architecture and separately versioned acceptance
changes while preserving the real dynamic running-trot target.

## First controlled hypothesis

Repair effective swing-lift accounting using the actual kernel value. This is
not a claim that the legacy checked and executed trajectories coincide: endpoint
blending, peak phase, plan expiry and stance anchors still require investigation.
Predeclare a flat development regression before the centered 5 cm canary with
the same velocity profile, gait parameters and initial conditions as prior
clean-source probes. Use `example/cpp/scripts/run_b1_research_probe.sh` with an
explicit clean exact SHA, unique run name, scene and duration. Domain 231 avoids
the observed Windows UDP reservation blocking domain 220; this changes transport
configuration and is recorded, not a control-policy change. No holdout tuning.

## Independent physical evidence

The simulator now records per-leg terrain-top world vertical force and non-top
terrain-contact force norm, resolved per MuJoCo contact. Classification uses the
normal oriented from terrain to robot and the box's world local +Z axis (dot
>= 0.99). Non-box and other normals are conservatively non-top. This applies to
the box-step test family; it is not generic arbitrary-mesh terrain certification.
Top vertical force includes transformed friction components; non-top force is
the full contact-force norm. The fields never enter controller observations.
Existing aggregate foot forces and nonfoot-collision fields remain unchanged.

The native simulator test creates real penetrating sphere/box contacts on top
and at the riser, checks independent generalized constraint vertical force and
reversed contact-order algebra plus unsupported geometry rejection. Eight checks now pass, including the full-robot rear bound. Full integration CSV alignment
and candidate judgments remain to be checked on new real runs.

## Lift correction validation

The kernel result carries the current tick's effective lift after slew and
adaptation; the runtime terrain supplement uses this value. Non-runtime modes
retain their prior convention. Missing/nonfinite lift stops command generation.
The CSV independently records requested and effective lift. This repairs scalar
accounting only, not the complete checked/executed swing-path mismatch.

Full native build and 36/36 CTest pass. Release test targets now retain asserts.
Two historical tests depended on assertions previously disabled by NDEBUG:
`test_velocity_command` and `test_wbc_runtime_gate`. An independent assertion-
enabled replay passed the WBC gate test and exposed a velocity test fixture
that expected a one-second qualification after only 0.502 s. The fixture now
checks 499 prequalification ticks (0.998 s), then the qualified boundary and
continuation. Production scheduler behavior is unchanged. Original failure
and source hashes remain under `_runs/b1_assert_audit_20260907_0001/`.
The first formal build also caught a missing Eigen dependency in the new test
target; corrected before the successful full build. Worker standalone syntax
and test results were not treated as final build acceptance.

## Clean-SHA lift feedback and next hypothesis

At runtime 8bd8eb2b55b40579871f1472fe686b7ca6d49010, the centered step
probe crossed: max base X=11.6001797446 m, normal controller/completion status.
Per-leg longest exact top-force support spans were FR 0.210 s, FL 0.188 s,
RR 0.072 s (RL is in the attached result). Nonfoot collisions remain, beginning
at truth t=18.848 s, base X=0.706609 m. This is a useful breakthrough, not a
candidate. The full-body rear-bound field was not yet logged, so old-log exit
proxies remain uncertified. Original baseline and sensor-only step failures
remain NOT_CERTIFIED under the independent bookkeeping analyzer.

Flat actuation completed but had pitch P95 4.527 deg, max 12.648 deg and ID-WBC
success fraction 0.999440. Its same-SHA flat sensor-only control had pitch P95
2.648 deg, max 4.240 deg and ID-WBC success 1.0. Both are short development
probes, not a complete B0 suite. The sensor-only short run's frozen stop-tail gate
failed; no full-profile acceptance is inferred.

Next: preserve committed target application/completion independently of current
plan expiry and preserve world height through stance. New-target preparation
still requires a usable plan; the next liftoff clears the completed target before
replacement. Missing new targets still invoke an uncertified legacy fallback.
This patch does not repair future MPC prefix coverage. Applied-mask telemetry
makes actual target application distinguishable from merely retaining an array.
An independent pitch-rotation test rejects copying only target_body.z while
keeping another point's body X/Y: the patch instead preserves the currently
commanded world X/Y and applies the desired world Z using the complete pose.

The new physical analyzer has 10/10 bookkeeping tests, and the native controller
suite has 37/37 tests. The rejected analyzer draft and corrected schema/clock
audit are indexed beside this report. Original frozen reports are preserved.

## Stance-hold authority correction

fa84f5185cb52de8fa2ef62d28cf3809998be23a failed its flat development
probe at gait_time=9.12001 with IK failure; no step canary was run. During the
initial zero-command Phase-1 stance hold, cyclic phase had incorrectly caused
terrain to prepare/apply targets for FL/RR (applied mask 6). Those latched world
heights were -5.329 mm on a flat floor, while later FR/RL targets were +26.818 mm.
The held old targets persisted and the last active base height reached 0.4103 m.
This is direct evidence of an execution-authority mismatch; the cause of the
initial map/world-height discrepancy is being audited separately.

The next patch derives terrain swing status from the same canonical Phase-1
stance-hold flag used by MPC/WBC. Hold means all four legs are in stance, so an
otherwise usable cyclic plan cannot prepare fictitious swing targets. It does
not add a local contact authority or alter the Phase-1 velocity command. The
three relevant native tests pass. Failure logs now also include commanded foot
positions, phase, duty, hold state and applied mask for the first IK failure.

## Stance-height ablation

Clean c7f20b2775ff19459871628106b3c45bd38aa3dc fixes initial canonical hold
(applied mask zero), but flat still fails IK at gait_time 9.39195. See
`hold_flat_failure.json`. Remove only active direct world-Z stance application
next; retain committed swing expiry semantics and coordinate utility tests.
The suspected loss of body-height feedback remains a hypothesis. Focused
velocity-command, commitment-lifecycle and WBC gate tests pass 3/3.

## Clean 764a21c ablation results and next isolation

Flat completes 25 s with ID-WBC fraction 1.0, no IK failure, but legacy posture
P95 gates still fail. First 5 cm step probe passes the empirical physical
subclaim: full collision geometry exits, four sustained top-support witnesses,
no nonfoot collisions, max pitch 10.636 deg, ID-WBC 1.0. Foot-riser contact
remains (rear-right max 315.788 N), and longest aerial witness is only 8 ms.
The identical-SHA repeat fails IK at gait_time 13.504 (FL commanded body Z
-0.437541 m). This is NOT a reproducible candidate. Raw failed run is preserved.
See `stance_ablation_results.json` for exact manifests, raw hashes and results.

Independent left-end force integration versus measured subtree momentum gives
100 ms impulse residual norm p50/p95/max 0.00654/0.02443/0.04881 N s during the
successful interaction. This is a consistency diagnostic, not a new tuned gate.
The tool has analytic constant/variable-force/aerial, injected-impulse and
coverage rejection tests. MuJoCo derived kinematic quantities after mj_step
retain pre-integration kinematics; allow the documented one-physics-step
alignment offset rather than claiming exact controller/GT pose synchronization.

Code inspection also finds that UpdateWbcFull puts the legacy 0.42 m BASE
height constant directly in COM reference[5]. Observed COM is about 27 mm below
base; neutral foot-site Z is about -0.35 m. A new default-OFF research ablation
`TROT_RESEARCH_NOMINAL_COM_HEIGHT=1` derives level-body base/COM height from the
lowest eligible measured/committed foot site, neutral geometry and measured
COM/base offset. It anticipates a lower committed foothold; it is not an IK or
trajectory certificate and does not change the historical default. Aerial
intervals with no eligible support use current COM reference, explicitly
uncertified. Ten independent geometry/input fixtures pass. This is to be
compared after capture-pose map registration, which is being implemented
separately. No runtime result is attributed to either pending change yet.
