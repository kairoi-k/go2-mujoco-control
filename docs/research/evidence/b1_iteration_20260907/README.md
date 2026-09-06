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
reversed contact-order algebra plus unsupported geometry rejection. Six checks pass. Full integration CSV alignment
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
