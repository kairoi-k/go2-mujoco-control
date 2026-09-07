# Joint articulated model and contact-point probe
This is model/architecture evidence, **not a B1 traversal release**. The research
objective remains genuine 5 cm closed-loop running traversal and then 10 cm.
See `docs/research/STAGE_C_ARTICULATED_ROUTE.md` for formulation and limitations.
The controller build and all 56 registered CTests pass, including best-first
joint search and the zero-event continuous-only case. Focused tests cover footprint-local freshness:
stale or unknown cells outside a queried support patch do not invalidate that
patch, while stale/unknown cells within it do. All historical tests remain.
The V4 analyzer passes 10 adversarial/registration tests without editing V3.
The real-MJCF kinematic finite-difference error is 1.72e-10 m/s; reconstructed
position residual is 2.20e-8 m. Generalized acceleration reconstruction matches
an independent finite-difference oracle to 4.34e-9. A coupled 40 ms candidate
has nine articulated samples, floating dynamics residuals 7.23e-11 N and
1.52e-7 Nm, maximum inverse-dynamics joint torque 3.295 Nm. These are model
sample results, not final motor torque or swept collision certificates.
`synthetic_latency.txt` is an isolated 31-repeat benchmark after three warmups,
under `/tmp/go2_mujoco_experiment.lock`; no concurrent build/archive was run.
This tiny 40 ms case measures p50/p95/max 2093.12/2133.39/2147.16 microseconds.
It is not a realistic multi-event horizon latency claim.
The registered next experiment is a same-source off/on comparison of
`TROT_RESEARCH_CONTACT_POINT_MODEL` on flat ground, running-trot, 0.14 s period,
32 s duration, the existing b1_v3_running_1mps.csv profile, seed 11/domain 231.
All other research flags match the isolated active-force baseline. The new
flag supplies actual-radius surface-point Jacobians using the declared normal;
missing terrain normals retain the logged flat assumption. No joint-planner
adoption or measured-normal claim follows from this flag.
`pre_run_binding.json` binds runtime source and binaries before these runs.
`reproduce.py` reuses and hashes the prior WBC replay implementation, verifies
clean runtime SHA/source/binaries, checks actual point-mode CSV adoption, and
independently reruns certificate/topology analyzers. Raw `_runs` remain untouched.
The wrapper's legacy profile-analyzer failure is not physical run failure or
success; raw manifest, controller/simulator status and analyzer JSON decide.
