# Shared model foot geometry observation seam
Baseline checkpoint: 1f2a2f415f748a790cc398066f67e37ad59f8ebd.
## Purpose and boundary
The actual Go2 MJCF distinguishes FK contact site, collision sphere center,
and terrain surface contact point. The former two are approximately 2 mm
apart in the local calf frame; the collision radius is 22 mm. Production
must read these values from the model, not encode those numbers as universal
geometry. A future site-to-surface conversion also needs the selected surface
normal and the future calf pose; base orientation alone does not suffice.
Extend the existing Go2RigidBody owner with immutable geometry metadata and
same-state site/geom observations. Keep foot_pos_world as the existing geom
center interface. Local geom/site positions retain separate parent body IDs.
A missing site or non-sphere invalidates the new geometry validity flag without
turning previously usable rigid-body dynamics into a failed Load. No raw
MuJoCo ownership escapes. All new numeric observations initialize explicitly.
This is a model observation seam, not a new point-conversion algorithm, future
body prediction, planner replacement or execution authorization. Existing
terrain flat-offset helpers remain a named legacy limitation.
## Independent verification
Use the actual production MJCF for every leg, independent mj_forward state
construction, body roll/pitch/yaw plus distinct leg joint positions, and
analytic FK site comparison. Verify local metadata/radius against the model,
world site and geom against independent evaluation, and demonstrate that calf
rotation differs from base rotation for the center-to-site displacement.
Explicit negative metadata cases must preserve unknown rather than fabricate
a valid sphere/site pair. Rebuild and run controller tests; no new closed-loop
performance claim is attached to this pure observation extension.

## Review and fixture correction
Root review removed a new skip-Jacobian branch to preserve the existing
dynamics path; the change remains observation-only. New Eigen values initialize
explicitly, and negative fixture files use exclusive unique temporary names.
The first negative fixtures lacked inertia on moving links and were rejected
by MuJoCo before reaching the intended metadata case. The failed CTest log is
retained in fixture_build_failure.txt; the fixture model is corrected instead
of weakening geometry validity checks or skipping the negative cases.

## Final validation
Full controller build succeeds; all45 controller CTest cases pass.
The focused geometry test passes production model/FK comparisons and both
negative models. Site/FK tolerance1e-7 m, independent world-point comparison
tolerance1e-10 m, radius/local-metadata tolerance1e-12 m. These are asserted
comparison tolerances, not measured maximum residual claims. Existing model
check reports mass15.2064 kg and inverse-dynamics residual0.0630302 in its
original aggregate norm; no new dynamics solver or tolerance was introduced.
No closed-loop run uses this metadata-only final commit. Last executed runtime
remains43c5f5a; all B1 status and flat behavior caveats remain unchanged.
