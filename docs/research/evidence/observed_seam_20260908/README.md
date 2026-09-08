# Observed collision seam correction
Research only; genuine B1 remains NOT_CERTIFIED. No controller, robot parameters,
frozen thresholds or unknown-cell coverage were changed.
Independent pre-fix one-sphere counterexample: plane normal14.3195N versus
adjacent-cell seam14.6866667N at center;2mm offset induces0.1275196m/s^2 lateral
acceleration on flat cells versus0 for a plane. Raw JSON and reproduction script
are retained verbatim from the independent323e51 review.
Correction source: 3d3a5f83bf1848bacc0d202ad1fad2861f8481c1. Equal-height, equal-yaw/source-compatible cells
merge into complete rectangles only. Original descriptor cells/provenance remain
immutable. Remaining edge-adjacent coplanar rectangles fail compilation, even if
metadata differs. L/T shapes are deliberately unsupported; no bounding-box hole
filling. Unequal heights, including5/10cm steps and slope cells, cannot merge.
This is a bounded correction, not arbitrary continuous terrain reconstruction.
Focused regression preserves robot mass/inertia/geometry/transmission/options,
90-degree capture yaw, real step sides, unknown/history rejection; checks a flat
compiled model against onebox and plane, old centered/2mm seams with a free sphere,
and one-step state agreement. At the recorded near-zero top (-5.96e-9m), whole
robot plane/box forces differ1.17333e-4N; this remains explicitly bounded by the
original isolated1e-3N fixture tolerance, not a B1 threshold. For a strict sphere
comparison both surfaces are translated equally by1m; force/vertical acceleration
differences are about7e-14 (1e-9 tolerance). Translation preserves relative
geometry/gravity. The simulator-specific near-zero plane difference is reported,
not asserted to be physically exact equivalence for arbitrary geometry/contact.
Build: cmake --build example/cpp/build --target test_stage_c_observed_collision_model -j2
Test: flock -n /tmp/go2_mujoco_experiment.lock example/cpp/build/test_stage_c_observed_collision_model
The final checkpoint records build/test stdout below. MuJoCo3.3.6 native WSL.
