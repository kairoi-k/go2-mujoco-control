# B1 physical evidence v2

This version supplies an independent empirical physical subclaim. It is not a
planner feasibility certificate or complete campaign acceptance. The original
frozen contracts, analyzers and result files remain unchanged. Its implementation
is `example/cpp/tools/analysis/analyze_b1_physical.py`; earlier unreviewed v2
analyzer drafts were rejected and are indexed in the iteration evidence packet.

Scope: one static world-attached axis-aligned box, ground at zero, top at 5 cm.
Other geometry/frame arrangements are unknown and rejected. Scene geometry and
MuJoCo truth are offline verification inputs only. The controller still receives
state estimates and lidar. A future campaign must independently bind clean source,
binaries, actual arguments, sensor provenance, residual checks, repeatability and
holdout results; a green physical report alone is not a candidate.

## Physical observation policy

The observation begins when the base reaches 0.5 m before the measured scene's
front edge. Completion needs a continuous 0.45 s interval in which every robot
collision geom is beyond the rear edge plus 20 mm, as are the foot sites.
`robot_collision_rear_bound_world_x_m` is the minimum geom center X minus MuJoCo's
orientation-independent bounding radius over collision geoms in the base-link
subtree. It is conservative and includes knees/calves, unlike a base-origin
proxy. Older logs lacking this field cannot pass the full-geometry gate; a
base/feet proxy may only bound their explicitly uncertified diagnostics.

Each leg must have at least 20 ms of observed >=10 N upward force on the box's
top, with its site inside the top rectangle. This is a load-bearing contact
witness, not a center-of-pressure margin. Actual top/non-top contact classification
is per MuJoCo contact and preserves both force channels. Nonfoot step contacts
fail this physical subclaim. Foot-riser force is reported independently; a
physical traversal never implies collision-free feet. Any contact-free claim
requires the separately reported nontop force to be zero within 1e-6 N.

The full observation must keep base height >=0.28 m and absolute roll/pitch <=15
degrees. The obstacle interaction interval must contain each diagonal force mask
and an aerial force interval for at least 4 ms. Leg contact uses full foot force
norm >=10 N; aerial uses total robot ground-reaction force norm <10 N, including
nonfoot force. A step-only contact mask is never an aerial measurement. Speeds,
effective duty range and solver fractions are reported for research judgment.
These brief witnesses do not alone establish a sustained, well-tracked running
gait; the campaign must also verify velocity tracking and schedule/force coherence.

Truth must be finite, strictly advancing, and have no gap >10 ms. Controller
association uses LowState state_tick_s (rounded MuJoCo physics time), never row
index or cmd_time_s. Duplicate state ticks are allowed; every truth row in the
observation needs a controller sample no more than 20 ms old. Estimated COM
stamps must agree within 4 ms and available lidar stamps must be no more than
200 ms old. These are timestamp/source coverage checks, not proof that an expired
planner trajectory remains valid. Active execution and sensor-only state are
checked throughout the associated window. Missing required coverage fails closed.

The policy was declared during failed development iteration, before a candidate.
Ten synthetic bookkeeping tests cover complete/incomplete exit, missing full-body
geometry, side-force misattribution, nonfoot collision, asynchronous/duplicate
state sampling, clock gaps, missing controller coverage, stale lidar, and NaN.
They are analyzer tests, not synthetic feasible robot dynamics. Native MuJoCo
contact fixtures independently verify logger force/frame/geom-order semantics.
