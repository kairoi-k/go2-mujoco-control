# World swing execution consistency, development only

This change is not a B1 candidate, a continuous collision proof, or a dynamic
feasibility certificate. It repairs the trajectory/coordinate/time contract
between candidate screening and execution before further closed-loop research.

A published plan owns the immutable TerrainModel used to create it. Runtime
latch uses that snapshot rather than a concurrently newer diagnostics model.
World foot-site curves share one evaluator for checking and command generation.
Heading XY is yaw-aligned and relative to the registered base; heading Z is
world-axis-aligned but relative to the same base Z. Full quaternion rotation
converts heading/world geometry to body-frame IK. Registered position, yaw,
map epoch and state stamp must match the curve frame.

Candidate generation checks normalized geometry only. Runtime must recheck
from the previous commanded world foot to the selected contact patch plus the
22 mm foot-site offset over the actual remaining Phase-1 swing. The planner's
50 ms contact grid is a candidate event association, not an exact touchdown
clock. No local swing retiming is introduced. A change to the subsequent global
gait schedule is not certified by this fixed-interval geometry check.

The world path does not inherit legacy unknown-anchor imputation, minimum-height
anchor forgiveness or unobserved-fringe skips. All queried patches must be
known; initial and terminal foot bottoms cannot penetrate patch maximum height
beyond 1 micrometre of float-conversion tolerance. Runtime masks individual
cells that become older than the existing 200 ms map-age limit by touchdown.
Fresh envelope timestamps do not refresh older cell observations.

The checker uses finite foot/shin samples and IK at a fixed snapshot body pose.
It does not certify swept geometry continuously, future body motion, joint
tracking, forces, impact, or MPC/WBC commitment consistency. Real MuJoCo evidence
and independent force/contact acceptance remain required. Missing or rejected
terrain trajectories retain the existing nominal fallback; that fallback is
explicitly uncertified for terrain traversal.

Independent tests include physical world-height arithmetic, positive grounded
swings, specific-reason unknown/fringe rejection using IK-reachable points,
initial 5 mm penetration, registered-pose mismatch, per-cell age and absolute
horizon limits. The corrected endpoint fixture removes a duplicated 22 mm
offset that previously allowed a floating endpoint to masquerade as grounded.

Synthetic latency source and raw output are preserved in
`example/cpp/experiments/_runs/b1_swing_validation_20260907_0001/`.
A flat 1600-cell snapshot, 78.4 ms swing and 1001 independent floor checks were
used. These timings are geometry-check timings, not a planner/control budget.
Full controller Release build and 42/42 CTest passed; simulator CTest 3/3 and
focused Python tests 48/48 passed (19 protocol, 15 V3 wrapper, 10 physical,
4 impulse). Raw initial failures are preserved: old interface fixtures lacked
mandatory quaternion provenance and placed foot sites on the contact plane;
the swing fixture required a reachable grounded endpoint. These were corrected
without changing legacy physical acceptance thresholds.

The final 1000-check benchmark returned p50/p95/max 3.006/3.055/46.328
microseconds. Dense independent flat-floor checking found minimum foot-bottom
height 0, endpoint Z error 0 and curve-parameter consistency error
1.11e-16 m. This is a simple flat geometry fixture, not a terrain dynamics oracle.
`source_binding.json` records 133 production/build input hashes and the controller
and simulator binary hashes. Final benchmark source, binary and output are also
retained with hashes in `swing_validation.json`.
