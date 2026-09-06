# Registered terrain intervals V2 research protocol

This is an observation-representation experiment, not a dynamics certificate
or a replacement for B1 Dynamic Traversal V3. The V1 map envelope, transport,
registration default and frozen acceptance remain identifiable and unchanged.

## Formulation and limitation

For a destination heading-frame cell, transform its corners into the capture
frame and conservatively enumerate the source-cell bounding box. V1 rejects
nonuniform heights. V2 retains the minimum and maximum of all finite, fresh
source heights, with the capture-to-current world-Z translation applied to
both bounds. Its scalar representative is the upper bound. Unknown, future,
stale or out-of-source coverage remains unknown. Landing patches must still
satisfy the original height-spread, slope, reachability and uncertainty gates;
swing checks use the upper bound. A mixed ground/top cell is thus useful as an
obstacle bound while remaining unsuitable as a level landing footprint.

This interval encloses the source piecewise-constant sampled heightfield,
not every point of the actual continuous MuJoCo terrain. Each source cell is
currently represented by a downward ray at its center. A subcell obstacle can
be missed. No continuous collision proof or real lidar fidelity is claimed.

## Activation and controlled tests

TROT_RESEARCH_MAP_INTERVALS_V2 accepts exactly 0 or 1; unset means V1.
The effective numeric policy is logged at initialization and the environment
is retained in each raw manifest. No wire format changes are needed.

Before actuation, run focused V1/V2 fixtures and all controller CTest. The
first closed-loop comparison uses the unchanged V3 1 m/s profile and default
0.14 s period, 0.44 duty, original lift, with V2 as the only functional change
relative to fb14417. Run flat first, then the 5 cm step if flat remains viable.
Keep runtime swing geometry checks, initial penetration rejection, per-cell
horizon age and unknown coverage unchanged. Planner diagnostic logging is off
for the timed comparison. Additional aggregate diagnostic runs are separate
and cannot establish acceptance or deterministic timing equivalence.

Outcomes: exact-SHA raw manifests, full runtime completion/safety/IK, actual
terrain preparation/application counts, first riser contact, top/nonfoot
forces, approach and interaction velocity, actual contact-cycle topology and
V3 verdict. A map-coverage improvement alone is not a B1 candidate.
