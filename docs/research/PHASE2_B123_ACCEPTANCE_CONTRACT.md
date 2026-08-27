# Phase 2 B1-B3 Acceptance Contract

Status: FROZEN before independent B1-B3 execution work. Version:
`phase2-b123-v1`. This contract extends, and never widens,
`PHASE2_B1_ACCEPTANCE_CONTRACT.md`.

## Common contract

Every B1-B3 verdict requires the frozen Phase 1 `steps` profile quantitative
PASS, clean source, zero lifecycle failures, lidar-only controller input, and
the exact inherited and terrain thresholds in the B1 contract. B2 and B3 use
the same thresholds. Controller and planner may not read scene XML, ground
truth, geom identity, obstacle coordinates, or step index. Ground truth is
harness-only post-run scoring.

Completion requires a planned foothold to be consumed by gait, SRBD-MPC, and
WBC; coherent planned/measured contact transfer; measured force-supported
touchdown by every leg on each crossed terrain surface; zero obstacle
collision; the body and all feet beyond the final terrain; and at least 0.45 s
of stable post-crossing evidence. A brake, required-plan rejection, stale-plan
failure, deadline miss, safety stop, or incomplete crossing is FAIL.
During a terrain transfer the effective MPC preview must retain at least two
contacts, its knot-zero mask must equal the applied WBC support mask, and its
first/last horizontal velocity references must match the shaped/applied Phase 1
v_cmd within 0.020 m/s. A separate planner-derived horizontal reference span
above 0.001 m is a second velocity authority and is FAIL.

## Milestones

B1 uses a 5 cm single step and its frozen development/holdout scenes. B2 uses
the 10 cm single-step fixture and must reproduce the same complete crossing.
B3 uses mixed/repeated rises and descents, requires multiple plan/map epochs,
and must reproduce the complete crossing without scene-specific behavior.

Every milestone run records the exact scene, profile, effective arguments,
source and dirty state, binary/scenario/profile/contract/analyzer hashes,
controller diagnostics, simulator ground truth, Phase 1 quantitative JSON,
and `phase2_terrain_analysis.json`. Development failures remain evidence but
never count as acceptance.
