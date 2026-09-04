# Go2 Phase 2 current route

Updated: 2026-09-04. This is the only route/status entrypoint for the lean
line. Historical documents and experiment logs are evidence, not orders.

## Exact state

- Worktree: `/home/che/dev/go2-workspace/lean-20260904`
- Branch: `phase2-b1-b3-lean-20260904`
- Certified behavior base: `5b95e8265c885a81f8488e4930e682aa55f05674`
- Formal evidence: `docs/research/evidence/order109b_c006i/`
- B0: PASS only for that exact code base and frozen Order-109b conditions. The
  lean cleanup commit is not a new formal B0 certification.
- B1: FAIL / not accepted. B2 and B3: not started.
- Rejected work is preserved at
  `archive/phase2-b1-b3-misrouted-20260904@f3b6e966`.

## Authoritative route

The target is a sensor-derived, dynamic 5 cm B1 crossing using the frozen
`phase2-b123-v1` and `b1-contract-v1.0` contracts. Running-trot remains the
gait. The Phase-1 shaper remains the single velocity authority. A successful
crossing does not brake or stop, keeps a coherent time-indexed contact,
foothold and body plan, permits the normal two-contact diagonal support
interval, and is consumed atomically by gait, SRBD-MPC and ID-WBC. Planned
contact and measured force-supported contact remain distinct.

The next implementation slice is one owner and one snapshot: remove the
remaining internal reachability of the old crawl/low-stance/transfer state
machines, then introduce one dynamic `TerrainExecutionState` shared by gait,
MPC and WBC.
Do not tune support thresholds or add another recovery state while doing it.

## Retired routes

The following are prohibited as normal, fallback, staging or recovery policy:

- quasi-static or scripted crawl;
- an `>=3` contact gate or support preload whose purpose is to preserve a
  three-leg interval;
- cap-to-zero, `<=0.05 m/s` arming, transfer hold, or low stance on the happy
  path;
- fixed leg order, scene coordinates, XML/ground-truth input, or outcome-picked
  retries;
- `PHASE2_B123_ACCEPTANCE_CONTRACT_V2.md`,
  `PHASE2_ORDER090_LOW_STANCE_CRAWL.md`, and the V2-B portion of the former
  Stage-C design.

The old terrain-actuation CLI is fail-closed on this branch. Only
`--terrain-sensor-only` is usable until the new dynamic execution path has an
explicit contract-aligned CLI and tests.

## Work discipline

One hypothesis, one committed clean source, one B0 development regression,
then one B1 development canary. Stop at the first information-bearing failure.
Never use dirty-source output as verification. After three failed probes at the
same blocker, return to architecture review. Builds and CTest are necessary,
not locomotion acceptance. A B1 development PASS still requires fresh formal
B0 and the frozen B1 holdout before any acceptance claim.

## Authority order

1. Human owner decisions recorded here.
2. `PHASE2_B0_ACCEPTANCE_CONTRACT.md`,
   `PHASE2_B1_ACCEPTANCE_CONTRACT.md`, and
   `PHASE2_B123_ACCEPTANCE_CONTRACT.md`.
3. `PHASE2_WORKFLOW.md` and frozen manifests.
4. Raw evidence and analyzers.

Everything else is historical context and cannot change the route.
