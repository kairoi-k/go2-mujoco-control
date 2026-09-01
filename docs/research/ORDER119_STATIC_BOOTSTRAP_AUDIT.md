# Order-119 static bootstrap audit

Status: STATIC REVIEW ONLY. No simulator run, no runtime authorization, no acceptance claim.

Baseline: `phase2-b1-b3` at `066348550c87cf6ee7142197f228987ed2fd74be`. C-002 tested source: `ba7c47ad3276f3e2adf2a48f7ac923d0501acfc7`.

## 1. C-002 evidence boundary

C-002 was explicitly SHADOW ONLY. The legacy/running-trot path moved the robot while the planner observed. The warmed-map shadow result therefore proves two separate facts: (1) the existing locomotion stack could physically traverse the 5 cm scene, and (2) the planner could construct a valid family on a sufficiently warmed map. It did not prove that the planner could generate, publish, adopt, and own the first live transition before any perception-warming motion.

The current branch is 55 commits ahead of the C-002 tested source. The important semantic change is the conversion from parallel shadow observation to a serial live dependency: planner validity/publication/adoption/contact guards now gate terrain execution.

## 2. Bootstrap diagnosis

The conceptual deadlock is real:

`no motion -> insufficient/unknown ROI -> no publishable C1 plan -> no motion`.

The dual-certificate C0/C1 design in `PHASE2_STAGE_C_BOOTSTRAP_ARCHITECTURE.md` is the correct architectural direction because C0 may perform bounded observation motion without waiting for C1, while C1 alone owns timed terrain transition execution.

The current blockage is no longer a conceptual architecture question. It is a scope/governance problem: Order-116 promoted several final-certification proof obligations into prerequisites for even a development prototype, then Order-117/118 started building passive observability infrastructure without an authorized runtime question that the observer can actually answer.

## 3. Static blocker triage

### A. Must fix before any live C1 development probe

1. **Fallback gait restoration seam.** `TerrainPlanExecutionAdapter::ApplyToKernel` writes gait pattern/period/duty/step/lift only when `using_plan_ && last_request_.valid`. When a plan expires or contact guard enters fallback, `Update()` creates a valid fallback request with the Phase-1 fallback gait values, but `ApplyToKernel()` clears the execution request and does not apply those fallback setters. A prior terrain plan can therefore leave terrain gait parameters resident in the kernel after fallback. This is a real control correctness defect, not merely a proof gap.

   Required static change: keep `request.has_execution_request` restricted to live timed-plan execution, but whenever `last_request_.valid` apply its pattern/period/duty/step/lift to the kernel. Add a regression test that first applies a terrain plan, then forces fallback, and proves the kernel receives the configured Phase-1 fallback gait values.

2. **Atomic first ownership boundary.** Before C1 drives a timed transition, gait and SRBD must consume the same adopted snapshot identity. C-004 already established the intended seam. A development bootstrap may reuse that exact atomic adoption rule; it does not need a new generalized lease/token framework before the first bounded probe.

3. **Measured/planned separation remains non-negotiable.** Planned contact must not become measured support or WBC safety contact. C-005 semantics remain required.

### B. Required for acceptance/safety certification, but not a prerequisite to a bounded simulation development probe

1. Per-cell raw-value/timestamp/ray provenance sufficient to formally certify C0 observation-viewpoint causality.
2. Full worst-case sensor-to-halt latency and discrete Dstop proof.
3. Generalized C0/C1 certificate leases, independent consumer ack deadlines, invalidation/preemption proofs.

These are legitimate final proof obligations, but treating all of them as prerequisites to run a single development-only 5 cm bootstrap experiment creates an infrastructure loop. The development probe must be explicitly non-acceptance and cannot weaken B1 thresholds or claim those obligations are satisfied.

### C. Stop extending unless tied to a concrete hypothesis

The standalone Order-117/118 observer is currently `interface_partial`; the latest LowCmd addition only counts messages and deliberately does not inspect command values. Runtime probing is still unauthorized. This cannot currently answer the central bootstrap question (whether bounded C0 observation motion warms the ROI enough for C1 publish/adopt), so no further observer expansion is justified until a specific required datum cannot be obtained from existing production/run telemetry.

## 4. Minimal development bootstrap contract

A single development-only path should be prepared behind an opt-in flag:

1. Start from the already validated Phase-1 locomotion surface.
2. Planner remains shadow while C0 owns motion.
3. C0 permits only generic bounded forward observation motion inside already-known flat/support-clear local terrain and the existing velocity shaper; no obstacle coordinate, fixed leg order, terrain FSM, or scripted perception maneuver.
4. If C0 local validity is lost, command brake/hold. Never enter unknown terrain to gain perception.
5. Once the ROI is sufficiently observed and one complete Family-A timed plan is publishable, stop/hold at a legal boundary, atomically publish/adopt that snapshot, verify gait/SRBD identity agreement, then transfer transition execution to C1.
6. If C1 cannot arm at that boundary, remain/return in Phase-1 brake/hold; do not repeatedly patch gates in the same run.
7. One fixed 5 cm development probe answers only: can bounded observation motion break the bootstrap deadlock and produce one live publish/adopt/first touchdown transaction? It is not B1 acceptance.

## 5. Stop rule

Do not add another generic diagnostic layer after this audit. Static work should be limited to the real fallback seam and the minimum bootstrap state/ownership interfaces needed for the development contract. After those compile/tests are available, the next information-bearing action is one bounded local build/CTest plus one 5 cm development probe. If that probe still cannot reach live adoption/first touchdown, classify the exact failure and revisit the planner/bootstrap architecture rather than beginning another open-ended instrumentation campaign.
