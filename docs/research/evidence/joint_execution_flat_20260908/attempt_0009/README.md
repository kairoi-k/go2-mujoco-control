# Proposal-based first acquisition: actual flat canary FAIL
Clean runtime6d0f75123d736f91c4ab114d37f8de178cfca54d, same0008 settings plus
TROT_RESEARCH_JOINT_INITIAL_PROPOSAL=1. Effective handover log confirms
initialization=proposal_reference; coherent_body=1. No state reset or later
handover relaxation. The new owner test passes and controller builds.
First applied21.026 (source21.020), five versions; stop21.332/count154 requests
reference_expired_or_unavailable. All154 active CSV rows have zero extra PD.
No logged posture stop or executor QP failure. Active-window max absolute IMU
roll15.470deg/pitch11.092deg; sampled max foot102.835mm/COM7.92756mm.
Sampled executor latency p50/p95/max291.771/325.1868/335.614us.
It still fails sustained flat execution, so no5cm or10cm attempt follows.
Initial nominal stance reference velocity is zero and has no commanded settling
curve. This is reference initialization, not an assertion of measured zero
velocity or a full articulated initial-state certificate.
Two distinct mechanisms appear after21.13. Planned touchdown precedes measured
support: at21.154 and21.174 nominal0110 versus measured0000. Several source
queries fail initial_contact_anchor_unavailable. Yet valid proposals also fail
to replace the active reference: source874@21.178 and876@21.246 take50.3124 and
49.8611ms in JointShadow. Owner logs status5 (commitment conflict) after21.236.
The old plan872 has next FL/RR touchdown21.284341264 and FR/RL21.348741264.
Proposal874 changes the former targets (FL x2.864579->2.869949); proposal876
preserves those but changes later FR/RL targets. CommittedEvents currently
exports only already-active leases, while RefreshActiveLeases can activate a
new swing during computation. This explains a concrete race mechanism; source
logs do not expose every rejected candidate/lease identity, so the exact
rejection branch should be reproduced with a deterministic delayed-adoption
fixture before changing the protocol. Missing anchors alone do not explain
reference starvation, and a longer force-validity lease is not a valid fix.
Next: establish a computation-aware frozen prefix/publication deadline within
the single execution owner, retaining true committed targets and fail-closed
contact provenance. Do not extend stale force validity or freeze the entire
future horizon merely to suppress rejection. No B1 acceptance claim.
