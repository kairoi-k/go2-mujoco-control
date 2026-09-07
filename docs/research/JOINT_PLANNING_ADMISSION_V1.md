# Joint planning admission protocol V1
Research option; historical behavior remains default. This is an asynchronous
planning/execution protocol, not contact evidence or traversal acceptance.
At control snapshot time t, an explicit admission budget B yields inclusive
latest_adoption=t+B. The owner exports currently active touchdown commitments
plus targets from its accepted plan whose liftoff<=latest_adoption and whose
touchdown is still future. These additional targets are producer constraints
for this solve, not measured contact or a reservation of every future step.
Their event identities, original target provenance and times remain unchanged.
The worker binds them through the existing candidate/commitment seam while
optimizing remaining footholds and continuous dynamics as before.
The immutable result carries latest_adoption independently of valid_until.
The owner rejects new results arriving after latest_adoption; previously
accepted results remain usable only within their original trajectory validity.
It still validates live leases at admission: the prefix is not permission to
change a swing already in progress. No old force trajectory is extended and
no unobserved support anchor is invented. An unresolved exported event fails
prefix construction. Zero budget keeps the prior protocol.
TROT_RESEARCH_JOINT_ADMISSION_BUDGET_S=0.080 registers a bounded flat canary0010
against0009, with all other settings retained.80ms is an explicit engineering
budget for this experiment, not a proven upper bound on solver latency. Actual
absolute-time admission checks enforce it. Publishing records source, deadline
and protected event count. Rejection/expiry remains a failure, not acceptance.
Deterministic owner test: source before a1.15s liftoff, adoption1.16s. An
unprotected changed target is rejected; the deadline1.18s protected target is
accepted. Admission1.19s fails although force coverage lasts to1.5s; an already
accepted proposal remains valid beyond admission deadline until its own end.
A deadline1.14s does not freeze the1.15s future event. No gain, analyzer or
historical threshold changes are part of this protocol.
