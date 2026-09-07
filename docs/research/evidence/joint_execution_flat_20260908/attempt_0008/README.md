# Coherent-body actual flat canary: FAIL
Clean runtime9c1ed03c9099809bcb22123fc7599c87e2759752, raw
example/cpp/experiments/_runs/joint_execution_flat_20260908_0008.
Actual first QP log confirms coherent_body=1, primary_orientation=1 and
solution_applied=1. Fixed-start21s, prior0.14s running schedule, all previous
canary settings; no threshold changes. Four focused tests passed first.
First appliedSTATE21.004, four accepted versions, last sampled21.278/count137;
136 CSV active-window rows have zero extra motor PD. Upstream posture stops
at roll-22.0233deg/pitch9.9677deg (later roll reaches-22.904deg). No executor
QP failure reported. first_stop=null means no executor stop log, NOT success.
Sampled max foot139.332mm, COM11.6249mm. Sampled executor latency
p50/p95/max287.864/334.1151/344.2us; this is not an all-tick latency bound.
First post-adoption proposal rejection at21.132 is initial_contact_anchor_unavailable.
Independent exact actual-QP replay matches the captured solution, preserving
original constraints; the numerical solver is not demonstrated as the cause.
A concrete handover discrepancy precedes failure. Initial legacy-command seed
stance vx=-0.2604/-0.2381m/s versus actual+0.2021/+0.2371m/s forFL/RR.
The command settling curve requests stance ax52.0847/47.6217m/s2; after feedback
actual WBC tasks request47.5594/42.9218m/s2. Swing seed vx2.4207/2.3976m/s
versus actual1.1937/1.2287m/s. Short counterfactual replay initializes from the
recorded state, without this legacy command handover. These are observations
that motivate a matched-handover comparison, not proof of sole causality.
The two short-phase improvements do not establish continuous execution.
Do not promote this candidate or sweep gains. Next compare first takeover
reference initialization while preserving actual q/dq and accepted later
commitments; separate command continuity from measured-state continuity.
B1 remains NOT_CERTIFIED, no new5cm or10cm run.
