# Shared orientation/swing priority flat experiment: FAILED
Actual clean runtime9c5c836fe98370812f0f1bc94b9c2ec718d638e6,
raw `example/cpp/experiments/_runs/joint_execution_flat_20260908_0007`.
Same fixed-start flat runner, explicitly adds
`TROT_RESEARCH_JOINT_SOFT_ORIENTATION=1`. First captured solve STATE21.024
confirms primary_orientation=0 and15equality rows. Other tasks, weights,
friction/torque limits and gait settings are unchanged. Focused tests verify
secondary H/g and force/torque constraints are identical across modes.
This candidate FAILS: last sampled applied command21.168, count73,2versions;
72raw zero-extra-PD rows in the analyzer's half-open sampled window. Upstream
posture gate reports roll22.8689deg,pitch12.02deg. No WBC failure occurs.
Sampled foot/COM errors0.428841/0.0151966m; latency260.8235/600.8545/654.548us
p50/p95/max. No B1 claim. first_stop=null is not successful completion: the
upstream posture gate bypasses the executor stop-request log.
The old hierarchy remains the default. This explicit negative candidate is
preserved, not promoted. Startup/adoption states differ across asynchronous
full runs, so elapsed execution counts alone do not prove a causal effect size.
The fixed-state attempt0006 remains the causal task-cost isolation evidence.
A priority change alone has not established body/leg/contact-consistent running.
Next review the connection between articulated planning, achievable motion
references and the unilateral compliant contact realization; do not continue
unbounded weight sweeps or change acceptance to bless this failure.
