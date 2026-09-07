# Bounded-admission actual flat canary: FAIL
Clean runtime97b6d02158a797f1bb02e9817495152964a3a322. Same0009 flags plus
TROT_RESEARCH_JOINT_ADMISSION_BUDGET_S=0.080. The deterministic owner test
reproduces delayed conflict and covers protected admission, deadline endpoint,
late rejection and unchanged accepted trajectory validity. Controller builds.
Actual first21.012,15 accepted versions, last sampled21.658/count325, then
posture roll23.8593deg/pitch14.4483deg. No executor expiry/QP stop logged;
first_stop=null does not mean success. Sampled max foot242.760mm/COM24.177mm.
Executor sampled p50/p95/max278.056/360.3623/456.382us. Still failed flat.
Publication logs record absolute deadlines and protected event counts. All15
observed admissions meet their deadline; maximum source age52ms. No sampled
commitment-conflict or stale-rejection statuses. Protected prefixes typically
contain4 of8 core events, leaving future targets available for optimization.
This supports the bounded-admission mechanism in this run; different initial
phases and timing prevent treating command counts across runs as a controlled
causal effect size. All original force validity and live-lease checks remain.
The persistent physical posture/foot failure is now the next issue. In the
coherent mode, body angular acceleration is derived from COM/Ldot and foot
acceleration targets; the independent attitude PD is deliberately bypassed.
This supplies task compatibility but does not regulate absolute body attitude.
A coherent horizon-level body/limb momentum reference or articulated planning
must provide that missing regulation, rather than adding a conflicting body
acceleration lock or relaxing posture/force thresholds. Contact realization
and strict full-body initial-state compatibility also remain unresolved.
No5cm/10cm or B1 acceptance claim. Raw joint_execution_flat_20260908_0010 is
preserved; execution_audit binds its hashes. admission_audit is sampled evidence.
