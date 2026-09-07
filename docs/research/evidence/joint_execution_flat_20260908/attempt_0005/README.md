# Executor tracking diagnostic: FAILED
Actual clean runtime4b09ccd217130c77b492040554c891d29746c7f5,
raw `example/cpp/experiments/_runs/joint_execution_flat_20260908_0005`.
Same fixed-start flat protocol. Logging is sampled and asynchronous start differs
from attempt0004; this is not an identical-state numerical replay.
At first sampled joint command STATE21.022, swing legs1/2 have requested versus
solved Cartesian acceleration norm errors41.76/70.42m/s2, while position errors
are12.8/12.0mm. Stance tasks are achieved in the model to displayed precision.
At21.060 leg1 is planned swing but measured contact; at21.066 all four measured
contacts coexist with planned aerial. Ground-truth confirmation is still needed;
these masks are sensors, not collision-truth replacement.
By21.086 leg1/2 position errors reach61.0/47.8mm. This evidence puts motion-task
compromise and actual contact timing before terminal numerical failure.
At21.402 the secondary QP fails active_KKT_numerical_failure, repeated identically;
planned mask6 versus measured1. Its exact matrices are retained separately.
191actual zero-extra-PD CSV commands,13versions, sampled maximum foot error
0.414984m. No B1 claim and no physical infeasibility claim from numerical failure.
Next use retained task/state evidence to investigate primary/secondary task
competition and contact-timing mismatch before further parameter experiments.
