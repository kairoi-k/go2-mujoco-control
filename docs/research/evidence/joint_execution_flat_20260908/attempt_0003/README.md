# Actual primary-QP numerical failure and regression
Runtime f58695069e832a8c58acc4b5614245e11154d965, raw run
`example/cpp/experiments/_runs/joint_execution_flat_20260908_0003`.
Flat diagnostic FAILED at STATE 21.110 after 53 actual zero-extra-PD commands.
Planned and measured masks both equal 6. No B1 claim.
The exported primary body/stance QP has 24 variables, 6 equalities and 36
inequalities. Independent HiGHS feasibility and original-coordinate KKT witness
prove feasibility and the strictly convex optimum 17524222.67510441.
Original solver reproducibly rejects iteration 12: an active constraint drifts
by 5.782286335835402e-7 in original units (limit 5e-7), despite whitened
violation about 1.03e-13. The saddle-point direction leaves active-row tangency
error about 1.99e-12. This is numerical failure, not physical infeasibility.
QR working-set nullspace projection preserves the same objective, constraints,
seed and acceptance tolerances. The regression obtains objective
17524222.675105095 in 16 iterations, equality residual 5.53e-14 and inequality
violation 2.75e-11. Both this matrix and the previous runtime/analytic tests pass.
Run `ctest --test-dir example/cpp/build -R test_dense_qp_active_set --output-on-failure`.
This repair has not yet been tested in a new full runtime. The failed runtime
also reports sampled foot-reference error up to 0.118023 m; small inverse-dynamics
residuals do not certify tracking or traversal. Variable-period commitments and
initial contact-anchor availability remain unresolved.

Independent reproduction: `python3 docs/research/evidence/joint_execution_flat_20260908/attempt_0003/verify_kkt.py`.
The supplied active support is accepted only after positive-definite Hessian,
original-coordinate primal feasibility, stationarity and nonnegative dual checks.
