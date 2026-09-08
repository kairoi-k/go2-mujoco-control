# Whole-body MPC oracle evidence (2026-09-08)
This directory contains privileged, known-scene, full-state research-oracle evidence. It is not runtime authority and contains no B1 acceptance certificate. The raw run artifacts are archived under `raw/`; `MANIFEST.json` records each original path, raw SHA-256, archive SHA-256, byte counts, and the gzip byte-equality check.
## Curated raw runs
- `0002`: smoke run, source `caae517b60e9386f0f9f1ffb24e12120a8f01187`, V1, two chunks and ten executed steps, completed for plumbing coverage only.
- `0003`: V1, four solver iterations, failed at chunk 20 after twenty completed chunks. Its code/protocol numeric gate mismatch is retained: the gate covered only state, force, and torque. `failure_audit.json` and `same_state_20_iterations.json` are offline diagnostics, not a certificate or execution admission.
- `0004`: V2 source `584e714dc8889f55e01c6fbc940ecfcb7117c184`, with the corrected gate, failed at chunk 35 after thirty-five completed chunks. The 175-step replay has state, force, and motor deltas of zero; normwise dynamics residual is `1.3569075029961537e-9`; the historical absolute residual is `4.3351886347409163e-7` and its check fails.
The first real scene contacts in 0004 occur at `t=21.286 s` for FL and `t=21.316 s` for FR. Both are vertical sidewall contacts (`normal_world=[1,0,0]`), not top contacts. The audit records 31 contact samples, zero top-contact samples, and a maximum sidewall normal force of `126.4267624 N`. The failed chunk 35 baseline violates the future FL calf upper-joint-limit constraint at component 37. Solver latency p50/p95/max is `13.7525583/15.3832912/15.6065959 s`.
No B1 success is claimed. Root-owned V3 work addresses support affordance and geometry clearance; it is not part of these archives.
## Earlier retained evidence
Source `99d256dd02c811a83f62f76dcd14fe500be1685f` enumerated all 25 first simultaneous in-flight touchdown pairs at recorded 0012 source 21.020. All centroidal candidates were feasible, but all failed the shared initial articulated torque check; the best `(2,2)` was `58.3664345292 Nm` against a `35 Nm` limit. This rules out only searching those existing footholds with the same stitched trajectory at that initialization, not global trajectory or 5 cm infeasibility.
The static target audit independently reproduces FL/RR displacements of `30.533/31.196 mm`. The legacy target is already `25.715/26.384 mm` ahead of the actual geometry; the geometry/site correction is about `-1.6 mm` and future COM translation `6.414 mm`. The old actual commanded curve is absent, so bootstrap inheritance fault is not established by this evidence.
The native evaluator audit source `89040b13185794177d9353f3f4d709425537ecfc` used the actual 5 cm scene with the original x=2.577 initial state, so it did not encounter the obstacle. Native/Python constraint deltas were zero, maximum cost delta was `3.47e-18`, repeated outputs were exact, and ten fail-closed checks passed. This is evaluator equivalence, not full-cycle movement or terrain capability.
The sequential verifier fixture source `56e0b1e2c8649283ef0a39a50beccbd9b366bdcf` had zero state/force/motor discrepancy on one-step replay and rejected deliberate saved-state and force tampering. It is a verifier fixture, not traversal evidence.

## V3/V4 and explicit user stop
Source1f36bc4 top-support audit0005: retained wall-contact witness changes from
legacy min inequality0 to-161.701N in forbidden-contact component; native/Python
maxdelta1.42e-14. Flat fixture remains accepted. Seven geometry envelope tests pass.
V3 canary0006 stopped after65steps before actual collision. Its failed horizon
predicted a forbidden wall contact. Inspection showed only5committed controls
were retained; the described65step seed transport was absent.
Source5033240 fixes transport and uses4correction nodes[5,35,64,69]. On the exact
0006 failed state, transported65steps remain feasible; the full solve finds a
strictly sampled feasible witness in15.6666s with exact prefix and Python min g0.
This is a same-state counterfactual, not completed traversal.
Run0007 source503324035880a70d40530337d1e348872fa1bfb9 was interrupted at the user's
quota-stop request after2completed chunks/10executed steps. Original run.json
status started is retained verbatim, NOT interpreted as a live process.
user_stop_request.json/shutdown_receipt.json record SIGINT, KeyboardInterrupt,
confirmed process disappearance and free experiment lock. No research process
is intentionally left running. No further experiment until user resumes.
